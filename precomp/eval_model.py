"""
eval_model.py
=============
Eval a trained YOLO model: per-class mAP report + stratified inference grid.

Outputs:
  runs/eval/report.json
    {
      "model": "path/to/best.pt",
      "splits": {
        "val":  { "mAP50": ..., "mAP50_95": ..., "precision": ..., "recall": ...,
                  "per_class": {"yellow_barrel": {"mAP50": ..., "images": 6}, ...} },
        "test": { ... }   # only if test split present
      }
    }
  runs/eval/grid.jpg
    Stratified: at least one image per class. Each tile shows ground-truth (green)
    + predictions (red) overlaid. Classes with zero examples get a banner tile.
"""

import argparse
import json
import os
import random
import sys
from glob import glob
from typing import Dict, List, Optional

import cv2
import numpy as np
import yaml


def _per_class_from_val(val_results, class_names: List[str]) -> Dict[str, dict]:
    """Pull per-class mAP50 from ultralytics validation object."""
    out: Dict[str, dict] = {}
    try:
        box = val_results.box
        # ap50 is shape (nc,); maps is per-class mAP50-95
        ap50 = box.ap50.tolist() if hasattr(box, "ap50") else []
        maps = box.maps.tolist() if hasattr(box, "maps") else []
        for i, name in enumerate(class_names):
            out[name] = {
                "mAP50": float(ap50[i]) if i < len(ap50) else 0.0,
                "mAP50_95": float(maps[i]) if i < len(maps) else 0.0,
            }
    except Exception as e:
        print(f"[eval] WARN per-class extraction failed: {e}", file=sys.stderr)
        for name in class_names:
            out[name] = {"mAP50": 0.0, "mAP50_95": 0.0}
    return out


def _label_class_ids(labels_dir: str, stem: str) -> List[int]:
    path = os.path.join(labels_dir, stem + ".txt")
    if not os.path.isfile(path):
        return []
    ids = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 1:
                try:
                    ids.append(int(parts[0]))
                except ValueError:
                    pass
    return ids


def _stratified_samples(img_dir: str, lbl_dir: str, class_names: List[str], per_class: int, rng: random.Random) -> Dict[str, List[str]]:
    """Group images by which class they contain; sample up to `per_class` per class."""
    by_class: Dict[str, List[str]] = {n: [] for n in class_names}
    for img_path in sorted(glob(os.path.join(img_dir, "*.jpg")) + glob(os.path.join(img_dir, "*.jpeg")) + glob(os.path.join(img_dir, "*.png"))):
        stem = os.path.splitext(os.path.basename(img_path))[0]
        ids = _label_class_ids(lbl_dir, stem)
        for c in set(ids):
            if 0 <= c < len(class_names):
                by_class[class_names[c]].append(img_path)
    picked: Dict[str, List[str]] = {}
    for name in class_names:
        pool = by_class[name]
        rng.shuffle(pool)
        picked[name] = pool[:per_class]
    return picked


def _draw_boxes(img: np.ndarray, boxes: List[tuple], color: tuple, label_prefix: str = ""):
    for (x1, y1, x2, y2, label) in boxes:
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        if label:
            text = f"{label_prefix}{label}"
            cv2.putText(img, text, (int(x1), max(int(y1) - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)


def _gt_boxes(lbl_path: str, img_w: int, img_h: int, class_names: List[str]):
    out = []
    if not os.path.isfile(lbl_path):
        return out
    with open(lbl_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            try:
                cid = int(parts[0]); cx, cy, w, h = (float(p) for p in parts[1:])
            except ValueError:
                continue
            x1 = (cx - w / 2) * img_w; y1 = (cy - h / 2) * img_h
            x2 = (cx + w / 2) * img_w; y2 = (cy + h / 2) * img_h
            name = class_names[cid] if 0 <= cid < len(class_names) else str(cid)
            out.append((x1, y1, x2, y2, name))
    return out


def _annotate(img_path: str, lbl_path: str, model, class_names: List[str], conf: float = 0.25) -> np.ndarray:
    img = cv2.imread(img_path)
    if img is None:
        return np.zeros((320, 320, 3), dtype=np.uint8)
    h, w = img.shape[:2]
    gt = _gt_boxes(lbl_path, w, h, class_names)
    _draw_boxes(img, gt, (0, 200, 0), label_prefix="gt:")
    try:
        results = model(img, verbose=False, conf=conf)
        preds = []
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                cls = int(box.cls[0].cpu().item())
                cf = float(box.conf[0].cpu().item())
                name = class_names[cls] if 0 <= cls < len(class_names) else str(cls)
                preds.append((x1, y1, x2, y2, f"{name} {cf:.2f}"))
        _draw_boxes(img, preds, (0, 0, 220), label_prefix="pr:")
    except Exception as e:
        cv2.putText(img, f"infer err: {e}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return img


def _placeholder_tile(text: str, size: int = 320) -> np.ndarray:
    img = np.full((size, size, 3), 40, dtype=np.uint8)
    cv2.putText(img, text, (10, size // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def _build_grid(tiles: List[np.ndarray], cols: int = 3) -> np.ndarray:
    if not tiles:
        return _placeholder_tile("no tiles")
    h, w = 320, 320
    norm = []
    for t in tiles:
        if t.shape[:2] != (h, w):
            norm.append(cv2.resize(t, (w, h)))
        else:
            norm.append(t)
    rows = (len(norm) + cols - 1) // cols
    canvas = np.full((rows * h, cols * w, 3), 0, dtype=np.uint8)
    for i, t in enumerate(norm):
        r, c = divmod(i, cols)
        canvas[r * h : (r + 1) * h, c * w : (c + 1) * w] = t
    return canvas


def _eval_split(model, data_yaml: str, split: str, class_names: List[str]) -> dict:
    """Run ultralytics val on a split. ultralytics uses 'val' by default; for 'test'
    we override `split='test'` (requires test: in data.yaml)."""
    try:
        if split == "val":
            res = model.val(data=data_yaml, verbose=False)
        else:
            res = model.val(data=data_yaml, split="test", verbose=False)
    except Exception as e:
        print(f"[eval] WARN val on split={split} failed: {e}", file=sys.stderr)
        return {"error": str(e)}
    out = {
        "mAP50": float(res.box.map50) if hasattr(res, "box") and hasattr(res.box, "map50") else None,
        "mAP50_95": float(res.box.map) if hasattr(res, "box") and hasattr(res.box, "map") else None,
        "precision": float(res.box.mp) if hasattr(res, "box") and hasattr(res.box, "mp") else None,
        "recall": float(res.box.mr) if hasattr(res, "box") and hasattr(res.box, "mr") else None,
        "per_class": _per_class_from_val(res, class_names),
    }
    return out


def run(ctx=None, **overrides) -> dict:
    if ctx is None:
        class _C: pass
        ctx = _C()
        for k, v in overrides.items(): setattr(ctx, k, v)

    data_yaml = getattr(ctx, "data_yaml", None)
    if data_yaml is None:
        data_yaml = os.path.join(getattr(ctx, "data_root", "data"), "_smoke" if getattr(ctx, "smoke", False) else "", "data.yaml")
        if getattr(ctx, "smoke", False):
            data_yaml = os.path.join(ctx.data_root, "_smoke", "data.yaml")
    if not os.path.isfile(data_yaml):
        raise FileNotFoundError(f"{data_yaml} missing")

    model_path = getattr(ctx, "model", None)
    if not model_path:
        raise ValueError("ctx.model (path to best.pt) required")
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"{model_path} missing")

    out_dir = getattr(ctx, "eval_dir", "runs/eval")
    os.makedirs(out_dir, exist_ok=True)

    with open(data_yaml) as f:
        d = yaml.safe_load(f)
    class_names = list(d["names"]) if isinstance(d["names"], list) else [d["names"][i] for i in sorted(d["names"])]
    data_root = d["path"]

    from ultralytics import YOLO
    model = YOLO(model_path)

    report = {"model": model_path, "data_yaml": data_yaml, "splits": {}}
    report["splits"]["val"] = _eval_split(model, data_yaml, "val", class_names)
    if "test" in d:
        report["splits"]["test"] = _eval_split(model, data_yaml, "test", class_names)

    report_path = os.path.join(out_dir, "report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)

    # Stratified inference grid (on val split)
    val_img_dir = os.path.join(data_root, d["val"])
    val_lbl_dir = val_img_dir.replace("/images", "/labels")
    rng = random.Random(int(getattr(ctx, "seed", 42)))
    picked = _stratified_samples(val_img_dir, val_lbl_dir, class_names, per_class=3, rng=rng)
    tiles: List[np.ndarray] = []
    for name in class_names:
        imgs = picked[name]
        if not imgs:
            tiles.append(_placeholder_tile(f"no examples for {name}"))
            continue
        for img_path in imgs:
            stem = os.path.splitext(os.path.basename(img_path))[0]
            lbl_path = os.path.join(val_lbl_dir, stem + ".txt")
            tiles.append(_annotate(img_path, lbl_path, model, class_names))
    grid = _build_grid(tiles, cols=3)
    grid_path = os.path.join(out_dir, "grid.jpg")
    cv2.imwrite(grid_path, grid)

    val_m = report["splits"]["val"].get("mAP50")
    test_m = report["splits"].get("test", {}).get("mAP50")
    print(f"[eval] OK report={report_path} grid={grid_path} val_mAP50={val_m} test_mAP50={test_m}")
    return report


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to best.pt")
    ap.add_argument("--data-yaml", required=True)
    ap.add_argument("--eval-dir", default="runs/eval")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()
    class _C: pass
    ctx = _C()
    for k, v in vars(args).items(): setattr(ctx, k.replace("-", "_"), v)
    try:
        run(ctx)
    except (FileNotFoundError, ValueError) as e:
        print(f"[eval] FAIL {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    _cli()
