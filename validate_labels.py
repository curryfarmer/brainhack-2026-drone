"""
validate_labels.py
==================
HARD GATE for the YOLO pipeline. Refuses to let bad data move forward.

Checks (per the plan):
  (a) every .jpg opens via cv2.imread, shape is (H, W, 3)
  (b) every image has a matching .txt; empty .txt is allowed (background)
  (c) each label line = int(class_id) float(cx) float(cy) float(w) float(h)
  (d) w > 0 and h > 0
  (e) bbox stays inside [0, 1] on both axes (cx ± w/2)
  (f) bbox pixel area >= 2x2 after multiplying by image dims
  (g) 0 <= class_id < nc
  (h) per-class image counts: HARD fail < 5/class, WARN < 20/class

On success: writes data/.validated.flag and data/validation_report.json.
On failure: prints the first violation as `path:line: <reason>` and exits 1.

Layout:
  Pass --data-root pointing at a directory that contains train/{images,labels}.
  classes.txt should be at the directory's parent or specified via --classes-file.
"""

import argparse
import json
import os
import sys
from collections import Counter
from glob import glob
from typing import List, Tuple

import cv2


def _read_classes(path: str) -> List[str]:
    with open(path) as f:
        return [l.strip() for l in f if l.strip()]


def _find_classes_file(data_root: str, explicit: str) -> str:
    if explicit and os.path.isfile(explicit):
        return explicit
    candidates = [
        os.path.join(data_root, "classes.txt"),
        os.path.join(os.path.dirname(data_root.rstrip(os.sep)), "classes.txt"),
        "classes.txt",
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(f"classes.txt not found (tried: {candidates})")


def _parse_label_line(line: str) -> Tuple[int, float, float, float, float]:
    parts = line.strip().split()
    if len(parts) != 5:
        raise ValueError(f"expected 5 tokens, got {len(parts)}")
    try:
        cls_id = int(parts[0])
    except ValueError as e:
        raise ValueError(f"class_id not int: {parts[0]!r}") from e
    if "." in parts[0] or "e" in parts[0].lower():
        raise ValueError(f"class_id must be int, got {parts[0]!r}")
    cx, cy, w, h = (float(x) for x in parts[1:])
    return cls_id, cx, cy, w, h


def _check_bbox(cx: float, cy: float, w: float, h: float, img_w: int, img_h: int) -> str:
    if not (w > 0 and h > 0):
        return f"w/h must be > 0 (got w={w}, h={h})"
    x_lo, x_hi = cx - w / 2, cx + w / 2
    y_lo, y_hi = cy - h / 2, cy + h / 2
    if not (0.0 <= x_lo and x_hi <= 1.0):
        return f"x out of [0,1]: x_lo={x_lo:.4f}, x_hi={x_hi:.4f}"
    if not (0.0 <= y_lo and y_hi <= 1.0):
        return f"y out of [0,1]: y_lo={y_lo:.4f}, y_hi={y_hi:.4f}"
    if w * img_w < 2.0 or h * img_h < 2.0:
        return f"bbox pixel size < 2x2 (got {w*img_w:.1f}x{h*img_h:.1f})"
    return ""


class ValidationError(Exception):
    pass


def _validate_split(images_dir: str, labels_dir: str, nc: int) -> Tuple[int, Counter, int]:
    """Returns (num_images, per_class_counts, num_background_images)."""
    if not os.path.isdir(images_dir):
        raise ValidationError(f"{images_dir}: directory missing")

    img_paths = sorted(
        glob(os.path.join(images_dir, "*.jpg"))
        + glob(os.path.join(images_dir, "*.jpeg"))
        + glob(os.path.join(images_dir, "*.png"))
    )
    if not img_paths:
        raise ValidationError(f"{images_dir}: no images found")

    per_class = Counter()
    background = 0

    for img_path in img_paths:
        stem = os.path.splitext(os.path.basename(img_path))[0]
        lbl_path = os.path.join(labels_dir, stem + ".txt")

        img = cv2.imread(img_path)
        if img is None:
            raise ValidationError(f"{img_path}:0: corrupt or unreadable image")
        if img.ndim != 3 or img.shape[2] != 3:
            raise ValidationError(f"{img_path}:0: expected (H,W,3), got {img.shape}")
        img_h, img_w = img.shape[:2]

        if not os.path.isfile(lbl_path):
            raise ValidationError(f"{img_path}:0: missing label file {lbl_path}")

        with open(lbl_path) as f:
            lines = [(i + 1, l) for i, l in enumerate(f) if l.strip()]

        if not lines:
            background += 1
            continue

        classes_in_img = set()
        for lineno, raw in lines:
            try:
                cls_id, cx, cy, w, h = _parse_label_line(raw)
            except ValueError as e:
                raise ValidationError(f"{lbl_path}:{lineno}: {e}")
            if not (0 <= cls_id < nc):
                raise ValidationError(f"{lbl_path}:{lineno}: class_id {cls_id} not in [0,{nc})")
            err = _check_bbox(cx, cy, w, h, img_w, img_h)
            if err:
                raise ValidationError(f"{lbl_path}:{lineno}: {err}")
            classes_in_img.add(cls_id)

        for c in classes_in_img:
            per_class[c] += 1

    return len(img_paths), per_class, background


def run(ctx=None, **overrides) -> dict:
    """Pipeline entry point.

    Args via ctx: ctx.data_root (default: 'data'), ctx.classes_file ('classes.txt').
    """
    if ctx is None:
        class _C: pass
        ctx = _C()
        ctx.data_root = overrides.get("data_root", "data")
        ctx.classes_file = overrides.get("classes_file", "classes.txt")
        ctx.smoke = overrides.get("smoke", False)

    data_root = ctx.data_root
    if getattr(ctx, "smoke", False):
        data_root = os.path.join(ctx.data_root, "_smoke")

    classes_path = _find_classes_file(data_root, ctx.classes_file)
    classes = _read_classes(classes_path)
    nc = len(classes)

    splits = []
    for split in ("train", "validation", "test"):
        img_dir = os.path.join(data_root, split, "images")
        lbl_dir = os.path.join(data_root, split, "labels")
        if os.path.isdir(img_dir):
            splits.append((split, img_dir, lbl_dir))

    if not any(s[0] == "train" for s in splits):
        raise ValidationError(f"{data_root}: train/images missing")

    report = {"classes": classes, "nc": nc, "splits": {}}
    train_per_class = Counter()
    for split, img_dir, lbl_dir in splits:
        n_imgs, per_class, bg = _validate_split(img_dir, lbl_dir, nc)
        report["splits"][split] = {
            "num_images": n_imgs,
            "background_images": bg,
            "per_class": {classes[k]: v for k, v in per_class.items()},
        }
        if split == "train":
            train_per_class = per_class

    warnings = []
    hard_failures = []
    for cls_id, cls_name in enumerate(classes):
        n = train_per_class.get(cls_id, 0)
        if n < 5:
            hard_failures.append(f"{cls_name}: {n} images, need >= 5")
        elif n < 20:
            warnings.append(f"{cls_name}: {n} images, recommend >= 20")

    report["warnings"] = warnings
    if hard_failures:
        report["hard_failures"] = hard_failures
        raise ValidationError("class imbalance: " + "; ".join(hard_failures))

    os.makedirs(data_root, exist_ok=True)
    with open(os.path.join(data_root, "validation_report.json"), "w") as f:
        json.dump(report, f, indent=2)
    with open(os.path.join(data_root, ".validated.flag"), "w") as f:
        f.write("ok\n")

    for w in warnings:
        print(f"[validate] WARN {w}")
    print(f"[validate] OK  data_root={data_root}  nc={nc}  splits={list(report['splits'].keys())}")
    return report


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="data", help="Directory containing train/{images,labels} (and optionally validation/, test/)")
    ap.add_argument("--classes-file", default="classes.txt")
    args = ap.parse_args()
    class _C: pass
    ctx = _C()
    ctx.data_root = args.data_root
    ctx.classes_file = args.classes_file
    ctx.smoke = False
    try:
        run(ctx)
    except ValidationError as e:
        print(f"[validate] FAIL {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    _cli()
