"""
split_train_val.py
==================
Pose-aware 80/20 train/val split using collect_yolo_data.py's session_meta JSON
sidecars. Whole spatial bins go either to train OR val — never both — to
prevent near-duplicate frames from leaking across the split.

Falls back to random split if:
  - <3 spatial bins are occupied (single-location capture), OR
  - >20% of images lack a sidecar

Requires data/.validated.flag (from validate_labels.py).

Idempotent: if data/validation/images already has files, this script is a no-op
(prints a notice). Re-run by deleting the validation/ directory first.
"""

import argparse
import json
import os
import random
import shutil
import sys
from collections import defaultdict
from glob import glob
from typing import Dict, List, Tuple

BIN_SIZE_M = 0.5
LABEL_EXTS = (".jpg", ".jpeg", ".png")


def _stems_in(images_dir: str) -> List[str]:
    if not os.path.isdir(images_dir):
        return []
    out = []
    for ext in LABEL_EXTS:
        for p in glob(os.path.join(images_dir, "*" + ext)):
            out.append(os.path.basename(p))
    return sorted(out)


def _pose_bin_for_stem(meta_dir: str, image_filename: str) -> Tuple[int, int, int] | None:
    stem = os.path.splitext(image_filename)[0]
    j = os.path.join(meta_dir, stem + ".json")
    if not os.path.isfile(j):
        return None
    try:
        with open(j) as f:
            d = json.load(f)
        n = d["pose_ned"]["n"]
        e = d["pose_ned"]["e"]
        dn = d["pose_ned"]["d"]
        return (
            int(round(n / BIN_SIZE_M)),
            int(round(e / BIN_SIZE_M)),
            int(round(dn / BIN_SIZE_M)),
        )
    except (KeyError, ValueError, json.JSONDecodeError):
        return None


def _pose_bins(meta_dir: str, images: List[str]) -> Tuple[Dict[Tuple[int, int, int], List[str]], int]:
    bins: Dict[Tuple[int, int, int], List[str]] = defaultdict(list)
    missing = 0
    for img in images:
        b = _pose_bin_for_stem(meta_dir, img)
        if b is None:
            missing += 1
        else:
            bins[b].append(img)
    return bins, missing


def _pose_split(bins: Dict[Tuple[int, int, int], List[str]], target_val_frac: float, seed: int) -> Tuple[List[str], List[str]]:
    """Greedy bin assignment: walk bins by descending size, send to val until ~target_val_frac reached."""
    rng = random.Random(seed)
    bin_keys = list(bins.keys())
    rng.shuffle(bin_keys)
    total = sum(len(v) for v in bins.values())
    target_val = int(round(total * target_val_frac))

    val_images: List[str] = []
    train_images: List[str] = []
    for k in bin_keys:
        if len(val_images) < target_val:
            val_images.extend(bins[k])
        else:
            train_images.extend(bins[k])
    return train_images, val_images


def _random_split(images: List[str], target_val_frac: float, seed: int) -> Tuple[List[str], List[str]]:
    rng = random.Random(seed)
    pool = list(images)
    rng.shuffle(pool)
    n_val = int(round(len(pool) * target_val_frac))
    return pool[n_val:], pool[:n_val]


def _move_pair(stem: str, src_img: str, src_lbl: str, dst_img_dir: str, dst_lbl_dir: str):
    os.makedirs(dst_img_dir, exist_ok=True)
    os.makedirs(dst_lbl_dir, exist_ok=True)
    img_name = os.path.basename(src_img)
    lbl_name = stem + ".txt"
    shutil.move(src_img, os.path.join(dst_img_dir, img_name))
    src_lbl_path = os.path.join(src_lbl, lbl_name)
    if os.path.isfile(src_lbl_path):
        shutil.move(src_lbl_path, os.path.join(dst_lbl_dir, lbl_name))


def run(ctx=None, **overrides) -> dict:
    if ctx is None:
        class _C: pass
        ctx = _C()
        ctx.data_root = overrides.get("data_root", "data")
        ctx.seed = overrides.get("seed", 42)
        ctx.smoke = overrides.get("smoke", False)

    data_root = ctx.data_root
    if getattr(ctx, "smoke", False):
        data_root = os.path.join(ctx.data_root, "_smoke")

    flag = os.path.join(data_root, ".validated.flag")
    if not os.path.isfile(flag):
        raise FileNotFoundError(f"{flag} missing — run validate_labels.py first")

    train_img_dir = os.path.join(data_root, "train", "images")
    train_lbl_dir = os.path.join(data_root, "train", "labels")
    val_img_dir = os.path.join(data_root, "validation", "images")
    val_lbl_dir = os.path.join(data_root, "validation", "labels")
    meta_dir = os.path.join(data_root, "session_meta")

    val_existing = _stems_in(val_img_dir)
    if val_existing:
        log = {"status": "skipped", "reason": f"validation/images already has {len(val_existing)} files"}
        with open(os.path.join(data_root, ".split.log"), "w") as f:
            json.dump(log, f, indent=2)
        print(f"[split] SKIP validation set already populated ({len(val_existing)} files)")
        return log

    images = _stems_in(train_img_dir)
    if len(images) < 10:
        raise ValueError(f"too few images to split: {len(images)} (need >= 10)")

    bins, missing = _pose_bins(meta_dir, images)
    missing_frac = missing / len(images) if images else 1.0
    use_pose = (len(bins) >= 3) and (missing_frac <= 0.20)

    if use_pose:
        train_set, val_set = _pose_split(bins, 0.2, ctx.seed)
        mode = "pose"
    else:
        train_set, val_set = _random_split(images, 0.2, ctx.seed)
        mode = "random"
        if len(bins) < 3:
            print(f"[split] WARN only {len(bins)} bins occupied — using random fallback")
        if missing_frac > 0.20:
            print(f"[split] WARN {missing_frac*100:.1f}% of images missing pose sidecars — using random fallback")

    for img_name in val_set:
        stem = os.path.splitext(img_name)[0]
        _move_pair(
            stem=stem,
            src_img=os.path.join(train_img_dir, img_name),
            src_lbl=train_lbl_dir,
            dst_img_dir=val_img_dir,
            dst_lbl_dir=val_lbl_dir,
        )

    log = {
        "status": "ok",
        "mode": mode,
        "seed": ctx.seed,
        "total": len(images),
        "train": len(train_set),
        "val": len(val_set),
        "occupied_bins": len(bins),
        "missing_sidecars": missing,
        "missing_frac": round(missing_frac, 4),
    }
    with open(os.path.join(data_root, ".split.log"), "w") as f:
        json.dump(log, f, indent=2)
    print(f"[split] OK mode={mode} train={len(train_set)} val={len(val_set)} bins={len(bins)}")
    return log


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="data")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()
    class _C: pass
    ctx = _C()
    ctx.data_root = args.data_root
    ctx.seed = args.seed
    ctx.smoke = False
    try:
        run(ctx)
    except (FileNotFoundError, ValueError) as e:
        print(f"[split] FAIL {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    _cli()
