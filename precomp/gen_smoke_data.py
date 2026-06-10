"""
gen_smoke_data.py
=================
Generate a small, deterministic, *learnable* synthetic dataset for end-to-end
pipeline smoke testing.

Each image:
  - 320x320 RGB
  - Random noise background
  - ONE filled rectangle (yellow for class 0, red for class 1)
  - Matching label .txt with the exact bbox (so the model can actually fit)

Also writes pose sidecar JSONs spread across multiple spatial bins so the
pose-aware split has something to bin (otherwise it falls back to random).

Two modes:
  default (smoke)  -> data/_smoke/{train,validation,test}/{images,labels}
  --persist        -> tests/fixtures/data_ok/{train,validation,test}/{...},
                      tests/fixtures/data_bad_bbox/{train,validation}/...,
                      tests/fixtures/sample.jpg

Run:
  python gen_smoke_data.py
  python gen_smoke_data.py --persist
"""

import argparse
import json
import os
import random
import shutil
import time
from typing import Tuple

import cv2
import numpy as np

CLASSES = ["yellow_barrel", "red_barrel"]
CLASS_COLORS_BGR = {
    0: (0, 220, 220),   # yellow
    1: (0, 0, 220),     # red
}
IMGSZ = 320


def _make_image(rng: random.Random, cls_id: int) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    """Random noise bg + one filled rect for class. Returns (img_bgr, normalized bbox cx,cy,w,h)."""
    bg = np.zeros((IMGSZ, IMGSZ, 3), dtype=np.uint8)
    noise = np.random.RandomState(rng.randint(0, 2**31 - 1)).randint(0, 60, (IMGSZ, IMGSZ, 3), dtype=np.uint8)
    bg[:] = noise

    # rect dims as fraction of image
    w_frac = rng.uniform(0.18, 0.40)
    h_frac = rng.uniform(0.18, 0.40)
    cx = rng.uniform(0.25, 0.75)
    cy = rng.uniform(0.25, 0.75)

    x1 = int((cx - w_frac / 2) * IMGSZ)
    y1 = int((cy - h_frac / 2) * IMGSZ)
    x2 = int((cx + w_frac / 2) * IMGSZ)
    y2 = int((cy + h_frac / 2) * IMGSZ)
    cv2.rectangle(bg, (x1, y1), (x2, y2), CLASS_COLORS_BGR[cls_id], thickness=-1)

    return bg, (cx, cy, w_frac, h_frac)


def _write_sidecar(meta_dir: str, stem: str, ts: float, bin_xyz: Tuple[float, float, float], yaw_deg: float, class_hint: str):
    """Pose sidecar matching collect_yolo_data.py schema."""
    sidecar = {
        "ts": ts,
        "image": stem + ".jpg",
        "pose_ned": {"n": bin_xyz[0], "e": bin_xyz[1], "d": bin_xyz[2]},
        "yaw_deg": yaw_deg,
        "altitude_m": -bin_xyz[2],
        "class_hint": class_hint,
        "trigger": "smoke",
    }
    with open(os.path.join(meta_dir, stem + ".json"), "w") as f:
        json.dump(sidecar, f, indent=2)


def _gen_split(out_root: str, split: str, n_per_class: int, seed: int, bins) -> int:
    img_dir = os.path.join(out_root, split, "images")
    lbl_dir = os.path.join(out_root, split, "labels")
    meta_dir = os.path.join(out_root, "session_meta")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    os.makedirs(meta_dir, exist_ok=True)

    rng = random.Random(seed)
    written = 0
    for cls_id, cls_name in enumerate(CLASSES):
        for i in range(n_per_class):
            img, (cx, cy, w, h) = _make_image(rng, cls_id)
            ts = time.time() + written
            ts_ms = int(ts * 1000)
            stem = f"{cls_name.split('_')[0]}_{ts_ms}_{i}"
            cv2.imwrite(os.path.join(img_dir, stem + ".jpg"), img)
            with open(os.path.join(lbl_dir, stem + ".txt"), "w") as f:
                f.write(f"{cls_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")

            bin_xyz = bins[written % len(bins)]
            _write_sidecar(meta_dir, stem, ts, bin_xyz, rng.uniform(-180, 180), cls_name.split("_")[0])
            written += 1
    return written


def _write_classes_txt(root: str):
    with open(os.path.join(root, "classes.txt"), "w") as f:
        for c in CLASSES:
            f.write(c + "\n")


def _gen_one_dataset(out_root: str, train_n: int, val_n: int, test_n: int, seed: int):
    if os.path.isdir(out_root):
        shutil.rmtree(out_root)
    os.makedirs(out_root, exist_ok=True)

    bins = [
        (0.0, 0.0, -2.0),
        (5.0, 0.0, -2.0),
        (0.0, 5.0, -2.0),
        (5.0, 5.0, -2.0),
        (2.5, 2.5, -3.5),
        (2.5, 2.5, -5.0),
    ]
    n_train = _gen_split(out_root, "train", train_n, seed,         bins)
    n_val   = _gen_split(out_root, "validation", val_n, seed + 1,  bins)
    n_test  = _gen_split(out_root, "test", test_n, seed + 2,       bins) if test_n > 0 else 0

    _write_classes_txt(out_root)
    return n_train, n_val, n_test


def _gen_bad_bbox_fixture(out_root: str):
    """One image with an invalid label (w > 1.0) — for validator's negative test."""
    if os.path.isdir(out_root):
        shutil.rmtree(out_root)
    img_dir = os.path.join(out_root, "train", "images")
    lbl_dir = os.path.join(out_root, "train", "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)

    rng = random.Random(0)
    img, _ = _make_image(rng, 0)
    cv2.imwrite(os.path.join(img_dir, "bad_0.jpg"), img)
    with open(os.path.join(lbl_dir, "bad_0.txt"), "w") as f:
        f.write("0 0.5 0.5 2.0 0.5\n")   # w=2.0 invalid (off-image)
    _write_classes_txt(out_root)


def _gen_sample_fixture(path: str):
    rng = random.Random(7)
    img, _ = _make_image(rng, 0)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, img)


def run(ctx=None) -> dict:
    """Pipeline entry point.

    Args:
        ctx: PipelineCtx (optional). When None, defaults to smoke mode at data/_smoke.
    """
    if ctx is None:
        out_root = "data/_smoke"
        seed = 42
    else:
        out_root = os.path.join(ctx.data_root, "_smoke") if ctx.smoke else ctx.data_root
        seed = ctx.seed

    n_train, n_val, n_test = _gen_one_dataset(
        out_root=out_root,
        train_n=15,        # 30 train (15/class)
        val_n=6,           # 12 val
        test_n=6,          # 12 test
        seed=seed,
    )
    print(f"[smoke-data] generated train={n_train} val={n_val} test={n_test} at {out_root}")
    return {"out_root": out_root, "train": n_train, "val": n_val, "test": n_test}


def _persist_fixtures():
    fx_root = "tests/fixtures"
    os.makedirs(fx_root, exist_ok=True)
    _gen_one_dataset(os.path.join(fx_root, "data_ok"), train_n=10, val_n=6, test_n=6, seed=123)
    _gen_bad_bbox_fixture(os.path.join(fx_root, "data_bad_bbox"))
    _gen_sample_fixture(os.path.join(fx_root, "sample.jpg"))
    print(f"[smoke-data] persisted fixtures under {fx_root}/")


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--persist", action="store_true", help="Write committed fixtures to tests/fixtures/")
    ap.add_argument("--out-root", default="data/_smoke")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--train-n", type=int, default=15, help="Per class")
    ap.add_argument("--val-n", type=int, default=5, help="Per class")
    ap.add_argument("--test-n", type=int, default=5, help="Per class")
    args = ap.parse_args()

    if args.persist:
        _persist_fixtures()
        return
    _gen_one_dataset(args.out_root, args.train_n, args.val_n, args.test_n, args.seed)
    print(f"[smoke-data] done at {args.out_root}")


if __name__ == "__main__":
    _cli()
