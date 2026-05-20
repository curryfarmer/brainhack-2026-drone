"""
train_yolo.py
=============
YOLO trainer with:
  - Auto device selection (cuda > mps > cpu)
  - Seed: random, numpy, torch all seeded BEFORE model construction
  - Batch auto sizing with OOM retry (halves on RuntimeError)
  - Val-size guard: hard-requires >= max(min_val, 5*nc) val images
  - Explicit sim-to-real augmentation flags (real ultralytics args only)
  - --resume support (ultralytics native checkpoint recovery)
"""

import argparse
import os
import random
import sys
from glob import glob
from typing import Optional, Tuple

import numpy as np
import torch
import yaml


AUG_DEFAULTS = dict(
    hsv_h=0.015,
    hsv_s=0.7,
    hsv_v=0.4,
    degrees=10.0,
    translate=0.1,
    scale=0.5,
    fliplr=0.5,
    flipud=0.0,
    mosaic=1.0,
    mixup=0.1,
    erasing=0.2,
)

AUG_LIGHT = dict(
    hsv_h=0.0,
    hsv_s=0.0,
    hsv_v=0.0,
    degrees=0.0,
    translate=0.0,
    scale=0.0,
    fliplr=0.5,
    flipud=0.0,
    mosaic=0.0,
    mixup=0.0,
    erasing=0.0,
)


def _autodetect_device(explicit: str) -> str:
    if explicit and explicit != "auto":
        return explicit
    if torch.cuda.is_available():
        return "0"   # cuda:0
    if torch.backends.mps.is_available():
        os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
        return "mps"
    print("[train] WARN no CUDA/MPS — falling back to CPU. Training will be slow.", file=sys.stderr)
    return "cpu"


def _default_batch(device: str) -> int:
    if device == "cpu":
        return 4
    if device == "mps":
        return 8
    return 16   # cuda


def _set_seeds(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _count_val(data_yaml_path: str) -> Tuple[int, int]:
    with open(data_yaml_path) as f:
        d = yaml.safe_load(f)
    root = d["path"]
    val_rel = d["val"]
    val_dir = os.path.join(root, val_rel)
    n = sum(len(glob(os.path.join(val_dir, "*" + ext))) for ext in (".jpg", ".jpeg", ".png"))
    return n, int(d["nc"])


def _is_oom(e: BaseException) -> bool:
    msg = str(e).lower()
    return "out of memory" in msg or "cuda error" in msg or "mps backend out of memory" in msg or "failed allocation" in msg


def run(ctx=None, **overrides) -> dict:
    if ctx is None:
        class _C: pass
        ctx = _C()
        for k, v in overrides.items():
            setattr(ctx, k, v)

    data_yaml = getattr(ctx, "data_yaml", None) or os.path.join(
        getattr(ctx, "data_root", "data"), "_smoke" if getattr(ctx, "smoke", False) else "", "data.yaml"
    )
    # if smoke, data_yaml lives at data_root/_smoke/data.yaml
    if getattr(ctx, "smoke", False) and getattr(ctx, "data_yaml", None) is None:
        data_yaml = os.path.join(ctx.data_root, "_smoke", "data.yaml")

    if not os.path.isfile(data_yaml):
        raise FileNotFoundError(f"{data_yaml} missing — run gen_data_yaml.py first")

    epochs = int(getattr(ctx, "epochs", 50))
    imgsz = int(getattr(ctx, "imgsz", 640))
    seed = int(getattr(ctx, "seed", 42))
    model_path = getattr(ctx, "model", "yolov10n.pt")
    name = getattr(ctx, "name", "barrel_v1")
    resume = bool(getattr(ctx, "resume", False))
    smoke = bool(getattr(ctx, "smoke", False))
    min_val = int(getattr(ctx, "min_val", 20))
    batch_in = getattr(ctx, "batch", "auto")

    device = _autodetect_device(getattr(ctx, "device", "auto"))

    n_val, nc = _count_val(data_yaml)
    required = max(min_val, 5 * nc)
    if n_val < required:
        raise ValueError(f"val set has {n_val} images, need >= {required}")

    if batch_in in (None, "auto"):
        batch = _default_batch(device)
    else:
        batch = int(batch_in)

    _set_seeds(seed)

    # Lazy import — heavy.
    from ultralytics import YOLO

    print(f"[train] device={device} batch={batch} imgsz={imgsz} epochs={epochs} model={model_path} name={name}")
    aug = AUG_LIGHT if smoke else AUG_DEFAULTS

    last_err: Optional[BaseException] = None
    while batch >= 1:
        try:
            model = YOLO(model_path)
            results = model.train(
                data=data_yaml,
                epochs=epochs,
                imgsz=imgsz,
                batch=batch,
                device=device,
                seed=seed,
                name=name,
                resume=resume,
                exist_ok=True,
                verbose=True,
                **aug,
            )
            # find produced best.pt
            run_dir = getattr(results, "save_dir", None) or os.path.join("runs", "detect", name)
            best = os.path.join(str(run_dir), "weights", "best.pt")
            if not os.path.isfile(best):
                raise FileNotFoundError(f"training reported success but {best} missing")
            print(f"[train] OK best={best} device={device} batch={batch}")
            return {"best_pt": best, "device": device, "batch": batch, "run_dir": str(run_dir)}
        except RuntimeError as e:
            last_err = e
            if _is_oom(e) and batch > 1:
                new_batch = batch // 2
                print(f"[train] OOM at batch={batch}, retrying with batch={new_batch}", file=sys.stderr)
                batch = new_batch
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
                continue
            raise

    raise RuntimeError(f"training failed at batch=1: {last_err}")


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-yaml", required=True)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", default="auto")
    ap.add_argument("--model", default="yolov10n.pt")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--name", default="barrel_v1")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--smoke", action="store_true", help="Use light augmentation (smoke-safe)")
    ap.add_argument("--min-val", type=int, default=20)
    args = ap.parse_args()
    class _C: pass
    ctx = _C()
    for k, v in vars(args).items():
        setattr(ctx, k.replace("-", "_"), v)
    ctx.data_root = "data"
    try:
        run(ctx)
    except (FileNotFoundError, ValueError, RuntimeError) as e:
        print(f"[train] FAIL {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    _cli()
