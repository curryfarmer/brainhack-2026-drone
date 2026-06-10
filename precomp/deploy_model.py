"""
deploy_model.py
===============
Versioned, no-source-edit deployment of a trained YOLO model.

Layout:
  models/yolo_<UTC-ts>/best.pt           <- the weights
  models/yolo_<UTC-ts>/args.yaml         <- copy of ultralytics train args
  models/yolo_<UTC-ts>/report.json       <- copy of eval report (if available)
  models/latest_path.txt                 <- one line: relative path to current best.pt
  model_config.json                      <- live config consumed by Detector.py

NO symlinks (git fragile on macOS / cross-platform).
NO Detector.py source edits — Detector.__init__ already accepts config_path kwarg.

Post-deploy: instantiate Detector(config_path='model_config.json') as a smoke check.
"""

import argparse
import datetime as _dt
import json
import os
import shutil
import sys
from typing import Optional


def _utc_ts() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _maybe_copy(src: str, dst: str) -> bool:
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        return True
    return False


def _instantiate_detector(config_path: str) -> None:
    """Smoke: just load the model via Detector. Don't open a display window."""
    from Detector import Detector
    d = Detector(
        config_path=config_path,
        device="cpu",
        enable_display=False,
        num_workers=0,           # don't spawn workers we won't use
    )
    try:
        d.stop()
    except Exception:
        pass


def run(ctx=None, **overrides) -> dict:
    if ctx is None:
        class _C: pass
        ctx = _C()
        for k, v in overrides.items(): setattr(ctx, k, v)

    weights = getattr(ctx, "weights", None)
    if not weights:
        raise ValueError("ctx.weights (path to best.pt) required")
    if not os.path.isfile(weights):
        raise FileNotFoundError(f"{weights} missing")

    name = getattr(ctx, "name", "barrel_v1")
    eval_dir = getattr(ctx, "eval_dir", "runs/eval")
    models_root = getattr(ctx, "models_root", "models")

    ts = _utc_ts()
    target_dir = os.path.join(models_root, f"yolo_{ts}")
    os.makedirs(target_dir, exist_ok=True)
    target_weights = os.path.join(target_dir, "best.pt")
    shutil.copy2(weights, target_weights)

    # copy args.yaml from train run dir (sibling of weights/)
    train_run_dir = os.path.dirname(os.path.dirname(weights))
    _maybe_copy(os.path.join(train_run_dir, "args.yaml"), os.path.join(target_dir, "args.yaml"))

    # copy report.json
    metrics = {}
    report_src = os.path.join(eval_dir, "report.json")
    if _maybe_copy(report_src, os.path.join(target_dir, "report.json")):
        try:
            metrics = json.load(open(report_src)).get("splits", {})
        except Exception:
            metrics = {}

    # model_config.json
    rel_weights = os.path.relpath(target_weights, start=os.getcwd())
    cfg = {
        "model_path": rel_weights,
        "model_version": name,
        "created_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "metrics": metrics,
    }
    cfg_path = "model_config.json"
    with open(cfg_path, "w") as f:
        json.dump(cfg, f, indent=2)

    # latest_path.txt (no symlink)
    with open(os.path.join(models_root, "latest_path.txt"), "w") as f:
        f.write(rel_weights + "\n")

    # post-deploy smoke: instantiate Detector
    try:
        _instantiate_detector(cfg_path)
    except Exception as e:
        raise RuntimeError(f"post-deploy Detector(config_path='{cfg_path}') failed: {e}")

    print(f"[deploy] OK weights={rel_weights} config={cfg_path}")
    return {"target_dir": target_dir, "weights": rel_weights, "config": cfg_path}


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--name", default="barrel_v1")
    ap.add_argument("--eval-dir", default="runs/eval")
    ap.add_argument("--models-root", default="models")
    args = ap.parse_args()
    class _C: pass
    ctx = _C()
    for k, v in vars(args).items(): setattr(ctx, k.replace("-", "_"), v)
    try:
        run(ctx)
    except (FileNotFoundError, ValueError, RuntimeError) as e:
        print(f"[deploy] FAIL {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    _cli()
