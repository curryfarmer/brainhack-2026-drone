"""
pipeline.py
===========
In-process orchestrator for the YOLO pipeline. No subprocess overhead — each
stage is imported and called via its `run(ctx)` entry point.

Stages (in DAG order):
  import_roboflow   (optional; only if --rf-zip given)
  gen_smoke_data    (only in --smoke)
  validate_labels
  split_train_val
  gen_data_yaml
  train_yolo
  eval_model
  deploy_model
  post_deploy_smoke (only in --smoke; loads Detector and runs predict)

Flags:
  --from STAGE / --to STAGE / --skip STAGE,STAGE
  --smoke         synthetic data, full DAG, <90s on CPU
  --dry-run       print stage list and exit
  --verbose       per-stage stdout
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional


@dataclass
class PipelineCtx:
    data_root: str = "data"
    classes_file: str = "classes.txt"
    name: str = "barrel_v1"
    seed: int = 42
    device: str = "auto"
    smoke: bool = False
    epochs: int = 50
    imgsz: int = 640
    batch: object = "auto"
    model: str = "yolov10n.pt"
    min_val: int = 20
    rf_zip: Optional[str] = None
    target_split: str = "train"
    eval_dir: str = "runs/eval"
    models_root: str = "models"
    data_yaml: Optional[str] = None
    weights: Optional[str] = None
    resume: bool = False


def _stage_import_roboflow(ctx: PipelineCtx):
    if not ctx.rf_zip:
        print("[pipe] import_roboflow: skipped (no --rf-zip)")
        return None
    import import_roboflow
    ctx.zip = ctx.rf_zip
    return import_roboflow.run(ctx)


def _stage_gen_smoke_data(ctx: PipelineCtx):
    if not ctx.smoke:
        print("[pipe] gen_smoke_data: skipped (not --smoke)")
        return None
    import gen_smoke_data
    return gen_smoke_data.run(ctx)


def _stage_validate_labels(ctx: PipelineCtx):
    import validate_labels
    return validate_labels.run(ctx)


def _stage_split_train_val(ctx: PipelineCtx):
    import split_train_val
    return split_train_val.run(ctx)


def _stage_gen_data_yaml(ctx: PipelineCtx):
    import gen_data_yaml
    out = gen_data_yaml.run(ctx)
    ctx.data_yaml = out
    return out


def _stage_train_yolo(ctx: PipelineCtx):
    import train_yolo
    result = train_yolo.run(ctx)
    ctx.weights = result["best_pt"]
    return result


def _stage_eval_model(ctx: PipelineCtx):
    import eval_model
    # eval_model expects ctx.model (path to weights)
    ctx.model = ctx.weights or ctx.model
    return eval_model.run(ctx)


def _stage_deploy_model(ctx: PipelineCtx):
    import deploy_model
    return deploy_model.run(ctx)


def _stage_post_deploy_smoke(ctx: PipelineCtx):
    """Smoke check: load Detector via config and run one prediction on a fixture."""
    if not ctx.smoke:
        # in real runs, deploy_model already instantiates Detector once
        return None
    if not os.path.isfile("model_config.json"):
        raise FileNotFoundError("model_config.json not found")
    from Detector import Detector
    import cv2
    fixture = "tests/fixtures/sample.jpg"
    if not os.path.isfile(fixture):
        raise FileNotFoundError(f"{fixture} missing — run gen_smoke_data.py --persist first")
    d = Detector(
        config_path="model_config.json",
        device="cpu",
        enable_display=False,
        num_workers=0,
    )
    img = cv2.imread(fixture)
    if img is None:
        raise RuntimeError(f"failed to read {fixture}")
    # raw inference, bypassing the worker queue (no workers in smoke)
    results = d.model(img, verbose=False, conf=0.001)
    n_boxes = 0
    for r in results:
        if r.boxes is not None:
            n_boxes += len(r.boxes)
    d.stop()
    print(f"[pipe] post_deploy_smoke: Detector predict returned {n_boxes} boxes (smoke threshold: >=0)")
    return {"boxes": n_boxes}


STAGES: List[tuple] = [
    ("import_roboflow",   _stage_import_roboflow),
    ("gen_smoke_data",    _stage_gen_smoke_data),
    ("validate_labels",   _stage_validate_labels),
    ("split_train_val",   _stage_split_train_val),
    ("gen_data_yaml",     _stage_gen_data_yaml),
    ("train_yolo",        _stage_train_yolo),
    ("eval_model",        _stage_eval_model),
    ("deploy_model",      _stage_deploy_model),
    ("post_deploy_smoke", _stage_post_deploy_smoke),
]


def _resolve_range(from_stage: Optional[str], to_stage: Optional[str]) -> List[tuple]:
    names = [n for n, _ in STAGES]
    start = names.index(from_stage) if from_stage else 0
    end = names.index(to_stage) + 1 if to_stage else len(names)
    if start >= end:
        raise ValueError(f"--from {from_stage} is at or after --to {to_stage}")
    return STAGES[start:end]


def run(args) -> int:
    skip = set(s.strip() for s in args.skip.split(",")) if args.skip else set()

    ctx = PipelineCtx(
        data_root=args.data_root,
        classes_file=args.classes_file,
        name=args.name,
        seed=args.seed,
        device=args.device,
        smoke=args.smoke,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        model=args.model,
        min_val=args.min_val,
        rf_zip=args.rf_zip,
        eval_dir=args.eval_dir,
        models_root=args.models_root,
    )

    # smoke knobs (per plan): tiny dataset, tiny imgsz, 1 epoch, batch 1, low min_val
    if ctx.smoke:
        ctx.epochs = 1
        ctx.imgsz = 320
        ctx.batch = 1
        ctx.min_val = 5
        ctx.device = "cpu"
        ctx.name = "smoke"

    stages = _resolve_range(args.from_, args.to)
    stages = [(n, fn) for n, fn in stages if n not in skip]

    if args.dry_run:
        print("[pipe] DRY-RUN stage list:")
        for n, _ in stages:
            print(f"  - {n}")
        return 0

    t0 = time.time()
    for name, fn in stages:
        if name == "import_roboflow" and not ctx.rf_zip:
            continue
        if name == "gen_smoke_data" and not ctx.smoke:
            continue
        if name == "post_deploy_smoke" and not ctx.smoke:
            continue
        ts = time.time()
        print(f"\n[pipe] >>> {name} start")
        try:
            fn(ctx)
        except Exception as e:
            elapsed = time.time() - ts
            print(f"[pipe] !!! {name} FAILED after {elapsed:.1f}s: {e}", file=sys.stderr)
            if args.verbose:
                import traceback; traceback.print_exc()
            return 1
        elapsed = time.time() - ts
        print(f"[pipe] <<< {name} done in {elapsed:.1f}s")

    total = time.time() - t0
    print(f"\n[pipe] ALL DONE in {total:.1f}s")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="from_", default=None, help="start at this stage")
    ap.add_argument("--to", default=None, help="stop after this stage")
    ap.add_argument("--skip", default="", help="comma-separated stages to skip")
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--data-root", default="data")
    ap.add_argument("--classes-file", default="classes.txt")
    ap.add_argument("--name", default="barrel_v1")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", default="auto")
    ap.add_argument("--model", default="yolov10n.pt")
    ap.add_argument("--min-val", type=int, default=20)
    ap.add_argument("--rf-zip", default=None, help="optional Roboflow zip to import first")
    ap.add_argument("--eval-dir", default="runs/eval")
    ap.add_argument("--models-root", default="models")
    args = ap.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
