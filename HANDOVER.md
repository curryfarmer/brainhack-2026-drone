# Handover — YOLO Training Pipeline

Short notes for picking up the work.

## What this is

A full pipeline for training the barrel-detector YOLO model. Drone needs to detect two classes: `yellow_barrel` and `red_barrel`. Pipeline takes you from "raw frames in Gazebo" to "deployed model that `Detector.py` can load".

## Quick start

```bash
# 1. Set up env (Python 3.10+ recommended)
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 2. Run end-to-end smoke test (~45s on CPU)
python pipeline.py --smoke

# 3. Dry-run to see the stage list
python pipeline.py --dry-run
```

If the smoke prints `[pipe] ALL DONE` and exits 0, your environment is good.

## The pipeline

Nine scripts. Each runs standalone. `pipeline.py` chains them in-process.

| Stage | Script | What it does |
|---|---|---|
| collect | `collect_yolo_data.py` | Keyboard-fly drone in Gazebo. Saves frames + pose JSON. |
| import | `import_roboflow.py` | Unzip a Roboflow export into the dataset. Optional. |
| validate | `validate_labels.py` | Hard gate. Refuses bad labels, corrupt images, off-image bboxes. |
| split | `split_train_val.py` | 80/20 train/val. Uses pose sidecars to avoid near-duplicate leak. |
| yaml | `gen_data_yaml.py` | Writes `data/data.yaml` for ultralytics. |
| train | `train_yolo.py` | Trains. Auto-picks cuda > mps > cpu. Sets seeds. OOM retry. |
| eval | `eval_model.py` | Per-class mAP + stratified inference grid. |
| deploy | `deploy_model.py` | Versions weights under `models/yolo_<ts>/`. Writes `model_config.json`. |

`Detector.py` was patched to accept `config_path="model_config.json"`. Old callers still work via `model_path=...`.

## How to train for real

1. Fly the drone with `collect_yolo_data.py`. Hit `T` to take off. Press `O` to start auto-save. Press `N` to switch between yellow/red/mixed filename prefix. Press `P` for burst capture. Land with `L`. Quit with `Q`.
2. Label the captured images. Use Roboflow or labelImg. Two classes: `yellow_barrel`, `red_barrel`. Aim for at least 200 images per class.
3. Import (only if you used Roboflow): `python import_roboflow.py --zip your_export.zip`.
4. Run the pipeline: `python pipeline.py --from validate_labels --to deploy --epochs 50`.
5. The trained model lands at `models/yolo_<timestamp>/best.pt`. `model_config.json` points to it.
6. Any code that uses `Detector(config_path="model_config.json")` now picks up the new weights.

## What to know before you train

- Pipeline assumes the training data lives in `data/train/{images,labels}`. The collector and the Roboflow importer both write there.
- Validator requires at least 5 images per class. Warns under 20. Hard fail under 5.
- Trainer requires at least `max(20, 5*nc) = 20` validation images. Drop `--min-val` if you want to train on less.
- Default model is `yolov10n.pt` (already in the repo). Swap to `yolo11s.pt` or larger if you have a real GPU.
- Augmentation flags are tuned for sim-to-real. See `train_yolo.py` `AUG_DEFAULTS`.
- Training output goes to `runs/detect/<name>/`. Deletes safely; everything important gets copied to `models/yolo_<ts>/`.

## Smoke test

`python pipeline.py --smoke` does the entire DAG against synthetic data in ~45s on CPU. Use it after any change to confirm nothing broke. The synthetic dataset is generated fresh into `data/_smoke/` — does not touch real data.

Persistent fixtures live at `tests/fixtures/data_ok` and `tests/fixtures/data_bad_bbox`. Use them to smoke individual stages:

```bash
python validate_labels.py --data-root tests/fixtures/data_ok          # should pass
python validate_labels.py --data-root tests/fixtures/data_bad_bbox    # should fail
```

If fixtures are missing, regenerate: `python gen_smoke_data.py --persist`.

## File map

New (this work):
- `pipeline.py` — orchestrator
- `gen_smoke_data.py`, `validate_labels.py`, `split_train_val.py`, `gen_data_yaml.py`
- `train_yolo.py`, `eval_model.py`, `deploy_model.py`, `import_roboflow.py`
- `tests/fixtures/` — committed mini-datasets
- `model_config.json` — points at the deployed model
- `HANDOVER.md` — this file
- `.gitignore` — keeps runs/models/data out of the repo

Existing (don't break):
- `collect_yolo_data.py` — keyboard-fly + auto-capture
- `Detector.py` — runtime YOLO worker. Patched to accept `config_path`. Old callers untouched.
- `keyboardcontrol.py`, `save_photo.py` — the collector was built on top of these
- `Train_YOLO_Models_new.ipynb` — Colab notebook. Same data layout. Still works.

## Gotchas

- On Mac without CUDA, training falls back to MPS. Some ops fall back to CPU automatically (`PYTORCH_ENABLE_MPS_FALLBACK=1` is set by `train_yolo.py`).
- ultralytics expects absolute paths in `data.yaml`. `gen_data_yaml.py` writes absolute. If you move the project, regenerate.
- The pose-aware split needs at least 3 distinct spatial bins. Otherwise it falls back to random. If you fly in one spot the whole session, expect random.
- `validate_labels.py` writes a flag file. `split_train_val.py` refuses to run without it. Order matters.
- `split_train_val.py` is idempotent. If `validation/images/` already has files, it skips. Delete the dir to redo the split.

## Open questions for the team

- Red barrel height in the real arena. Unknown. Training data needs to cover multiple altitudes to be safe.
- Whether sim training transfers to real cameras. Augmentation is tuned for it but not validated.
- Whether `Train_YOLO_Models_new.ipynb` Colab path is still needed. The new `train_yolo.py` does the same job locally.

## How to export this work

1. Commit everything except runs/, models/, data/, .venv/, __pycache__/. The `.gitignore` handles this.
2. Optionally zip with: `git archive --format=zip HEAD -o handover.zip` if it's a git repo. Or just `zip -r handover.zip . -x '*.venv*' -x '*runs*' -x '*models/yolo_*' -x '*__pycache__*' -x '*data/_smoke*' -x '*data/train*' -x '*data/validation*' -x '*data/test*'`.
3. Share `yolov10n.pt` separately if needed (5.6 MB, COCO baseline).
4. If you have a trained barrel model already, share `models/yolo_<ts>/best.pt` separately too. Update `model_config.json` to point at the right path on the receiver side.

## Contact

Pipeline was iterated against two rounds of adversarial critique. See `/Users/jp/.claude/plans/can-i-get-a-partitioned-catmull.md` for the full plan and the critique-resolved checklist.
