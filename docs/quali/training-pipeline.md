# YOLO Training Pipeline — capture → label → train → deploy

A full pipeline for training the barrel-detector YOLO model. The drone needs to detect two classes: `yellow_barrel` and `red_barrel`. The pipeline takes you from "raw frames in Gazebo" to "deployed model that `Detector.py` can load".

Nine scripts, each runnable standalone, chained in-process by `pipeline.py`. For how these files fit into the rest of the repo, see the [codebase reference](codebase.md). For why the pipeline is structured this way, see the [design rationale](design-rationale.md).

## Environment check

Environment setup (venv, requirements, torch flavour, Gazebo bindings) is covered in the [deployment guide](deployment.md) — do that first. Then verify the pipeline works end to end:

```bash
# Run end-to-end smoke test (~45s on CPU)
python pipeline.py --smoke

# Dry-run to see the stage list
python pipeline.py --dry-run
```

If the smoke prints `[pipe] ALL DONE` and exits 0, your environment is good.

## Pipeline stages

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

(That's eight stages plus the orchestrator `pipeline.py` — nine scripts total.)

## 1. Collect images

`collect_yolo_data.py` records training data while you fly. The capture keys come from `keyboardcontrol.py`'s key handler:

| Key | Effect |
|---|---|
| `C` | Single shot now |
| `P` | Burst — queue 10 frames |
| `O` | Toggle continuous auto-save |
| `[` `]` | Slow / speed the auto-save tick (0.25 / 0.5 / 1.0 / 2.0 s) |
| `N` | Cycle filename prefix (`yellow` → `red` → `mixed`) |

The frame source is automatic:

1. **Primary — Gazebo camera.** `gz.transport13.Node().subscribe()` listens on `CAMERA_TOPIC` (default `/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image`). Every frame fills `_latest_frame_bgr`.
2. **Fallback — screen grab via [`mss`](https://pypi.org/project/mss/).** If the Gazebo subscribe call returns `False`, or `_latest_frame_bgr` is still `None` when a capture is requested, `_grab_screen()` snapshots the primary monitor and returns it as BGR. Put the Gazebo viewport on the primary monitor and the recorded JPEGs will be the cropped Gazebo view; if Gazebo is offline, you get the desktop instead — same path through the rest of the pipeline.

If `mss` can't be imported the fallback prints a one-line warning and silently no-ops — the script still flies fine, you just can't screen-capture.

> **History:** the May 22 upload reverted `requirements.txt` and dropped the `mss` entry (along with the `torch>=2.1` line and the `ultralytics<9` pin); all three were restored on 2026-06-06, so a fresh `pip install -r requirements.txt` covers `mss`. See [codebase.md known issues](codebase.md).

Output paths: `data/train/images/<prefix>_<ts>.jpg` for the image, `session_meta/<prefix>_<ts>.json` for the pose sidecar.

## 2. Label

After `collect_yolo_data.py` you have a folder of un-labelled images at `data/train/images/<prefix>_<ts>.jpg` plus pose sidecars at `session_meta/<prefix>_<ts>.json`. To train YOLO you also need a `.txt` next to each image with the bounding boxes — that's what the labelling step produces.

### What format the rest of the pipeline expects

Every captured image must have a matching label file with the same stem:

```
data/train/images/yellow_1779194401320_2.jpg
data/train/labels/yellow_1779194401320_2.txt   <-- you create this
```

Each line of the `.txt` is one bounding box, **normalized to image dimensions**:

```
<class_id> <cx> <cy> <w> <h>
```

- `class_id` is the 0-based index into `classes.txt`. The project file content is:
  ```
  yellow_barrel
  red_barrel
  ```
  so class **0 = yellow_barrel**, class **1 = red_barrel**.
- `cx, cy, w, h` are all floats in `[0, 1]`. `cx, cy` is the **centre** of the box, `w, h` are the **full width and height**, all divided by image width / image height.

> **Create `classes.txt` first.** The repo root does **not** currently ship a `classes.txt` — create one at the root before running any command that references it (one class name per line, in the order above; see `tests/fixtures/data_ok/classes.txt` for the exact format).

An image with no barrels visible should produce an **empty** `.txt` file (still required — it's the "background" class signal). `validate_labels.py` accepts empty files.

### Pick a labelling tool

All three options output the exact format above — pick whichever matches your workflow.

| Tool | Install | UI | AI-assist | Best for |
|---|---|---|---|---|
| **Roboflow** (web) | sign up at roboflow.com — nothing to install locally | browser | yes (auto-label after a few examples) | most users; output goes straight into `import_roboflow.py` |
| **labelImg** | `pip install labelImg` | Qt desktop app | none | working offline, small batches |
| **X-AnyLabeling** | `pip install x-anylabeling` | Qt desktop app | SAM one-click box | thousands of images, want speed |

#### Roboflow (recommended)

1. Make a free account at https://roboflow.com.
2. Create a new **Object Detection** project. Class names exactly `yellow_barrel` and `red_barrel` (matches `classes.txt` — order matters because Roboflow assigns class IDs by the order you create them).
3. Upload the folder `data/train/images/`. Roboflow batches them automatically.
4. Label each frame:
   - Press `B` to draw a bounding box. Click-drag a tight rectangle around each barrel.
   - Press the number key for the class (`1` = yellow_barrel, `2` = red_barrel by default) OR pick from the dropdown.
   - Press `→` for next image.
5. Once enough images are labelled (≥20 per class for a sanity train, ≥200 per class for a real model), Roboflow's **Smart Polygon / Auto-Label** can pre-label the rest. Always verify the auto-labels before exporting.
6. **Export → YOLOv8** → "Show download code" → choose `.zip`. Save the file as `data/roboflow_export.zip`.
7. Merge it into the project layout:
   ```bash
   ./run.sh import_roboflow.py --zip data/roboflow_export.zip --target-split train
   ```
   This dumps labels into `data/train/labels/`, validates the class set against `classes.txt`, and refuses to overwrite existing files (so you can re-run safely).

#### labelImg (offline)

```bash
pip install labelImg
mkdir -p data/train/labels
labelImg data/train/images classes.txt
```

The last argument is the predefined class list (point it at the root `classes.txt` you created above). Key bindings worth knowing:

| Key | Action |
|---|---|
| `W` | Create a new rectangle |
| `D` | Next image |
| `A` | Previous image |
| `Ctrl+S` | Save |
| `Ctrl+Shift+S` | Set save dir |

**Before drawing:** click `View → Auto Save mode` and switch the format dropdown (top-left) to **YOLO**, not PascalVOC. Then set the save directory to `data/train/labels`. Each save produces a `.txt` with the correct YOLO format.

#### X-AnyLabeling (SAM-assisted, fastest for big batches)

```bash
pip install x-anylabeling
x-anylabeling
```

- `Open Dir` → `data/train/images`
- `Change Output Dir` → `data/train/labels`
- File → Auto Labeling → load `SAM` model (downloads on first use, ~400 MB)
- Click on a barrel → SAM proposes a box → confirm with `Enter`, assign class.

Saves in YOLO format if you switch the format toggle in the bottom-right corner.

### After labelling — validate → split → train

Run these in order. All accept `./run.sh` since they don't touch MAVSDK, but plain `python3` works too:

```bash
# 1. Validate. Refuses to proceed if any image is broken, mis-sized, or
#    has fewer than 5 images per class. Writes data/.validated.flag.
python3 validate_labels.py --data-root data --classes-file classes.txt

# 2. 80/20 train/val split. Pose-aware: whole spatial bins go to one side
#    so near-duplicate frames don't leak across the split.
python3 split_train_val.py --data-root data --meta-dir session_meta

# 3. Generate Ultralytics data.yaml for the trainer.
python3 gen_data_yaml.py --data-root data --classes-file classes.txt

# 4. Train.
python3 train_yolo.py --data data/data.yaml --epochs 50 --device cuda
```

`train_yolo.py` writes weights to `models/yolo_<timestamp>/weights/best.pt` and the latest path to `models/latest_path.txt`. Plug that into `qualifier_run.py --weights <path>` to fly the mission with your new model.

### Common labelling mistakes

- **Wrong class order.** If you label `red_barrel` as class 0 in Roboflow but `classes.txt` says `yellow_barrel` is class 0, `import_roboflow.py` will reject the merge with `class set mismatch`. Fix the export-side class order; do not edit `classes.txt`.
- **Tight vs loose boxes.** Boxes should hug the barrel — extra margin trains the model to detect "rectangle of stuff around a barrel". `validate_labels.py` enforces a 2×2 pixel minimum but won't catch generous-margin labels.
- **Skipping background frames.** Empty-label `.txt` files (no barrels visible) are valuable training signal. Don't delete them.
- **Class imbalance.** Aim for roughly equal counts per class. `validate_labels.py` HARD-fails below 5 images per class and WARNs below 20.

## 3. Train for real

1. Fly the drone with `collect_yolo_data.py`. Hit `T` to take off. Press `O` to start auto-save. Press `N` to switch between yellow/red/mixed filename prefix. Press `P` for burst capture. Land with `L`. Quit with `Q`.
2. Label the captured images. Use Roboflow or labelImg (see above). Two classes: `yellow_barrel`, `red_barrel`. Aim for at least 200 images per class.
3. Import (only if you used Roboflow): `python import_roboflow.py --zip your_export.zip`.
4. Run the pipeline: `python pipeline.py --from validate_labels --to deploy --epochs 50`.
5. The trained model lands at `models/yolo_<timestamp>/best.pt`. `model_config.json` points to it. To actually fly with the new weights, pass the path explicitly (`qualifier_run.py --weights <path>`) — `Detector(config_path=...)` exists again (regressed May 22, restored 2026-06-06), but the real gotcha is `model_config.json` pointing at the wrong weights, so verify what it points at before relying on it (see [Deploy](#deploy) below).

## Preconditions & defaults

- Pipeline assumes the training data lives in `data/train/{images,labels}`. The collector and the Roboflow importer both write there.
- Validator requires at least 5 images per class. Warns under 20. Hard fail under 5.
- Trainer requires at least `max(20, 5*nc) = 20` validation images. Drop `--min-val` if you want to train on less.
- Default base model is `yolov10n.pt` (already in the repo, COCO-pretrained — **untrained for barrels**). Swap to `yolo11s.pt` or larger if you have a real GPU.
- Augmentation flags are tuned for sim-to-real. See `train_yolo.py` `AUG_DEFAULTS`.
- Training output goes to `runs/detect/<name>/`. Deletes safely; everything important gets copied to `models/yolo_<ts>/`.

## Smoke tests

`python pipeline.py --smoke` does the entire DAG against synthetic data in ~45s on CPU. Use it after any change to confirm nothing broke. The synthetic dataset is generated fresh into `data/_smoke/` — does not touch real data.

Persistent fixtures live at `tests/fixtures/data_ok` and `tests/fixtures/data_bad_bbox`. Use them to smoke individual stages:

```bash
python validate_labels.py --data-root tests/fixtures/data_ok          # should pass
python validate_labels.py --data-root tests/fixtures/data_bad_bbox    # should fail
```

If fixtures are missing, regenerate: `python gen_smoke_data.py --persist`.

## Deploy

`deploy_model.py` versions the trained weights under `models/yolo_<ts>/` and writes the deployed path into `model_config.json`; `train_yolo.py` also records the latest weights path in `models/latest_path.txt`.

**How the mission code picks up weights — read carefully, this changed:**

- `Detector.py` was patched to accept `config_path="model_config.json"` so callers would auto-pick-up the deployed model. That patch was lost in the May 22 upload and **restored 2026-06-06** — `Detector(config_path="model_config.json")` works again. Just make sure `model_config.json` points at the intended weights (next bullet), or pass the weights path explicitly. See [codebase.md known issues](codebase.md).
- In practice, pass weights on the command line: `python qualifier_run.py --weights models/yolo_<ts>/best.pt`. The alternative entry point `qualifier_main.py` (the teammate-built DFS exploration runner — see [codebase.md](codebase.md) for how the two mains differ) wires its own detector; verify which weights it loads before a scored run.
- A trained `best.pt` (6.2 MB) now sits at the repo root, but `model_config.json` **still points at the COCO `yolov10n.pt` placeholder**. Nothing reconciles these automatically — before any scored run, verify which weights file is actually being loaded by the entry point you're flying.

## Gotchas

- On Mac without CUDA, training falls back to MPS. Some ops fall back to CPU automatically (`PYTORCH_ENABLE_MPS_FALLBACK=1` is set by `train_yolo.py`).
- ultralytics expects absolute paths in `data.yaml`. `gen_data_yaml.py` writes absolute. If you move the project, regenerate.
- The pose-aware split needs at least 3 distinct spatial bins. Otherwise it falls back to random. If you fly in one spot the whole session, expect random.
- `validate_labels.py` writes a flag file. `split_train_val.py` refuses to run without it. Order matters.
- `split_train_val.py` is idempotent. If `validation/images/` already has files, it skips. Delete the dir to redo the split.

## Open questions

- Whether sim training transfers to real cameras. Augmentation is tuned for it but not validated.
- Whether the `Train_YOLO_Models_new.ipynb` Colab path is still needed. `train_yolo.py` does the same job locally.

Mission-level open questions (red-barrel height in the real arena, altitude strategy, pose source) live in the [design rationale](design-rationale.md).
