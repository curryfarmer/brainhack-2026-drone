"""
import_roboflow.py
===================
Unzip a Roboflow YOLOv8 export and merge it into the project's data/train
directory. Validates structure before merging; detects filename collisions
and class-set mismatches up front.

Expected zip structure (Roboflow YOLOv8 export, single split form):

  export.zip
    images/         <- *.jpg
    labels/         <- *.txt (YOLO format)
    classes.txt     (or data.yaml with names: [...])

Roboflow can also export split-form (`train/`, `valid/`, `test/`). This script
accepts both. Everything is merged into data/train/{images,labels} unless
--target-split is given.
"""

import argparse
import json
import os
import shutil
import sys
import tempfile
import zipfile
from glob import glob
from typing import List, Optional, Tuple

import yaml


def _list_zip_top(root: str) -> List[str]:
    return sorted([n for n in os.listdir(root) if not n.startswith(".")])


def _find_classes(extracted: str) -> List[str]:
    # Prefer classes.txt
    for c in (
        os.path.join(extracted, "classes.txt"),
        os.path.join(extracted, "data.yaml"),
    ):
        if os.path.isfile(c):
            if c.endswith(".txt"):
                with open(c) as f:
                    return [l.strip() for l in f if l.strip()]
            else:
                with open(c) as f:
                    d = yaml.safe_load(f)
                names = d.get("names")
                if isinstance(names, list):
                    return list(names)
                if isinstance(names, dict):
                    return [names[k] for k in sorted(names)]
    raise FileNotFoundError("zip is missing classes.txt or data.yaml with names: [...]")


def _locate_split_dirs(extracted: str) -> List[Tuple[str, str]]:
    """Return list of (images_dir, labels_dir) found inside the extracted zip."""
    # single-split form
    img = os.path.join(extracted, "images")
    lbl = os.path.join(extracted, "labels")
    if os.path.isdir(img) and os.path.isdir(lbl):
        return [(img, lbl)]
    # multi-split form (train/valid/test)
    out = []
    for split in ("train", "valid", "val", "validation", "test"):
        si = os.path.join(extracted, split, "images")
        sl = os.path.join(extracted, split, "labels")
        if os.path.isdir(si) and os.path.isdir(sl):
            out.append((si, sl))
    if not out:
        raise FileNotFoundError("zip has no images/+labels/ (single or split form)")
    return out


def _existing_classes_or_none(data_root: str) -> Optional[List[str]]:
    p = os.path.join(data_root, "classes.txt")
    if not os.path.isfile(p):
        return None
    with open(p) as f:
        return [l.strip() for l in f if l.strip()]


def _collect_filenames(images_dir: str) -> List[str]:
    return sorted(os.path.basename(p) for p in glob(os.path.join(images_dir, "*")))


def run(ctx=None, **overrides) -> dict:
    if ctx is None:
        class _C: pass
        ctx = _C()
        for k, v in overrides.items(): setattr(ctx, k, v)

    zip_path = getattr(ctx, "zip", None)
    if not zip_path or not os.path.isfile(zip_path):
        raise FileNotFoundError(f"zip {zip_path} missing")

    data_root = getattr(ctx, "data_root", "data")
    target_split = getattr(ctx, "target_split", "train")

    target_img = os.path.join(data_root, target_split, "images")
    target_lbl = os.path.join(data_root, target_split, "labels")
    os.makedirs(target_img, exist_ok=True)
    os.makedirs(target_lbl, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="rfimport_") as tmp:
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(tmp)
        # nested top-level dir? unwrap one level if present
        top = _list_zip_top(tmp)
        root = tmp
        if len(top) == 1 and os.path.isdir(os.path.join(tmp, top[0])):
            root = os.path.join(tmp, top[0])

        new_classes = _find_classes(root)
        splits = _locate_split_dirs(root)

        existing = _existing_classes_or_none(data_root)
        if existing and existing != new_classes:
            raise ValueError(
                f"class set mismatch: existing={existing} zip={new_classes}. "
                "Refusing to merge — fix the source export or remove existing classes.txt."
            )

        # collision check
        existing_names = set(_collect_filenames(target_img))
        new_names: List[str] = []
        for si, sl in splits:
            for n in _collect_filenames(si):
                new_names.append(n)
        collisions = sorted(set(new_names) & existing_names)
        if collisions:
            head = ", ".join(collisions[:10]) + (f", ... ({len(collisions)} total)" if len(collisions) > 10 else "")
            raise ValueError(f"filename collisions with existing data: {head}")

        # copy
        copied_imgs = 0
        copied_lbls = 0
        for si, sl in splits:
            for img in glob(os.path.join(si, "*")):
                shutil.copy2(img, target_img)
                copied_imgs += 1
            for lbl in glob(os.path.join(sl, "*.txt")):
                shutil.copy2(lbl, target_lbl)
                copied_lbls += 1

        # write classes.txt if not present
        cls_path = os.path.join(data_root, "classes.txt")
        if not os.path.isfile(cls_path):
            with open(cls_path, "w") as f:
                for c in new_classes:
                    f.write(c + "\n")

        log = {
            "zip": zip_path,
            "target_split": target_split,
            "classes": new_classes,
            "copied_images": copied_imgs,
            "copied_labels": copied_lbls,
        }
        log_path = os.path.join(data_root, "import.log")
        with open(log_path, "a") as f:
            f.write(json.dumps(log) + "\n")
        print(f"[import] OK imgs={copied_imgs} lbls={copied_lbls} classes={new_classes}")
        return log


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zip", required=True)
    ap.add_argument("--data-root", default="data")
    ap.add_argument("--target-split", default="train")
    args = ap.parse_args()
    class _C: pass
    ctx = _C()
    for k, v in vars(args).items(): setattr(ctx, k.replace("-", "_"), v)
    try:
        run(ctx)
    except (FileNotFoundError, ValueError) as e:
        print(f"[import] FAIL {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    _cli()
