"""
gen_data_yaml.py
================
Generates Ultralytics-compatible data.yaml.

Schema:
  path: <absolute path to data_root>
  train: train/images
  val:   validation/images
  test:  test/images          # only if directory present and non-empty
  nc:    <int>
  names: [...]
"""

import argparse
import os
import sys
from glob import glob
from typing import List

import yaml


def _read_classes(path: str) -> List[str]:
    with open(path) as f:
        names = [l.strip() for l in f if l.strip()]
    if not names:
        raise ValueError(f"{path}: empty classes file")
    return names


def _find_classes_file(data_root: str, explicit: str) -> str:
    if explicit and os.path.isfile(explicit):
        return explicit
    for c in (
        os.path.join(data_root, "classes.txt"),
        os.path.join(os.path.dirname(data_root.rstrip(os.sep)), "classes.txt"),
        "classes.txt",
    ):
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(f"classes.txt not found near {data_root}")


def _has_images(p: str) -> bool:
    return os.path.isdir(p) and any(
        glob(os.path.join(p, "*" + ext)) for ext in (".jpg", ".jpeg", ".png")
    )


def run(ctx=None, **overrides) -> str:
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
    names = _read_classes(classes_path)

    train_dir = os.path.join(data_root, "train", "images")
    val_dir = os.path.join(data_root, "validation", "images")
    test_dir = os.path.join(data_root, "test", "images")

    if not _has_images(train_dir):
        raise FileNotFoundError(f"{train_dir}: no images")
    if not _has_images(val_dir):
        raise FileNotFoundError(f"{val_dir}: no images (run split_train_val.py first)")

    abs_root = os.path.abspath(data_root)
    payload = {
        "path": abs_root,
        "train": "train/images",
        "val": "validation/images",
        "nc": len(names),
        "names": names,
    }
    if _has_images(test_dir):
        payload["test"] = "test/images"

    out_path = os.path.join(data_root, "data.yaml")
    with open(out_path, "w") as f:
        yaml.safe_dump(payload, f, sort_keys=False)

    # round-trip sanity
    loaded = yaml.safe_load(open(out_path))
    assert loaded["nc"] == len(names) and os.path.isdir(os.path.join(loaded["path"], loaded["train"]))
    print(f"[data-yaml] OK -> {out_path}  nc={len(names)}  test_set={'yes' if 'test' in payload else 'no'}")
    return out_path


def _cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="data")
    ap.add_argument("--classes-file", default="classes.txt")
    args = ap.parse_args()
    class _C: pass
    ctx = _C()
    ctx.data_root = args.data_root
    ctx.classes_file = args.classes_file
    ctx.smoke = False
    try:
        run(ctx)
    except (FileNotFoundError, ValueError, AssertionError) as e:
        print(f"[data-yaml] FAIL {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    _cli()
