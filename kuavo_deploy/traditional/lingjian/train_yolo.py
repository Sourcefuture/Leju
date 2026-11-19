#!/usr/bin/env python3
"""Thin, reproducible wrapper around Ultralytics custom-model training."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/letools-yolo-config")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True, help="Ultralytics dataset YAML")
    parser.add_argument("--model", required=True, help="Pretrained checkpoint, e.g. yolo26n.pt or yolo26n-obb.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--project", type=Path, default=Path("outputs/lingjian_yolo"))
    parser.add_argument("--name", default="parts")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError("Install requirements_yolo.txt before training") from error

    model = YOLO(args.model)
    model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.image_size,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(args.project),
        name=args.name,
        pretrained=True,
        patience=20,
        cache=False,
    )


if __name__ == "__main__":
    main()
