#!/usr/bin/env python3
"""Train and export a YOLO11 detector with Ultralytics."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=project_root / "dataset" / "data.yaml")
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--project", type=Path, default=project_root / "runs" / "detect")
    parser.add_argument("--name", default="firesafety_yolo11n")
    parser.add_argument("--export-onnx", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.data.exists():
        raise SystemExit(f"Dataset YAML not found: {args.data}")

    try:
        from ultralytics import YOLO
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "ultralytics is not installed. Install with: pip install ultralytics"
        ) from exc

    model = YOLO(args.model)
    result = model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(args.project),
        name=args.name,
        cache=False,
        plots=True,
    )

    if args.export_onnx:
        best = Path(result.save_dir) / "weights" / "best.pt"
        YOLO(str(best)).export(format="onnx", imgsz=args.imgsz, opset=12, simplify=True)


if __name__ == "__main__":
    main()
