#!/usr/bin/env python3
"""HSV and contour baseline for Lingjian bins and yellow parts."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class Detection:
    label: str
    center_px: tuple[float, float]
    angle_deg: float
    area_px: float
    box_px: list[list[int]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="JPEG/PNG image or directory")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.yaml"),
    )
    return parser.parse_args()


def roi_mask(shape: tuple[int, int], normalized_roi: list[float]) -> np.ndarray:
    height, width = shape
    x0, y0, x1, y1 = normalized_roi
    x0, x1 = int(round(x0 * width)), int(round(x1 * width))
    y0, y1 = int(round(y0 * height)), int(round(y1 * height))
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[max(0, y0) : min(height, y1), max(0, x0) : min(width, x1)] = 255
    return mask


def color_mask(hsv: np.ndarray, spec: dict, workspace: np.ndarray, kernel_size: int) -> np.ndarray:
    lower = np.asarray(spec["hsv_lower"], dtype=np.uint8)
    upper = np.asarray(spec["hsv_upper"], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.bitwise_and(mask, workspace)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def contours_to_detections(mask: np.ndarray, label: str, spec: dict) -> list[Detection]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: list[Detection] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not (float(spec["min_area"]) <= area <= float(spec["max_area"])):
            continue
        (cx, cy), (width, height), angle = cv2.minAreaRect(contour)
        if width < height:
            angle += 90.0
        box = cv2.boxPoints(((cx, cy), (width, height), angle)).astype(np.int32)
        detections.append(
            Detection(
                label=label,
                center_px=(round(float(cx), 2), round(float(cy), 2)),
                angle_deg=round(float(angle), 2),
                area_px=round(area, 2),
                box_px=box.tolist(),
            )
        )
    return sorted(detections, key=lambda detection: detection.area_px, reverse=True)


def exclude_bin_interiors(mask: np.ndarray, bins: list[Detection], margin: int) -> np.ndarray:
    filtered = mask.copy()
    for detection in bins:
        points = np.asarray(detection.box_px, dtype=np.int32)
        x, y, width, height = cv2.boundingRect(points)
        x0, y0 = max(0, x - margin), max(0, y - margin)
        x1, y1 = min(mask.shape[1], x + width + margin), min(mask.shape[0], y + height + margin)
        filtered[y0:y1, x0:x1] = 0
    return filtered


def select_bins(detections: list[Detection], image_shape: tuple[int, int], spec: dict) -> list[Detection]:
    """Keep the largest tabletop bin on each side of the image."""
    height, width = image_shape
    min_center_y = float(spec.get("min_center_y", 0.0)) * height
    max_per_side = int(spec.get("max_per_image_side", 1))
    selected: list[Detection] = []
    for left_side in (True, False):
        candidates = [
            detection
            for detection in detections
            if detection.center_px[1] >= min_center_y
            and ((detection.center_px[0] < width / 2) == left_side)
        ]
        selected.extend(candidates[:max_per_side])
    return selected


def detect(image: np.ndarray, config: dict) -> tuple[list[Detection], dict[str, np.ndarray]]:
    perception = config["perception"]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    workspace = roi_mask(image.shape[:2], perception["workspace_roi"])
    kernel_size = int(perception["morphology_kernel"])

    blue_mask = color_mask(hsv, perception["blue_bin"], workspace, kernel_size)
    bins = contours_to_detections(blue_mask, "blue_bin", perception["blue_bin"])
    bins = select_bins(bins, image.shape[:2], perception["blue_bin"])

    yellow_mask = color_mask(hsv, perception["yellow_part"], workspace, kernel_size)
    yellow_mask = exclude_bin_interiors(
        yellow_mask,
        bins,
        int(perception["bin_exclusion_margin_px"]),
    )
    yellow_parts = contours_to_detections(yellow_mask, "yellow_part", perception["yellow_part"])
    return bins + yellow_parts, {"blue_bin": blue_mask, "yellow_part": yellow_mask}


def annotate(image: np.ndarray, detections: list[Detection]) -> np.ndarray:
    output = image.copy()
    colors = {"blue_bin": (255, 120, 0), "yellow_part": (0, 255, 255)}
    for detection in detections:
        color = colors[detection.label]
        box = np.asarray(detection.box_px, dtype=np.int32)
        center = tuple(int(round(value)) for value in detection.center_px)
        cv2.polylines(output, [box], True, color, 2, cv2.LINE_AA)
        cv2.circle(output, center, 4, color, -1)
        text = f"{detection.label} {detection.angle_deg:.0f}deg"
        cv2.putText(output, text, (center[0] + 5, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return output


def iter_images(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    suffixes = {".jpg", ".jpeg", ".png"}
    return sorted(candidate for candidate in path.rglob("*") if candidate.suffix.lower() in suffixes)


def main() -> None:
    args = parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)

    images = iter_images(args.input)
    if not images:
        raise FileNotFoundError(f"No images found under {args.input}")

    records: list[str] = []
    for path in images:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Unable to read {path}")
        detections, masks = detect(image, config)
        annotated = annotate(image, detections)
        cv2.imwrite(str(args.output_dir / path.name), annotated)
        for label, mask in masks.items():
            cv2.imwrite(str(args.output_dir / f"{path.stem}_{label}_mask.png"), mask)
        records.append(
            json.dumps(
                {"file": path.name, "detections": [asdict(item) for item in detections]},
                ensure_ascii=False,
            )
        )

    (args.output_dir / "detections.jsonl").write_text("\n".join(records) + "\n", encoding="utf-8")
    print(f"Processed {len(images)} image(s); results: {args.output_dir}")


if __name__ == "__main__":
    main()
