"""Optional Ultralytics YOLO backend for the Lingjian sorting scene."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class YoloDetection:
    label: str
    confidence: float
    center_px: tuple[float, float]
    box_xyxy: tuple[float, float, float, float]
    angle_deg: float | None = None
    polygon_px: list[list[float]] | None = None


def _long_edge_angle(polygon: np.ndarray) -> float:
    """Return the undirected long-edge angle in degrees in [-90, 90)."""
    (_, _), (width, height), angle = cv2.minAreaRect(polygon.astype(np.float32))
    if width < height:
        angle += 90.0
    while angle >= 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return float(angle)


class YoloDetector:
    """Load one YOLO model and expose a stable, ROS-friendly result format.

    A standard/custom checkpoint is loaded with ``YOLO``. Supplying prompts switches
    to ``YOLOWorld`` and is intended for zero-shot trials or annotation bootstrapping.
    Ultralytics is imported lazily so the classical detector remains usable without it.
    """

    def __init__(
        self,
        model: str | Path,
        *,
        prompts: Sequence[str] | None = None,
        confidence: float = 0.25,
        iou: float = 0.5,
        image_size: int = 640,
        device: str = "cpu",
        max_detections: int = 50,
    ) -> None:
        try:
            from ultralytics import YOLO, YOLOWorld
        except ImportError as error:
            raise RuntimeError(
                "Ultralytics is optional. Install requirements_yolo.txt before using the YOLO backend."
            ) from error

        self.prompts = [item.strip() for item in prompts or [] if item.strip()]
        self.model = YOLOWorld(str(model)) if self.prompts else YOLO(str(model))
        if self.prompts:
            self.model.set_classes(self.prompts)
        self.confidence = confidence
        self.iou = iou
        self.image_size = image_size
        self.device = device
        self.max_detections = max_detections

    def predict(self, image: np.ndarray) -> tuple[list[YoloDetection], np.ndarray, dict[str, float]]:
        result = self.model.predict(
            image,
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.image_size,
            device=self.device,
            max_det=self.max_detections,
            verbose=False,
        )[0]
        detections = self._parse_result(result)
        speed = {key: float(value) for key, value in result.speed.items()}
        return detections, result.plot(), speed

    @staticmethod
    def _parse_result(result: Any) -> list[YoloDetection]:
        detections: list[YoloDetection] = []

        if result.obb is not None and len(result.obb):
            polygons = result.obb.xyxyxyxy.detach().cpu().numpy()
            centers = result.obb.xywhr.detach().cpu().numpy()
            classes = result.obb.cls.detach().cpu().numpy().astype(int)
            confidences = result.obb.conf.detach().cpu().numpy()
            for polygon, center, class_id, confidence in zip(polygons, centers, classes, confidences):
                x0, y0 = polygon.min(axis=0)
                x1, y1 = polygon.max(axis=0)
                detections.append(
                    YoloDetection(
                        label=result.names[int(class_id)],
                        confidence=round(float(confidence), 5),
                        center_px=(round(float(center[0]), 2), round(float(center[1]), 2)),
                        box_xyxy=tuple(round(float(value), 2) for value in (x0, y0, x1, y1)),
                        angle_deg=round(_long_edge_angle(polygon), 2),
                        polygon_px=np.round(polygon, 2).tolist(),
                    )
                )
            return detections

        if result.boxes is None:
            return detections

        boxes = result.boxes.xyxy.detach().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        confidences = result.boxes.conf.detach().cpu().numpy()
        mask_polygons = result.masks.xy if result.masks is not None else []

        for index, (box, class_id, confidence) in enumerate(zip(boxes, classes, confidences)):
            x0, y0, x1, y1 = (float(value) for value in box)
            polygon: np.ndarray | None = None
            angle: float | None = None
            if index < len(mask_polygons) and len(mask_polygons[index]) >= 3:
                polygon = np.asarray(mask_polygons[index], dtype=np.float32)
                angle = _long_edge_angle(polygon)
                center = polygon.mean(axis=0)
            else:
                center = np.asarray([(x0 + x1) / 2.0, (y0 + y1) / 2.0])

            detections.append(
                YoloDetection(
                    label=result.names[int(class_id)],
                    confidence=round(float(confidence), 5),
                    center_px=(round(float(center[0]), 2), round(float(center[1]), 2)),
                    box_xyxy=tuple(round(value, 2) for value in (x0, y0, x1, y1)),
                    angle_deg=None if angle is None else round(angle, 2),
                    polygon_px=None if polygon is None else np.round(polygon, 2).tolist(),
                )
            )
        return detections
