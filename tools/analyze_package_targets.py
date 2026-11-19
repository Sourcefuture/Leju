#!/usr/bin/env python3
"""Detect the three parcel targets in a saved Kuavo head RGB-D frame.

The current camera driver publishes an optical TF tree that is disconnected from
the robot TF tree.  This tool therefore estimates camera roll/pitch from the
table plane and uses the camera origin reported by the robot URDF.  The result
is suitable for planning a conservative hover pose, but should be validated
before commanding the arm.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def components(mask: np.ndarray, min_area: int = 300):
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), 8
    )
    result = []
    for idx in range(1, n):
        x, y, w, h, area = stats[idx]
        if area >= min_area:
            result.append(
                {
                    "mask": labels == idx,
                    "bbox_xywh": [int(x), int(y), int(w), int(h)],
                    "area_px": int(area),
                    "centroid_uv": centroids[idx].tolist(),
                }
            )
    return result


def deproject(mask, depth_m, intrinsics):
    fy, fx = intrinsics[4], intrinsics[0]
    cx, cy = intrinsics[2], intrinsics[5]
    valid = mask & np.isfinite(depth_m) & (depth_m > 0.3) & (depth_m < 2.0)
    v, u = np.nonzero(valid)
    if len(u) < 20:
        raise ValueError("not enough valid depth pixels in target mask")
    z = depth_m[v, u]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    points = np.column_stack((x, y, z))
    return np.median(points, axis=0), int(len(points))


def fit_table_plane(rgb, depth_m, intrinsics):
    """Fit y = ax + bz + c over unobstructed wood pixels."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    polygon = np.zeros(rgb.shape[:2], np.uint8)
    cv2.fillConvexPoly(
        polygon,
        np.array([[255, 250], [590, 250], [847, 477], [110, 455]], np.int32),
        1,
    )
    mask = (
        (polygon > 0)
        & (depth_m > 0.3)
        & (depth_m < 1.5)
        & (hsv[:, :, 0] >= 5)
        & (hsv[:, :, 0] <= 40)
        & (hsv[:, :, 1] >= 20)
        & (hsv[:, :, 1] <= 180)
        & (hsv[:, :, 2] >= 60)
    )
    # Remove parcels, bin, scale, and robot hands from the fit.
    for x1, y1, x2, y2 in (
        (285, 250, 410, 330),
        (230, 320, 380, 450),
        (420, 300, 545, 430),
        (550, 250, 848, 480),
        (0, 390, 848, 480),
    ):
        mask[y1:y2, x1:x2] = False

    fy, fx = intrinsics[4], intrinsics[0]
    cx, cy = intrinsics[2], intrinsics[5]
    v, u = np.nonzero(mask)
    z = depth_m[v, u]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    design = np.column_stack((x, z, np.ones_like(x)))

    # Two robust least-squares passes are enough because the color mask is clean.
    coeff, *_ = np.linalg.lstsq(design, y, rcond=None)
    for threshold in (0.015, 0.008):
        residual = np.abs(y - design @ coeff)
        keep = residual < threshold
        coeff, *_ = np.linalg.lstsq(design[keep], y[keep], rcond=None)
    residual = np.abs(y - design @ coeff)
    keep = residual < 0.008

    a, b, c = coeff
    # The table upward normal expressed in optical coordinates.  It also equals
    # the z row of a camera-to-waist rotation matrix.
    normal = np.array([a, -1.0, b], dtype=float)
    normal /= np.linalg.norm(normal)
    return {
        "normal_camera": normal,
        "plane_offset_m": float(c / np.linalg.norm([a, -1.0, b])),
        "inliers": int(keep.sum()),
        "samples": int(len(keep)),
        "median_residual_m": float(np.median(residual[keep])),
    }


def camera_to_waist_rotation(table_normal):
    # Camera +x points approximately toward robot right, so robot +y (left)
    # starts at camera -x.  Project it into the table plane to absorb camera roll.
    robot_z = np.asarray(table_normal, dtype=float)
    robot_y = np.array([-1.0, 0.0, 0.0])
    robot_y -= robot_z * np.dot(robot_y, robot_z)
    robot_y /= np.linalg.norm(robot_y)
    robot_x = np.cross(robot_y, robot_z)
    robot_x /= np.linalg.norm(robot_x)
    return np.vstack((robot_x, robot_y, robot_z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument(
        "--camera-origin-waist",
        nargs=3,
        type=float,
        default=[0.094, 0.044, 0.569],
        metavar=("X", "Y", "Z"),
        help="camera optical origin in waist_yaw_link, metres",
    )
    parser.add_argument("--hover-clearance", type=float, default=0.15)
    args = parser.parse_args()

    rgb = cv2.imread(str(args.capture_dir / "rgb.jpg"))
    depth_m = np.load(args.capture_dir / "depth_mm.npy").astype(float) / 1000.0
    metadata = json.loads((args.capture_dir / "camera.json").read_text())
    intrinsics = metadata["K"]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    purple = (
        (hsv[:, :, 0] >= 115)
        & (hsv[:, :, 0] <= 165)
        & (hsv[:, :, 1] >= 25)
        & (hsv[:, :, 1] <= 180)
        & (hsv[:, :, 2] >= 100)
    )
    purple[:230, :] = False
    purple[:, :180] = False
    purple_parts = sorted(
        components(purple, min_area=800), key=lambda item: item["centroid_uv"][1]
    )
    if len(purple_parts) != 2:
        raise RuntimeError(f"expected two purple parcels, detected {len(purple_parts)}")

    dark = hsv[:, :, 2] <= 100
    dark[:280, :] = False
    dark[400:, :] = False
    dark[:, :540] = False
    black_parts = sorted(components(dark, min_area=1000), key=lambda x: -x["area_px"])
    if not black_parts:
        raise RuntimeError("black parcel was not detected")

    detections = [
        ("purple_far", purple_parts[0], (255, 0, 255)),
        ("purple_near", purple_parts[1], (255, 0, 255)),
        ("black_in_bin", black_parts[0], (0, 255, 255)),
    ]
    plane = fit_table_plane(rgb, depth_m, intrinsics)
    rotation = camera_to_waist_rotation(plane["normal_camera"])
    translation = np.asarray(args.camera_origin_waist)
    table_z = float(translation[2] - plane["plane_offset_m"])

    annotated = rgb.copy()
    output = {
        "source": str(args.capture_dir),
        "frame": "waist_yaw_link (estimated camera extrinsic)",
        "camera_origin_waist_m": translation.tolist(),
        "camera_to_waist_rotation": rotation.tolist(),
        "table": {
            "z_waist_m": table_z,
            "fit_inliers": plane["inliers"],
            "fit_samples": plane["samples"],
            "median_residual_m": plane["median_residual_m"],
        },
        "targets": [],
        "warning": (
            "Camera and robot TF trees are disconnected. Roll/pitch are fitted "
            "from the table; translation and yaw use the physical mount assumption."
        ),
    }
    for label, detection, color in detections:
        # Erode target masks to avoid mixed depth at parcel edges.
        core = cv2.erode(detection["mask"].astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        point_camera, count = deproject(core, depth_m, intrinsics)
        point_waist = rotation @ point_camera + translation
        hover = point_waist.copy()
        hover[2] += args.hover_clearance
        item = {
            "id": label,
            "bbox_xywh": detection["bbox_xywh"],
            "centroid_uv": [round(v, 2) for v in detection["centroid_uv"]],
            "valid_depth_pixels": count,
            "center_camera_m": np.round(point_camera, 4).tolist(),
            "center_waist_m_est": np.round(point_waist, 4).tolist(),
            "hover_waist_m_est": np.round(hover, 4).tolist(),
            "height_above_table_m": round(float(point_waist[2] - table_z), 4),
        }
        output["targets"].append(item)

        x, y, w, h = detection["bbox_xywh"]
        cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
        text = f"{label}: hover [{hover[0]:.2f},{hover[1]:.2f},{hover[2]:.2f}]m"
        cv2.putText(
            annotated,
            text,
            (max(5, x - 20), max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2,
            cv2.LINE_AA,
        )

    (args.capture_dir / "targets.json").write_text(
        json.dumps(output, indent=2, ensure_ascii=False) + "\n"
    )
    cv2.imwrite(str(args.capture_dir / "targets_annotated.jpg"), annotated)
    print(json.dumps(output, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
