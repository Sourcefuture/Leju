#!/usr/bin/python3
"""Capture one Kuavo RGB-D snapshot without publishing ROS commands.

The RGB and depth messages are acquired sequentially.  Their header timestamps
are recorded so callers can decide whether the pair is sufficiently close for
their application.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--camera", default="cam_h", choices=("cam_h", "cam_l", "cam_r", "cam_w")
    )
    parser.add_argument(
        "--depth-topic",
        help="override the camera-specific default depth Image topic",
    )
    parser.add_argument(
        "--camera-info-topic",
        help="override the camera-specific default depth CameraInfo topic",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser.parse_args()


def decode_depth(msg: Image) -> np.ndarray:
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")

    dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    values_per_row = msg.step // dtype.itemsize
    depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, values_per_row)
    return depth[:, : msg.width].astype(np.uint16, copy=True)


def stamp(msg) -> float:
    return msg.header.stamp.to_sec()


def field_of_view(width: int, height: int, matrix) -> dict:
    """Return the effective pinhole FOV represented by a 3x3 intrinsic matrix."""
    fx, fy = float(matrix[0]), float(matrix[4])
    cx, cy = float(matrix[2]), float(matrix[5])
    if min(width, height) <= 0 or min(fx, fy) <= 0:
        return {}

    to_deg = lambda value: math.degrees(value)
    left = to_deg(math.atan(cx / fx))
    right = to_deg(math.atan((width - cx) / fx))
    top = to_deg(math.atan(cy / fy))
    bottom = to_deg(math.atan((height - cy) / fy))
    return {
        "horizontal_deg": left + right,
        "vertical_deg": top + bottom,
        "left_deg": left,
        "right_deg": right,
        "top_deg": top,
        "bottom_deg": bottom,
    }


def camera_info_dict(msg: CameraInfo) -> dict:
    return {
        "topic_frame_id": msg.header.frame_id,
        "stamp": stamp(msg),
        "width": int(msg.width),
        "height": int(msg.height),
        "distortion_model": msg.distortion_model,
        "D": list(msg.D),
        "K": list(msg.K),
        "R": list(msg.R),
        "P": list(msg.P),
        "binning_x": int(msg.binning_x),
        "binning_y": int(msg.binning_y),
        "roi": {
            "x_offset": int(msg.roi.x_offset),
            "y_offset": int(msg.roi.y_offset),
            "height": int(msg.roi.height),
            "width": int(msg.roi.width),
            "do_rectify": bool(msg.roi.do_rectify),
        },
        "fov_from_K": field_of_view(msg.width, msg.height, msg.K),
        "fov_from_P": field_of_view(
            msg.width,
            msg.height,
            (msg.P[0], 0.0, msg.P[2], 0.0, msg.P[5], msg.P[6], 0.0, 0.0, 1.0),
        ),
    }


def main():
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    rospy.init_node("capture_kuavo_rgbd", anonymous=True, disable_signals=True)
    root = f"/{args.camera}"
    default_depth_topics = {
        "cam_h": f"{root}/depth/image_raw",
        "cam_l": f"{root}/depth/image_rect_raw",
        "cam_r": f"{root}/aligned_depth_to_color/image_raw",
        "cam_w": f"{root}/depth/image_raw",
    }
    default_info_topics = {
        "cam_h": f"{root}/depth/camera_info",
        "cam_l": f"{root}/depth/camera_info",
        "cam_r": f"{root}/aligned_depth_to_color/camera_info",
        "cam_w": f"{root}/depth/camera_info",
    }
    depth_topic = args.depth_topic or default_depth_topics[args.camera]
    camera_info_topic = args.camera_info_topic or default_info_topics[args.camera]

    rgb_msg = rospy.wait_for_message(
        f"{root}/color/image_raw/compressed", CompressedImage, timeout=args.timeout
    )
    depth_msg = rospy.wait_for_message(
        depth_topic, Image, timeout=args.timeout
    )
    info_msg = rospy.wait_for_message(
        camera_info_topic, CameraInfo, timeout=args.timeout
    )

    rgb = cv2.imdecode(np.frombuffer(rgb_msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if rgb is None:
        raise RuntimeError("Failed to decode the compressed RGB frame")
    depth = decode_depth(depth_msg)

    cv2.imwrite(str(args.output / "rgb.jpg"), rgb)
    np.save(str(args.output / "depth_mm.npy"), depth)

    valid = depth > 0
    preview = np.zeros_like(depth, dtype=np.uint8)
    if valid.any():
        low, high = np.percentile(depth[valid], (2, 98))
        if high > low:
            preview[valid] = np.clip(
                (depth[valid].astype(np.float32) - low) * 255.0 / (high - low),
                0,
                255,
            ).astype(np.uint8)
    cv2.imwrite(str(args.output / "depth_preview.png"), cv2.applyColorMap(preview, cv2.COLORMAP_TURBO))

    metadata = {
        "camera": args.camera,
        "rgb_topic": f"{root}/color/image_raw/compressed",
        "depth_topic": depth_topic,
        "camera_info_topic": camera_info_topic,
        "frame_id": depth_msg.header.frame_id,
        "rgb_frame_id": rgb_msg.header.frame_id,
        "depth_frame_id": depth_msg.header.frame_id,
        "rgb_stamp": stamp(rgb_msg),
        "depth_stamp": stamp(depth_msg),
        "stamp_delta_seconds": abs(stamp(rgb_msg) - stamp(depth_msg)),
        "rgb_format": rgb_msg.format,
        "rgb_compressed_bytes": len(rgb_msg.data),
        "rgb_shape_hwc": list(rgb.shape),
        "rgb_decoded_color_order": "BGR",
        "depth_encoding": depth_msg.encoding,
        "depth_is_bigendian": bool(depth_msg.is_bigendian),
        "depth_step_bytes": int(depth_msg.step),
        "depth_units": "mm for 16UC1/mono16",
        "width": depth_msg.width,
        "height": depth_msg.height,
        "camera_info": camera_info_dict(info_msg),
        # Keep K and D at the top level for compatibility with earlier captures.
        "K": list(info_msg.K),
        "D": list(info_msg.D),
        "fov_from_K": field_of_view(info_msg.width, info_msg.height, info_msg.K),
        "valid_depth_fraction": float(valid.mean()),
        "valid_depth_min_mm": int(depth[valid].min()) if valid.any() else None,
        "valid_depth_max_mm": int(depth[valid].max()) if valid.any() else None,
        "valid_depth_median_mm": float(np.median(depth[valid])) if valid.any() else None,
        "valid_depth_p95_mm": float(np.percentile(depth[valid], 95)) if valid.any() else None,
    }
    (args.output / "camera.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
