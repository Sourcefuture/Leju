#!/usr/bin/python3
"""Read-only Kuavo TF inspection and point transformation utility.

Examples:
  kuavo_tf_inspector.py list
  kuavo_tf_inspector.py lookup --target base_link --source cam_h_color_optical_frame
  kuavo_tf_inspector.py lookup --target base_link --source cam_h_color_optical_frame \
      --point 0.10 0.05 0.80

``lookup_transform(target, source)`` returns the transform that maps coordinates
from ``source`` into ``target``.  This program only subscribes to TF topics.
"""

from __future__ import annotations

import argparse
import json
import math
import sys

import rospy
import tf2_ros
import yaml


DEFAULT_CAMERA_FRAMES = (
    "cam_h_color_optical_frame",
    "cam_l_color_optical_frame",
    "cam_r_color_optical_frame",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=2.0,
        help="seconds to collect /tf and /tf_static before querying",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="list frames, parents and roots")
    list_parser.add_argument(
        "--check-cameras",
        action="store_true",
        help="also check whether each known camera frame connects to base_link",
    )

    lookup = subparsers.add_parser("lookup", help="look up source -> target")
    lookup.add_argument("--target", required=True, help="output coordinate frame")
    lookup.add_argument("--source", required=True, help="input coordinate frame")
    lookup.add_argument(
        "--point",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="optional point in source frame, in metres",
    )
    lookup.add_argument("--timeout-s", type=float, default=2.0)
    return parser.parse_args()


def frame_graph(buffer):
    raw = yaml.safe_load(buffer.all_frames_as_yaml()) or {}
    frames = {}
    for name, info in raw.items():
        frames[str(name)] = {
            "parent": str(info.get("parent", "")),
            "broadcaster": str(info.get("broadcaster", "")),
            "rate_hz": float(info.get("rate", 0.0)),
            "most_recent_transform": float(info.get("most_recent_transform", 0.0)),
            "oldest_transform": float(info.get("oldest_transform", 0.0)),
            "buffer_length": float(info.get("buffer_length", 0.0)),
        }
    children = set(frames)
    parents = {info["parent"] for info in frames.values() if info["parent"]}
    roots = sorted(parents - children)
    return frames, roots


def transform_dict(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        "target_frame": transform.header.frame_id,
        "source_frame": transform.child_frame_id,
        "stamp": transform.header.stamp.to_sec(),
        "translation_m": [t.x, t.y, t.z],
        "quaternion_xyzw": [q.x, q.y, q.z, q.w],
    }


def rotate_vector(vector, quaternion):
    """Rotate a vector by an xyzw unit quaternion without extra dependencies."""
    x, y, z = vector
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("TF quaternion has zero norm")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return [
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    ]


def apply_transform(point, payload):
    rotated = rotate_vector(point, payload["quaternion_xyzw"])
    return [
        rotated[index] + payload["translation_m"][index] for index in range(3)
    ]


def camera_connectivity(buffer):
    results = {}
    for frame in DEFAULT_CAMERA_FRAMES:
        ok, reason = buffer.can_transform(
            "base_link", frame, rospy.Time(0), return_debug_tuple=True
        )[:2]
        results[frame] = {"connected_to_base_link": bool(ok), "reason": str(reason)}
    return results


def main():
    args = parse_args()
    rospy.init_node("kuavo_tf_inspector", anonymous=True, disable_signals=True)
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0), debug=False)
    listener = tf2_ros.TransformListener(buffer)
    rospy.sleep(max(args.warmup_s, 0.1))

    frames, roots = frame_graph(buffer)
    if args.command == "list":
        payload = {
            "frame_count": len(frames),
            "roots": roots,
            "frames": frames,
        }
        if args.check_cameras:
            payload["camera_connectivity"] = camera_connectivity(buffer)
        print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
        return 0

    try:
        transform = buffer.lookup_transform(
            args.target,
            args.source,
            rospy.Time(0),
            rospy.Duration(args.timeout_s),
        )
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as error:
        print(
            json.dumps(
                {
                    "ok": False,
                    "target_frame": args.target,
                    "source_frame": args.source,
                    "error": str(error),
                    "known_roots": roots,
                    "known_frames": sorted(frames),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 2

    payload = {"ok": True, "transform": transform_dict(transform)}
    if args.point is not None:
        payload["point_source_m"] = list(args.point)
        payload["point_target_m"] = apply_transform(args.point, payload["transform"])
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
