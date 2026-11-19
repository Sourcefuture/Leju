#!/usr/bin/env python3
"""ROS sidecar for real-time YOLO inference; it never publishes robot commands."""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/letools-yolo-config")

from kuavo_deploy.traditional.lingjian.yolo_backend import YoloDetector


def load_ros_modules() -> None:
    """Import ROS lazily so ``--help`` and static checks work outside ROS shells."""
    global rospy, CompressedImage, String
    try:
        import rospy as rospy_module
        from sensor_msgs.msg import CompressedImage as compressed_image_type
        from std_msgs.msg import String as string_type
    except ImportError as error:
        raise RuntimeError(
            "ROS Python modules are unavailable. Source /opt/ros/noetic/setup.bash before launching the node."
        ) from error
    rospy = rospy_module
    CompressedImage = compressed_image_type
    String = string_type


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--prompts", nargs="*")
    parser.add_argument("--input-topic", default="/cam_h/color/image_raw/compressed")
    parser.add_argument("--output-topic", default="/lingjian/yolo/annotated/compressed")
    parser.add_argument("--detections-topic", default="/lingjian/yolo/detections")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--max-rate", type=float, default=10.0)
    return parser.parse_args()


class LiveYoloDetector:
    def __init__(self, args: argparse.Namespace) -> None:
        self.detector = YoloDetector(
            args.model,
            prompts=args.prompts,
            confidence=args.confidence,
            iou=args.iou,
            image_size=args.image_size,
            device=args.device,
        )
        self.min_interval = 0.0 if args.max_rate <= 0 else 1.0 / args.max_rate
        self.last_processed = 0.0
        self.image_publisher = rospy.Publisher(args.output_topic, CompressedImage, queue_size=1)
        self.detection_publisher = rospy.Publisher(args.detections_topic, String, queue_size=1)
        self.subscriber = rospy.Subscriber(
            args.input_topic,
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2**24,
        )

    def callback(self, message: CompressedImage) -> None:
        now = time.monotonic()
        if now - self.last_processed < self.min_interval:
            return
        self.last_processed = now

        image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn_throttle(2.0, "Unable to decode head-camera frame")
            return

        begin = time.perf_counter()
        detections, annotated, model_speed = self.detector.predict(image)
        wall_ms = (time.perf_counter() - begin) * 1000.0
        cv2.putText(
            annotated,
            f"YOLO {wall_ms:.1f}ms",
            (10, annotated.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        ok, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            rospy.logwarn_throttle(2.0, "Unable to encode annotated YOLO frame")
            return

        annotated_message = CompressedImage()
        annotated_message.header = message.header
        annotated_message.format = "jpeg"
        annotated_message.data = jpeg.tobytes()
        self.image_publisher.publish(annotated_message)
        self.detection_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "stamp": message.header.stamp.to_sec(),
                        "wall_latency_ms": round(wall_ms, 3),
                        "model_speed_ms": model_speed,
                        "detections": [asdict(item) for item in detections],
                    },
                    ensure_ascii=False,
                )
            )
        )


def main() -> None:
    args = parse_args()
    load_ros_modules()
    rospy.init_node("lingjian_yolo_detector", anonymous=False)
    LiveYoloDetector(args)
    rospy.loginfo("Lingjian YOLO sidecar started; this node does not publish robot commands")
    rospy.spin()


if __name__ == "__main__":
    main()
