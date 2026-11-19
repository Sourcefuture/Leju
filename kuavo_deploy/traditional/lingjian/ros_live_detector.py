#!/usr/bin/env python3
"""ROS sidecar node that detects Lingjian parts without publishing robot commands."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from kuavo_deploy.traditional.lingjian.classical_detector import annotate, detect


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/cam_h/color/image_raw/compressed")
    parser.add_argument("--output-topic", default="/lingjian/perception/annotated/compressed")
    parser.add_argument("--detections-topic", default="/lingjian/perception/detections")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.yaml"))
    parser.add_argument("--max-rate", type=float, default=10.0)
    return parser.parse_args()


class LiveDetector:
    def __init__(self, args: argparse.Namespace):
        self.config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
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

        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn_throttle(2.0, "Unable to decode head-camera frame")
            return

        begin = time.perf_counter()
        detections, _ = detect(image, self.config)
        latency_ms = (time.perf_counter() - begin) * 1000.0
        output = annotate(image, detections)
        cv2.putText(output, f"detector {latency_ms:.1f}ms", (10, output.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

        ok, output_jpeg = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            rospy.logwarn_throttle(2.0, "Unable to encode annotated frame")
            return

        annotated_message = CompressedImage()
        annotated_message.header = message.header
        annotated_message.format = "jpeg"
        annotated_message.data = output_jpeg.tobytes()
        self.image_publisher.publish(annotated_message)

        payload = {
            "stamp": message.header.stamp.to_sec(),
            "latency_ms": round(latency_ms, 3),
            "detections": [asdict(item) for item in detections],
        }
        self.detection_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main() -> None:
    args = parse_args()
    rospy.init_node("lingjian_live_detector", anonymous=False)
    LiveDetector(args)
    rospy.loginfo("Lingjian live detector started; this node does not publish robot commands")
    rospy.spin()


if __name__ == "__main__":
    main()
