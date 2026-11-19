#!/usr/bin/python3
"""Read-only live viewer for Kuavo head, left-wrist and right-wrist cameras."""

from __future__ import annotations

import argparse
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage


CAMERAS = {
    "head": "/cam_h/color/image_raw/compressed",
    "left": "/cam_l/color/image_raw/compressed",
    "right": "/cam_r/color/image_raw/compressed",
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera",
        choices=("all", *CAMERAS),
        default="all",
        help="camera panel(s) to display",
    )
    parser.add_argument("--panel-width", type=int, default=480)
    parser.add_argument("--stale-s", type=float, default=1.0)
    return parser.parse_args()


class Viewer:
    def __init__(self, names, panel_width, stale_s):
        self.names = names
        self.panel_width = panel_width
        self.panel_height = round(panel_width * 480 / 848)
        self.stale_s = stale_s
        self.lock = threading.Lock()
        self.frames = {}
        self.received_at = {}
        self.subscribers = [
            rospy.Subscriber(
                CAMERAS[name],
                CompressedImage,
                self.callback,
                callback_args=name,
                queue_size=1,
                buff_size=2**22,
                tcp_nodelay=True,
            )
            for name in names
        ]

    def callback(self, message, name):
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        with self.lock:
            self.frames[name] = image
            self.received_at[name] = time.monotonic()

    def panel(self, name):
        with self.lock:
            frame = self.frames.get(name)
            received_at = self.received_at.get(name)
            frame = None if frame is None else frame.copy()

        if frame is None:
            panel = np.zeros((self.panel_height, self.panel_width, 3), np.uint8)
            status = "WAITING FOR FRAMES"
            color = (0, 165, 255)
        else:
            panel = cv2.resize(
                frame,
                (self.panel_width, self.panel_height),
                interpolation=cv2.INTER_AREA,
            )
            age = time.monotonic() - received_at
            status = f"age {age:.2f}s"
            color = (0, 255, 0) if age <= self.stale_s else (0, 0, 255)

        cv2.rectangle(panel, (0, 0), (self.panel_width, 54), (0, 0, 0), -1)
        cv2.putText(
            panel,
            f"{name}: {CAMERAS[name]}",
            (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            status,
            (10, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        return panel

    def run(self):
        window = "Kuavo cameras (q/Esc to quit)"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        while not rospy.is_shutdown():
            image = np.hstack([self.panel(name) for name in self.names])
            cv2.imshow(window, image)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    rospy.init_node("view_kuavo_cameras", anonymous=True, disable_signals=True)
    names = list(CAMERAS) if args.camera == "all" else [args.camera]
    Viewer(names, args.panel_width, args.stale_s).run()


if __name__ == "__main__":
    main()
