#!/usr/bin/python3
"""Show a read-only Kuavo JPEG stream over SSH; local ROS is not required.

Run with /usr/bin/python3 for WSLg/OpenCV windows. q/Esc closes the viewer;
s saves the original frame. --save-latest atomically updates a local JPEG for
other vision programs. No robot control topic or service is used.
"""
from __future__ import annotations

import argparse
import getpass
import json
import os
from pathlib import Path
import shlex
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np
import paramiko

TOPICS = {"head": "/cam_h/color/image_raw/compressed",
          "left": "/cam_l/color/image_raw/compressed",
          "right": "/cam_r/color/image_raw/compressed"}
HEADER = struct.Struct("!4sId")
REMOTE = r'''
import sys,time,struct,threading
import rospy
from sensor_msgs.msg import CompressedImage
rospy.init_node('kuavo_ssh_readonly_camera',anonymous=True,disable_signals=True,log_level=rospy.ERROR)
lock=threading.Lock();latest=[None];last_received=[time.monotonic()]
def receive(msg):
    with lock:
        latest[0]=msg
        last_received[0]=time.monotonic()
sub=rospy.Subscriber(__TOPIC__,CompressedImage,receive,queue_size=1,buff_size=2**22,tcp_nodelay=True)
last_stamp=None
try:
    while not rospy.is_shutdown():
        time.sleep(1.0/__FPS__)
        with lock:
            msg=latest[0];age=time.monotonic()-last_received[0]
        if age>20:raise RuntimeError('No camera messages for 20 seconds')
        if msg is None:continue
        stamp=msg.header.stamp.to_sec()
        if stamp==last_stamp:continue
        data=bytes(msg.data)
        sys.stdout.buffer.write(struct.pack('!4sId',b'KJPG',len(data),stamp)+data)
        sys.stdout.buffer.flush();last_stamp=stamp
except (BrokenPipeError,IOError):
    pass
finally:
    sub.unregister()
    rospy.signal_shutdown('SSH viewer disconnected')
'''


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f".{os.getpid()}.tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


class Receiver(threading.Thread):
    def __init__(self, channel):
        super().__init__(daemon=True)
        self.channel = channel
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.serial = 0

    def read_exact(self, size):
        chunks = []
        remaining = size
        while remaining and not self.stop_event.is_set():
            while self.channel.recv_stderr_ready():
                message = self.channel.recv_stderr(4096).decode("utf-8", "replace").strip()
                if message:
                    print(message, file=sys.stderr, flush=True)
            try:
                part = self.channel.recv(remaining)
            except socket.timeout:
                continue
            if not part:
                raise EOFError("Remote camera stream ended")
            chunks.append(part)
            remaining -= len(part)
        if remaining:
            raise EOFError("Viewer closed")
        return b"".join(chunks)

    def run(self):
        try:
            while not self.stop_event.is_set():
                magic, size, stamp = HEADER.unpack(self.read_exact(HEADER.size))
                if magic != b"KJPG" or not 1 <= size <= 10_000_000:
                    raise ValueError("Invalid camera stream header")
                payload = self.read_exact(size)
                with self.lock:
                    self.serial += 1
                    self.latest = (self.serial, stamp, payload, time.monotonic())
        except Exception as error:
            if not self.stop_event.is_set():
                self.error = f"{type(error).__name__}: {error}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.26.1")
    parser.add_argument("--user", default="lab")
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--camera", choices=TOPICS, default="head")
    parser.add_argument("--fps", type=float, default=15)
    parser.add_argument("--password-env", default="KUAVO_SSH_PASSWORD")
    parser.add_argument("--save-latest", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/robot_camera/snapshots"))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--seconds", type=float, help="Stop after this many seconds of received video")
    args = parser.parse_args()
    if not .5 <= args.fps <= 30:
        parser.error("--fps must be in 0.5..30")
    if args.max_frames is not None and args.max_frames < 1:
        parser.error("--max-frames must be positive")
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    if not args.headless and any("GUI:" in line and "NONE" in line for line in cv2.getBuildInformation().splitlines()):
        parser.error("This OpenCV is headless. Use /usr/bin/python3 or --headless.")
    password = os.environ.get(args.password_env)
    if password is None:
        password = getpass.getpass(f"SSH password for {args.user}@{args.host}: ")
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    receiver = None
    frames = 0
    first_received = None
    first_stamp = last_stamp = None
    window = f"Kuavo {args.camera} - SSH read-only (q/Esc: exit, s: save)"
    try:
        client.connect(args.host, port=args.port, username=args.user, password=password,
                       look_for_keys=False, allow_agent=False, timeout=8, banner_timeout=8, auth_timeout=8)
        password = None
        source = REMOTE.replace("__TOPIC__", repr(TOPICS[args.camera])).replace("__FPS__", repr(args.fps))
        command = "bash --noprofile --norc -c " + shlex.quote(
            "source /opt/ros/noetic/setup.bash && " +
            shlex.join(["env", f"ROS_MASTER_URI=http://{args.host}:11311", f"ROS_IP={args.host}", "python3", "-"]))
        stdin, stdout, stderr = client.exec_command(command)
        stdin.write(source)
        stdin.flush()
        stdin.channel.shutdown_write()
        channel = stdout.channel
        channel.settimeout(1)
        receiver = Receiver(channel)
        receiver.start()
        print(f"Connected: {args.user}@{args.host} | {TOPICS[args.camera]} | limit {args.fps:g} FPS", flush=True)
        if not args.headless:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 1018, 576)
        started = time.monotonic()
        serial = -1
        image = payload = None
        while True:
            with receiver.lock:
                latest = receiver.latest
            if latest and latest[0] != serial:
                serial, stamp, payload, received = latest
                image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("Cannot decode camera JPEG")
                first_received = received if first_received is None else first_received
                first_stamp = stamp if first_stamp is None else first_stamp
                last_stamp = stamp
                frames += 1
                if frames == 1:
                    print(f"Receiving {image.shape[1]}x{image.shape[0]} video. q/Esc: exit; s: save.", flush=True)
                if args.save_latest:
                    atomic_write(args.save_latest, payload)
                    meta = dict(topic=TOPICS[args.camera], stamp=stamp, width=image.shape[1], height=image.shape[0],
                                received_at_unix=time.time(), motion_commands_sent=False)
                    atomic_write(args.save_latest.with_suffix(".json"), json.dumps(meta, indent=2).encode())
            if receiver.error:
                raise RuntimeError(receiver.error)
            now = time.monotonic()
            if first_received is None and now-started > 25:
                raise TimeoutError("No head frames within 25s; check remote camera publisher")
            if not args.headless:
                panel = image.copy() if image is not None else np.zeros((480,848,3), np.uint8)
                age = now-latest[3] if latest else None
                text = f"{args.camera} | age {age:.2f}s | q/Esc exit, s save" if age is not None else "Waiting for remote camera..."
                cv2.rectangle(panel, (0,0), (panel.shape[1],30), (0,0,0), -1)
                cv2.putText(panel, text, (8,21), cv2.FONT_HERSHEY_SIMPLEX, .55,
                            (0,255,0) if age is not None and age<1 else (0,120,255), 1, cv2.LINE_AA)
                cv2.imshow(window, panel)
                key = cv2.waitKey(15) & 255
                # OpenCV 4.2/GTK does not implement WND_PROP_VISIBLE and
                # returns -1 even for a valid window. AUTOSIZE is supported.
                if key in (27, ord('q')) or cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE)<0:
                    break
                if key == ord('s') and payload is not None:
                    path = args.output_dir / (time.strftime(f"{args.camera}_%Y%m%d_%H%M%S")+f"_{time.time_ns()%1000000000:09d}.jpg")
                    atomic_write(path, payload)
                    print(f"Saved {path}", flush=True)
            else:
                time.sleep(.01)
            if args.max_frames and frames >= args.max_frames:
                break
            if args.seconds and first_received is not None and now-first_received >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if receiver:
            receiver.stop_event.set()
            receiver.channel.close()
            receiver.join(timeout=2)
        client.close()
        if not args.headless:
            cv2.destroyAllWindows()
    print(json.dumps(dict(frames=frames, first_stamp=first_stamp,
                          last_stamp=last_stamp, motion_commands_sent=False)), flush=True)


if __name__ == "__main__":
    main()
