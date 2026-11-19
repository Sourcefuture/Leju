#!/usr/bin/env python3
"""Kuavo hand targets and robot-side ROS execution (0=open, 100=closed).

The SSH entry point is kuavo_remote_hand.py. ROS imports are deferred so target
generation and interpolation can also be checked on WSL without ROS packages.
"""
from __future__ import annotations

import argparse
import json
import math
import numbers
import threading
import time


HAND_NAMES = {
    side: [prefix + name for name in
           ("thumb", "thumb_aux", "index", "middle", "ring", "pinky")]
    for side, prefix in (("left", "l_"), ("right", "r_"))
}
COMMAND_TOPIC = "/control_robot_hand_position"
FEEDBACK_TOPIC = "/dexhand/state"


def positions(values):
    """Validate six integer position channels; never silently clip input."""
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("each hand needs exactly six integer values in 0..100")
    if any(isinstance(v, bool) or not isinstance(v, numbers.Integral)
           or not 0 <= v <= 100 for v in values):
        raise ValueError("hand positions must be integers in 0..100")
    return [int(v) for v in values]


def validate_request(request):
    command = request.get("command")
    if command not in ("state", "close", "open", "send"):
        raise ValueError("command must be state, close, open, or send")
    if type(request.get("execute", False)) is not bool:
        raise ValueError("execute must be a boolean")
    for key, default, low, high in (("duration_s", 2., .5, 30.),
                                    ("timeout_s", 15., 1., 60.)):
        value = request.get(key, default)
        if isinstance(value, bool) or not isinstance(value, numbers.Real) \
                or not math.isfinite(value) or not low <= value <= high:
            raise ValueError("%s must be in %s..%s" % (key, low, high))
    if command == "state":
        if request.get("execute", False):
            raise ValueError("state does not accept execute")
        return
    if request.get("hand", "left") not in ("left", "right", "both"):
        raise ValueError("hand must be left, right, or both")
    level = request.get("level", 80)
    if command == "close" and (type(level) is not int or not 0 <= level <= 100):
        raise ValueError("level must be an integer in 0..100")
    thumb = request.get("thumb_rotation")
    if thumb is not None and (type(thumb) is not int or not 0 <= thumb <= 100):
        raise ValueError("thumb_rotation must be an integer in 0..100")
    if command == "send":
        hand = request.get("hand", "left")
        if hand == "both":
            if request.get("values") is not None:
                raise ValueError("both hands: use left_values and right_values")
            positions(request.get("left_values"))
            positions(request.get("right_values"))
        else:
            if request.get("left_values") is not None or request.get("right_values") is not None:
                raise ValueError("single hand: use values")
            positions(request.get("values"))


def targets_from_feedback(current, request):
    """Use live feedback for the unselected hand and preserved thumb rotation."""
    validate_request(request)
    target = {side: positions(current[side]) for side in HAND_NAMES}
    command = request["command"]
    if command == "state":
        return target
    selected = tuple(HAND_NAMES) if request.get("hand", "left") == "both" \
        else (request.get("hand", "left"),)
    for side in selected:
        if command == "send":
            key = side + "_values" if len(selected) == 2 else "values"
            target[side] = positions(request[key])
        else:
            level = request.get("level", 80) if command == "close" else 0
            thumb = request.get("thumb_rotation")
            # Actual robot's installed linker_hand_mapping.h: channel 0 is
            # thumb flexion; channel 1 is thumb abduction / side swing.
            target[side] = [level, target[side][1] if thumb is None else thumb] + [level] * 4
    return target


def ramp_targets(current, target, duration_s, rate_hz=10):
    """Interpolate both hands together, retaining an exact final target."""
    count = max(1, int(math.ceil(duration_s * rate_hz)))
    for index in range(1, count + 1):
        fraction = index / count
        yield {side: [int(round(a + (b - a) * fraction))
                      for a, b in zip(current[side], target[side])]
               for side in HAND_NAMES}


class KuavoHandController:
    def __init__(self, timeout_s=15):
        import rospy
        from sensor_msgs.msg import JointState
        self.rospy = rospy
        self.timeout_s = timeout_s
        self.lock = threading.Lock()
        self.latest = None
        self.publisher = None
        if not rospy.core.is_initialized():
            rospy.init_node("kuavo_ssh_hand_control", anonymous=True, disable_signals=True)
        self.subscriber = rospy.Subscriber(
            FEEDBACK_TOPIC, JointState, self._receive, queue_size=1)

    def _receive(self, message):
        with self.lock:
            self.latest = (time.monotonic(), message)

    def read_state(self):
        started = time.monotonic()
        while not self.rospy.is_shutdown():
            with self.lock:
                sample = self.latest
            if sample is not None and sample[0] >= started - .1:
                received, msg = sample
                mapping = dict(zip(msg.name, msg.position))
                result = {"topic": FEEDBACK_TOPIC,
                          "stamp": msg.header.stamp.to_sec(),
                          "received_at_unix": time.time(),
                          "age_s": time.monotonic() - received,
                          "names": list(msg.name), "effort": list(msg.effort)}
                for side, names in HAND_NAMES.items():
                    if any(name not in mapping for name in names):
                        raise RuntimeError("unexpected hand feedback joint names")
                    raw = [float(mapping[name]) for name in names]
                    if any(not math.isfinite(v) or not 0 <= v <= 100 for v in raw):
                        raise RuntimeError("feedback is outside the compatible 0..100 range")
                    result[side] = positions([int(round(v)) for v in raw])
                return result
            if time.monotonic() - started >= self.timeout_s:
                raise RuntimeError("no fresh feedback on " + FEEDBACK_TOPIC)
            self.rospy.sleep(.02)
        raise RuntimeError("ROS has shut down")

    def _connect_publisher(self):
        from kuavo_msgs.msg import robotHandPosition
        self.publisher = self.rospy.Publisher(
            COMMAND_TOPIC, robotHandPosition, queue_size=1, latch=False)
        started = time.monotonic()
        # The current real robot has ONE subscriber, /nodelet_manager. Requiring
        # two connections caused the previous temporary scripts to wait 25s.
        while self.publisher.get_num_connections() < 1:
            if time.monotonic() - started >= self.timeout_s:
                raise RuntimeError("no TCPROS subscriber on " + COMMAND_TOPIC)
            self.rospy.sleep(.02)

    def run(self, request, progress=None):
        validate_request(request)
        before = self.read_state()
        result = {"schema_version": 1, "request": request,
                  "command_topic": COMMAND_TOPIC, "pre_state": before,
                  "post_state": before, "command_sent": False,
                  "grasp_verified": False, "status": "read"}
        if request["command"] == "state":
            return result
        result["target"] = targets_from_feedback(before, request)
        result["status"] = "planned"
        if not request.get("execute", False):
            return result  # No command publisher is even created in preview.
        self._connect_publisher()
        # Re-read AFTER connecting: startup must not make the initial state old.
        before = self.read_state()
        target = targets_from_feedback(before, request)
        result.update(pre_state=before, post_state=before, target=target,
                      status="executing", published_messages=0)
        from kuavo_msgs.msg import robotHandPosition
        duration_s = float(request.get("duration_s", 2.))
        steps = max(1, int(math.ceil(duration_s * 10)))
        started = time.monotonic()
        try:
            for index, values in enumerate(ramp_targets(before, target, duration_s), 1):
                message = robotHandPosition()
                message.header.stamp = self.rospy.Time.now()
                message.left_hand_position = values["left"]
                message.right_hand_position = values["right"]
                self.publisher.publish(message)
                result.update(command_sent=True, published_messages=index,
                              last_sent=values)
                if progress is not None and (index == steps or index % 5 == 0):
                    progress({"step": index, "of": steps, **values})
                remaining = started + index * duration_s / steps - time.monotonic()
                if remaining > 0:
                    self.rospy.sleep(remaining)
            self.rospy.sleep(.8)
            after = self.read_state()
            selected = tuple(HAND_NAMES) if request.get("hand", "left") == "both" \
                else (request.get("hand", "left"),)
            errors = {side: max(abs(a - b) for a, b in zip(after[side], target[side]))
                      for side in selected}
            result.update(post_state=after, selected_hand_max_error=errors,
                          status="executed" if max(errors.values()) <= 10
                          else "feedback_mismatch")
        except (Exception, KeyboardInterrupt) as error:
            result.update(status="interrupted" if isinstance(error, KeyboardInterrupt)
                          else "error", error=str(error) or type(error).__name__)
            try:
                result["post_state"] = self.read_state()
            except Exception as feedback_error:
                result["feedback_error"] = str(feedback_error)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request-json", required=True)
    args = parser.parse_args()
    request = json.loads(args.request_json)
    validate_request(request)
    controller = KuavoHandController(request.get("timeout_s", 15.))
    result = controller.run(
        request, lambda value: print("HAND_PROGRESS:" + json.dumps(value), flush=True))
    print("HAND_RESULT:" + json.dumps(result), flush=True)
    return 0 if result["status"] in ("read", "planned", "executed") else 3


if __name__ == "__main__":
    raise SystemExit(main())
