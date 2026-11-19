#!/usr/bin/env python3
"""Offline checks for hand isolation, preview and changing feedback; no ROS."""
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

from kuavo_hand_controller import (
    KuavoHandController, positions, ramp_targets, targets_from_feedback, validate_request,
)
from kuavo_remote_hand import save_record


class TargetTests(unittest.TestCase):
    def setUp(self):
        self.current = {"left": [94, 0, 34, 34, 34, 34],
                        "right": [91, 20, 30, 40, 50, 60]}

    def test_malformed_positions_fail_instead_of_clipping(self):
        for bad in ([0] * 5, [0] * 7, [-1] + [0] * 5,
                    [101] + [0] * 5, [True] + [0] * 5, [2.5] + [0] * 5):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                positions(bad)

    def test_close_only_selected_hand_and_keep_thumb_rotation(self):
        target = targets_from_feedback(self.current, {"command": "close", "hand": "left"})
        self.assertEqual(target["left"], [80, 0, 80, 80, 80, 80])
        self.assertEqual(target["right"], self.current["right"])
        self.assertEqual(self.current["left"][2], 34)

    def test_open_both_preserves_each_thumb_and_override_is_explicit(self):
        target = targets_from_feedback(self.current, {"command": "open", "hand": "both"})
        self.assertEqual(target["left"], [0, 0, 0, 0, 0, 0])
        self.assertEqual(target["right"], [0, 20, 0, 0, 0, 0])
        target = targets_from_feedback(self.current, {
            "command": "open", "hand": "right", "thumb_rotation": 0})
        self.assertEqual(target["right"], [0] * 6)

    def test_both_custom_arrays_are_independent(self):
        target = targets_from_feedback(self.current, {
            "command": "send", "hand": "both", "left_values": [10] * 6,
            "right_values": [80] * 6})
        self.assertEqual(target, {"left": [10] * 6, "right": [80] * 6})
        with self.assertRaises(ValueError):
            validate_request({"command": "send", "hand": "both", "values": [10] * 6})

    def test_ramp_never_changes_unselected_hand_or_overshoots(self):
        target = targets_from_feedback(self.current, {"command": "close", "hand": "left"})
        frames = list(ramp_targets(self.current, target, .7))
        self.assertEqual(frames[-1], target)
        for frame in frames:
            self.assertEqual(frame["right"], self.current["right"])
            for value in frame["left"][2:]:
                self.assertTrue(34 <= value <= 80)

    def test_invalid_timing_or_execute_fail_before_ros(self):
        for request in ({"command": "state", "timeout_s": float("inf")},
                        {"command": "close", "duration_s": 0},
                        {"command": "close", "execute": "false"},
                        {"command": "close", "hand": "lef"}):
            with self.subTest(request=request), self.assertRaises(ValueError):
                validate_request(request)


class ControllerTests(unittest.TestCase):
    def test_preview_does_not_create_a_command_publisher(self):
        controller = object.__new__(KuavoHandController)
        controller.read_state = lambda: {"left": [95, 0, 35, 35, 35, 35], "right": [0] * 6}
        controller._connect_publisher = lambda: self.fail("preview created a publisher")
        result = controller.run({"command": "close", "hand": "left", "execute": False})
        self.assertFalse(result["command_sent"])
        self.assertEqual(result["status"], "planned")
        self.assertEqual(result["target"]["left"][0], 80)
        self.assertEqual(result["target"]["left"][1], 0)

    def test_execution_rereads_feedback_after_connection(self):
        initial = {"left": [90, 0, 0, 0, 0, 0], "right": [0] * 6}
        fresh = {"left": [85, 20, 20, 20, 20, 20], "right": [10] * 6}
        final = {"left": [80, 20, 80, 80, 80, 80], "right": [10] * 6}
        states = iter([initial, fresh, final])
        messages = []
        controller = object.__new__(KuavoHandController)
        controller.read_state = lambda: next(states)
        controller._connect_publisher = lambda: None
        controller.publisher = types.SimpleNamespace(publish=messages.append)
        controller.rospy = types.SimpleNamespace(
            Time=types.SimpleNamespace(now=lambda: 123), sleep=lambda seconds: None)
        class Message:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None)
        module = types.ModuleType("kuavo_msgs.msg")
        module.robotHandPosition = Message
        with patch.dict("sys.modules", {"kuavo_msgs": types.ModuleType("kuavo_msgs"),
                                        "kuavo_msgs.msg": module}):
            result = controller.run({"command": "close", "hand": "left",
                                     "duration_s": .5, "execute": True})
        self.assertEqual(result["pre_state"], fresh)
        self.assertEqual(result["status"], "executed")
        self.assertTrue(result["command_sent"])
        self.assertEqual(result["target"]["left"][1], 20)
        self.assertTrue(all(m.right_hand_position == [10] * 6 for m in messages))

    def test_history_and_latest_feedback_are_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            result = {"action_id": "example", "post_state": {"left": [0] * 6},
                      "status": "planned", "command_sent": False}
            save_record(directory, result)
            root = Path(directory)
            self.assertEqual(json.loads((root / "latest_state.json").read_text()), result["post_state"])
            self.assertEqual(json.loads((root / "actions/example.json").read_text()), result)
            self.assertEqual(len((root / "history.jsonl").read_text().splitlines()), 1)


if __name__ == "__main__":
    unittest.main()
