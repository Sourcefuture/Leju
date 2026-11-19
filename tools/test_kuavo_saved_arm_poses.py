#!/usr/bin/env python3
"""Saved-arm isolation and replay preparation checks; no robot commands."""
import json
from pathlib import Path
import tempfile
import types
import unittest

from kuavo_arm_controller import KuavoArmController, saved_pose_target
from kuavo_remote_arm import (
    build_parser, controller_arguments, list_saved_poses, parse_remote_json,
    save_arm_pose, saved_pose_path,
)


def pose(arm):
    return {"pose_type": "kuavo_saved_arm_pose", "name": "example",
            "robot": "5", "robot_version": 55, "arm": arm,
            "arm7_deg": [10.] * 7,
            # This old other-arm state MUST NOT be used as a target.
            "saved_state": {"arm14_deg": [99.] * 14}}


class SavedArmTests(unittest.TestCase):
    def test_left_restore_keeps_live_right(self):
        current = list(range(14))
        target, arm = saved_pose_target(pose("left"), current)
        self.assertEqual(target, [10.] * 7 + current[7:])
        self.assertEqual(current, list(range(14)))
        self.assertEqual(arm, "left")

    def test_right_restore_keeps_live_left(self):
        current = list(range(14))
        target, _ = saved_pose_target(pose("right"), current)
        self.assertEqual(target, current[:7] + [10.] * 7)

    def test_wrong_arm_and_bad_angles_fail(self):
        with self.assertRaises(ValueError):
            saved_pose_target(pose("left"), [0.] * 14, "right")
        for bad in ([1.] * 6, [float("nan")] * 7):
            p = pose("left")
            p["arm7_deg"] = bad
            with self.assertRaises(ValueError):
                saved_pose_target(p, [0.] * 14)

    def test_select_one_arm_from_a_both_arm_capture(self):
        p = pose("both")
        p["arm14_deg"] = [20.] * 7 + [30.] * 7
        target, _ = saved_pose_target(p, [1.] * 14, "right")
        self.assertEqual(target, [1.] * 7 + [30.] * 7)

    def test_preview_never_waits_for_publisher_or_calls_a_mode_service(self):
        controller = object.__new__(KuavoArmController)
        controller.robot = "5"
        controller.rospy = types.SimpleNamespace(get_param=lambda key, default: 55)
        controller.read_current_state = lambda **unused: {"arm14_deg": list(range(14))}
        controller._run_joint_target = lambda **kwargs: kwargs
        result = controller.move_to_saved_pose(pose("right"), execute=False)
        self.assertEqual(result["target_deg"], list(range(7)) + [10.] * 7)
        self.assertFalse(result["execute"])
        self.assertFalse(result["extra"]["finger_commands_sent"])

    def test_wrong_robot_version_is_rejected_before_read_or_publish(self):
        controller = object.__new__(KuavoArmController)
        controller.robot = "5"
        controller.rospy = types.SimpleNamespace(get_param=lambda key, default: 52)
        with self.assertRaises(ValueError):
            controller.move_to_saved_pose(pose("left"), execute=True)

    def test_execute_prepares_other_arm_from_feedback_after_connections_are_ready(self):
        controller = object.__new__(KuavoArmController)
        controller.robot, controller.publisher_timeout = "5", 1.
        connections = iter([1, 2])
        waited = []
        controller.publisher = types.SimpleNamespace(get_num_connections=lambda: next(connections))
        graph = [[], [["/kuavo_arm_target_poses", ["/bridge", "/recorder"]]],
                 [["/arm_traj_change_mode", ["/bridge"]]]]
        controller.rospy = types.SimpleNamespace(
            get_param=lambda key, default: 55,
            get_master=lambda: types.SimpleNamespace(getSystemState=lambda: (1, "ok", graph)),
            sleep=lambda seconds: waited.append(True))
        def read(**unused):
            self.assertTrue(waited)
            return {"arm14_deg": [5.] * 7 + [42.] * 7}
        controller.read_current_state = read
        controller._run_joint_target = lambda **kwargs: kwargs
        result = controller.move_to_saved_pose(pose("left"), execute=True)
        self.assertEqual(result["target_deg"], [10.] * 7 + [42.] * 7)

    def test_save_archives_old_capture_and_lists_only_named_poses(self):
        state = {"schema_version": 1, "robot": "5", "robot_version": 55,
                 "feedback_stamp": 123., "source": "live", "arm14_deg": list(range(14))}
        with tempfile.TemporaryDirectory() as directory:
            first, path = save_arm_pose(directory, "left_parcel", "left", state)
            state["arm14_deg"][0] = 42.
            save_arm_pose(directory, "left_parcel", "left", state)
            self.assertEqual(first["arm7_deg"][0], 0.)
            self.assertEqual(json.loads(path.read_text())["arm7_deg"][0], 42.)
            (Path(directory) / "unrelated.json").write_text("[]")
            self.assertEqual(len(list_saved_poses(directory)), 1)
            self.assertGreaterEqual(len(list((Path(directory) / ".versions").glob("*.json"))), 3)
        with self.assertRaises(ValueError):
            saved_pose_path("/tmp", "../not_a_pose")

    def test_save_dispatches_read_only_state_and_goto_does_not_execute_by_default(self):
        args = build_parser().parse_args(["save-pose", "example", "--arm", "left"])
        self.assertEqual(controller_arguments(args)[4], "state")
        args = build_parser().parse_args(["goto-pose", "example"])
        args.saved_pose = pose("left")
        self.assertNotIn("--execute", controller_arguments(args))
        args.execute = True
        self.assertIn("--execute", controller_arguments(args))

    def test_json_parser_tolerates_ros_log_prefix(self):
        value = {"schema_version": 1, "arm14_deg": list(range(14))}
        raw = "[INFO] ROS initialized\n" + json.dumps(value, indent=2) + "\n"
        self.assertEqual(parse_remote_json(raw), value)


if __name__ == "__main__":
    unittest.main()
