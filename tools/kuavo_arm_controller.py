#!/usr/bin/env python3
"""Stateful Kuavo-5 arm pose and Cartesian-step controller.

The public methods default to planning only.  Real motion requires
``execute=True``.  Every plan starts from fresh ``/sensors_data_raw`` feedback;
after a real motion, fresh feedback is written to ``latest_state.json`` and an
append-only JSONL history.  A saved state is never reused as an IK seed when
live feedback is unavailable.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import tempfile
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path


ARM14_LAYOUT = [
    "left_j1", "left_j2", "left_j3", "left_j4", "left_j5", "left_j6", "left_j7",
    "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6", "right_j7",
]

# Angles are degrees because this is also the unit expected by
# /kuavo_arm_target_poses.  FK/IK and feedback are converted to radians.
STANDARD_POSES_DEG = {
    "arms_down": {
        "description": "双臂完全放下；与项目 arm_init 的 14 维零位一致",
        "values": [0.0] * 14,
    },
    "ready": {
        "description": "双臂自然前摆并略微屈肘的准备姿态",
        "values": [
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
        ],
    },
    "arms_horizontal_palms_down": {
        "description": "双臂向两侧平举，手心向下",
        "values": [
            0.0, 15.0, 70.0, -70.0, 90.0, 0.0, -20.0,
            0.0, -15.0, -70.0, -70.0, -90.0, 0.0, 20.0,
        ],
    },
    "elbows_90_palms_down": {
        "description": "双臂向前、肘部约 90 度、手心向下",
        "values": [
            0.0, 0.0, 0.0, -90.0, -84.0, 0.0, 0.0,
            0.0, 0.0, 0.0, -90.0, 84.0, 0.0, 0.0,
        ],
    },
}

POSE_ALIASES = {
    "initial": "arms_down",
    "home": "arms_down",
    "down": "arms_down",
    "horizontal": "arms_horizontal_palms_down",
    "t_pose": "arms_horizontal_palms_down",
    "elbows_90": "elbows_90_palms_down",
}

DIRECTIONS = {
    "forward": (1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
}

ARM_SLICES = {"4pro": (12, 26), "5": (13, 27), "5w": (4, 18)}
DEFAULT_REAL_STATE_DIR = "outputs/robot_control/kuavo5_arm"
DEFAULT_MUJOCO_STATE_DIR = "outputs/robot_control/kuavo5_mujoco"


def now_iso():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def resolve_pose_name(name):
    resolved = POSE_ALIASES.get(name, name)
    if resolved not in STANDARD_POSES_DEG:
        choices = ", ".join(sorted(STANDARD_POSES_DEG))
        raise ValueError(f"unknown pose {name!r}; available poses: {choices}")
    return resolved


def get_standard_pose_deg(name):
    """Return a new 14-element list, so callers cannot mutate the preset."""
    return list(STANDARD_POSES_DEG[resolve_pose_name(name)]["values"])


def extract_arm14(joint_q, robot="5"):
    begin, end = ARM_SLICES[robot]
    result = list(joint_q[begin:end])
    if len(result) != 14:
        raise ValueError(
            f"expected 14 joints from joint_q[{begin}:{end}], got {len(result)}"
        )
    return result


def translated_xyz(xyz, direction, step_m):
    if direction not in DIRECTIONS:
        raise ValueError(f"unknown direction {direction!r}")
    if not 0.0 < step_m <= 0.05:
        raise ValueError("step_m must be in (0, 0.05]")
    return [
        float(value) + step_m * axis
        for value, axis in zip(xyz, DIRECTIONS[direction])
    ]


def angular_distance_deg(a_rad, b_rad):
    return max(abs(math.degrees(float(a) - float(b))) for a, b in zip(a_rad, b_rad))


def position_error_mm(a, b):
    return 1000.0 * math.sqrt(
        sum((float(x) - float(y)) ** 2 for x, y in zip(a, b))
    )


def saved_pose_target(pose, current_deg, arm=None):
    """Merge one saved arm into LIVE 14D feedback, never its old other arm."""
    if pose.get("pose_type") != "kuavo_saved_arm_pose":
        raise ValueError("not a kuavo_saved_arm_pose file")
    saved_arm = pose.get("arm")
    arm = arm or saved_arm
    if saved_arm not in ("left", "right", "both") or arm not in ("left", "right", "both"):
        raise ValueError("invalid saved arm selection")
    if saved_arm != "both" and arm != saved_arm:
        raise ValueError("saved pose belongs to %s; cannot restore %s" % (saved_arm, arm))
    current = [float(v) for v in current_deg]
    if len(current) != 14 or not all(math.isfinite(v) for v in current):
        raise ValueError("fresh current state must have 14 finite joint angles")
    key, count = ("arm14_deg", 14) if saved_arm == "both" else ("arm7_deg", 7)
    saved = [float(v) for v in pose.get(key, [])]
    if len(saved) != count or not all(math.isfinite(v) for v in saved):
        raise ValueError("saved pose must have %s finite degree values" % count)
    if arm in ("left", "both"):
        current[:7] = saved[:7]
    if arm in ("right", "both"):
        current[7:] = saved[7:] if saved_arm == "both" else saved
    return current, arm


class StateStore:
    """Atomic latest-state file plus append-only action history."""

    def __init__(self, directory):
        self.directory = Path(directory).expanduser().resolve()
        self.latest_path = self.directory / "latest_state.json"
        self.history_path = self.directory / "history.jsonl"

    def _ensure_directory(self):
        self.directory.mkdir(parents=True, exist_ok=True)

    def write_latest(self, state):
        self._ensure_directory()
        fd, temporary = tempfile.mkstemp(
            prefix="latest_state.", suffix=".tmp", dir=str(self.directory)
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(state, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.latest_path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def append_history(self, record):
        self._ensure_directory()
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        # A single append is used so normal one-controller operation cannot
        # leave a partial record between separate writes.
        with self.history_path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")
            stream.flush()
            os.fsync(stream.fileno())


class KuavoArmController:
    """Generate standard poses and state-relative Cartesian movements."""

    def __init__(
        self,
        robot="5",
        state_dir="outputs/robot_control/kuavo5_arm",
        feedback_timeout=5.0,
        publisher_timeout=30.0,
        max_position_error_mm=5.0,
        max_joint_speed_deg_s=25.0,
        init_ros=True,
    ):
        if robot not in ARM_SLICES:
            raise ValueError(f"unsupported robot type: {robot}")
        self.robot = robot
        self.feedback_timeout = float(feedback_timeout)
        self.publisher_timeout = float(publisher_timeout)
        self.max_position_error_mm = float(max_position_error_mm)
        self.max_joint_speed_deg_s = float(max_joint_speed_deg_s)
        self.store = StateStore(state_dir)

        self.rospy = None
        self.messages = {}
        self.services = {}
        self.publisher = None
        if init_ros:
            self._init_ros()

    def _init_ros(self):
        import rospy
        from kuavo_msgs.msg import armTargetPoses, ikSolveParam, sensorsData, twoArmHandPoseCmd
        from kuavo_msgs.srv import changeArmCtrlMode, fkSrv, twoArmHandPoseCmdSrv
        from std_msgs.msg import Bool

        self.rospy = rospy
        self.messages = {
            "armTargetPoses": armTargetPoses,
            "Bool": Bool,
            "ikSolveParam": ikSolveParam,
            "sensorsData": sensorsData,
            "twoArmHandPoseCmd": twoArmHandPoseCmd,
        }
        self.services = {
            "changeArmCtrlMode": changeArmCtrlMode,
            "fkSrv": fkSrv,
            "twoArmHandPoseCmdSrv": twoArmHandPoseCmdSrv,
        }
        if not rospy.core.is_initialized():
            rospy.init_node("kuavo_arm_controller", anonymous=True)
        self.publisher = rospy.Publisher(
            "/kuavo_arm_target_poses", armTargetPoses, queue_size=1, latch=False
        )

    def _require_ros(self):
        if self.rospy is None:
            raise RuntimeError("ROS is not initialized")

    def diagnose(self, wait_s=30.0):
        """Inspect the arm-control ROS graph without changing control mode."""
        self._require_ros()
        started = time.monotonic()
        deadline = started + float(wait_s)
        while self.publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            self.rospy.sleep(0.05)
        connection_wait_s = time.monotonic() - started

        system_state = self.rospy.get_master().getSystemState()
        if (
            len(system_state) == 3
            and isinstance(system_state[0], int)
            and isinstance(system_state[2], (list, tuple))
        ):
            code, status_message, system_state = system_state
            if code != 1:
                raise RuntimeError(
                    f"ROS master getSystemState failed: {status_message}"
                )
        publishers, subscribers, services = system_state

        def nodes_for(entries, name):
            for resource_name, node_names in entries:
                if resource_name == name:
                    return list(node_names)
            return []

        return {
            "schema_version": 1,
            "recorded_at": now_iso(),
            "read_only": True,
            "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
            "ros_ip": os.environ.get("ROS_IP"),
            "ros_hostname": os.environ.get("ROS_HOSTNAME"),
            "node_name": self.rospy.get_name(),
            "node_uri": self.rospy.get_node_uri(),
            "target_topic": "/kuavo_arm_target_poses",
            "target_topic_master_subscribers": nodes_for(
                subscribers, "/kuavo_arm_target_poses"
            ),
            "target_topic_tcpros_connections": self.publisher.get_num_connections(),
            "connection_wait_s": connection_wait_s,
            "mode_service_nodes": nodes_for(services, "/arm_traj_change_mode"),
            "feedback_topic_publishers": nodes_for(
                publishers, "/sensors_data_raw"
            ),
            "ready": bool(
                self.publisher.get_num_connections() > 0
                and nodes_for(services, "/arm_traj_change_mode")
                and nodes_for(publishers, "/sensors_data_raw")
            ),
        }

    def _call_fk(self, arm14_rad):
        self._require_ros()
        self.rospy.wait_for_service("/ik/fk_srv", timeout=self.feedback_timeout)
        service = self.rospy.ServiceProxy("/ik/fk_srv", self.services["fkSrv"])
        result = service(list(arm14_rad))
        if not result.success:
            raise RuntimeError("FK service failed")
        return result.hand_poses

    @staticmethod
    def _pose_dict(pose):
        return {
            "position_m": [float(value) for value in pose.pos_xyz],
            "quaternion_xyzw": [float(value) for value in pose.quat_xyzw],
        }

    def read_current_state(self, reason="read", save=True):
        """Read fresh feedback. Saved state is never used as a fallback."""
        self._require_ros()
        message = self.rospy.wait_for_message(
            "/sensors_data_raw",
            self.messages["sensorsData"],
            timeout=self.feedback_timeout,
        )
        arm14_rad = extract_arm14(message.joint_data.joint_q, self.robot)
        state = {
            "schema_version": 1,
            "recorded_at": now_iso(),
            "reason": reason,
            "source": "/sensors_data_raw (fresh feedback)",
            "robot": self.robot,
            "robot_version": self.rospy.get_param("/robot_version", None),
            "urdf_file": self.rospy.get_param("/urdfFile", None),
            "reference_frame": "base_link",
            "feedback_stamp": float(message.header.stamp.to_sec()),
            "layout": ARM14_LAYOUT,
            "arm14_rad": arm14_rad,
            "arm14_deg": [math.degrees(value) for value in arm14_rad],
        }
        try:
            hand_poses = self._call_fk(arm14_rad)
            state["left_eef"] = self._pose_dict(hand_poses.left_pose)
            state["right_eef"] = self._pose_dict(hand_poses.right_pose)
        except Exception as error:
            # Joint feedback is still the authoritative current state even if
            # the optional FK service is temporarily unavailable.
            state["left_eef"] = None
            state["right_eef"] = None
            state["fk_error"] = f"{type(error).__name__}: {error}"
        if save:
            self.store.write_latest(state)
        return state

    def _duration_for_target(self, current_rad, target_rad, requested_s):
        largest_change = angular_distance_deg(current_rad, target_rad)
        minimum = largest_change / self.max_joint_speed_deg_s
        return max(float(requested_s), minimum, 0.5), largest_change

    def _arm_is_executing(self):
        try:
            message = self.rospy.wait_for_message(
                "/humanoid/mpc/is_arm_executing",
                self.messages["Bool"],
                timeout=1.0,
            )
            return bool(message.data)
        except Exception:
            return None

    def _ensure_external_control(self):
        self.rospy.wait_for_service("/arm_traj_change_mode", timeout=self.feedback_timeout)
        service = self.rospy.ServiceProxy(
            "/arm_traj_change_mode", self.services["changeArmCtrlMode"]
        )
        result = service(control_mode=2)
        if not result.result:
            raise RuntimeError(f"failed to enter arm control mode 2: {result.message}")

    def _publish_target(self, target_deg, duration_s):
        if self._arm_is_executing() is True:
            raise RuntimeError("another arm trajectory is already executing")
        # On the current Kuavo-5 ROS graph, a newly registered publisher can
        # take about 18 seconds to receive TCPROS connections even though the
        # subscribers already appear in the master's system state.
        deadline = time.monotonic() + self.publisher_timeout
        while self.publisher.get_num_connections() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "/kuavo_arm_target_poses has no TCPROS subscriber after "
                    f"{self.publisher_timeout:.1f}s; run the read-only doctor command"
                )
            self.rospy.sleep(0.05)
        # Change the controller mode only after the outbound topic connection
        # is ready, so a network failure cannot leave the arm in mode 2.
        self._ensure_external_control()
        message = self.messages["armTargetPoses"]()
        message.times = [float(duration_s)]
        message.values = [float(value) for value in target_deg]
        message.frame = 0
        self.publisher.publish(message)

    def _wait_for_motion(self, duration_s):
        deadline = time.monotonic() + duration_s + 5.0
        saw_running = False
        while time.monotonic() < deadline:
            status = self._arm_is_executing()
            if status is True:
                saw_running = True
            if status is False and (saw_running or time.monotonic() >= deadline - 4.5):
                break
            self.rospy.sleep(0.05)
        self.rospy.sleep(0.35)
        return {"saw_running": saw_running, "final_executing": self._arm_is_executing()}

    def _run_joint_target(
        self,
        kind,
        name,
        target_deg,
        duration_s,
        execute,
        extra=None,
        pre_state=None,
    ):
        if len(target_deg) != 14:
            raise ValueError(f"target must contain 14 joints, got {len(target_deg)}")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        action_id = str(uuid.uuid4())
        pre = pre_state or self.read_current_state(
            reason=f"before:{kind}:{name}", save=True
        )
        target_rad = [math.radians(float(value)) for value in target_deg]
        actual_duration, largest_change = self._duration_for_target(
            pre["arm14_rad"], target_rad, duration_s
        )
        record = {
            "schema_version": 1,
            "action_id": action_id,
            "created_at": now_iso(),
            "kind": kind,
            "name": name,
            "execute_requested": bool(execute),
            "status": "planned",
            "pre_state": pre,
            "target_arm14_deg": list(target_deg),
            "target_arm14_rad": target_rad,
            "requested_duration_s": float(duration_s),
            "actual_duration_s": actual_duration,
            "largest_joint_change_deg": largest_change,
        }
        if extra:
            record.update(copy.deepcopy(extra))

        if not execute:
            self.store.append_history(record)
            return record

        try:
            self._publish_target(target_deg, actual_duration)
            record["motion_monitor"] = self._wait_for_motion(actual_duration)
            if record["motion_monitor"]["final_executing"] is True:
                raise RuntimeError("arm trajectory did not finish before timeout")
            post = self.read_current_state(reason=f"after:{kind}:{name}", save=True)
            record["post_state"] = post
            record["max_target_error_deg"] = angular_distance_deg(
                post["arm14_rad"], target_rad
            )
            record["target_reached"] = record["max_target_error_deg"] <= 3.0
            if kind == "cartesian_step" and extra and "ik" in extra:
                eef_key = "left_eef" if extra["arm"] == "left" else "right_eef"
                if post[eef_key] is None:
                    record["post_position_error_mm"] = None
                    record["target_reached"] = False
                else:
                    record["post_position_error_mm"] = position_error_mm(
                        post[eef_key]["position_m"], extra["ik"]["requested_xyz_m"]
                    )
                    record["target_reached"] = bool(
                        record["target_reached"]
                        and record["post_position_error_mm"]
                        <= self.max_position_error_mm + 2.0
                    )
            record["status"] = (
                "executed" if record["target_reached"] else "executed_with_error"
            )
            post["last_action_id"] = action_id
            post["last_action_status"] = record["status"]
            self.store.write_latest(post)
        except (Exception, KeyboardInterrupt) as error:
            record["status"] = "failed"
            record["error"] = f"{type(error).__name__}: {error}"
            try:
                record["post_state"] = self.read_current_state(
                    reason=f"after_failed:{kind}:{name}", save=True
                )
                record["post_state"]["last_action_id"] = action_id
                record["post_state"]["last_action_status"] = "failed"
                self.store.write_latest(record["post_state"])
            except Exception as feedback_error:
                record["post_feedback_error"] = (
                    f"{type(feedback_error).__name__}: {feedback_error}"
                )
            self.store.append_history(record)
            raise
        self.store.append_history(record)
        return record

    def move_to_standard_pose(
        self, name, duration_s=3.0, execute=False, arm="both"
    ):
        resolved = resolve_pose_name(name)
        if arm not in ("left", "right", "both"):
            raise ValueError("arm must be 'left', 'right', or 'both'")
        preset = get_standard_pose_deg(resolved)
        pre = None
        if arm != "both":
            # Preserve the other arm using fresh measured joint feedback.
            pre = self.read_current_state(
                reason=f"before:standard_pose:{arm}:{resolved}", save=True
            )
            target = list(pre["arm14_deg"])
            if arm == "left":
                target[:7] = preset[:7]
            else:
                target[7:] = preset[7:]
        else:
            target = preset
        return self._run_joint_target(
            kind="standard_pose",
            name=f"{arm}:{resolved}",
            target_deg=target,
            duration_s=duration_s,
            execute=execute,
            extra={
                "arm": arm,
                "pose_name": resolved,
                "description": STANDARD_POSES_DEG[resolved]["description"],
            },
            pre_state=pre,
        )

    def move_to_joint_target(self, values_deg, duration_s=3.0, execute=False):
        """Plan or execute an explicit [left J1..J7, right J1..J7] target."""
        values = [float(value) for value in values_deg]
        if len(values) != 14:
            raise ValueError(f"values_deg must contain 14 joints, got {len(values)}")
        return self._run_joint_target(
            kind="joint_target",
            name="explicit_arm14",
            target_deg=values,
            duration_s=duration_s,
            execute=execute,
            extra={"layout": ARM14_LAYOUT, "input_unit": "degree"},
        )

    def move_to_saved_pose(self, pose, arm=None, duration_s=6.0, execute=False):
        """Restore recorded joint angles; preserve the other arm's fresh state.

        This uses the existing joint interpolation API, not obstacle planning.
        Finger commands are deliberately separate from saved arm poses.
        """
        if pose.get("robot") != self.robot:
            raise ValueError("saved pose robot does not match the controller")
        saved_version = pose.get("robot_version")
        actual_version = self.rospy.get_param("/robot_version", None)
        if saved_version is not None and str(saved_version) != str(actual_version):
            raise ValueError("saved pose robot_version differs from the current robot")
        # Validate the file before waiting for connections or changing mode.
        saved_pose_target(pose, [0.] * 14, arm)
        if execute:
            system = self.rospy.get_master().getSystemState()
            if len(system) == 3 and isinstance(system[0], int):
                if system[0] != 1:
                    raise RuntimeError("could not read ROS graph")
                system = system[2]
            subscribers = dict(system[1]).get("/kuavo_arm_target_poses", [])
            owners = dict(system[2]).get("/arm_traj_change_mode", [])
            if not set(subscribers).intersection(owners):
                raise RuntimeError("arm trajectory service has no target topic subscriber")
            deadline = time.monotonic() + self.publisher_timeout
            while self.publisher.get_num_connections() < len(subscribers):
                if time.monotonic() >= deadline:
                    raise RuntimeError("saved-pose publisher TCPROS connections not ready")
                self.rospy.sleep(.05)
        # Read AFTER connection startup, which can otherwise age the other arm.
        pre = self.read_current_state(reason="before:saved_pose:" + pose["name"], save=True)
        target, selected = saved_pose_target(pose, pre["arm14_deg"], arm)
        return self._run_joint_target(
            kind="saved_pose", name=pose["name"], target_deg=target,
            duration_s=duration_s, execute=execute, pre_state=pre,
            extra={"arm": selected, "description": pose.get("description", ""),
                   "saved_reference_frame": pose.get("reference_frame"),
                   "saved_eef": pose.get("eef"),
                   "input_unit": "degree", "layout": ARM14_LAYOUT,
                   "other_arm_source": "fresh measured feedback",
                   "finger_commands_sent": False},
        )

    # Named wrappers make common poses convenient from Python code.
    def arms_down(self, duration_s=3.0, execute=False, arm="both"):
        return self.move_to_standard_pose("arms_down", duration_s, execute, arm)

    def ready(self, duration_s=3.0, execute=False, arm="both"):
        return self.move_to_standard_pose("ready", duration_s, execute, arm)

    def arms_horizontal_palms_down(
        self, duration_s=4.0, execute=False, arm="both"
    ):
        return self.move_to_standard_pose(
            "arms_horizontal_palms_down", duration_s, execute, arm
        )

    def elbows_90_palms_down(
        self, duration_s=4.0, execute=False, arm="both"
    ):
        return self.move_to_standard_pose(
            "elbows_90_palms_down", duration_s, execute, arm
        )

    def left_arm_to_pose(self, name, duration_s=3.0, execute=False):
        return self.move_to_standard_pose(
            name, duration_s=duration_s, execute=execute, arm="left"
        )

    def right_arm_to_pose(self, name, duration_s=3.0, execute=False):
        return self.move_to_standard_pose(
            name, duration_s=duration_s, execute=execute, arm="right"
        )

    @staticmethod
    def _copy_hand_pose(target, source):
        target.pos_xyz = list(source.pos_xyz)
        target.quat_xyzw = list(source.quat_xyzw)
        target.elbow_pos_xyz = [0.0, 0.0, 0.0]

    def _solve_cartesian_step(self, pre, arm, direction, step_m):
        current_q = pre["arm14_rad"]
        hand_poses = self._call_fk(current_q)
        command = self.messages["twoArmHandPoseCmd"]()
        command.use_custom_ik_param = True
        command.joint_angles_as_q0 = True
        command.ik_param = self.messages["ikSolveParam"](
            major_optimality_tol=1e-3,
            major_feasibility_tol=1e-3,
            minor_feasibility_tol=1e-3,
            major_iterations_limit=200,
            oritation_constraint_tol=1e-2,
            pos_constraint_tol=self.max_position_error_mm / 1000.0,
            pos_cost_weight=1.0,
            constraint_mode=0,
            elbow_cost_scale=0.0,
        )
        command.hand_poses.left_pose.joint_angles = current_q[:7]
        command.hand_poses.right_pose.joint_angles = current_q[7:]
        self._copy_hand_pose(command.hand_poses.left_pose, hand_poses.left_pose)
        self._copy_hand_pose(command.hand_poses.right_pose, hand_poses.right_pose)

        selected = (
            command.hand_poses.left_pose
            if arm == "left"
            else command.hand_poses.right_pose
        )
        current_xyz = list(selected.pos_xyz)
        requested_xyz = translated_xyz(current_xyz, direction, step_m)
        selected.pos_xyz = requested_xyz

        self.rospy.wait_for_service(
            "/ik/two_arm_hand_pose_cmd_srv", timeout=self.feedback_timeout
        )
        service = self.rospy.ServiceProxy(
            "/ik/two_arm_hand_pose_cmd_srv",
            self.services["twoArmHandPoseCmdSrv"],
        )
        result = service(command)
        achieved_xyz = list(
            result.hand_poses.left_pose.pos_xyz
            if arm == "left"
            else result.hand_poses.right_pose.pos_xyz
        )
        error_mm = position_error_mm(requested_xyz, achieved_xyz)
        result_q = list(result.q_arm)
        accepted = bool(
            result.success
            and len(result_q) == 14
            and error_mm <= self.max_position_error_mm
        )
        return {
            "ik_service_success": bool(result.success),
            "accepted": accepted,
            "current_xyz_m": current_xyz,
            "requested_xyz_m": requested_xyz,
            "achieved_xyz_m": achieved_xyz,
            "position_error_mm": error_mm,
            "target_arm14_rad": result_q,
            "target_arm14_deg": [math.degrees(value) for value in result_q],
        }

    def move_relative(
        self,
        arm,
        direction,
        step_m=0.02,
        duration_s=1.0,
        execute=False,
    ):
        if arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")
        # This fresh feedback is the sole starting point for every IK request.
        pre = self.read_current_state(
            reason=f"before:cartesian_step:{arm}:{direction}", save=True
        )
        solution = self._solve_cartesian_step(pre, arm, direction, step_m)
        if not solution["accepted"]:
            rejected = {
                "schema_version": 1,
                "action_id": str(uuid.uuid4()),
                "created_at": now_iso(),
                "kind": "cartesian_step",
                "name": f"{arm}:{direction}",
                "execute_requested": bool(execute),
                "status": "rejected",
                "pre_state": pre,
                "arm": arm,
                "direction": direction,
                "step_m": step_m,
                "ik": solution,
            }
            self.store.append_history(rejected)
            return rejected

        # Re-read immediately before handing the target to the execution path.
        # Re-solve if the robot changed materially while IK was being computed.
        latest = self.read_current_state(reason="cartesian_step:pre_publish", save=True)
        if angular_distance_deg(latest["arm14_rad"], pre["arm14_rad"]) > 1.0:
            pre = latest
            solution = self._solve_cartesian_step(pre, arm, direction, step_m)
            if not solution["accepted"]:
                rejected = {
                    "schema_version": 1,
                    "action_id": str(uuid.uuid4()),
                    "created_at": now_iso(),
                    "kind": "cartesian_step",
                    "name": f"{arm}:{direction}",
                    "execute_requested": bool(execute),
                    "status": "rejected_after_state_change",
                    "pre_state": pre,
                    "arm": arm,
                    "direction": direction,
                    "step_m": step_m,
                    "ik": solution,
                }
                self.store.append_history(rejected)
                return rejected

        return self._run_joint_target(
            kind="cartesian_step",
            name=f"{arm}:{direction}",
            target_deg=solution["target_arm14_deg"],
            duration_s=duration_s,
            execute=execute,
            extra={
                "arm": arm,
                "direction": direction,
                "step_m": step_m,
                "reference_frame": "waist_yaw_link",
                "ik": solution,
            },
            pre_state=pre,
        )


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=tuple(ARM_SLICES), default="5")
    parser.add_argument(
        "--backend", choices=("real", "mujoco"), default="real",
        help="real uses ROS; mujoco uses the local official Kuavo-5.2 model",
    )
    parser.add_argument("--scene", help="MuJoCo scene XML (mujoco backend only)")
    parser.add_argument(
        "--state-dir", default=DEFAULT_REAL_STATE_DIR
    )
    parser.add_argument(
        "--reset-sim",
        action="store_true",
        help="ignore persisted MuJoCo qpos and start from the model home pose",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list-poses")
    subparsers.add_parser("state")
    doctor = subparsers.add_parser(
        "doctor", help="read-only ROS graph and publisher connection check"
    )
    doctor.add_argument("--wait-s", type=float, default=30.0)

    pose = subparsers.add_parser("pose")
    pose.add_argument("name", choices=sorted(set(STANDARD_POSES_DEG) | set(POSE_ALIASES)))
    pose.add_argument("--arm", choices=("left", "right", "both"), default="both")
    pose.add_argument("--duration-s", type=float, default=3.0)
    pose.add_argument("--execute", action="store_true")

    move = subparsers.add_parser("move")
    move.add_argument("--arm", choices=("left", "right"), required=True)
    move.add_argument("--direction", choices=tuple(DIRECTIONS), required=True)
    move.add_argument("--step-m", type=float, default=0.02)
    move.add_argument("--duration-s", type=float, default=1.0)
    move.add_argument("--execute", action="store_true")

    joints = subparsers.add_parser(
        "joints", help="plan or execute an explicit 14-joint target in degrees"
    )
    joints.add_argument(
        "--values",
        nargs=14,
        type=float,
        required=True,
        metavar="DEG",
        help="left J1..J7 followed by right J1..J7",
    )
    joints.add_argument("--duration-s", type=float, default=3.0)
    joints.add_argument("--execute", action="store_true")

    saved = subparsers.add_parser("goto-pose", help="restore a user-saved arm pose")
    saved.add_argument("--pose-json", required=True, help="JSON supplied by SSH launcher")
    saved.add_argument("--arm", choices=("left", "right", "both"))
    saved.add_argument("--duration-s", type=float, default=6.)
    saved.add_argument("--max-joint-speed-deg-s", type=float, default=5.)
    saved.add_argument("--execute", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    if args.command == "list-poses":
        print(json.dumps(STANDARD_POSES_DEG, ensure_ascii=False, indent=2))
        return

    if args.command == "goto-pose":
        if args.backend != "real":
            raise SystemExit("saved real-arm poses require the real backend")
        if not math.isfinite(args.max_joint_speed_deg_s) or not 0 < args.max_joint_speed_deg_s <= 25:
            raise SystemExit("max joint speed must be in (0,25] degrees/s")
        if not math.isfinite(args.duration_s) or args.duration_s <= 0:
            raise SystemExit("duration must be positive and finite")
        pose_data = json.loads(args.pose_json)
        saved_pose_target(pose_data, [0.] * 14, args.arm)

    if args.backend == "mujoco":
        if args.robot != "5":
            raise SystemExit("the MuJoCo backend currently targets robot=5 only")
        from kuavo_mujoco_controller import DEFAULT_SCENE, MuJoCoKuavoArmController

        state_dir = args.state_dir
        if state_dir == DEFAULT_REAL_STATE_DIR:
            state_dir = DEFAULT_MUJOCO_STATE_DIR
        controller = MuJoCoKuavoArmController(
            scene=args.scene or DEFAULT_SCENE,
            state_dir=state_dir,
            resume=not args.reset_sim,
        )
    else:
        controller = KuavoArmController(
            robot=args.robot, state_dir=args.state_dir,
            max_joint_speed_deg_s=args.max_joint_speed_deg_s
            if args.command == "goto-pose" else 25.,
        )
    if args.command == "state":
        result = controller.read_current_state(reason="manual_state_query", save=True)
    elif args.command == "doctor":
        result = controller.diagnose(wait_s=args.wait_s)
    elif args.command == "pose":
        result = controller.move_to_standard_pose(
            args.name,
            duration_s=args.duration_s,
            execute=args.execute,
            arm=args.arm,
        )
    elif args.command == "joints":
        result = controller.move_to_joint_target(
            args.values,
            duration_s=args.duration_s,
            execute=args.execute,
        )
    elif args.command == "goto-pose":
        result = controller.move_to_saved_pose(
            pose_data, arm=args.arm, duration_s=args.duration_s, execute=args.execute)
    else:
        result = controller.move_relative(
            arm=args.arm,
            direction=args.direction,
            step_m=args.step_m,
            duration_s=args.duration_s,
            execute=args.execute,
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
