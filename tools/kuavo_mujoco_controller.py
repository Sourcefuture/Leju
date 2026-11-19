#!/usr/bin/env python3
"""MuJoCo backend for the stateful Kuavo arm controller."""

from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/kuavo-mujoco-cache")

import mujoco
import numpy as np

try:
    from kuavo_arm_controller import (
        ARM14_LAYOUT,
        KuavoArmController,
        now_iso,
        position_error_mm,
        translated_xyz,
    )
except ModuleNotFoundError:  # imported as tools.kuavo_mujoco_controller
    from tools.kuavo_arm_controller import (
        ARM14_LAYOUT,
        KuavoArmController,
        now_iso,
        position_error_mm,
        translated_xyz,
    )


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = (
    ROOT
    / ".sim/kuavo-ros-opensource/src/kuavo_assets/models/biped_s52/xml/scene.xml"
)


class MuJoCoKuavoArmController(KuavoArmController):
    """Run the existing stateful control flow against the Kuavo-5.2 MJCF."""

    def __init__(
        self,
        scene=DEFAULT_SCENE,
        state_dir="outputs/robot_control/kuavo5_mujoco",
        max_position_error_mm=5.0,
        resume=True,
    ):
        super().__init__(
            robot="5",
            state_dir=state_dir,
            max_position_error_mm=max_position_error_mm,
            init_ros=False,
        )
        self.scene_path = Path(scene).expanduser().resolve()
        if not self.scene_path.is_file():
            raise FileNotFoundError(f"Kuavo-5 MuJoCo scene not found: {self.scene_path}")
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[2] = 0.98
            self.data.qpos[3] = 1.0
        mujoco.mj_forward(self.model, self.data)

        self.joint_names = [f"zarm_l{i}_joint" for i in range(1, 8)]
        self.joint_names += [f"zarm_r{i}_joint" for i in range(1, 8)]
        self.joint_ids = [self._joint_id(name) for name in self.joint_names]
        self.qpos_addresses = [int(self.model.jnt_qposadr[j]) for j in self.joint_ids]
        self.dof_addresses = [int(self.model.jnt_dofadr[j]) for j in self.joint_ids]
        self.body_ids = {
            "left": self._body_id("zarm_l7_link"),
            "right": self._body_id("zarm_r7_link"),
        }
        # kuavo_v52/kuavo.json specifies hand_ref_length=0.193 m.
        self.eef_local_offset = np.array([0.0, 0.0, -0.193], dtype=np.float64)
        self.resumed_from_state = False
        if resume and self.store.latest_path.is_file():
            try:
                saved = json.loads(self.store.latest_path.read_text(encoding="utf-8"))
                saved_q = saved.get("arm14_rad")
                if (
                    saved.get("robot") == "5"
                    and str(saved.get("source", "")).startswith("MuJoCo Kuavo-5.2")
                    and isinstance(saved_q, list)
                    and len(saved_q) == 14
                ):
                    self.data.qpos[self.qpos_addresses] = np.asarray(saved_q, dtype=float)
                    self.data.qvel[:] = 0.0
                    mujoco.mj_forward(self.model, self.data)
                    self.resumed_from_state = True
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                # A malformed or non-MuJoCo state is never used as feedback.
                self.resumed_from_state = False
        self.trajectory = []
        self._active_label = "resumed" if self.resumed_from_state else "initial"
        self._active_meta = {}
        self._append_snapshot("initial")

    def _joint_id(self, name):
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if result < 0:
            raise RuntimeError(f"missing Kuavo-5 joint {name}")
        return result

    def _body_id(self, name):
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if result < 0:
            raise RuntimeError(f"missing Kuavo-5 body {name}")
        return result

    def _arm14_rad(self):
        return np.asarray(self.data.qpos[self.qpos_addresses], dtype=float).copy()

    def _eef(self, arm):
        body_id = self.body_ids[arm]
        rotation = np.asarray(self.data.xmat[body_id], dtype=float).reshape(3, 3)
        position = np.asarray(self.data.xpos[body_id], dtype=float) + rotation @ self.eef_local_offset
        quat_wxyz = np.asarray(self.data.xquat[body_id], dtype=float)
        quat_xyzw = np.r_[quat_wxyz[1:], quat_wxyz[0]]
        return position.copy(), quat_xyzw.copy(), quat_wxyz.copy()

    def _state(self, reason):
        mujoco.mj_forward(self.model, self.data)
        arm14 = self._arm14_rad()
        left_pos, left_quat, _ = self._eef("left")
        right_pos, right_quat, _ = self._eef("right")
        return {
            "schema_version": 1,
            "recorded_at": now_iso(),
            "reason": reason,
            "source": "MuJoCo Kuavo-5.2 current qpos",
            "robot": "5",
            "layout": ARM14_LAYOUT,
            "arm14_rad": arm14.tolist(),
            "arm14_deg": np.degrees(arm14).tolist(),
            "left_eef": {
                "position_m": left_pos.tolist(),
                "quaternion_xyzw": left_quat.tolist(),
            },
            "right_eef": {
                "position_m": right_pos.tolist(),
                "quaternion_xyzw": right_quat.tolist(),
            },
        }

    def read_current_state(self, reason="read", save=True):
        state = self._state(reason)
        if save:
            self.store.write_latest(state)
        return state

    def _append_snapshot(self, label=None):
        self.trajectory.append(
            {
                "label": label or self._active_label,
                "qpos": np.asarray(self.data.qpos, dtype=float).copy(),
                "meta": copy.deepcopy(self._active_meta),
            }
        )

    def _run_joint_target(self, *args, **kwargs):
        self._active_label = kwargs.get("name") or (args[1] if len(args) > 1 else "motion")
        self._active_meta = copy.deepcopy(kwargs.get("extra") or {})
        return super()._run_joint_target(*args, **kwargs)

    def _publish_target(self, target_deg, duration_s):
        start = self._arm14_rad()
        target = np.radians(np.asarray(target_deg, dtype=np.float64))
        frames = max(12, min(60, round(float(duration_s) * 20)))
        for index in range(1, frames + 1):
            x = index / frames
            alpha = x * x * (3.0 - 2.0 * x)
            self.data.qpos[self.qpos_addresses] = start * (1.0 - alpha) + target * alpha
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self._append_snapshot()

    def _wait_for_motion(self, duration_s):
        return {
            "backend": "mujoco_kinematic",
            "simulated_duration_s": float(duration_s),
            "saw_running": True,
            "final_executing": False,
        }

    def _solve_cartesian_step(self, pre, arm, direction, step_m):
        if arm not in ("left", "right"):
            raise ValueError("arm must be left or right")
        # Start exactly from the state passed through the existing controller.
        self.data.qpos[self.qpos_addresses] = np.asarray(pre["arm14_rad"], dtype=float)
        mujoco.mj_forward(self.model, self.data)
        current_position, _, target_quat = self._eef(arm)
        requested_position = np.asarray(
            translated_xyz(current_position, direction, step_m), dtype=np.float64
        )

        arm_slice = slice(0, 7) if arm == "left" else slice(7, 14)
        qpos_addresses = self.qpos_addresses[arm_slice]
        dof_addresses = self.dof_addresses[arm_slice]
        joint_ids = self.joint_ids[arm_slice]
        body_id = self.body_ids[arm]

        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        rotation_error = np.zeros(3, dtype=np.float64)
        iterations = 0
        for iterations in range(1, 301):
            mujoco.mj_forward(self.model, self.data)
            position, _, current_quat = self._eef(arm)
            position_error = requested_position - position
            mujoco.mju_subQuat(rotation_error, target_quat, current_quat)
            if np.linalg.norm(position_error) < 2e-5 and np.linalg.norm(rotation_error) < 2e-4:
                break

            jacp.fill(0.0)
            jacr.fill(0.0)
            mujoco.mj_jac(self.model, self.data, jacp, jacr, position, body_id)
            orientation_weight = 0.30
            jacobian = np.vstack(
                [jacp[:, dof_addresses], orientation_weight * jacr[:, dof_addresses]]
            )
            error = np.r_[position_error, orientation_weight * rotation_error]
            damping = 2e-3
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(6), error
            )
            largest = float(np.max(np.abs(delta)))
            if largest > 0.08:
                delta *= 0.08 / largest
            candidate = np.asarray(self.data.qpos[qpos_addresses], dtype=float) + delta
            for index, joint_id in enumerate(joint_ids):
                low, high = self.model.jnt_range[joint_id]
                candidate[index] = np.clip(candidate[index], low + 1e-5, high - 1e-5)
            self.data.qpos[qpos_addresses] = candidate

        mujoco.mj_forward(self.model, self.data)
        achieved_position, _, _ = self._eef(arm)
        error_mm = position_error_mm(requested_position, achieved_position)
        result_q = self._arm14_rad()
        accepted = error_mm <= self.max_position_error_mm

        # Planning must not mutate the simulated feedback state. Execution is
        # performed later by the unchanged _run_joint_target path.
        self.data.qpos[self.qpos_addresses] = np.asarray(pre["arm14_rad"], dtype=float)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return {
            "ik_service_success": bool(accepted),
            "solver": "MuJoCo damped-least-squares 6D IK",
            "iterations": iterations,
            "accepted": bool(accepted),
            "current_xyz_m": current_position.tolist(),
            "requested_xyz_m": requested_position.tolist(),
            "achieved_xyz_m": achieved_position.tolist(),
            "position_error_mm": float(error_mm),
            "target_arm14_rad": result_q.tolist(),
            "target_arm14_deg": np.degrees(result_q).tolist(),
        }
