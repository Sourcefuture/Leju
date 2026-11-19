#!/usr/bin/env python3
"""Live Kuavo-5.2 viewer driven by the controller's persistent history."""

from __future__ import annotations

import argparse
import json
import os
import time
from collections import deque
from kuavo_sim_bin_environment import apply_private_bin_poses
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "glfw")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/kuavo-mujoco-cache")

import mujoco
import mujoco.viewer
import numpy as np

try:
    from kuavo_planar_task import sample_motion, apply_objects, load_json
except ModuleNotFoundError:
    from tools.kuavo_planar_task import sample_motion, apply_objects, load_json
try:
    from kuavo_scale_vision import write_lcd_texture
except ModuleNotFoundError:
    from tools.kuavo_scale_vision import write_lcd_texture

try:
    from kuavo_mujoco_controller import DEFAULT_SCENE
except ModuleNotFoundError:
    from tools.kuavo_mujoco_controller import DEFAULT_SCENE


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STATE_DIR = ROOT / "outputs/robot_control/kuavo5_mujoco"


def arm_addresses(model):
    names = [f"zarm_l{i}_joint" for i in range(1, 8)]
    names += [f"zarm_r{i}_joint" for i in range(1, 8)]
    result = []
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"scene is not a Kuavo-5 arm model: missing {name}")
        result.append(int(model.jnt_qposadr[jid]))
    return result


def valid_arm_state(payload):
    values = payload.get("arm14_rad") if isinstance(payload, dict) else None
    return (
        isinstance(values, list)
        and len(values) == 14
        and payload.get("robot") == "5"
        and str(payload.get("source", "")).startswith("MuJoCo Kuavo-5.2")
    )


def read_latest(path):
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        return np.asarray(payload["arm14_rad"], dtype=float) if valid_arm_state(payload) else None
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None


def synchronize_latest(model, data, addresses, latest_path):
    """Recover missed history updates while idle, including a state reset."""
    latest = read_latest(latest_path)
    if latest is None or np.max(np.abs(latest - data.qpos[addresses])) < 1e-6:
        return False
    data.qpos[addresses] = latest
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    task_objects = load_json(latest_path, {}).get('task_objects', {})
    apply_objects(model, data, task_objects)
    apply_private_bin_poses(model, data, task_objects, latest_path.parent)
    return True


class HistoryFollower:
    def __init__(self, path, from_start=False):
        self.path = path
        self.position = path.stat().st_size if path.is_file() and not from_start else 0
        self.partial = ""

    def poll(self):
        if not self.path.is_file():
            return []
        size = self.path.stat().st_size
        if size < self.position:
            self.position = 0
            self.partial = ""
        with self.path.open("r", encoding="utf-8") as stream:
            stream.seek(self.position)
            chunk = stream.read()
            self.position = stream.tell()
        if not chunk:
            return []
        text = self.partial + chunk
        lines = text.splitlines(keepends=True)
        self.partial = ""
        if lines and not lines[-1].endswith("\n"):
            self.partial = lines.pop()
        records = []
        for line in lines:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            target = record.get("target_arm14_rad")
            if (
                record.get("status") == "executed"
                and isinstance(target, list)
                and len(target) == 14
            ):
                records.append(record)
        return records


def draw_point_checks(viewer, model, data, points, selection):
    """Label geometric test points and show actual proxy TCP, not wrist origin."""
    if not points:
        return ""
    viewer.user_scn.ngeom = 0
    for name, spec in points.items():
        if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
            break
        geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        position = np.asarray(spec['target_world_xyz_m']) + [0, 0, .025]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LABEL, np.zeros(3),
                           position, np.eye(3).ravel(), np.array([1., 1., 1., 1.]))
        geom.label = name
        viewer.user_scn.ngeom += 1
    point = selection.get('point')
    if point not in points:
        return "\nPoint check: preview A/B/C/D or R1/R2/R3/R4 in another terminal"
    spec = points[point]
    ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'check_tcp_' + spec['arm'])
    if ident < 0:
        return ""
    delta = (data.geom_xpos[ident] - spec['target_world_xyz_m']) * 1000
    return (f"\nCHECK {point} ({spec['arm']} TCP), target +50 mm above table"
            f"\nerror XYZ mm: {delta[0]:+.1f} {delta[1]:+.1f} {delta[2]:+.1f}"
            f"\n{selection.get('mode', '')}; human confirmation required")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--state-dir", type=Path, default=DEFAULT_STATE_DIR)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--camera", help="start with a named scene camera, or 'free' for mouse-controlled view")
    parser.add_argument('--replay',action='store_true',help='Replay existing recorded actions from their first posture')
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="playback speed multiplier; 2.0 plays motions twice as fast",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.fps <= 0 or args.speed <= 0:
        raise SystemExit("--fps and --speed must be positive")
    scene = args.scene.expanduser().resolve()
    state_dir = args.state_dir.expanduser().resolve()
    state_dir.mkdir(parents=True, exist_ok=True)
    latest_path = state_dir / "latest_state.json"
    history_path = state_dir / "history.jsonl"
    check_points = load_json(scene.parent / 'point_check_targets.json', {}).get('points', {})

    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
        data.qpos[2] = 0.98
        data.qpos[3] = 1.0
    addresses = arm_addresses(model)
    follower = HistoryFollower(history_path,from_start=args.replay)
    existing=follower.poll() if args.replay else []
    initial = (np.asarray(existing[0].get('trajectory_arm14_rad',[existing[0]['target_arm14_rad']])[0])
               if existing else read_latest(latest_path))
    if initial is not None:
        data.qpos[addresses] = initial
    head=(existing[0].get('head_yaw_pitch_deg',[]) if existing else load_json(latest_path,{}).get('head_yaw_pitch_deg',[]))
    for name,angle in zip(['zhead_1_joint','zhead_2_joint'],head):data.qpos[model.joint(name).qposadr[0]]=np.radians(angle)
    mujoco.mj_forward(model, data)
    task_objects = (existing[0].get('task_objects',{}) if existing else
                    load_json(latest_path, {}).get('task_objects', {}))
    apply_objects(model, data, task_objects)

    queue = deque(existing)
    active = None
    frame_period = 1.0 / args.fps
    display_action = "initial state"
    lcd_value = None
    print(f"GUI is watching: {state_dir}")
    print("Enter commands in another terminal. Close the window or press Ctrl+C to stop.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.05, 0.0, 1.02]
        viewer.cam.distance = 2.65
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -8
        if args.camera and args.camera != 'free':
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = model.camera(args.camera).id
        elif check_points:
            viewer.cam.lookat[:] = [.48, 0., .85]
            viewer.cam.distance = 2.4
            viewer.cam.elevation = -25
        while viewer.is_running():
            loop_started = time.monotonic()
            for record in follower.poll():
                queue.append(record)

            if active is None and queue:
                record = queue.popleft()
                active = {
                    "record": record,
                    "start_time": time.monotonic(),
                    "start": np.asarray(data.qpos[addresses], dtype=float).copy(),
                    "target": np.asarray(record["target_arm14_rad"], dtype=float),
                    "duration": max(
                        0.15,
                        float(record.get("actual_duration_s", 1.0)) / args.speed,
                    ),
                }
                display_action = record.get("name", "motion")
                task_objects = record.get('task_objects', task_objects)
                print(
                    f"playing {record.get('name', 'motion')} | "
                    f"duration={active['duration']:.2f}s"
                )

            with viewer.lock():
                playback_progress = 1.0
                drop_bag = (active['record'].get('bag') if active and active['record'].get('event')=='drop' else None)
                if active is not None:
                    for name,angle in zip(['zhead_1_joint','zhead_2_joint'],active['record'].get('head_yaw_pitch_deg',[])):
                        data.qpos[model.joint(name).qposadr[0]]=np.radians(angle)
                    elapsed = time.monotonic() - active["start_time"]
                    x = min(1.0, elapsed / active["duration"])
                    playback_progress = x
                    data.qpos[addresses] = sample_motion(active['record'], active['start'], x)
                    data.qvel[:] = 0.0
                    if x >= 1.0:
                        print(f"finished {active['record'].get('name', 'motion')}")
                        active = None
                elif not queue:
                    if synchronize_latest(model, data, addresses, latest_path):
                        display_action = "synchronized latest state"
                        print("synchronized latest_state.json")
                    task_objects = load_json(latest_path, {}).get('task_objects', task_objects)

                mujoco.mj_forward(model, data)
                apply_objects(model, data, task_objects, playback_progress, drop_bag)
                apply_private_bin_poses(model,data,task_objects,state_dir,playback_progress,drop_bag,
                    active['record'].get('action_id') if active else None)
                left = np.degrees(data.qpos[addresses[:7]])
                right = np.degrees(data.qpos[addresses[7:]])
                check_text = draw_point_checks(viewer, model, data, check_points,
                    load_json(state_dir / 'corner_check_selection.json', {}))
                phase = "MOVING" if active is not None else "IDLE"
                overlay_text = (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f"{phase}: {display_action}\nState: {state_dir.name}\n"
                    + "L deg: " + " ".join(f"{q:+.1f}" for q in left) + "\n"
                    + "R deg: " + " ".join(f"{q:+.1f}" for q in right) + check_text,
                    "",
                )
            viewer.set_texts(overlay_text)
            value = (active['record'].get('scale_display_kg',0) if active else
                     (lcd_value if queue and lcd_value is not None else
                      load_json(latest_path,{}).get('scale_display_kg',0)))
            if value != lcd_value:
                ident=write_lcd_texture(model,value)
                if ident is not None:viewer.update_texture(ident)
                lcd_value=value
            # Kinematic playback owns qpos; do not apply GUI physics edits back
            # into this state during the synchronization step.
            viewer.sync(state_only=True)
            time.sleep(max(0.0, frame_period - (time.monotonic() - loop_started)))


if __name__ == "__main__":
    main()
