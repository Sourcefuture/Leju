#!/usr/bin/env python3
"""Render Kuavo-5 arm commands with the official MuJoCo model.

This is a visual/kinematic smoke test.  It deliberately writes qpos directly so
it does not pretend to validate MPC/WBC dynamics or a trained VLA checkpoint.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

# The variable must be set before importing mujoco.  EGL works in headless WSL;
# --live switches to GLFW so the same script can open a WSLg window.
if "--live" in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "glfw")
else:
    os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/kuavo-mujoco-cache")

import cv2
import mujoco
import numpy as np

from kuavo_arm_controller import ARM14_LAYOUT, STANDARD_POSES_DEG


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = (
    ROOT
    / ".sim/kuavo-ros-opensource/src/kuavo_assets/models/biped_s52/xml/scene.xml"
)
DEFAULT_OUTPUT = ROOT / "outputs/kuavo_sim/kuavo5_arm_pose_demo.mp4"
POSE_SEQUENCE = [
    ("arms_down", 1.0, 0.8),
    ("ready", 1.8, 0.8),
    ("arms_horizontal_palms_down", 2.8, 1.2),
    ("elbows_90_palms_down", 2.8, 1.2),
    ("arms_down", 2.8, 1.0),
]


def smoothstep(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def joint_id(model: mujoco.MjModel, name: str) -> int:
    result = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if result < 0:
        raise RuntimeError(f"official model does not contain joint {name!r}")
    return result


def arm_qpos_addresses(model: mujoco.MjModel) -> list[int]:
    names = [f"zarm_l{i}_joint" for i in range(1, 8)]
    names += [f"zarm_r{i}_joint" for i in range(1, 8)]
    return [int(model.jnt_qposadr[joint_id(model, name)]) for name in names]


def validate_and_clip_pose(
    model: mujoco.MjModel, values_deg: list[float]
) -> tuple[np.ndarray, list[dict]]:
    if len(values_deg) != 14:
        raise ValueError(f"arm command must contain 14 values, got {len(values_deg)}")
    values = np.radians(np.asarray(values_deg, dtype=np.float64))
    clipped = values.copy()
    warnings = []
    names = [f"zarm_l{i}_joint" for i in range(1, 8)]
    names += [f"zarm_r{i}_joint" for i in range(1, 8)]
    for index, name in enumerate(names):
        jid = joint_id(model, name)
        low, high = (float(x) for x in model.jnt_range[jid])
        clipped[index] = np.clip(values[index], low, high)
        if not math.isclose(values[index], clipped[index], abs_tol=1e-9):
            warnings.append(
                {
                    "joint": ARM14_LAYOUT[index],
                    "requested_deg": float(values_deg[index]),
                    "clipped_deg": math.degrees(float(clipped[index])),
                }
            )
    return clipped, warnings


def reset_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
        data.qpos[2] = 0.98
        data.qpos[3] = 1.0
    mujoco.mj_forward(model, data)


def set_arm_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    addresses: list[int],
    values_rad: np.ndarray,
) -> None:
    data.qpos[addresses] = values_rad
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def make_camera() -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.05, 0.0, 1.02]
    camera.distance = 2.9
    camera.azimuth = 142.0
    camera.elevation = -8.0
    return camera


def scene_label(scene_path: Path) -> str:
    text = str(scene_path)
    if "biped_s52" in text or "kuavo52" in text:
        return "KUAVO 5.2"
    if "biped_s45" in text:
        return "KUAVO 4.5 | official challenge scene"
    return "KUAVO | official MuJoCo model"


def overlay(
    frame: np.ndarray, title: str, values_deg: np.ndarray, model_label: str
) -> np.ndarray:
    result = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    panel = result.copy()
    cv2.rectangle(panel, (0, 0), (result.shape[1], 96), (12, 18, 28), -1)
    cv2.addWeighted(panel, 0.76, result, 0.24, 0, result)
    cv2.putText(
        result,
        model_label,
        (22, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        result,
        title,
        (22, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        (80, 220, 255),
        2,
        cv2.LINE_AA,
    )
    left = "L: " + " ".join(f"{x:+.0f}" for x in values_deg[:7])
    right = "R: " + " ".join(f"{x:+.0f}" for x in values_deg[7:])
    cv2.putText(result, left, (22, result.shape[0] - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(result, right, (22, result.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1, cv2.LINE_AA)
    return result


def build_timeline(
    model: mujoco.MjModel, fps: int, custom_values: list[float] | None = None
):
    sequence = POSE_SEQUENCE
    custom_poses = {}
    if custom_values is not None:
        custom_poses["custom_arm14"] = custom_values
        sequence = [
            ("arms_down", 1.0, 0.8),
            ("custom_arm14", 3.0, 2.0),
            ("arms_down", 3.0, 1.0),
        ]
    poses = {}
    all_warnings = []
    for name, _, _ in sequence:
        values = list(
            custom_poses.get(name, STANDARD_POSES_DEG.get(name, {}).get("values", []))
        )
        poses[name], warnings = validate_and_clip_pose(model, values)
        all_warnings.extend({"pose": name, **warning} for warning in warnings)

    frames = []
    previous = poses[sequence[0][0]].copy()
    for name, move_seconds, hold_seconds in sequence:
        target = poses[name]
        move_frames = max(1, round(move_seconds * fps))
        for index in range(move_frames):
            alpha = smoothstep((index + 1) / move_frames)
            values = previous * (1.0 - alpha) + target * alpha
            frames.append((name, values))
        frames.extend((name, target.copy()) for _ in range(round(hold_seconds * fps)))
        previous = target.copy()
    return frames, all_warnings


def render_video(args, model, data, addresses, frames, warnings):
    args.output.parent.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = make_camera()
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(
        str(args.output), fourcc, args.fps, (args.width, args.height)
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not create the MP4 output")

    keyframes = {}
    model_label = scene_label(args.scene)
    try:
        for index, (name, values) in enumerate(frames):
            set_arm_pose(model, data, addresses, values)
            renderer.update_scene(data, camera=camera)
            image = overlay(renderer.render(), name, np.degrees(values), model_label)
            writer.write(image)
            keyframes[name] = image.copy()
    finally:
        writer.release()
        renderer.close()

    poster_names = list(keyframes)
    while len(poster_names) < 4:
        poster_names.append(poster_names[-1])
    poster_names = poster_names[:4]
    tiles = [keyframes[name] for name in poster_names]
    half_w, half_h = args.width // 2, args.height // 2
    tiles = [cv2.resize(tile, (half_w, half_h)) for tile in tiles]
    poster = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
    poster_path = args.output.with_name(args.output.stem + "_poses.png")
    cv2.imwrite(str(poster_path), poster)

    report = {
        "mode": "kinematic_visual_smoke_test",
        "robot_model": model_label,
        "scene": str(args.scene.resolve()),
        "arm_layout": ARM14_LAYOUT,
        "fps": args.fps,
        "frames": len(frames),
        "duration_seconds": len(frames) / args.fps,
        "video": str(args.output.resolve()),
        "poster": str(poster_path.resolve()),
        "joint_limit_clips": warnings,
        "validated": [
            "official MJCF loads",
            "14 arm joints resolve by name",
            "14D degree commands map to MuJoCo qpos",
            "offscreen rendering and MP4 encoding",
        ],
        "not_validated": [
            "MPC/WBC dynamics",
            "ROS simulator services",
            "camera observations into LingBot-VLA-V2",
            "trained checkpoint inference (checkpoint is absent)",
        ],
    }
    report_path = args.output.with_suffix(".json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return poster_path, report_path


def run_live(model, data, addresses, frames, fps: int):
    import mujoco.viewer

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.05, 0.0, 1.02]
        viewer.cam.distance = 2.9
        viewer.cam.azimuth = 142.0
        viewer.cam.elevation = -8.0
        delay = 1.0 / fps
        while viewer.is_running():
            for _, values in frames:
                if not viewer.is_running():
                    return
                started = time.monotonic()
                set_arm_pose(model, data, addresses, values)
                viewer.sync()
                time.sleep(max(0.0, delay - (time.monotonic() - started)))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--fps", type=int, default=30)
    # The official MJCF uses MuJoCo's default 640x480 offscreen framebuffer.
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--values",
        type=float,
        nargs=14,
        metavar="DEG",
        help="render one custom [left J1..J7, right J1..J7] command",
    )
    parser.add_argument("--live", action="store_true", help="loop in a WSLg window")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.scene.is_file():
        raise SystemExit(
            f"missing official scene: {args.scene}\n"
            "Clone LejuRobotics/kuavo-ros-opensource under .sim first."
        )
    model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
    data = mujoco.MjData(model)
    reset_home(model, data)
    addresses = arm_qpos_addresses(model)
    frames, warnings = build_timeline(model, args.fps, args.values)
    if args.live:
        run_live(model, data, addresses, frames, args.fps)
        return
    poster, report = render_video(args, model, data, addresses, frames, warnings)
    print(f"video:  {args.output.resolve()}")
    print(f"poster: {poster.resolve()}")
    print(f"report: {report.resolve()}")


if __name__ == "__main__":
    main()
