#!/usr/bin/env python3
"""Exercise both Kuavo-5 arms in all six Cartesian directions in MuJoCo."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/kuavo-mujoco-cache")

import cv2
import mujoco
import numpy as np

from kuavo_arm_controller import DIRECTIONS
from kuavo_mujoco_controller import DEFAULT_SCENE, MuJoCoKuavoArmController


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "outputs/kuavo_sim/cartesian_conversion_test"


def render(controller, output: Path, fps: int = 20):
    width, height = 640, 480
    renderer = mujoco.Renderer(controller.model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.05, 0.0, 1.02]
    camera.distance = 2.65
    camera.azimuth = 145
    camera.elevation = -8
    writer = cv2.VideoWriter(
        str(output.with_suffix(".mp4")),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError("could not open simulation video writer")
    try:
        for snapshot in controller.trajectory:
            controller.data.qpos[:] = snapshot["qpos"]
            mujoco.mj_forward(controller.model, controller.data)
            renderer.update_scene(controller.data, camera=camera)
            frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            shade = frame.copy()
            cv2.rectangle(shade, (0, 0), (width, 72), (10, 15, 24), -1)
            cv2.addWeighted(shade, 0.78, frame, 0.22, 0, frame)
            label = snapshot["label"]
            cv2.putText(frame, "KUAVO 5.2 Cartesian -> arm14 test", (18, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, label, (18, 57), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (80, 220, 255), 2, cv2.LINE_AA)
            writer.write(frame)
    finally:
        writer.release()
        renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--step-m", type=float, default=0.02)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    controller = MuJoCoKuavoArmController(
        scene=args.scene,
        state_dir=args.output.with_name(args.output.name + "_state"),
        max_position_error_mm=2.0,
    )
    controller.ready(duration_s=1.0, execute=True)
    records = []
    ordered_directions = ["forward", "backward", "left", "right", "up", "down"]
    previous_post = controller.read_current_state("test_start", save=True)
    for arm in ("left", "right"):
        for direction in ordered_directions:
            record = controller.move_relative(
                arm=arm,
                direction=direction,
                step_m=args.step_m,
                duration_s=0.5,
                execute=True,
            )
            continuity_deg = max(
                abs(a - b)
                for a, b in zip(
                    record["pre_state"]["arm14_deg"], previous_post["arm14_deg"]
                )
            )
            record["state_continuity_error_deg"] = continuity_deg
            previous_post = record.get("post_state", record["pre_state"])
            records.append(record)

    failures = [r for r in records if r["status"] != "executed"]
    report = {
        "robot": "Kuavo-5.2 official MJCF",
        "backend": "MuJoCo kinematic FK/IK",
        "step_m": args.step_m,
        "arms": ["left", "right"],
        "directions": ordered_directions,
        "tests": len(records),
        "passed": len(records) - len(failures),
        "failed": len(failures),
        "max_ik_position_error_mm": max(r["ik"]["position_error_mm"] for r in records),
        "max_post_position_error_mm": max(r["post_position_error_mm"] for r in records),
        "max_state_continuity_error_deg": max(r["state_continuity_error_deg"] for r in records),
        "records": records,
    }
    args.output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    render(controller, args.output)
    print(json.dumps({k: v for k, v in report.items() if k != "records"}, ensure_ascii=False, indent=2))
    print(f"video: {args.output.with_suffix('.mp4').resolve()}")
    print(f"report: {args.output.with_suffix('.json').resolve()}")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
