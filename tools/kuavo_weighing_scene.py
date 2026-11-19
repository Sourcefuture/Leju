#!/usr/bin/env python3
"""Render the Kuavo-5.2 weighing scene or verify its isolated scale physics."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np

from prepare_kuavo5_weighing_scene import DEFAULT_OUTPUT, PARCELS, PLATE_TOP, SCALE_XY, TABLE_Z


def reset(model):
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return data


def position_parcels(model, data, selected):
    for name, spec in PARCELS.items():
        joint = model.joint(f"parcel_{name}_free")
        adr = int(joint.qposadr[0])
        xy = SCALE_XY if name == selected else spec["xy"]
        top = PLATE_TOP if name == selected else TABLE_Z
        data.qpos[adr:adr+7] = [*xy, top+spec["half_size"][2]+.003, 1,0,0,0]
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


def measure(model, data):
    readings = []
    steps = round(2.0 / model.opt.timestep)
    for i in range(steps):
        mujoco.mj_step(model, data)
        if i >= 3*steps//4:
            readings.append(float(data.sensor("scale_force").data[2]))
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(readings)):
        raise RuntimeError("Non-finite scale dynamics")
    return float(np.mean(readings)), float(np.std(readings))


def validate(output):
    model = mujoco.MjModel.from_xml_path(str(output / "scale_bench.xml"))
    data = reset(model)
    position_parcels(model, data, "empty")
    tare, _ = measure(model, data)
    gravity = abs(model.opt.gravity[2])
    records = []
    for selected in ["a", "b", "c", "empty"]:
        data = reset(model)
        position_parcels(model, data, selected)
        force, std = measure(model, data)
        mass = abs(force-tare) / gravity
        expected = PARCELS[selected]["mass"] if selected in PARCELS else 0.0
        error = abs(mass-expected)
        records.append({"parcel_on_scale": selected, "expected_kg": expected,
                        "measured_kg": mass, "error_kg": error, "force_z_n": force,
                        "std_kg": std/gravity,
                        "passed": bool(error < .005 and std/gravity < .005)})
    report = {"mode": "isolated scale contact physics; robot not simulated dynamically",
              "tare_force_z_n": tare, "settle_time_s": 2.0, "averaging_window_s": .5,
              "formula": "abs(mean(force_z)-tare_force_z)/abs(gravity_z)",
              "passed": all(row["passed"] for row in records), "cases": records}
    (output / "scale_validation.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


def render(output):
    model = mujoco.MjModel.from_xml_path(str(output / "scene.xml"))
    data = reset(model)
    with mujoco.Renderer(model, height=720, width=960) as renderer:
        for camera in ["weighing_overview", "weighing_top"]:
            renderer.update_scene(data, camera=camera)
            frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            cv2.rectangle(frame, (0,0), (960,74), (24,28,35), -1)
            cv2.putText(frame, "KUAVO 5.2 | Weighing workbench", (24,30),
                        cv2.FONT_HERSHEY_SIMPLEX, .75, (255,255,255), 2, cv2.LINE_AA)
            cv2.putText(frame, "Blue 0.2 kg | Orange 0.5 kg | Purple 1.0 kg (configured masses)",
                        (24,58), cv2.FONT_HERSHEY_SIMPLEX, .53, (190,225,235), 1, cv2.LINE_AA)
            path = output / f"{camera}.png"
            if not cv2.imwrite(str(path), frame):
                raise RuntimeError(f"Could not save {path}")
            print(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("render", "scale-test"))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output.resolve()
    if not (output / "scene.xml").is_file():
        raise SystemExit("Build the scene first: bash tools/kuavo_weighing.sh build")
    if args.command == "render":
        render(output)
    else:
        validate(output)


if __name__ == "__main__":
    main()
