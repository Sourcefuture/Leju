#!/usr/bin/env python3
"""Build the existing Kuavo-5.2 model with a weighing workbench, offline."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / ".sim/kuavo-ros-opensource/src/kuavo_assets/models/biped_s52"
DEFAULT_OUTPUT = ROOT / "outputs/kuavo_sim/weighing"
PARCELS = {
    "a": {"mass": 0.2, "half_size": [0.045, 0.04, 0.035], "xy": [0.51, 0.32], "rgba": "0.20 0.58 0.90 1"},
    "b": {"mass": 0.5, "half_size": [0.055, 0.045, 0.045], "xy": [0.69, 0.32], "rgba": "0.92 0.49 0.15 1"},
    "c": {"mass": 1.0, "half_size": [0.065, 0.05, 0.055], "xy": [0.88, 0.32], "rgba": "0.54 0.36 0.76 1"},
}
TABLE_Z = 0.76
PLATE_TOP = 0.844
SCALE_XY = [0.60, 0.0]


def numbers(values):
    return " ".join(f"{v:.9g}" for v in values)


def box(parent, name, pos, size, rgba, **kwargs):
    return ET.SubElement(parent, "geom", name=name, type="box", pos=numbers(pos),
                         size=numbers(size), rgba=rgba, contype="8", conaffinity="8",
                         friction="0.8 0.01 0.001", **kwargs)


def camera(parent, name, pos, target):
    z = np.array(pos) - np.array(target)
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ET.SubElement(parent, "camera", name=name, pos=numbers(pos),
                  xyaxes=numbers(np.r_[x, y]), fovy="45")


def make_workbench(on_scale):
    world = ET.Element("worldbody")
    ET.SubElement(world, "light", pos="0.6 -0.8 3", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="weighing_floor", type="plane", size="3 3 0.05",
                  rgba="0.18 0.22 0.27 1", contype="8", conaffinity="8")
    box(world, "table_top", [.68, 0, TABLE_Z-.025], [.38, .62, .025], "0.60 0.43 0.28 1")
    for i, (x, y) in enumerate(( (.34,.56), (.34,-.56), (1.02,.56), (1.02,-.56) )):
        box(world, f"table_leg_{i}", [x,y,.355], [.022,.022,.355], "0.18 0.21 0.25 1")
    # Fixed scale base; a separate fixed child plate has a force sensor at its support.
    scale = ET.SubElement(world, "body", name="scale_base", pos="0.60 0 0.76")
    box(scale, "scale_housing", [0,0,.027], [.175,.155,.027], "0.16 0.20 0.26 1")
    plate = ET.SubElement(scale, "body", name="scale_plate", pos="0 0 0.074")
    box(plate, "scale_surface", [0,0,0], [.15,.13,.010], "0.80 0.84 0.88 1", mass="0.35")
    ET.SubElement(plate, "site", name="scale_load_site", pos="0 0 0", size="0.004", rgba="0 0 0 0")
    box(scale, "scale_display", [-.09,-.17,.035], [.065,.02,.012], "0.08 0.48 0.35 1")
    # Destination bin, to the robot's right. It is a task landmark, not a scorer.
    box(world, "output_bin_bottom", [.73,-.41,.772], [.18,.14,.012], "0.20 0.48 0.43 1")
    for name, pos, size in (
        ("front", [.55,-.41,.82], [.012,.14,.048]),
        ("back", [.91,-.41,.82], [.012,.14,.048]),
        ("left", [.73,-.27,.82], [.18,.012,.048]),
        ("right", [.73,-.55,.82], [.18,.012,.048]),
    ):
        box(world, f"output_bin_{name}", pos, size, "0.20 0.48 0.43 1")
    for name, spec in PARCELS.items():
        xy = SCALE_XY if name == on_scale else spec["xy"]
        top = PLATE_TOP if name == on_scale else TABLE_Z
        body = ET.SubElement(world, "body", name=f"parcel_{name}",
                             pos=numbers([*xy, top+spec["half_size"][2]+.003]))
        ET.SubElement(body, "freejoint", name=f"parcel_{name}_free")
        box(body, f"parcel_{name}_geom", [0,0,0], spec["half_size"], spec["rgba"], mass=str(spec["mass"]))
    ET.SubElement(world, "site", name="scale_hover", pos="0.60 0 1.02", type="sphere",
                  size="0.012", rgba="0.95 0.75 0.1 0.6")
    camera(world, "weighing_overview", [2.1,-2.5,2.0], [.38,0,.92])
    ET.SubElement(world, "camera", name="weighing_top", pos=".68 0 2.2",
                  xyaxes="0 -1 0 1 0 0", fovy="48")
    return world


def save(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build(output=DEFAULT_OUTPUT, on_scale="empty"):
    output = Path(output).resolve()
    source = MODEL_DIR / "xml/biped_s52.xml"
    if not source.is_file():
        raise FileNotFoundError(f"Local official Kuavo-5.2 model is missing: {source}")
    output.mkdir(parents=True, exist_ok=True)
    scene = ET.parse(source).getroot()
    scene.set("model", "Kuavo-5.2 weighing workbench")
    scene.find("compiler").set("meshdir", str((MODEL_DIR / "meshes").resolve()))
    # Approximate simulation mounts; these are not real-camera calibration.
    head = scene.find(".//body[@name='head_camera_depth_link']")
    ET.SubElement(head, "camera", name="sim_head", pos="0.015 0 0",
                  xyaxes="0 -1 0 0.5 0 0.866025404", fovy="60")
    for side in ("l", "r"):
        wrist = scene.find(f".//body[@name='zarm_{side}7_link']")
        ET.SubElement(wrist, "camera", name=f"sim_wrist_{side}", pos="0.025 0.065 -0.045",
                      xyaxes="1 0 0 0 1 0", fovy="60")
    ET.SubElement(scene, "option", timestep="0.002", gravity="0 0 -9.81")
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.65 0.65 0.65")
    workbench = make_workbench(on_scale)
    scene.find("worldbody").extend(copy.deepcopy(list(workbench)))
    ET.SubElement(scene.find("sensor"), "force", name="scale_force", site="scale_load_site")
    scene_path = output / "scene.xml"
    save(scene, scene_path)
    # Let MuJoCo determine all qpos addresses, including the three free parcels.
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    home = model.qpos0.copy()
    base = model.body("base_link").id
    home[model.jnt_qposadr[model.body_jntadr[base]] + 2] = .98
    ET.SubElement(ET.SubElement(scene, "keyframe"), "key", name="home", qpos=numbers(home))
    save(scene, scene_path)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    for arm in ("l", "r"):
        for j in range(1,8):
            model.joint(f"zarm_{arm}{j}_joint")

    # Use identical table, load cell and parcels for a gravity/contact bench test.
    # No unsupported humanoid balance controller is implied by this second file.
    bench = ET.Element("mujoco", model="weighing contact validation")
    ET.SubElement(bench, "option", timestep="0.002", gravity="0 0 -9.81")
    bench.append(copy.deepcopy(workbench))
    ET.SubElement(ET.SubElement(bench, "sensor"), "force", name="scale_force", site="scale_load_site")
    save(bench, output / "scale_bench.xml")
    metadata = {
        "robot": "Kuavo-5.2 biped_s52 (same local model as previous simulations)",
        "scene": str(scene_path), "coordinate_frame": "MuJoCo world; not waist_yaw_link",
        "table_top_z_m": TABLE_Z, "scale_top_xyz_m": [*SCALE_XY, PLATE_TOP],
        "scale_hover_xyz_m": [.60,0,1.02], "parcel_on_scale": on_scale,
        "parcels": PARCELS, "scale_sensor": "scale_force (3D force, N)",
        "robot_mode": "Existing arm viewer/controller use kinematic qpos playback",
        "scale_validation": "scale_bench.xml uses gravity and contact dynamics, without robot",
        "sim_cameras": {
            "head": "sim_head", "left": "sim_wrist_l", "right": "sim_wrist_r",
            "top": "weighing_top", "overview": "weighing_overview",
            "mount_note": "Approximate simulation mounts, not calibrated real-camera extrinsics",
        },
    }
    (output / "scene_info.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+"\n")
    return scene_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--on-scale", choices=["empty", *PARCELS], default="empty")
    args = parser.parse_args()
    print(build(args.output, args.on_scale))


if __name__ == "__main__":
    main()
