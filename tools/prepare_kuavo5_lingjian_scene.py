#!/usr/bin/env python3
"""Compose the official challenge Task-1 room with the official Kuavo-5.2 model.

The challenge branch ships its ROS controller and scorer for Kuavo 4.5 only.
This helper swaps the visual model to Kuavo 5.2 for camera/pose validation.  It
does not claim controller or automatic-scoring compatibility.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHALLENGE_ROOT = ROOT / ".sim/kuavo-data-challenge-branch/src/data_challenge_simulator"
KUAVO52_ROOT = ROOT / ".sim/kuavo-ros-opensource/src/kuavo_assets/models/biped_s52"
OUTPUT_DIR = ROOT / "outputs/kuavo_sim/scenes"


def main():
    challenge_scene = CHALLENGE_ROOT / "models/biped_s45/xml/scene1.xml"
    kuavo_model = KUAVO52_ROOT / "xml/biped_s52.xml"
    if not challenge_scene.is_file() or not kuavo_model.is_file():
        raise SystemExit("official challenge scene or Kuavo-5.2 model is missing under .sim")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    model_output = OUTPUT_DIR / "biped_s52_visual.xml"
    scene_output = OUTPUT_DIR / "lingjian_task1_kuavo52_visual.xml"

    model_text = kuavo_model.read_text(encoding="utf-8")
    model_text = model_text.replace(
        'meshdir="../meshes/"',
        f'meshdir="{(KUAVO52_ROOT / "meshes").resolve()}/"',
        1,
    )
    model_output.write_text(model_text, encoding="utf-8")

    scene_text = challenge_scene.read_text(encoding="utf-8")
    scene_text = scene_text.replace(
        '<include file="biped_s45.xml" />',
        '<include file="biped_s52_visual.xml" />',
        1,
    )
    scene_text = scene_text.replace(
        'file="../../assets/',
        f'file="{(CHALLENGE_ROOT / "models/assets").resolve()}/',
    )
    scene_output.write_text(scene_text, encoding="utf-8")
    print(scene_output.resolve())


if __name__ == "__main__":
    main()
