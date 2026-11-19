#!/usr/bin/env python3
"""Fetch pinned simulation submodules and restore this project's visual XMLs.

Run after cloning: python3 tools/setup_kuavo_sim_dependencies.py
Use --skip-update when the two .sim repositories are already checked out.
Existing files with different content are never overwritten.
"""

import argparse
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
DEPENDENCIES = (".sim/kuavo-ros-opensource", ".sim/kuavo-data-challenge-branch")
RELATIVE_XML = Path("src/data_challenge_simulator/models/biped_s45/xml")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-update", action="store_true")
    args = parser.parse_args()
    if not args.skip_update:
        subprocess.run(
            ["git", "submodule", "update", "--init", "--", *DEPENDENCIES],
            cwd=ROOT, check=True,
        )
    meshes = ROOT / DEPENDENCIES[0] / "src/kuavo_assets/models/biped_s52/meshes"
    destination = ROOT / DEPENDENCIES[1] / RELATIVE_XML
    if not meshes.is_dir() or not destination.is_dir():
        parser.error("Simulation dependencies are missing; run without --skip-update")
    source = ROOT / "configs/sim/visual_overrides"
    pending = []
    for name in ("biped_s52_visual.xml", "scene1_kuavo52_visual.xml"):
        text = (source / name).read_text(encoding="utf-8")
        if name == "biped_s52_visual.xml":
            text, count = re.subn(
                r'meshdir="[^"]*"', lambda _: 'meshdir="' + str(meshes) + '/"',
                text, count=1,
            )
            if count != 1:
                raise ValueError("Expected one meshdir in the visual robot template")
        target = destination / name
        if target.exists() and target.read_text(encoding="utf-8") != text:
            parser.error("Refusing to overwrite a modified scene: " + str(target))
        pending.append((target, text))
    for target, text in pending:
        if not target.exists():
            target.write_text(text, encoding="utf-8")
        print(target.relative_to(ROOT))


if __name__ == "__main__":
    main()
