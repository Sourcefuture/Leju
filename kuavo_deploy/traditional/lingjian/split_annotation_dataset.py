#!/usr/bin/env python3
"""Split extracted LeRobot frames by episode for CVAT annotation."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--validation-period",
        type=int,
        default=5,
        help="Assign every Nth selected episode to validation (default: 5).",
    )
    return parser.parse_args()


def copy_image(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def main() -> None:
    args = parse_args()
    if args.validation_period < 2:
        raise ValueError("--validation-period must be at least 2")

    source_manifest = args.input_dir / "samples.jsonl"
    if not source_manifest.is_file():
        raise FileNotFoundError(f"Missing extraction manifest: {source_manifest}")

    records = [
        json.loads(line)
        for line in source_manifest.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    episodes = sorted({int(record["episode_index"]) for record in records})
    episode_split = {
        episode: ("validation" if (position + 1) % args.validation_period == 0 else "train")
        for position, episode in enumerate(episodes)
    }

    split_records: list[dict] = []
    for record in records:
        filename = str(record["file"])
        source = args.input_dir / filename
        if not source.is_file():
            raise FileNotFoundError(source)
        split = episode_split[int(record["episode_index"])]
        destination = args.output_dir / "images" / split / filename
        copy_image(source, destination)
        split_records.append({**record, "split": split})

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = args.output_dir / "split_manifest.jsonl"
    manifest.write_text(
        "\n".join(json.dumps(record, ensure_ascii=False) for record in split_records) + "\n",
        encoding="utf-8",
    )

    image_counts = Counter(record["split"] for record in split_records)
    episode_counts = Counter(episode_split.values())
    summary = {
        "split_strategy": f"every {args.validation_period}th selected episode is validation",
        "image_counts": dict(sorted(image_counts.items())),
        "episode_counts": dict(sorted(episode_counts.items())),
        "total_images": len(split_records),
        "total_episodes": len(episodes),
        "episode_overlap": False,
    }
    (args.output_dir / "split_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
