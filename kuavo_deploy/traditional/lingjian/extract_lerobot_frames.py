#!/usr/bin/env python3
"""Extract evenly spaced, low-redundancy frames from a local LeRobot dataset."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np

# LeRobot may initialize a Hugging Face cache even for local datasets.
os.environ.setdefault("HF_HOME", "/tmp/letools-hf-cache")

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--camera", default="observation.images.head_cam_h")
    parser.add_argument("--count", type=int, default=200)
    parser.add_argument(
        "--per-episode",
        type=int,
        default=0,
        help="If positive, sample this many frames from each selected episode instead of --count global frames.",
    )
    parser.add_argument("--episode-step", type=int, default=1)
    parser.add_argument("--video-backend", default="pyav")
    return parser.parse_args()


def tensor_to_bgr(image) -> np.ndarray:
    array = image.detach().cpu().permute(1, 2, 0).numpy()
    if array.dtype != np.uint8:
        array = np.clip(array * 255.0, 0, 255).astype(np.uint8)
    return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)


def main() -> None:
    args = parse_args()
    if args.count < 1 or args.per_episode < 0 or args.episode_step < 1:
        raise ValueError("--count/--episode-step must be positive and --per-episode must be non-negative")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    dataset = LeRobotDataset(
        repo_id="local/beijing-lingjian",
        root=args.dataset_root,
        video_backend=args.video_backend,
    )
    if args.camera not in dataset.meta.video_keys:
        raise KeyError(f"Unknown camera {args.camera!r}; available: {dataset.meta.video_keys}")

    if args.per_episode:
        sampled_indices: list[int] = []
        for episode_index in range(0, len(dataset.meta.episodes), args.episode_step):
            episode = dataset.meta.episodes[episode_index]
            start = int(episode["dataset_from_index"])
            stop = int(episode["dataset_to_index"]) - 1
            sampled_indices.extend(
                np.linspace(start, stop, min(args.per_episode, stop - start + 1), dtype=np.int64).tolist()
            )
        indices = np.asarray(sampled_indices, dtype=np.int64)
    else:
        indices = np.linspace(0, len(dataset) - 1, min(args.count, len(dataset)), dtype=np.int64)
    manifest_path = args.output_dir / "samples.jsonl"
    records: list[str] = []

    for index in indices:
        item = dataset[int(index)]
        episode = int(item["episode_index"].item())
        frame = int(item["frame_index"].item())
        filename = f"ep{episode:06d}_frame{frame:06d}_index{int(index):09d}.jpg"
        image = tensor_to_bgr(item[args.camera])
        if not cv2.imwrite(str(args.output_dir / filename), image):
            raise OSError(f"Failed to write {filename}")

        record = {
            "file": filename,
            "dataset_index": int(index),
            "episode_index": episode,
            "frame_index": frame,
            "timestamp": float(item["timestamp"].item()),
            "task": item["task"],
            "camera": args.camera,
            "observation_state": item["observation.state"].tolist(),
            "action": item["action"].tolist(),
        }
        records.append(json.dumps(record, ensure_ascii=False))

    manifest_path.write_text("\n".join(records) + "\n", encoding="utf-8")
    print(f"Extracted {len(records)} frames to {args.output_dir}")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
