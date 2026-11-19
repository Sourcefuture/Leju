#!/usr/bin/env python3
"""Replay one LeRobot v3 episode through the classical detector and write an annotated video."""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict
from pathlib import Path

import av
import cv2
import numpy as np
import yaml

os.environ.setdefault("HF_HOME", "/tmp/letools-hf-cache")

from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata

from kuavo_deploy.traditional.lingjian.classical_detector import annotate, detect


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--episode", type=int, required=True)
    parser.add_argument("--camera", default="observation.images.head_cam_h")
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.yaml"))
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--display", action="store_true")
    return parser.parse_args()


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else 0.0


def main() -> None:
    args = parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    metadata = LeRobotDatasetMetadata("local/beijing-lingjian", root=args.dataset_root)
    if not 0 <= args.episode < metadata.total_episodes:
        raise IndexError(f"episode must be in [0, {metadata.total_episodes - 1}]")
    if args.camera not in metadata.video_keys:
        raise KeyError(f"Unknown camera {args.camera!r}; available: {metadata.video_keys}")

    episode = metadata.episodes[args.episode]
    prefix = f"videos/{args.camera}"
    start_s = float(episode[f"{prefix}/from_timestamp"])
    stop_s = float(episode[f"{prefix}/to_timestamp"])
    video_path = args.dataset_root / metadata.get_video_file_path(args.episode, args.camera)

    fps = float(metadata.fps)
    episode_frames = int(round((stop_s - start_s) * fps))
    if args.max_frames > 0:
        episode_frames = min(episode_frames, args.max_frames)

    container = av.open(str(video_path))
    stream = container.streams.video[0]
    stream.thread_type = "AUTO"
    width, height = stream.width, stream.height
    seek_offset = int(start_s / float(stream.time_base))
    container.seek(seek_offset, stream=stream, backward=True, any_frame=False)

    args.output_video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise OSError(f"Unable to create output video: {args.output_video}")

    latencies_ms: list[float] = []
    records: list[str] = []
    processed = 0
    try:
        for frame in container.decode(stream):
            frame_time = float(frame.pts * stream.time_base)
            if frame_time + 0.5 / fps < start_s:
                continue
            if frame_time >= stop_s or processed >= episode_frames:
                break
            local_frame = processed
            image = frame.to_ndarray(format="bgr24")
            begin = time.perf_counter()
            detections, _ = detect(image, config)
            latency_ms = (time.perf_counter() - begin) * 1000.0
            latencies_ms.append(latency_ms)

            output = annotate(image, detections)
            yellow_count = sum(item.label == "yellow_part" for item in detections)
            bin_count = sum(item.label == "blue_bin" for item in detections)
            status = f"ep={args.episode} frame={local_frame} bins={bin_count} yellow={yellow_count} {latency_ms:.1f}ms"
            cv2.putText(output, status, (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
            writer.write(output)

            records.append(
                json.dumps(
                    {
                        "episode": args.episode,
                        "frame": local_frame,
                        "timestamp_s": round(local_frame / fps, 4),
                        "latency_ms": round(latency_ms, 3),
                        "detections": [asdict(item) for item in detections],
                    },
                    ensure_ascii=False,
                )
            )
            processed += 1

            if args.display:
                cv2.imshow("lingjian replay", output)
                if cv2.waitKey(max(1, int(round(1000 / fps)))) & 0xFF in (27, ord("q")):
                    break
    finally:
        container.close()
        writer.release()
        if args.display:
            cv2.destroyAllWindows()

    result_path = args.output_video.with_suffix(".jsonl")
    result_path.write_text("\n".join(records) + ("\n" if records else ""), encoding="utf-8")
    summary = {
        "episode": args.episode,
        "source_video": str(video_path),
        "processed_frames": processed,
        "fps": fps,
        "detector_latency_ms_p50": round(percentile(latencies_ms, 50), 3),
        "detector_latency_ms_p95": round(percentile(latencies_ms, 95), 3),
    }
    args.output_video.with_suffix(".summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
