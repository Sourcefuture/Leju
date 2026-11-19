#!/usr/bin/env python3
"""Run a custom YOLO or prompted YOLO-World model on one LeRobot episode."""

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

os.environ.setdefault("HF_HOME", "/tmp/letools-hf-cache")
os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/letools-yolo-config")

from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata

from kuavo_deploy.traditional.lingjian.yolo_backend import YoloDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--episode", type=int, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--camera", default="observation.images.head_cam_h")
    parser.add_argument(
        "--prompts",
        nargs="*",
        help="Enable YOLO-World with these English prompts; omit for a custom/closed-set checkpoint.",
    )
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--max-frames", type=int, default=0)
    return parser.parse_args()


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else 0.0


def main() -> None:
    args = parse_args()
    metadata = LeRobotDatasetMetadata("local/beijing-lingjian", root=args.dataset_root)
    if not 0 <= args.episode < metadata.total_episodes:
        raise IndexError(f"episode must be in [0, {metadata.total_episodes - 1}]")
    if args.camera not in metadata.video_keys:
        raise KeyError(f"Unknown camera {args.camera!r}; available: {metadata.video_keys}")

    detector = YoloDetector(
        args.model,
        prompts=args.prompts,
        confidence=args.confidence,
        iou=args.iou,
        image_size=args.image_size,
        device=args.device,
    )

    episode = metadata.episodes[args.episode]
    prefix = f"videos/{args.camera}"
    start_s = float(episode[f"{prefix}/from_timestamp"])
    stop_s = float(episode[f"{prefix}/to_timestamp"])
    source_path = args.dataset_root / metadata.get_video_file_path(args.episode, args.camera)
    fps = float(metadata.fps)
    frame_limit = int(round((stop_s - start_s) * fps))
    if args.max_frames > 0:
        frame_limit = min(frame_limit, args.max_frames)

    container = av.open(str(source_path))
    stream = container.streams.video[0]
    stream.thread_type = "AUTO"
    container.seek(int(start_s / float(stream.time_base)), stream=stream, backward=True, any_frame=False)

    args.output_video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (stream.width, stream.height),
    )
    if not writer.isOpened():
        raise OSError(f"Unable to create output video: {args.output_video}")

    records: list[str] = []
    wall_latencies_ms: list[float] = []
    processed = 0
    try:
        for frame in container.decode(stream):
            frame_time = float(frame.pts * stream.time_base)
            if frame_time + 0.5 / fps < start_s:
                continue
            if frame_time >= stop_s or processed >= frame_limit:
                break

            image = frame.to_ndarray(format="bgr24")
            begin = time.perf_counter()
            detections, annotated, model_speed = detector.predict(image)
            wall_ms = (time.perf_counter() - begin) * 1000.0
            wall_latencies_ms.append(wall_ms)
            cv2.putText(
                annotated,
                f"ep={args.episode} frame={processed} detections={len(detections)} wall={wall_ms:.1f}ms",
                (10, annotated.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
            writer.write(annotated)
            records.append(
                json.dumps(
                    {
                        "episode": args.episode,
                        "frame": processed,
                        "timestamp_s": round(processed / fps, 4),
                        "wall_latency_ms": round(wall_ms, 3),
                        "model_speed_ms": model_speed,
                        "detections": [asdict(item) for item in detections],
                    },
                    ensure_ascii=False,
                )
            )
            processed += 1
    finally:
        container.close()
        writer.release()

    result_path = args.output_video.with_suffix(".jsonl")
    result_path.write_text("\n".join(records) + ("\n" if records else ""), encoding="utf-8")
    summary = {
        "episode": args.episode,
        "source_video": str(source_path),
        "model": str(args.model),
        "prompts": args.prompts or [],
        "processed_frames": processed,
        "fps": fps,
        "wall_latency_ms_p50": round(percentile(wall_latencies_ms, 50), 3),
        "wall_latency_ms_p95": round(percentile(wall_latencies_ms, 95), 3),
    }
    args.output_video.with_suffix(".summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
