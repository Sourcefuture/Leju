#!/usr/bin/env python3
"""Create CVAT train/validation tasks and upload split image directories."""

from __future__ import annotations

import argparse
import os
import time
from contextlib import ExitStack
from pathlib import Path

import requests


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://localhost:8080")
    parser.add_argument("--username", required=True)
    parser.add_argument("--password-env", default="CVAT_PASSWORD")
    parser.add_argument("--project-id", type=int, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--task-prefix", default="lingjian_parts_v1")
    parser.add_argument("--segment-size", type=int, default=100)
    parser.add_argument("--timeout", type=int, default=900)
    return parser.parse_args()


def checked(response: requests.Response) -> requests.Response:
    if not response.ok:
        raise RuntimeError(
            f"CVAT returned HTTP {response.status_code} for {response.request.method} "
            f"{response.request.url}: {response.text[:1000]}"
        )
    return response


def wait_for_request(
    session: requests.Session,
    base_url: str,
    headers: dict[str, str],
    request_id: str,
    timeout: int,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = checked(
            session.get(f"{base_url}/api/requests/{request_id}", headers=headers, timeout=30)
        )
        payload = response.json()
        status = payload["status"]
        progress = payload.get("progress")
        print(f"  CVAT request {request_id}: {status}, progress={progress}", flush=True)
        if status == "finished":
            return
        if status == "failed":
            raise RuntimeError(f"CVAT processing failed: {payload.get('message', '')}")
        time.sleep(2)
    raise TimeoutError(f"CVAT request {request_id} did not finish within {timeout}s")


def upload_images(
    session: requests.Session,
    base_url: str,
    headers: dict[str, str],
    task_id: int,
    image_paths: list[Path],
    timeout: int,
) -> str:
    """Use CVAT's start/multiple/finish upload protocol for client files."""
    data_url = f"{base_url}/api/tasks/{task_id}/data/"
    checked(
        session.post(
            data_url,
            headers={**headers, "Upload-Start": ""},
            timeout=30,
        )
    )

    with ExitStack() as stack:
        files = [
            (
                f"client_files[{index}]",
                (path.name, stack.enter_context(path.open("rb")), "image/jpeg"),
            )
            for index, path in enumerate(image_paths)
        ]
        checked(
            session.post(
                data_url,
                headers={**headers, "Upload-Multiple": ""},
                data={"image_quality": "95"},
                files=files,
                timeout=timeout,
            )
        )

    finish = checked(
        session.post(
            data_url,
            headers={**headers, "Upload-Finish": ""},
            files={
                "image_quality": (None, "95"),
                "sorting_method": (None, "natural"),
                "use_zip_chunks": (None, "true"),
            },
            timeout=30,
        )
    )
    return str(finish.json()["rq_id"])


def main() -> None:
    args = parse_args()
    password = os.environ.get(args.password_env)
    if not password:
        raise RuntimeError(f"Set {args.password_env} without placing the password in command arguments")

    base_url = args.url.rstrip("/")
    session = requests.Session()
    login = checked(
        session.post(
            f"{base_url}/api/auth/login",
            json={"username": args.username, "password": password},
            timeout=30,
        )
    )
    headers = {"Authorization": f"Token {login.json()['key']}"}

    project = checked(
        session.get(f"{base_url}/api/projects/{args.project_id}", headers=headers, timeout=30)
    ).json()
    print(f"Using CVAT project {project['id']}: {project['name']}")

    existing_response = checked(
        session.get(
            f"{base_url}/api/tasks",
            headers=headers,
            params={"project_id": args.project_id, "page_size": 100},
            timeout=30,
        )
    ).json()
    existing = {task["name"]: task for task in existing_response.get("results", [])}

    for split in ("train", "validation"):
        task_name = f"{args.task_prefix}_{split}"
        image_dir = args.dataset_root / "images" / split
        image_paths = sorted(image_dir.glob("*.jpg"))
        if not image_paths:
            raise FileNotFoundError(f"No JPEG images found in {image_dir}")

        if task_name in existing:
            task = checked(
                session.get(
                    f"{base_url}/api/tasks/{existing[task_name]['id']}",
                    headers=headers,
                    timeout=30,
                )
            ).json()
            if task.get("size", 0) == len(image_paths):
                print(
                    f"Already ready: task={task['id']}, subset={task.get('subset')}, "
                    f"size={task['size']}; skipping upload",
                    flush=True,
                )
                continue
            if task.get("size", 0) != 0:
                raise RuntimeError(
                    f"Task {task_name!r} has {task['size']} frames, expected {len(image_paths)}; "
                    "refusing to modify it"
                )
            task_id = task["id"]
            print(f"Reusing empty task {task_id}: {task_name}", flush=True)
        else:
            task_payload = {
                "name": task_name,
                "project_id": args.project_id,
                "subset": split,
                "segment_size": args.segment_size,
                "overlap": 0,
            }
            task = checked(
                session.post(
                    f"{base_url}/api/tasks",
                    headers=headers,
                    json=task_payload,
                    timeout=30,
                )
            ).json()
            task_id = task["id"]
            print(f"Created task {task_id}: {task_name}", flush=True)

        print(f"Uploading {len(image_paths)} images", flush=True)
        request_id = upload_images(
            session, base_url, headers, task_id, image_paths, args.timeout
        )
        wait_for_request(session, base_url, headers, request_id, args.timeout)
        task = checked(
            session.get(f"{base_url}/api/tasks/{task_id}", headers=headers, timeout=30)
        ).json()
        print(
            f"Ready: task={task_id}, subset={task.get('subset')}, size={task.get('size')}, "
            f"jobs={task.get('jobs', {}).get('count')}",
            flush=True,
        )


if __name__ == "__main__":
    main()
