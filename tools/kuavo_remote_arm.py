#!/usr/bin/python3
"""Run the Kuavo arm controller on the robot through SSH.

This launcher runs on WSL.  It uploads ``kuavo_arm_controller.py`` to the
robot, sources the robot's ROS workspace, executes the selected command, and
streams the remote output back to the local terminal.  Commands plan by
default; real movement still requires the explicit ``--execute`` option.
"""

from __future__ import annotations

import argparse
import getpass
import json
import math
import os
from pathlib import Path
import re
import shlex
import socket
import sys
import uuid

import paramiko

try:
    from .kuavo_arm_controller import StateStore, now_iso, saved_pose_target
except ImportError:
    from kuavo_arm_controller import StateStore, now_iso, saved_pose_target


ROOT = Path(__file__).resolve().parents[1]
LOCAL_CONTROLLER = ROOT / "tools/kuavo_arm_controller.py"
DEFAULT_POSE_DIR = ROOT / "outputs/robot_control/pose_library"
DEFAULT_REPLAY_STATE_DIR = ROOT / "outputs/robot_control/saved_pose_replay"


def saved_pose_path(directory, name):
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]{0,63}", name):
        raise ValueError("pose name must use 1..64 letters, digits, '_' or '-'")
    return Path(directory) / (name + ".json")


def save_arm_pose(directory, name, arm, state, description=""):
    path = saved_pose_path(directory, name)
    if arm not in ("left", "right", "both"):
        raise ValueError("invalid arm")
    values = [float(v) for v in state.get("arm14_deg", [])]
    if len(values) != 14 or not all(math.isfinite(v) for v in values):
        raise ValueError("capture must have 14 finite feedback angles")
    pose = {"schema_version": 1, "pose_type": "kuavo_saved_arm_pose",
            "name": name, "description": description, "arm": arm,
            "robot": state["robot"], "robot_version": state.get("robot_version"),
            "reference_frame": state.get("reference_frame", "base_link"),
            "recorded_at": state.get("recorded_at", now_iso()),
            "feedback_stamp": state["feedback_stamp"], "source": state["source"],
            "saved_state": state, "restore_finger_state": False}
    if arm == "both":
        pose.update(arm14_deg=values, arm14_rad=[math.radians(v) for v in values],
                    eef={side: state.get(side + "_eef") for side in ("left", "right")})
    else:
        selected = values[:7] if arm == "left" else values[7:]
        pose.update(arm7_deg=selected, arm7_rad=[math.radians(v) for v in selected],
                    eef=state.get(arm + "_eef"))
    saved_pose_target(pose, values)  # Validate the stored representation.
    path.parent.mkdir(parents=True, exist_ok=True)
    archive = path.parent / ".versions"
    archive.mkdir(exist_ok=True)
    serial = uuid.uuid4().hex
    if path.exists():
        (archive / (name + "_previous_" + serial + ".json")).write_text(path.read_text())
    data = json.dumps(pose, ensure_ascii=False, indent=2) + "\n"
    (archive / (name + "_" + serial + ".json")).write_text(data)
    temporary = path.with_name(path.name + "." + serial + ".tmp")
    temporary.write_text(data)
    os.replace(temporary, path)
    return pose, path


def list_saved_poses(directory):
    poses = []
    for path in sorted(Path(directory).glob("*.json")):
        try:
            value = json.loads(path.read_text())
        except (ValueError, OSError):
            continue
        if isinstance(value, dict) and value.get("pose_type") == "kuavo_saved_arm_pose":
            poses.append({key: value.get(key) for key in
                          ("name", "description", "arm", "robot_version", "recorded_at")})
            poses[-1]["path"] = str(path)
    return poses


def parse_remote_json(output):
    decoder = json.JSONDecoder()
    offset = 0
    for line in output.splitlines(keepends=True):
        if line.lstrip().startswith("{"):
            try:
                value, _ = decoder.raw_decode(output, offset + len(line) - len(line.lstrip()))
                if isinstance(value, dict) and value.get("schema_version") == 1:
                    return value
            except ValueError:
                pass
        offset += len(line)
    raise ValueError("robot returned no arm state/action JSON")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.26.1")
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--user", default="lab")
    parser.add_argument(
        "--password-env",
        default="KUAVO_SSH_PASSWORD",
        help="environment variable containing the SSH password; prompt if unset",
    )
    parser.add_argument(
        "--accept-new-host-key",
        action="store_true",
        help="trust and store a previously unknown host key",
    )
    parser.add_argument("--connect-timeout-s", type=float, default=8.0)
    parser.add_argument("--command-timeout-s", type=float, default=120.0)
    parser.add_argument(
        "--remote-repo", default="/home/lab/kuavo-ros-opensource"
    )
    parser.add_argument(
        "--remote-script", default="/tmp/kuavo_arm_controller.py"
    )
    parser.add_argument(
        "--remote-state-dir", default="/tmp/kuavo5_arm_state"
    )
    parser.add_argument(
        "--remote-ros-ip",
        default="192.168.26.1",
        help="IPv4 address advertised by the ROS node running on the robot",
    )
    parser.add_argument(
        "--no-upload",
        action="store_true",
        help="reuse the remote script instead of uploading the local copy",
    )
    parser.add_argument("--pose-dir", type=Path, default=DEFAULT_POSE_DIR)
    parser.add_argument("--local-state-dir", type=Path, default=DEFAULT_REPLAY_STATE_DIR)

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("state", help="read current arm state; never moves")
    doctor = subparsers.add_parser("doctor", help="read-only ROS connection diagnostics")
    doctor.add_argument("--wait-s", type=float, default=30.0)
    subparsers.add_parser("list-poses", help="show standard 14D poses")

    joints = subparsers.add_parser("joints", help="explicit 14D degree target")
    joints.add_argument("--values", nargs=14, type=float, required=True)
    joints.add_argument("--duration-s", type=float, default=3.0)
    joints.add_argument("--execute", action="store_true")

    pose = subparsers.add_parser("pose", help="standard named pose")
    pose.add_argument(
        "name",
        choices=(
            "arms_down",
            "ready",
            "arms_horizontal_palms_down",
            "elbows_90_palms_down",
        ),
    )
    pose.add_argument("--arm", choices=("left", "right", "both"), default="both")
    pose.add_argument("--duration-s", type=float, default=3.0)
    pose.add_argument("--execute", action="store_true")

    move = subparsers.add_parser("move", help="relative Cartesian arm step")
    move.add_argument("--arm", choices=("left", "right"), required=True)
    move.add_argument(
        "--direction",
        choices=("forward", "backward", "left", "right", "up", "down"),
        required=True,
    )
    move.add_argument("--step-m", type=float, default=0.02)
    move.add_argument("--duration-s", type=float, default=1.0)
    move.add_argument("--execute", action="store_true")

    subparsers.add_parser("list-saved-poses", help="list saved real-arm poses without connecting")
    save = subparsers.add_parser("save-pose", help="record fresh feedback without moving")
    save.add_argument("name")
    save.add_argument("--arm", choices=("left", "right", "both"), required=True)
    save.add_argument("--description", default="")

    saved = subparsers.add_parser("goto-pose", help="restore a saved pose; preview by default")
    saved.add_argument("name")
    saved.add_argument("--arm", choices=("left", "right", "both"),
                       help="defaults to saved arm; a single-arm pose cannot be used by the opposite arm")
    saved.add_argument("--duration-s", type=float, default=6.)
    saved.add_argument("--max-joint-speed-deg-s", type=float, default=5.)
    saved.add_argument("--execute", action="store_true")
    return parser


def password_for(args):
    password = os.environ.get(args.password_env)
    if password is not None:
        return password
    return getpass.getpass(
        f"SSH password for {args.user}@{args.host} "
        f"(or set {args.password_env}): "
    )


def controller_arguments(args):
    result = [
        "--robot",
        "5",
        "--state-dir",
        args.remote_state_dir,
        "state" if args.command == "save-pose" else args.command,
    ]
    if args.command == "joints":
        result += ["--values", *[str(value) for value in args.values]]
        result += ["--duration-s", str(args.duration_s)]
        if args.execute:
            result.append("--execute")
    elif args.command == "pose":
        result += [args.name, "--arm", args.arm, "--duration-s", str(args.duration_s)]
        if args.execute:
            result.append("--execute")
    elif args.command == "move":
        result += [
            "--arm",
            args.arm,
            "--direction",
            args.direction,
            "--step-m",
            str(args.step_m),
            "--duration-s",
            str(args.duration_s),
        ]
        if args.execute:
            result.append("--execute")
    elif args.command == "doctor":
        result += ["--wait-s", str(args.wait_s)]
    elif args.command == "goto-pose":
        result += ["--pose-json", json.dumps(args.saved_pose, ensure_ascii=True),
                   "--duration-s", str(args.duration_s),
                   "--max-joint-speed-deg-s", str(args.max_joint_speed_deg_s)]
        if args.arm is not None:
            result += ["--arm", args.arm]
        if args.execute:
            result.append("--execute")
    return result


def upload_controller(client, local_path, remote_path):
    temporary = remote_path + ".upload"
    with client.open_sftp() as sftp:
        sftp.put(str(local_path), temporary)
        sftp.chmod(temporary, 0o755)
        try:
            # OpenSSH supports an atomic overwrite extension.
            sftp.posix_rename(temporary, remote_path)
        except (AttributeError, IOError):
            # Older SFTP servers only provide the standard rename operation,
            # which cannot replace an existing destination.
            try:
                sftp.remove(remote_path)
            except IOError:
                pass
            sftp.rename(temporary, remote_path)


def remote_command(args):
    controller = ["python3", args.remote_script, *controller_arguments(args)]
    shell_script = " && ".join(
        [
            f"cd {shlex.quote(args.remote_repo)}",
            "source devel/setup.bash",
            "export ROS_MASTER_URI=" + shlex.quote(f"http://{args.host}:11311"),
            f"export ROS_IP={shlex.quote(args.remote_ros_ip)}",
            "unset ROS_HOSTNAME",
            shlex.join(controller),
        ]
    )
    return "bash --noprofile --norc -c " + shlex.quote(shell_script)


def connect(args, password):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    if args.accept_new_host_key:
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    else:
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(
        hostname=args.host,
        port=args.port,
        username=args.user,
        password=password,
        look_for_keys=False,
        allow_agent=False,
        timeout=args.connect_timeout_s,
        banner_timeout=args.connect_timeout_s,
        auth_timeout=args.connect_timeout_s,
    )
    return client


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "list-saved-poses":
        print(json.dumps(list_saved_poses(args.pose_dir), ensure_ascii=False, indent=2))
        return 0
    if args.command in ("save-pose", "goto-pose"):
        try:
            path = saved_pose_path(args.pose_dir, args.name)
            if args.command == "goto-pose":
                args.saved_pose = json.loads(path.read_text())
                saved_pose_target(args.saved_pose, [0.] * 14, args.arm)
                if not math.isfinite(args.duration_s) or args.duration_s <= 0:
                    raise ValueError("duration must be positive and finite")
                if not math.isfinite(args.max_joint_speed_deg_s) or not 0 < args.max_joint_speed_deg_s <= 25:
                    raise ValueError("max joint speed must be in (0,25] degrees/s")
        except (OSError, ValueError) as error:
            parser.error(str(error))
    if not LOCAL_CONTROLLER.is_file() and not args.no_upload:
        raise SystemExit(f"local controller not found: {LOCAL_CONTROLLER}")

    password = password_for(args)
    client = None
    try:
        client = connect(args, password)
        if not args.no_upload:
            upload_controller(client, LOCAL_CONTROLLER, args.remote_script)

        command = remote_command(args)
        if getattr(args, "execute", False):
            print(
                "[kuavo] Motion was explicitly requested. The remote controller may "
                "wait up to 30 seconds for TCPROS connections before movement starts.",
                file=sys.stderr,
                flush=True,
            )
        stdin, stdout, stderr = client.exec_command(
            command, timeout=args.command_timeout_s
        )
        stdin.close()
        exit_status = stdout.channel.recv_exit_status()
        remote_stdout = stdout.read().decode("utf-8", errors="replace")
        remote_stderr = stderr.read().decode("utf-8", errors="replace")
        if args.command == "save-pose" and exit_status == 0:
            state = parse_remote_json(remote_stdout)
            pose, path = save_arm_pose(args.pose_dir, args.name, args.arm, state, args.description)
            print(json.dumps({"saved_pose_file": str(path), "pose": pose,
                              "motion_commands_sent": False}, ensure_ascii=False, indent=2))
        elif args.command == "goto-pose" and exit_status == 0:
            record = parse_remote_json(remote_stdout)
            record.update(saved_pose_file=str(path), robot_host=args.host)
            store = StateStore(args.local_state_dir)
            store.append_history(record)
            store.write_latest(record.get("post_state") or record["pre_state"])
            print(json.dumps(record, ensure_ascii=False, indent=2))
            if args.execute and record.get("status") != "executed":
                exit_status = 3
        elif remote_stdout:
            print(remote_stdout, end="")
        if remote_stderr:
            print(remote_stderr, end="", file=sys.stderr)
        return exit_status
    except paramiko.BadHostKeyException as error:
        print(f"SSH host key mismatch: {error}", file=sys.stderr)
        return 10
    except paramiko.AuthenticationException:
        print("SSH authentication failed", file=sys.stderr)
        return 11
    except ValueError as error:
        print(f"Arm pose data error: {error}", file=sys.stderr)
        return 13
    except (paramiko.SSHException, socket.timeout, OSError) as error:
        print(f"SSH execution failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 12
    finally:
        password = None
        if client is not None:
            client.close()


if __name__ == "__main__":
    sys.exit(main())
