#!/usr/bin/python3
"""从 WSL 经 SSH 控制 Kuavo 左手、右手或双手：state / close / open / send。

0 表示张开，100 表示收紧位置，不代表力度。close 默认 80；close/open 默认
保留拇指侧摆通道（第 1 项）。动作默认只预览，--execute 才发送；本地不需要 ROS。
"""
from __future__ import annotations

import argparse
from datetime import datetime
import getpass
import json
import os
from pathlib import Path
import shlex
import sys
import threading
import uuid

try:
    from .kuavo_hand_controller import validate_request
except ImportError:
    from kuavo_hand_controller import validate_request


ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "tools/kuavo_hand_controller.py"
DEFAULT_STATE_DIR = ROOT / "outputs/robot_control/kuavo5_hands"


def save_record(directory, result):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    serial = result["action_id"]
    for name, value in (("latest_result.json", result),
                        ("latest_state.json", result["post_state"]),
                        ("actions/" + serial + ".json", result)):
        path = directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + "." + serial + ".tmp")
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
        os.replace(temporary, path)
    with (directory / "history.jsonl").open("a") as stream:
        stream.write(json.dumps(result, ensure_ascii=False) + "\n")


class KuavoRemoteHand:
    """Python API. Each action begins with live feedback and returns its record.

    with KuavoRemoteHand() as hands:
        hands.close("left", level=80, execute=True)
        hands.open("right", execute=True)
        hands.send("left", values=[95, 60, 80, 80, 80, 80], execute=True)
    """
    def __init__(self, host="192.168.26.1", user="lab", port=22, password=None,
                 password_env="KUAVO_SSH_PASSWORD", ros_ip=None,
                 remote_repo="/home/lab/kuavo-ros-opensource",
                 state_dir=DEFAULT_STATE_DIR, connect_timeout_s=8,
                 accept_new_host_key=False, progress=None):
        self.host, self.user, self.port = host, user, port
        self.password, self.password_env = password, password_env
        self.ros_ip, self.remote_repo = ros_ip or host, remote_repo
        self.state_dir, self.connect_timeout_s = Path(state_dir), connect_timeout_s
        self.accept_new_host_key, self.progress = accept_new_host_key, progress
        self.client = None
        self.lock = threading.Lock()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *unused):
        self.disconnect()

    def connect(self):
        if self.client is not None:
            return
        import paramiko
        password = self.password
        if password is None:
            password = os.environ.get(self.password_env)
        if password is None:
            password = getpass.getpass("SSH password for %s@%s: " % (self.user, self.host))
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        policy = paramiko.AutoAddPolicy if self.accept_new_host_key else paramiko.RejectPolicy
        client.set_missing_host_key_policy(policy())
        try:
            client.connect(self.host, port=self.port, username=self.user,
                           password=password, look_for_keys=False, allow_agent=False,
                           timeout=self.connect_timeout_s,
                           banner_timeout=self.connect_timeout_s,
                           auth_timeout=self.connect_timeout_s)
        except Exception:
            client.close()
            raise
        self.client = client
        self.password = None

    def disconnect(self):
        if self.client is not None:
            self.client.close()
            self.client = None

    def _request(self, command, **options):
        request = dict(command=command, **options)
        validate_request(request)
        with self.lock:
            self.connect()
            script = CONTROLLER.read_text()
            inner = "source " + shlex.quote(self.remote_repo + "/devel/setup.bash")
            inner += " && " + shlex.join([
                "env", "-u", "ROS_HOSTNAME",
                "ROS_MASTER_URI=http://%s:11311" % self.host,
                "ROS_IP=" + self.ros_ip, "python3", "-", "--request-json",
                json.dumps(request)])
            shell = "bash --noprofile --norc -c " + shlex.quote(inner)
            timeout = request.get("timeout_s", 15.) * 4 + request.get("duration_s", 2.) + 15
            stdin, stdout, stderr = self.client.exec_command(shell, timeout=timeout)
            # Drain stderr concurrently so a ROS diagnostic cannot fill the SSH
            # window and deadlock while we are reading the result from stdout.
            error_chunks = []
            def drain_errors():
                while True:
                    chunk = stderr.read(4096)
                    if not chunk:
                        return
                    error_chunks.append(chunk)
            error_thread = threading.Thread(target=drain_errors, daemon=True)
            error_thread.start()
            try:
                stdin.write(script)
                stdin.flush()
                stdin.channel.shutdown_write()
                result = None
                other = []
                for line in stdout:
                    if line.startswith("HAND_RESULT:"):
                        result = json.loads(line.split(":", 1)[1])
                    elif line.startswith("HAND_PROGRESS:"):
                        if self.progress is not None:
                            self.progress(json.loads(line.split(":", 1)[1]))
                    else:
                        other.append(line)
                status = stdout.channel.recv_exit_status()
                error_thread.join(timeout=3)
                errors = b"".join(error_chunks).decode("utf-8", "replace")
                if result is None:
                    raise RuntimeError("remote hand command failed (exit %s): %s" %
                                       (status, (errors + "".join(other))[-3000:]))
                result.update(action_id=str(uuid.uuid4()),
                              recorded_at=datetime.now().astimezone().isoformat(),
                              robot_host=self.host, remote_exit_status=status)
                if errors.strip():
                    result["remote_stderr"] = errors
                save_record(self.state_dir, result)
                return result
            finally:
                stdout.channel.close()

    def state(self, timeout_s=15):
        return self._request("state", timeout_s=timeout_s)

    def close(self, hand="left", level=80, duration_s=2., execute=False,
              thumb_rotation=None, timeout_s=15):
        return self._request("close", hand=hand, level=level, duration_s=duration_s,
                             execute=execute, thumb_rotation=thumb_rotation,
                             timeout_s=timeout_s)

    def open(self, hand="left", duration_s=2., execute=False,
             thumb_rotation=None, timeout_s=15):
        return self._request("open", hand=hand, duration_s=duration_s,
                             execute=execute, thumb_rotation=thumb_rotation,
                             timeout_s=timeout_s)

    def send(self, hand="left", values=None, *, left_values=None, right_values=None,
             duration_s=2., execute=False, timeout_s=15):
        return self._request("send", hand=hand, values=values,
                             left_values=left_values, right_values=right_values,
                             duration_s=duration_s, execute=execute, timeout_s=timeout_s)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.26.1")
    parser.add_argument("--user", default="lab")
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--password-env", default="KUAVO_SSH_PASSWORD")
    parser.add_argument("--accept-new-host-key", action="store_true")
    parser.add_argument("--ros-ip", help="机器人端临时 ROS 节点的地址，默认与 --host 相同")
    parser.add_argument("--remote-repo", default="/home/lab/kuavo-ros-opensource")
    parser.add_argument("--state-dir", type=Path, default=DEFAULT_STATE_DIR)
    parser.add_argument("--timeout-s", type=float, default=15)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("state", help="只读查询双手反馈")
    for command, text in (("close", "收紧"), ("open", "张开"), ("send", "发送自定义六维参数")):
        action = sub.add_parser(command, help=text)
        action.add_argument("--hand", choices=("left", "right", "both"), default="left")
        action.add_argument("--duration-s", type=float, default=2)
        action.add_argument("--execute", action="store_true", help="实际发送；省略时只读取反馈并预览")
        if command == "close":
            action.add_argument("--level", type=int, default=80, help="收拢位置 0..100，默认 80，不代表力度")
        if command in ("close", "open"):
            action.add_argument("--thumb-rotation", type=int, help="拇指侧摆通道（第 1 项）0..100，默认保留实时反馈")
        if command == "send":
            action.add_argument("--values", type=int, nargs=6, help="单手的六个通道")
            action.add_argument("--left-values", type=int, nargs=6, help="双手模式中的左手六通道")
            action.add_argument("--right-values", type=int, nargs=6, help="双手模式中的右手六通道")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    request = {"command": args.command, "timeout_s": args.timeout_s}
    for key in ("hand", "duration_s", "execute", "level", "thumb_rotation",
                "values", "left_values", "right_values"):
        if hasattr(args, key):
            request[key] = getattr(args, key)
    try:
        validate_request(request)
    except ValueError as error:
        parser.error(str(error))
    def progress(value):
        print("HAND_PROGRESS:" + json.dumps(value), file=sys.stderr, flush=True)
    try:
        with KuavoRemoteHand(host=args.host, user=args.user, port=args.port,
                             password_env=args.password_env, ros_ip=args.ros_ip,
                             remote_repo=args.remote_repo, state_dir=args.state_dir,
                             accept_new_host_key=args.accept_new_host_key,
                             progress=progress) as hands:
            result = hands._request(**request)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return result["remote_exit_status"]
    except Exception as error:
        print("Hand command failed: " + str(error), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
