#!/usr/bin/env bash
# Read-only readiness check for Kuavo arm teleoperation.

set -u

MASTER_URI="${ROS_MASTER_URI:-http://192.168.26.1:11311}"
LOCAL_IP="${ROS_IP:-}"

usage() {
  cat <<'EOF'
Usage: tools/check_kuavo_arm_control.sh [--master URI] [--ip LOCAL_IP]

This script only reads the ROS graph. It never publishes a topic or calls a
service that can move the robot.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --master)
      MASTER_URI="$2"
      shift 2
      ;;
    --ip)
      LOCAL_IP="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v rosnode >/dev/null 2>&1 || ! command -v rostopic >/dev/null 2>&1; then
  echo "FAIL ROS command-line tools are unavailable. Source the robot ROS workspace first."
  exit 1
fi

export ROS_MASTER_URI="$MASTER_URI"
if [[ -n "$LOCAL_IP" ]]; then
  export ROS_IP="$LOCAL_IP"
fi

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=${ROS_IP:-<unset>}"

if ! nodes="$(timeout 5 rosnode list 2>&1)"; then
  echo "FAIL Cannot reach the ROS master: $nodes"
  exit 1
fi

node_count="$(printf '%s\n' "$nodes" | sed '/^[[:space:]]*$/d' | wc -l)"
echo "PASS ROS master reachable ($node_count nodes)"

failures=0

check_topic() {
  local topic="$1"
  local requirement="$2"
  local info

  if ! info="$(timeout 5 rostopic info "$topic" 2>&1)"; then
    echo "FAIL $topic is unavailable"
    failures=$((failures + 1))
    return
  fi

  local type publishers subscribers
  type="$(printf '%s\n' "$info" | sed -n 's/^Type: //p')"
  publishers="$(printf '%s\n' "$info" | sed -n '/^Publishers:/,/^Subscribers:/p' | sed '1d;$d' | sed '/^[[:space:]]*$/d')"
  subscribers="$(printf '%s\n' "$info" | sed -n '/^Subscribers:/,$p' | sed '1d' | sed '/^[[:space:]]*$/d')"

  echo "INFO $topic type=${type:-unknown}"
  printf '     publishers: %s\n' "${publishers:-None}"
  printf '     subscribers: %s\n' "${subscribers:-None}"

  case "$requirement" in
    publisher)
      if [[ -z "$publishers" || "$publishers" == *"None"* ]]; then
        echo "FAIL $topic has no state publisher"
        failures=$((failures + 1))
      else
        echo "PASS $topic has a state publisher"
      fi
      ;;
    subscriber)
      if [[ -z "$subscribers" || "$subscribers" == *"None"* ]]; then
        echo "FAIL $topic has no controller subscriber"
        failures=$((failures + 1))
      else
        echo "PASS $topic has a controller subscriber"
      fi
      ;;
  esac
}

check_topic /sensors_data_raw publisher
check_topic /kuavo_arm_traj subscriber
check_topic /kuavo_arm_target_poses subscriber

echo "INFO End-effector topics (optional during arm-only tests):"
for topic in /leju_claw_state /leju_claw_command; do
  if timeout 3 rostopic info "$topic" >/dev/null 2>&1; then
    echo "PASS $topic is present"
  else
    echo "WARN $topic is absent"
  fi
done

if (( failures > 0 )); then
  echo "RESULT NOT READY ($failures required checks failed)"
  exit 1
fi

echo "RESULT ROS GRAPH READY"
echo "NOTE Verify stance, collision checking, fresh joint feedback, and a single command publisher before enabling motion."
