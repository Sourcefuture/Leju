#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="/home/source/miniforge3/envs/letools/bin/python"

cd "${ROOT_DIR}"
exec "${PYTHON_BIN}" tools/kuavo_arm_controller.py --backend mujoco "$@"
