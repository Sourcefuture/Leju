#!/usr/bin/env bash
# Local-only scene, kinematic arm control, and isolated scale validation.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="/home/source/miniforge3/envs/letools/bin/python"
SCENE_DIR="${ROOT_DIR}/outputs/kuavo_sim/weighing"
STATE_DIR="${ROOT_DIR}/outputs/robot_control/kuavo5_weighing"
cd "${ROOT_DIR}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Missing letools Python: ${PYTHON_BIN}" >&2
  exit 1
fi
command_name="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "${command_name}" in
  build)
    exec "${PYTHON_BIN}" tools/prepare_kuavo5_weighing_scene.py "$@" ;;
  gui|ctl|render|scale-test|camera)
    if [[ ! -f "${SCENE_DIR}/scene.xml" ]]; then
      echo "Build first: bash tools/kuavo_weighing.sh build" >&2
      exit 1
    fi
    case "${command_name}" in
      gui)
        export MUJOCO_GL=glfw
        exec "${PYTHON_BIN}" -u tools/kuavo_mujoco_live_viewer.py \
          --scene "${SCENE_DIR}/scene.xml" --state-dir "${STATE_DIR}" \
          --camera weighing_overview "$@" ;;
      ctl)
        exec "${PYTHON_BIN}" tools/kuavo_arm_controller.py --backend mujoco \
          --scene "${SCENE_DIR}/scene.xml" --state-dir "${STATE_DIR}" "$@" ;;
      camera)
        export MUJOCO_GL=egl
        exec "${PYTHON_BIN}" tools/kuavo_sim_camera.py \
          --scene "${SCENE_DIR}/scene.xml" --state-dir "${STATE_DIR}" "$@" ;;
      render|scale-test)
        export MUJOCO_GL=egl
        exec "${PYTHON_BIN}" tools/kuavo_weighing_scene.py "${command_name}" "$@" ;;
    esac ;;
  help|--help|-h)
    cat <<'EOF'
Usage: bash tools/kuavo_weighing.sh COMMAND [options]
  build [--on-scale empty|a|b|c]     Generate Kuavo-5.2 scene and scale bench
  gui                              Open scene viewer (terminal A)
  ctl [--reset-sim] pose ready --execute
  ctl move --arm left --direction up --step-m 0.02 --execute
  ctl state                        Read saved local simulation arm state
  render                           Save overview and top-view PNGs
  camera --camera head --live       Live simulated head RGB (s saves RGB-D)
  camera --camera all               Save head/left/right RGB-D snapshots
  camera --list                     List MJCF cameras
  scale-test                       Check 0/0.2/0.5/1 kg load-cell readings
Only local MuJoCo is used. No SSH or ROS control messages are sent.
EOF
    ;;
  *) echo "Unknown command: ${command_name}" >&2; exit 2 ;;
esac
