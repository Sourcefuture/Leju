#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="/home/source/miniforge3/envs/letools/bin/python"
SCENE_DIR="${ROOT_DIR}/outputs/kuavo_sim/tabletop_calibration"
STATE_DIR="${ROOT_DIR}/outputs/robot_control/kuavo5_tabletop"
# Respect explicit backends instead of silently forcing EGL. CPU rendering is
# useful on WSL installations whose DRI device is unavailable to the user.
OFFSCREEN_BACKEND="${KUAVO_RENDER_BACKEND:-${MUJOCO_GL:-egl}}"
if [[ "${KUAVO_SOFTWARE_RENDER:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=true
  export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
fi
export PYTHONUNBUFFERED=1
export PYTHONFAULTHANDLER=1
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
cd "${ROOT_DIR}"
command_name="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "${command_name}" in
  build)
    export MUJOCO_GL="${OFFSCREEN_BACKEND}"
    exec "${PYTHON_BIN}" tools/prepare_kuavo5_tabletop_scene.py --render "$@" ;;
  gui)
    export MUJOCO_GL=glfw
    exec "${PYTHON_BIN}" -u tools/kuavo_mujoco_live_viewer.py \
      --scene "${SCENE_DIR}/scene.xml" --state-dir "${STATE_DIR}" --camera weighing_overview "$@" ;;
  camera)
    export MUJOCO_GL="${OFFSCREEN_BACKEND}"
    exec "${PYTHON_BIN}" tools/kuavo_sim_camera.py \
      --scene "${SCENE_DIR}/scene.xml" --state-dir "${STATE_DIR}" \
      --output "${SCENE_DIR}/captures" "$@" ;;
  render)
    export MUJOCO_GL="${OFFSCREEN_BACKEND}"
    exec "${PYTHON_BIN}" tools/prepare_kuavo5_tabletop_scene.py --preview-only "$@" ;;
  ctl)
    exec "${PYTHON_BIN}" tools/kuavo_arm_controller.py --backend mujoco \
      --scene "${SCENE_DIR}/scene.xml" --state-dir "${STATE_DIR}" "$@" ;;
  planar)
    export MUJOCO_GL="${OFFSCREEN_BACKEND}"
    exec "${PYTHON_BIN}" tools/kuavo_planar_task.py \
      --scene "${SCENE_DIR}/scene.xml" --state-dir "${STATE_DIR}" "$@" ;;
  closed-loop)
    export MUJOCO_GL="${OFFSCREEN_BACKEND}"
    if "${PYTHON_BIN}" tools/kuavo_planar_closed_loop.py "$@"; then
      printf '[closed-loop] exit code: 0\n'
    else
      result_code=$?
      printf '[closed-loop] exit code: %s\n' "$result_code" >&2
      exit "$result_code"
    fi ;;
  calibration)
    export MUJOCO_GL="${OFFSCREEN_BACKEND}"
    exec "${PYTHON_BIN}" tools/kuavo_manual_calibration.py "$@" ;;
  corner-check)
    exec "${PYTHON_BIN}" tools/kuavo_corner_check.py "$@" ;;
  scene-editor)
    export MUJOCO_GL="${OFFSCREEN_BACKEND}"
    exec "${PYTHON_BIN}" tools/kuavo_scene_editor.py "$@" ;;
  help|--help|-h)
    cat <<'EOF'
Local Kuavo-5.2 competition tabletop (120 x 60 cm)
  bash tools/kuavo_tabletop.sh build [--table-height 0.76] [--table-gap 0.22]
  bash tools/kuavo_tabletop.sh gui
  bash tools/kuavo_tabletop.sh gui --camera weighing_top
  bash tools/kuavo_tabletop.sh gui --scene SCENE.xml --state-dir STATE --replay
  bash tools/kuavo_tabletop.sh camera --camera head --live
  bash tools/kuavo_tabletop.sh render
  bash tools/kuavo_tabletop.sh ctl state
  bash tools/kuavo_tabletop.sh planar init
  bash tools/kuavo_tabletop.sh planar audit
  bash tools/kuavo_tabletop.sh planar approach A --execute
  bash tools/kuavo_tabletop.sh planar record A --confirm
  bash tools/kuavo_tabletop.sh planar fit
  bash tools/kuavo_tabletop.sh planar import-calibration CORNERS.json
  bash tools/kuavo_tabletop.sh planar cycle --rgb RGB.png [--execute]
  bash tools/kuavo_tabletop.sh closed-loop --scene SCENE.xml --corners CORNERS.json --state-dir NEW_STATE [--execute]
  bash tools/kuavo_tabletop.sh calibration --scene SCENE.xml --state-dir CAL_DIR gui --image RGB.png
  bash tools/kuavo_tabletop.sh calibration --scene SCENE.xml --state-dir CAL_DIR record-tcp A --arm left --confirm
  bash tools/kuavo_tabletop.sh corner-check prepare
  bash tools/kuavo_tabletop.sh corner-check preview R1
  bash tools/kuavo_tabletop.sh corner-check jog --arm left --delta .01 0 0 --execute
  bash tools/kuavo_tabletop.sh corner-check confirm R1 --confirm
  bash tools/kuavo_tabletop.sh corner-check archive-ab --output NEW_ARCHIVE_DIR
  bash tools/kuavo_tabletop.sh scene-editor --source-scene SCENE.xml --config EDIT.json --output NEW_SCENE_DIR
Use KUAVO_SOFTWARE_RENDER=1 for CPU rendering on restricted WSL DRI devices.
Tasks accept supplied known corner XYZ; manual recording requires confirmation. Local simulation only.
EOF
    ;;
  *) echo "Unknown command: ${command_name}" >&2; exit 2 ;;
esac
