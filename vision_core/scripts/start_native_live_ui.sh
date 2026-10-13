#!/usr/bin/env bash
# Manual desktop entry point: executing this script opts in to the real camera.
set -eo pipefail
if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: bash start_native_live_ui.sh [additional UI options]'
  echo 'Starts real D435i only when manually run in the Ubuntu desktop.'
  echo 'MTC_VISION_VENV overrides the existing venv; MTC_VISION_OUTPUT_ROOT overrides private saves.'
  exit 0
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY. Open Terminal in the VMware Ubuntu desktop; do not guess DISPLAY.' >&2
  exit 2
fi
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
venv_path=${MTC_VISION_VENV:-$HOME/.venvs/mtc-vision-jazzy-py312}
if [[ ! -f "$venv_path/bin/activate" ]]; then
  echo "Existing venv not found: $venv_path (set MTC_VISION_VENV); no installation attempted." >&2
  exit 2
fi
source "$venv_path/bin/activate"
cd "$workspace"
export PYTHONPATH="$workspace/vision_core:$workspace/src/mtc_vision_adapter${PYTHONPATH:+:$PYTHONPATH}"
output_root=${MTC_VISION_OUTPUT_ROOT:-$HOME/private_data/mtc_live_ui}
run_output="$output_root/live_$(date +%Y%m%d_%H%M%S_%N)"
echo "LIVE diagnostic output: $run_output"
echo 'q/ESC or Ctrl+C exits; s saves; Body/Ring Pose INVALID; no ROS2 publisher.'
exec python vision_core/scripts/live_rgbd_preview.py --live --preview-authorized \
  --color-size 1280 720 --depth-size 640 480 --fps 15 \
  --run-output "$run_output" "$@"
