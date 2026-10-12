#!/usr/bin/env bash
# Manual H2 entry: one H1 camera pipeline with an attached ROS2 result sink.
set -eo pipefail
if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: bash start_native_live_ros_ui.sh [additional UI options]'
  echo 'Manually starts the single LIVE UI with native ROS2 topics; no standalone camera publisher.'
  exit 0
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY. Run in VMware Ubuntu desktop Terminal.' >&2
  exit 2
fi
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
exec bash "$workspace/vision_core/scripts/start_native_live_ui.sh" --ros-publish "$@"
