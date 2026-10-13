#!/usr/bin/env bash
# Manual H2 entry: one H1 camera pipeline with an attached ROS2 result sink.
set -eo pipefail
if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: bash start_native_live_ros_ui.sh [additional UI options]'
  echo 'Manually starts the single LIVE UI with native ROS2 topics; no standalone camera publisher.'
  echo 'MTC_VISION_ROS_SETUP overrides the ROS overlay setup.bash path.'
  exit 0
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY. Run in VMware Ubuntu desktop Terminal.' >&2
  exit 2
fi
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/jazzy/setup.bash
ros_setup=${MTC_VISION_ROS_SETUP:-$workspace/install/setup.bash}
if [[ -z "${MTC_VISION_ROS_SETUP:-}" && ! -f "$ros_setup" && -f "$workspace/install/yuze/setup.bash" ]]; then
  ros_setup="$workspace/install/yuze/setup.bash"
fi
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS overlay not found: $ros_setup (set MTC_VISION_ROS_SETUP)." >&2
  exit 2
fi
source "$ros_setup"
exec bash "$workspace/vision_core/scripts/start_native_live_ui.sh" --ros-publish "$@"
