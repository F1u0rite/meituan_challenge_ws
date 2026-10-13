#!/usr/bin/env bash
# Manual desktop entry. Executing this explicitly starts one real camera.
set -eo pipefail
export MTC_VISION_OUTPUT_ROOT=${MTC_VISION_OUTPUT_ROOT:-$HOME/private_data/v45_live}
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/start_native_live_ros_ui.sh" \
  --perception-pipeline simplified "$@"
