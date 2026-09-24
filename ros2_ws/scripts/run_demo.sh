#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is missing in this Linux distribution. Use Ubuntu-24.04.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
cd "$workspace_dir"
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$workspace_dir/src/urc_simulation/config/fastdds.xml}"
# Explicitly use the WSLg/X11 Qt backend. Software rendering is an optional fallback.
export QT_QPA_PLATFORM=xcb
if [[ "${URC_SOFTWARE_RENDERING:-0}" == 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
fi
if [[ "${URC_SKIP_BUILD:-0}" != 1 ]]; then
  colcon build --event-handlers console_cohesion+
fi
source install/setup.bash
exec 9>"/tmp/urc-local-demo-${UID}.lock"
if ! flock -n 9; then
  echo 'The local demo is already running. Close its RViz/launcher first.' >&2
  exit 1
fi
echo 'Starting isolated simulation on ROS domain 71. No CAN or physical camera.'
exec ros2 launch urc_simulation demo.launch.py "$@"
