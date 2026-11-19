#!/usr/bin/env bash
set -eo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sim_root/scripts/env.bash"
source "$sim_root/install/setup.bash"
export ROS_DOMAIN_ID=86
export IGN_PARTITION=fr5_gazebo_sim
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
if ! ros2 pkg prefix gz_ros2_control >/dev/null 2>&1 || ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "Install Gazebo first: bash $sim_root/scripts/install_gazebo.sh" >&2; exit 1
fi
exec ros2 launch fr5_act_sim gazebo.launch.py "$@"
