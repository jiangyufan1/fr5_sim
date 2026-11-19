#!/usr/bin/env bash
set -eo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sim_root/scripts/env.bash"
source "$sim_root/install/setup.bash"
export ROS_DOMAIN_ID=86
exec /usr/bin/python3 "$sim_root/scripts/gazebo_gripper.py" "$@"
