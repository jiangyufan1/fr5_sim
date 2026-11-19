#!/usr/bin/env bash
set -eo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sim_root/scripts/env.bash"
export ROS_DOMAIN_ID=86
exec ros2 run rqt_image_view rqt_image_view /wrist_camera/image
