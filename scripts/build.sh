#!/usr/bin/env bash
set -eo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sim_root/scripts/env.bash"
cd "$sim_root"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
