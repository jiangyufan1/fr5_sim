#!/usr/bin/env bash
set -eo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sim_root/scripts/env.bash"
if [ ! -f "$sim_root/install/setup.bash" ]; then
  echo 'Build first: bash scripts/build.sh' >&2; exit 1
fi
source "$sim_root/install/setup.bash"
exec ros2 launch fr5_act_sim sim.launch.py "$@"
