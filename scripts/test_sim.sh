#!/usr/bin/env bash
set -eo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sim_root/scripts/env.bash"
source "$sim_root/install/setup.bash"
exec /usr/bin/python3 "$sim_root/scripts/test_trajectory.py"
