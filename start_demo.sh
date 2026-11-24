#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
if [[ -f install/m2_env.bash ]]; then
  source install/m2_env.bash
else
  source /opt/ros/humble/setup.bash
  source install/setup.bash
fi
exec python3 src/mobile_manipulator_autonomy/scripts/demo_console.py "$@"
