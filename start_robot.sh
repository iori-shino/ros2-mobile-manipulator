#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p log/panel
# Keep one desktop panel; its existing supervisor owns simulation and SLAM.
exec flock -n log/panel/desktop.lock python3 src/mobile_manipulator_gazebo/scripts/robot_panel.py --workspace "$PWD" --start-all >>log/panel/desktop.log 2>&1
