# Lightweight control panel

Run from a built workspace after sourcing ROS and `install/setup.bash`:

```bash
ros2 run mobile_manipulator_gazebo robot_panel.py --workspace "$PWD"
```

Tkinter is the only added GUI toolkit. The panel calls existing commands and ROS interfaces; the robot does not depend on the panel.

| Button | Existing interface / behavior |
|---|---|
| Start simulation | `sim.launch.py` with the structured SLAM room; refuses duplicate simulation |
| Stop simulation | Sends stop, then stops verified simulation/SLAM/teleop process groups |
| Keyboard teleop | Ensures the shared router and opens `teleop_keyboard.py` in GNOME Terminal |
| Gamepad teleop | `teleop.launch.py`; reuses an existing joy_node |
| Arm demo | `arm_control.py demo`; may take several minutes; do not use as a routine health check |
| Arm / gripper home | Existing `arm_control.py home` action client |
| Health check | Existing read-only `system_health.py`; PASS/FAIL and details in `log/panel/` |
| Start SLAM | Existing `slam.launch.py`; refuses duplicate SLAM |
| Save map | Existing `save_slam_map.py` service client; new names under `maps/generated/`, never overwrites |
| STOP | Zero `/cmd_vel` through the existing guard, `/teleop/key` stop event, cancel requests to arm/gripper actions |

## Stop and lifecycle semantics

- STOP is a **software command stop**, not a certified, persistent hardware emergency stop. New commands can resume motion. The physical robot has not been implemented.
- Stop cancels a panel-launched arm operation and requests cancellation of active arm/gripper trajectory goals. A controller normally holds after cancellation; this is not power removal.
- The gamepad retains its existing L-release/recentre rearming rules. The panel does not change limits, controller gains or deadman logic.
- Other programs publishing `/cmd_vel` directly are outside the teleop router's arbitration; do not run them concurrently.
- Closing/crashing the panel does not kill ROS sessions. Their command watchdogs remain independent. The CLI continues to work.
- Only same-user, same-workspace sessions with a verified command, process group and start time can be stopped. Processes launched elsewhere show `Running (external)` and must be stopped in their own terminal.
- Process logs and session identifiers are runtime-only and excluded from the public release.
- Stopping simulation ends the current live SLAM session; save the map first if it matters. Starting it again begins a new simulation.

The panel does not require a theme, plotting library, Qt application or browser service. Without a desktop/display, use README CLI commands.
