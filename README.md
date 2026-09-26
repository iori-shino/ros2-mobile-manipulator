# ROS 2 Mobile Manipulator Simulation

A course project that integrates a self-designed SolidWorks mobile manipulator with ROS 2 Humble and Gazebo Fortress. The model has a four-wheel skid-steer base, a four-joint arm, a parallel gripper, a 2D LiDAR and an RGB camera.

**Robot motion, sensing, SLAM and quantitative results in this repository are simulation results. No physical robot, payload, grasping or field-navigation experiment is claimed.** A real USB gamepad was used to control the simulated robot.

## Demo / screenshots

![RViz robot, TF, scan and SLAM map](docs/images/rviz.png)

![Lightweight Tkinter control panel](docs/images/panel.png)

These are captures from the running project. The room map is partial; unknown space is not evidence of complete room coverage.

## System architecture

```mermaid
flowchart LR
    CAD[Self-authored STL / Xacro] --> RSP[robot_state_publisher]
    CAD --> Model[prepare_model.py: temporary URDF / SDF]
    Model --> GZ[Gazebo Fortress]
    USB[USB gamepad] --> Joy[joy_node]
    Keys[Keyboard] --> Router[teleop_router]
    Joy --> Router
    Router --> Cmd[/cmd_vel]
    Cmd --> Guard[cmd_vel_guard: limits + wall-time timeout]
    Guard --> Bridge[ros_gz_bridge]
    Bridge --> GZ
    Arm[arm_control.py] --> JTC[arm / gripper trajectory controllers]
    JTC --> Control[gz_ros2_control]
    Control --> GZ
    GZ --> Sensors[scan / image / odom / joint_states / clock]
    Sensors --> SLAM[SLAM Toolbox]
    SLAM --> Map[map + map-to-odom TF]
    Sensors --> RViz[RViz]
    RSP --> RViz
    Map --> RViz
    Panel[Tkinter panel] -. existing CLI / ROS interfaces .-> Arm
    Panel -. existing CLI / ROS interfaces .-> Router
    Panel -. launch / map save / health .-> SLAM
```

## Robot model and software stack

| Item | Implemented configuration |
|---|---|
| Base | Approximately 0.40 × 0.30 m; four wheels, radius 0.05 m, track 0.35 m |
| Arm / gripper | J1 yaw, J2–J4 pitch; two prismatic jaws, approximately 20–50 mm opening |
| Meshes | Ten author-created STL files; millimetres converted to metres with scale 0.001 |
| Sensor appearance | LiDAR cylinder and camera box/cylinders; no downloaded sensor CAD |
| OS / middleware | Ubuntu 22.04, ROS 2 Humble, Python 3.10 |
| Simulation | Gazebo Fortress; `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` |
| Controllers | JointTrajectoryController for both arm and gripper; arm position interface, gripper effort/PID interface |
| Mapping / display | SLAM Toolbox online asynchronous mapping, RViz 2 |
| Optional panel | Python Tkinter; no Qt application framework |

Reference versions recorded during development: Fortress 6.18.0, gz_ros2_control 0.7.21, joint_trajectory_controller 2.54.0, slam_toolbox 2.6.10. They describe the tested environment, not a guarantee for every package update.

## Features and scope

- RViz model and TF; physics-based base, arm and symmetric jaw motion.
- Simulated 360-sample LiDAR and 320 × 240 RGB camera, configured at 10 Hz in simulation time.
- SLAM map generation and checked YAML/PGM saving.
- Keyboard and measured USB controller mapping with enable, stop, deadzone and wall-time timeouts.
- Read-only system checks and isolated teleoperation regression tests.
- A small panel that starts existing commands, reports status and sends software stop requests.

There is no Nav2 navigation, MoveIt, route planning, automatic exploration, object recognition or real-robot driver.

## Repository structure

```text
src/mobile_manipulator_description/   Xacro, original STL, RViz configuration
src/mobile_manipulator_gazebo/        launches, worlds, controllers and Python tools
tools/acceptance/                    portable calibration and regression utilities
tools/prepare_release.py             allowlisted, history-free release export
tools/check_release.py               static release checks
docs/                               architecture, evidence, issues and project facts
maps/                               saved example map
```

## Installation / dependencies

Use Ubuntu 22.04 with an existing [ROS 2 Humble desktop installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and Gazebo Fortress. The [Humble gz_ros2_control documentation](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html) describes the matching simulator plugin.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-tk python3-yaml python3-pil gnome-terminal \
  ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ign-ros2-control \
  ros-humble-ros2-controllers ros-humble-slam-toolbox ros-humble-joy \
  ros-humble-teleop-twist-joy ros-humble-joint-state-publisher-gui
```

Clone or unpack the repository into a workspace directory, for example `~/robot_ws`. Run the following **from the repository root**. Initialize rosdep once if the machine has not been initialized (`sudo rosdep init`), then:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

The panel needs a graphical desktop. The robot CLI does not depend on Tkinter. The panel's keyboard button uses GNOME Terminal; the keyboard script can also run directly in another interactive terminal.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

In each subsequent terminal, source ROS and this workspace again. Do not launch duplicate simulation, SLAM, joy or teleop nodes.

## Running

The smallest complete mapping demonstration:

```bash
ros2 launch mobile_manipulator_gazebo mapping_demo.launch.py
```

This starts the room simulation, SLAM and RViz. Driving remains manual. For simulation without SLAM use `ros2 launch mobile_manipulator_gazebo sim.launch.py`; for a model-only RViz session use `ros2 launch mobile_manipulator_description display.launch.py`. Do not run model-only joint-state publishing together with physics simulation.

Optional panel, from the workspace root:

```bash
ros2 run mobile_manipulator_gazebo robot_panel.py --workspace "$PWD"
```

The panel can start/stop verified simulation sessions, open keyboard input, start gamepad teleop/SLAM, run the existing arm demo and health check, and save a map. It detects running ROS nodes and refuses duplicate launches. Closing the panel leaves ROS sessions running. **STOP is a software command stop**, sent through the existing safety chain plus action cancellation; it is not a certified or latched hardware emergency stop. New commands can resume motion. See [GUI operation](docs/GUI.md).

Arm and gripper examples:

```bash
ros2 run mobile_manipulator_gazebo arm_control.py J2 0.4
ros2 run mobile_manipulator_gazebo arm_control.py open
ros2 run mobile_manipulator_gazebo arm_control.py close
ros2 run mobile_manipulator_gazebo arm_control.py home
```

`arm_control.py demo` runs the longer previously validated sequence; it is optional, can take several minutes in a slow VM, and is not required for routine health checks. No grasped object or payload is tested.

## Keyboard / gamepad controls

Start the shared router and USB driver once:

```bash
ros2 launch mobile_manipulator_gazebo teleop.launch.py
```

Use `start_joy:=false` if a joy_node already runs. With no gamepad, use `keyboard_only:=true`. Then open a separate interactive terminal:

```bash
ros2 run mobile_manipulator_gazebo teleop_keyboard.py
```

W/S = forward/backward, A/D = left/right, Space = stop, Q/Ctrl-C = stop and quit. Hold/repeat a key for continuous motion. Keyboard speed is 0.04 m/s and 0.3 rad/s; a 0.30 s wall-time input timeout handles missing key-up events.

Measured mapping for **Nintendo Co., Ltd. Pro Controller**, using `joy_node`, zero-based indices:

| Input | `/joy` index / direction |
|---|---|
| Left stick horizontal / vertical | axes 0 / 1; left / up positive |
| Right stick horizontal / vertical | axes 2 / 3; recorded but unused for driving |
| A / B / X / Y | buttons 1 / 0 / 2 / 3 |
| L / R | buttons 5 / 6 |
| ZL / ZR | buttons 7 / 8 |
| Minus / Plus | buttons 9 / 10 |

**Release L, centre the left stick, then hold L to enable; move the stick to drive. Release L to stop. B is a separate stop button.** Gamepad speed is limited to 0.05 m/s (3 m/min) and 0.4 rad/s. The router uses a 0.15 deadzone after the driver's 0.05 deadzone and a 0.30 s wall-time input timeout. The existing downstream guard separately clamps speed and stops stale commands after 0.5 s wall time. Held gamepad enable takes priority over movement keys; Space can stop either input. Do not simultaneously run other tools that publish `/cmd_vel` directly.

Mappings vary with device/driver. To repeat calibration for this Pro Controller, run `ros2 run joy joy_node`, then `python3 tools/acceptance/joy_calibrate.py` and follow the labelled prompts. The calibrator only reads `/joy`. It generates `config/teleop_mapping.yaml` in the Gazebo package; rebuild to install a newly generated file. The helper names this device explicitly and expects button triggers. Other hardware requires reviewing the device name and adapting trigger handling before calibration. Do not assume an Xbox, PS or generic Switch layout.

## SLAM demo and map saving

For a simulation that is already running, start SLAM separately:

```bash
ros2 launch mobile_manipulator_gazebo slam.launch.py
ros2 run mobile_manipulator_gazebo save_slam_map.py maps/generated/demo_room
```

Use a new map name each time; existing maps are not overwritten. `maps/stage5_room_20260927.yaml` and its PGM are a saved, partial simulation example. The fixed `slam_test_route.py` is an acceptance utility for a **fresh room/origin only**, not a navigation or general autonomous-driving feature.

## Automated validation

With simulation, SLAM and full teleoperation running:

```bash
ros2 run mobile_manipulator_gazebo system_health.py
python3 -m pytest -q tools/acceptance/test_teleop_core.py tools/acceptance/test_panel_runtime.py
python3 tools/acceptance/teleop_integration.py
```

Health checks are read-only and take about 25 wall seconds. Use `--keyboard-only` for keyboard-only operation or `--skip-teleop` when no teleop is started. Integration tests use isolated `/stage6_test/*` topics and do not move the running robot. Actual USB input acceptance is guided by `python3 tools/acceptance/teleop_physical_acceptance.py` and requires a human operator.

Recorded results, their limits and exact evidence fields are in [validation results](docs/VALIDATION.md) and [machine-readable evidence](docs/evidence/results.json). Historical runs are distinguished from final-stage checks. Do not interpret near-zero ideal-simulation drift as real-world accuracy.

## Known limitations

- Simulation only; estimated inertias and simplified collision geometry. No real-robot safety, payload or grasping validation.
- Self-collision is disabled; validated demo poses do not guarantee arbitrary arm configurations are collision-free.
- Four-wheel skid-steer uses tuned anisotropic friction/slip for this world. It is not a calibrated tire/ground model.
- The VM can run well below real time. 0.05 m/s is per simulation second; model scaling is not an extra speed multiplier. Keep wall-time command watchdogs independent of simulation time.
- The map has unknown/occluded regions; full-room coverage and navigation are not claimed.
- Physical USB disconnect/reconnect was not separately accepted. Input silence and router failure were tested, but a driver repeating frozen input cannot be detected solely by message arrival time.
- Software stop is not a hardware emergency stop. The GUI supervises only its own or verified same-workspace sessions; external sessions may need to be stopped in their terminal.

See [engineering log](docs/ENGINEERING_LOG.md) for symptoms, evidence, fixes and remaining uncertainty.

## Asset provenance and publication

The author confirmed the ten robot STL files are self-created and contain no third-party CAD. Sensor appearances and room geometry are primitives created in this project. No SolidWorks installer, commercial CAD library, downloaded standard-part mesh or dependency source is redistributed. File hashes and scope are in [asset provenance](docs/ASSET_PROVENANCE.md).

No open-source licence has been selected by the author yet; existing package metadata remains `Proprietary`. Public visibility would not grant a permissive reuse licence. See [NOTICE](NOTICE.md). The prepared release contains no development Git history or local handoff/session records; see [release checks](docs/RELEASE_CHECKLIST.md).

## Development assistance / acknowledgements

Parts of the ROS 2 integration, debugging workflow, test automation, GUI and documentation were developed with assistance from OpenAI Codex. The project author defined the project requirements, created the CAD model, reviewed integration decisions, operated the simulation/gamepad tests and performed final acceptance. AI assistance included implementing and executing integration code and automated checks under that review; the code is not presented as entirely hand-written by the author.

ROS 2, Gazebo, ros2_control and SLAM Toolbox are upstream dependencies installed separately under their own licences.
