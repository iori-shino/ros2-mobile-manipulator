# Validation results and evidence

All robot results are **Gazebo simulation**, not real-robot accuracy, payload or safety results. A physical USB gamepad supplied human input to the simulation. The project author performed manual acceptance.

`evidence/results.json` contains selected original numeric records and the SHA-256 of each original local file. Long sample streams, runtime session information and developer-specific paths are not published. The source paths listed inside that JSON identify the retained local originals; those raw files are deliberately outside the release. Small selected records retain their original field names for traceability.

| Record | Fields / result | Scope |
|---|---|---|
| `stage2_base` | `tests`: +0.2004397 m forward, −0.2000394 m backward; yaw +1.3843573 / −1.3810415 rad; `watchdog_stopped=true` | One short base sequence; Gazebo chassis pose, not wheel odometry alone |
| `stage2_base` | Turning planar displacement = hypot(`actual_forward_m`, `actual_lateral_m`), about 0.0912 / 0.0915 mm | Derived from the recorded ideal-simulation test; not a real-world skid-steer accuracy claim |
| `stage3_arm_gripper` | 12 arm setpoints plus four jaw setpoints, `passed=true`; `max_jaw_asymmetry_m=0.0000734592` | Bounded joint/demo targets, no payload or arbitrary collision-free planning |
| `stage3_base_regression` | Existing base sequence and watchdog still passed after arm integration | Historical regression; not rerun for GUI-only work |
| `stage5_route` | Four approximately 0.5 m sides; endpoint error 0.00039993 m and heading error 0.01698168 rad | Physical route endpoint discrepancy, **not** SLAM localization accuracy |
| `stage5_route` | Maximum SLAM-vs-physics position/heading discrepancy 0.01784162 m / 0.13989816 rad | One simulated short route |
| `stage5_geometry` | 119×98 map, ~0.05 m resolution; 8946 known cells; observed occupied-boundary median/p95 error 0.008411 / 0.058411 m | Observed surfaces only; northern outer wall/occlusions are incomplete |
| `stage6_health` | All 24 checks true | Read-only graph, data, TF, mapping and active controllers |
| `stage6_integration` | All 22 isolated ROS checks true | Remapped test topics; input loss, guard limits, process failure, keyboard/terminal lifecycle |
| `stage6_physical` | Four directions, L-release stop, B stop, keyboard transition, stable physical pose passed | Human USB operation; recorded release-to-zero-output 23.6–74.5 ms is not physical stopping time |
| `speed_clock_sample` | Approximately 10 wall seconds / 1.152 simulation seconds; RTF 0.1152 | A VM load snapshot, not a performance target |

The original stage 6 core tests passed 11 cases; measured mapping validation passed 31 checks. The final GUI adds process-ownership tests and a bounded GUI integration report, included in `evidence/final_checks.json` after verification. The GUI test invokes a real arm demo, cancels its first active goal and verifies cancellation; it does not repeat the lengthy full stage 3 physical sequence.

The final candidate passed 18 unit tests, 22 isolated ROS checks, 17 live GUI checks and the 24-check system health run. Both packages built using only the candidate overlay and the existing ROS installation; five launch descriptions, four script help entry points and model generation were checked. This is not a fresh-OS dependency-installation test. Existing simulation/SLAM sessions were preserved; simulation teardown ownership was exercised with controlled test processes, without restarting the live map session.

## Repeatable checks

After building, source ROS and `install/setup.bash`, and run from the workspace root:

```bash
ros2 run mobile_manipulator_gazebo system_health.py
python3 -m pytest -q tools/acceptance/test_teleop_core.py tools/acceptance/test_panel_runtime.py
python3 tools/acceptance/teleop_integration.py
```

Use the appropriate health flag if teleop is intentionally omitted (`--skip-teleop`) or keyboard-only (`--keyboard-only`). An absent required topic produces FAIL; low wall-clock topic rate alone is not treated as proof of a broken simulator.

`teleop_physical_acceptance.py` requires deliberate human input. `acceptance_test.py`, `arm_control.py demo` and `slam_test_route.py` can move the simulated robot; use them only in their documented initial conditions. They are not part of the default read-only health command.
