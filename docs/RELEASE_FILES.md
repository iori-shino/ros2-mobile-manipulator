# Final-stage change inventory

The release manifest lists every published file and its SHA-256. This document groups the final-stage changes; previously accepted robot control, geometry and SLAM algorithms were retained.

| Area | Files / change |
|---|---|
| Optional GUI | `src/mobile_manipulator_gazebo/scripts/robot_panel.py`, `panel_runtime.py`: Tkinter panel and verified-session process management |
| Packaging | Gazebo package `CMakeLists.txt`, `package.xml`: install panel/helper and declare Tkinter dependency |
| Portable outputs | Existing `scripts/acceptance_test.py`, `scripts/arm_control.py`: relative default log output paths only |
| Keyboard startup | `scripts/teleop_keyboard.py`: wait conditionally for DDS subscribers, up to five seconds; key mapping, command limits and stop behavior unchanged |
| Portable acceptance tools | `tools/acceptance/joy_calibrate.py`, `teleop_integration.py`, `teleop_physical_acceptance.py`: derive workspace location from the script; create required output directories on first run; repeat held synthetic joy input like the driver while retaining explicit silence tests |
| GUI regression | `tools/acceptance/conftest.py`, `test_panel_runtime.py`: import setup and process ownership/lifecycle checks |
| Release preparation | `.gitignore`, `tools/prepare_release.py`, `tools/check_release.py`: exclusions, allowlisted history-free snapshot and static/manifest audit |
| Public documentation | `README.md`, `NOTICE.md`; `docs/GUI.md`, `ENGINEERING_LOG.md`, `PROJECT_FACTS.md`, `VALIDATION.md`, `ASSET_PROVENANCE.md`, `RELEASE_CHECKLIST.md`, this inventory |
| Public evidence | `docs/evidence/`: selected historical records, final checks, STL provenance/hashes; `docs/images/`: reviewed application screenshots and example map |

Stage 6 teleoperation scripts, measured mapping, health checker and existing acceptance tools are included as already accepted work. They are not presented as newly added robot functionality in this stage. Local handoff notes and raw execution logs are retained privately and excluded from the snapshot.

The author approved public publication after reviewing the prepared snapshot. A fresh Git history is created from these files; earlier development history and local runtime files are excluded.
