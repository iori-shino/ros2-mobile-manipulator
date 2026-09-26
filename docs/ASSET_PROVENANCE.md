# Asset provenance

The project author explicitly confirmed that all ten STL files below were self-modelled and contain no downloaded third-party CAD. This is an author declaration supported by the project inventory, not an independent legal ownership investigation.

| File | Bytes | SHA-256 |
|---|---:|---|
| `arm_base.STL` | 20884 | `daec0659a0f2e538ff5723b01106f041cbae6549066fd6266f23864ceb85c27f` |
| `base_plate.STL` | 41084 | `5e1dc779f5bee14c3d780e8ab273ab310cb6290472d83c72563f80ddc67d116a` |
| `gripper_base.STL` | 684 | `988852e39ec278162448b7173babfe9caa4a5efba368b7cdf9baaf07091e4e0d` |
| `gripper_jaw-1.STL` | 1084 | `2498b3f0e674f179191e02db89f9b3c780cee0e65d246e87f29efcb3e2475420` |
| `joint1_platform.STL` | 19784 | `9b0355c2793dd0613323179c51139f410a9d496606078e09ea6db6e811b4ccb1` |
| `link1.STL` | 23084 | `afc14dd61e6abc9ea5cd2d045dc67cf3f7ce7f380ab117e87436adedd58710bf` |
| `link2.STL` | 22284 | `8b8201e3f31e38b09589d80342f967f136e18a817161dbeb900a8b29985b7de3` |
| `link3.STL` | 23084 | `c5b77cb5fdff08ef31cf648e7ecf05f26b7d750bb66858e0bc0cfdc04a6dff23` |
| `wheel.STL` | 19284 | `98c251c3e910877fe54d42ba250bb8beb701a3faed8185496fd3b2e39770dd2e` |
| `wrist_mount.STL` | 9084 | `f3f9700e3dbe1f892b9455e42ab4c522b2a9418be7546fcc3ce7e2238ba3665c` |

The wheel mesh is reused four times; the jaw mesh is reused for the two jaws. Binary/ASCII STL remains in its original millimetre coordinates; Xacro applies scale 0.001.

LiDAR is a cylinder; the RGB camera consists of boxes/cylinders defined in `src/mobile_manipulator_description/urdf/sensors.xacro`. The scan sensor and image sensor are Gazebo components, not third-party CAD assets. Room walls, pillar, partition and coloured targets are basic geometry in the project's SDF files.

No third-party LiDAR/camera STL, SolidWorks project file, commercial standard-part library, installer or raw screen recording is included. Screenshots show this project in RViz and the panel; they are not claims of affiliation with upstream projects. Dependencies are installed separately under their own licences.

The author has not selected a redistribution/reuse licence; package metadata remains Proprietary. Do not claim a permissive open-source licence before the author selects one. Machine-readable inventory: `evidence/assets.json`.
