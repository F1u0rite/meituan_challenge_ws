# V4.4-A native Ubuntu RGB-D / ROS2 offline feasibility

Date: 2026-10-12. Workspace: /home/shrmeituan/meituan_vision_ws.
Branch: feature/perception-adapter. Protected HEAD:
0cb1db98ab65058d5acebcb440edb1744029d574 (V4.3-A localhost TCP bridge).
No reset, overwrite of teammate code, install, camera start, push or merge occurred.
TCP bridge remains a fallback; replay_publisher and vision_core algorithms are unchanged.

## Current environment

Python 3.12.3, Linux x86_64, glibc 2.39.
D435i (8086:0b3a) is attached to the guest, but lsusb -t reports 480M
under ehci-pci: USB 2.0, not demonstrated USB 3.x.
Neither rs-enumerate-devices nor realsense-viewer is on PATH.
pyrealsense2, OCP/cadquery-ocp and Open3D are not installed/importable.

Existing imports pass: NumPy 1.26.4, OpenCV 4.6.0, SciPy 1.11.4,
PyYAML 6.0.1, Matplotlib 3.6.3, pytest 7.4.4.
OpenCV/Matplotlib/pytest are below vision_core pyproject declared minimums
4.8/3.8/8, despite the observed offline tests passing. Dependency-complete
installation is not claimed. Open3D is not used by the tested core path.

Official release metadata confirms that the pinned cadquery-ocp 7.7.2 has
a cp312 manylinux_2_35_x86_64 wheel; this guest's Python/platform/glibc tags
match. This is distribution-tag compatibility, NOT tested CAD runtime:
https://pypi.org/project/cadquery-ocp/7.7.2/
The official pyrealsense2 package also has Python 3.12 Linux distributions:
https://pypi.org/project/pyrealsense2/2.56.5.9235/
No downloaded package was installed. A stale core error mentioning a Python
3.10 CAD runtime does not prove Python 3.12 incompatibility.

## Real offline results

- Existing vision_core tests: 80 passed, 1 skipped (private evidence unavailable).
- Combined vision_core + ROS2 adapter directed tests: 198 passed, 1 skipped,
  34.72 s. Includes existing TCP and replay natural/SIGINT regressions.
- colcon: mtc_interfaces and mtc_vision_adapter, 2 packages finished (5.04 s).
- Installed native_rgbd_publisher processed four lossless, explicitly
  SIMULATED_NATIVE_TEST_ONLY RGB-D samples with the actual FastPerception core.
  Independent native ROS2 subscriber received four BatteryDetectionArray
  messages plus four diagnostic messages. Each frame had one red visible-depth
  candidate at 0.6 m. No real camera was constructed/started in offline mode.
- Confirmed ROS arrays were EMPTY on purpose: private CAD dimensions/table
  confirmation evidence is absent, so existing confirmation gates are not bypassed.
- Known confirmed diagnostic-interface fixture conversion is separately tested
  with native ROS message classes: IDs/colors preserved; every body/ring pose
  component NaN and ring_pose_valid=false even if a fixture supplies a candidate.
  This fixture does not constitute real detection/pose accuracy acceptance.
- RGB-D deprojection/alignment/depth outliers, offline visible/body geometry,
  colors, confirmation evidence and candidate interface tests pass.
  Ring candidate field/matrix propagation passes existing synthetic interface
  tests; actual STEP-derived body/ring fitting is NOT RUN.
- Native camera mode without explicit camera_authorized=true rejects BEFORE
  creating the source or importing pyrealsense2. Real authorized camera mode
  and its stream selection/performance are NOT RUN.

## Prepared native node (independent of TCP)

Architecture: RGBDFrame -> existing FastPerception (+ existing RGBDVisionPipeline
if genuine CAD resources are complete and source hashes match) -> core.to_ros
-> native BatteryDetectionArray publisher. No new vision algorithm or IDL.

An offline node needs recorded sample folders and the existing core source:

```sh
cd /home/shrmeituan/meituan_vision_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mtc_vision_adapter native_rgbd_publisher --ros-args \
  -p input_mode:=offline \
  -p vision_core_path:=/home/shrmeituan/meituan_vision_ws/vision_core \
  -p offline_dir:=/path/to/recorded_or_explicit_test_session
```

The test's actual generated session path is recorded in native_publisher.log
and available under /tmp/mtc_v44a_native_20261012/synthetic_rgbd*.
No implicit camera fallback exists for missing/empty offline samples.
ROS QoS: RELIABLE KEEP_LAST(5) VOLATILE.
Only CONFIRMED_OBJECT items are eligible; UNKNOWN/unconfirmed/stale items are excluded.
For production select one producer per domain/topic: native OR TCP fallback.
Offline tests isolate their ROS domains and do not change the existing bridge.

Diagnostics: /mtc/perception/native_diagnostics (std_msgs/String).
Original color/depth SDK ms, domains, host_utc, capture-source fallback labels,
confirmation evidence and available candidates remain in diagnostic JSON.
ROS Header stamp is ZERO_UNMAPPED_CAMERA_CLOCK (0 sec, 0 ns) until an explicit
trusted camera-to-ROS clock mapping exists. It is neither sensor time nor
Ubuntu dispatch time. Local ROS dispatch time is separately labelled.
clock mapping, motion_usable, body_pose_valid and ring_pose_valid remain false.
Every published body/ring XYZ/quaternion component is NaN. No TF is published.
Shared IDL has no body_pose_valid field; nonfinite body pose remains the team's
existing invalid-pose signal. Actual C++ FSM rejection is NOT VERIFIED here.

## Missing private assets / safe handoff from Windows

No genuine CAD resources or RGB-D private samples were present. Copy only
authorized original files into these ignored Ubuntu locations, never into Git:

- vision_core/assets/cad/电池道具.stp
- vision_core/assets/cad/舌规V1.STEP
- Optional existing caches: vision_core/outputs/cad_analysis/geometry.json and
  vision_core/outputs/cad_analysis/cad_meshes.npz, matching the originals' SHA256.
  Windows cache metadata may contain Windows source paths; the existing pipeline
  must successfully resolve and hash-check them on Linux. Prefer regeneration
  with the existing analyze_cad_rgbd.py after official OCP installation.
- For real offline evidence, a complete existing sample_* session:
  raw_color.png, aligned_depth.png (lossless Z16), camera_intrinsics.json,
  metadata.json with depth scale, stream profiles, host/SDK timestamps/domains.
  Include raw depth/experiment metadata when recorded; preserve file hashes
  and timestamps. No personal photos or private models are published.

No public STEP replacement, fabricated CAD cache or guessed model was supplied.

## Evidence and next gate

Host-local logs, not committed:
- /tmp/mtc_v44a_environment.log
- /tmp/mtc_v44a_vision_pytest.log
- /tmp/mtc_v44a_native_pytest.log
- /tmp/mtc_v44a_all_pytest.log
- /tmp/mtc_v44a_colcon.log
- /tmp/mtc_v44a_native_20261012/native_results.json
- /tmp/mtc_v44a_native_20261012/native_publisher.log
- /tmp/mtc_v44a_native_20261012/native_received.json
- /tmp/mtc_v44a_native_20261012/native_diagnostics.json

Ubuntu acquisition is NOT READY/NOT VERIFIED: missing SDK and guest USB2 path.
User must configure VMware USB 3.x and connect D435i to Ubuntu, checking
lsusb -t for a SuperSpeed path (5000M or higher), and resolve trusted SDK
dependencies before device acceptance. No VM settings were changed here.
Only after the user's explicit “准备好了，启动预览” may a real preview be started.
No AUBO S3, FSM, MoveIt, Motion Executor or shared IDL was changed.
Offline stage complete; await user input.
