# V4.5 modular RGB-D perception

Opt-in replacement; V4.4 remains the default. No new IDL, robot commands, YOLO,
models or dependencies. Same capture source, RGB-D admission, UI and ROS sink.

| Module | V4.4 | V4.5 |
|---|---|---|
| Identity | RGB/depth, table-cache age, whole motion history, tracking | Current RGB color/shape plus tracking; independent of depth/table/CAD |
| Body | Full async CAD fit and cached table support | Same-frame mask depth, outlier filtering, observed surface PCA/plane and optional CAD dimensions |
| Ring | Mostly full-body CAD prediction | Observed color-hole topology, rim depth plane, through-hole background separation, CAD aperture-size check |
| ROS/UI | Existing team topics and fixed canvas | Reused; identity, geometry and execution evidence explicitly separated |

Implementation: `perception/simplified_rgbd.py` contains BatteryDetection,
BodyPoseEstimation, RingPoseEstimation and their orchestrator. ROS2 output remains
`mtc_vision_adapter.native_result_publisher.NativeResultPublisher`.
`config/simplified_rgbd.yaml` is independently configurable; V4.4 files unchanged.

## Identity semantics

CONFIRMED_OBJECT means an engineering RGB color/shape/track hypothesis, not a
verified battery class probability or a grasp target. Two observations establish
association. HSV itself can confuse colored background objects. Identity confidence
is contour area / minimum-area rectangle area; `confidence_semantics` labels it.
`color_confidence` is the HSV pixel fraction, also not a calibrated probability.

V4.5 tightens chromatic support (S>=100) because actual H4 woodgrain contaminated
legacy contours. RGB-only fill >=0.70 rejects irregular extra H3 patches; it is
separate from the unchanged V4.4 0.80 combined geometric confirmation rule. Area,
short side, aspect and image-edge checks are provisional camera-view priors. They
require separate four-color, illumination, occlusion and negative-scene validation.
This update does not change any 3D-validity rule or claim detection accuracy.

`battery_mask` is supplied as the current-frame boolean view Mask, with its contour
and bbox serialized. The outer silhouette is filled; raw HSV internal holes are
examined separately. No mask pixels or geometry are copied across captures.

## Body and ring evidence

Body uses accepted same-domain RGB-D and verified SDK alignment. Reject repeated
IDs/non-increasing sensor timestamps, incomparable clocks, skew >100ms, missing
alignment, insufficient depth/points. Robust depth filtering precedes deprojection.
Surface centroid is explicitly separate from body center. A center candidate is
only inferred from a near-planar face with CAD-compatible span plus half-height;
face identity remains unverified. PCA yaw is an azimuth in optical X/Y, modulo pi,
not a unique full 3D rotation. Both yaw directions are retained.

Ring requires a visible enclosed RGB hole, sufficient local rim depth, a near-planar
rim, a reliable viewing ray, observed deeper background inside the hole, and
agreement with optional CAD aperture dimensions. Hole centers are ray/observed-rim
plane intersections, never body-position offsets or background depth at the void.
Color holes are proposals: dark paint, glare, occlusion and missing depth can reject
them. Candidate insertion normals have two signs; roll, entry accessibility and
precision remain unverified. Full-body CAD fitting is disabled in simplified mode.

Body/Ring validity and motion_usable remain false in this version. Ground-truth
position/orientation validation, face identification and direction resolution are
not complete. No plausible candidate is promoted into an executable Pose.
Optional geometry cache uses existing relative `outputs/cad_analysis/geometry.json`;
provide locally verified STEP/cache via the established private asset arrangement.
Missing cache does not prevent RGB identity or observed surface candidates.

## ROS interface

- `/mtc/perception/batteries`: `mtc_interfaces/msg/BatteryDetectionArray`.
- `/mtc/perception/native_diagnostics`: `std_msgs/msg/String` with detection, mask
  contour, quality, candidates, separate validity flags and filter reasons.
- QoS unchanged: RELIABLE, KEEP_LAST depth 5, VOLATILE.
- Coordinates: `camera_color_optical`, metres; optical X right, Y down, Z forward.
- Current confirmed RGB hypotheses may appear in arrays with NaN body/ring Pose,
  ring_pose_valid=false. IDL has no body_pose_valid: consult diagnostics and existing
  invalid-Pose convention; never drive an arm from an identity alone.
- SDK timestamps remain original. ROS header is zero because SDK-to-ROS clock
  mapping remains unverified; dispatch time in diagnostics is not capture time.
- Historical replay remains stale and is not ROS target-admitted by replacing
  capture time. Candidates remain observable in diagnostics. No new topic required.

## Launch and rollback

From VMware Ubuntu graphical desktop (existing Jazzy overlay/venv provisioned):

```bash
cd "$HOME/meituan_vision_ws"
bash vision_core/scripts/start_simplified_live_ros_ui.sh
```

This manual command starts the real D435i, preferred RGB 1280x720, depth 640x480
Z16 at 15 FPS, with one shared capture source. No automatic hardware validation is
part of offline delivery. q/Esc or Ctrl+C exits/releases; s saves; r starts/stops
bounded real recording. Do not start another camera publisher. The wrapper uses
the existing venv and ROS setup, does not install anything, and rejects no DISPLAY.

Offline saved H3/H4 `frame_*` directories:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "$HOME/.venvs/mtc-vision-jazzy-py312/bin/activate"
export PYTHONPATH="$PWD/vision_core:$PWD/src/mtc_vision_adapter:$PYTHONPATH"
python vision_core/scripts/live_rgbd_preview.py --perception-pipeline simplified \
  --full-vision --dual-rate --headless --replay /path/to/recorded_frames \
  --run-output /path/to/new_private_output --ros-publish
```

Remove `--headless` to open the existing UI on the desktop. V4.4 comparison uses
`--perception-pipeline legacy`. Approved V4.4-D NPY manifest replay remains supported.
For Git rollback, commit/save work then `git switch codex/v44c-native-runtime`;
do not reset/rewrite history. That branch preserves H4 commit 80b1c753.

Offline completion does not establish reliable four-color identification, valid
ring pose, calibrated XYZ accuracy, camera-to-base TF or grasp readiness. Fixed
and controlled-moving real scenes, real blue/yellow/green, colored distractors,
occluded apertures and independent metrology remain required.

## Offline delivery snapshot

117 H3 + 353 H4 original saved pairs were processed. Detection-valid observations:
105 + 343 (all red); body XYZ candidates: 13 + 16; yaw candidates: 115 + 344;
local ring-center candidates: 0 + 1. Body/Ring Valid are zero. H4 maintains one ID
for its 343 identity observations. Extra early H3 blue/green proposals were not
confirmed; real four-color accuracy and negative-scene accuracy are not measured.

Compute-only mean/P95: H3 67.10/83.77 ms; H4 41.83/55.64 ms. Corresponding inverse
mean capacities 14.90/23.90 FPS are not camera or UI FPS. Independent ROS replay
received 20 arrays and 20 diagnostics, with 19 identity observations; old captures
were filtered, zero executable poses. Its actual rolling result-update rate was
6.45 Hz and full-process wall throughput 3.16 Hz including initialization/rendering,
I/O and DDS. CAD is disabled; Camera/UI-window FPS were not measured offline.

New targeted tests: 6 passed. Full regression: 310 passed, 10 skipped, 1 known legacy
baseline failure (`test_native_real_offline_ros_process`, synthetic red fixture
expects an object under the original CAD-aware policy). This is not an all-green
suite, and does not certify real detection or poses. `colcon build --packages-select
mtc_vision_adapter --symlink-install` passed. No IDL/FSM/MoveIt changes or physical
camera/robot tests were performed. Raw data, CAD and full test logs remain private.
