# Automatic top and hole coordinate delivery

This opt-in sender runs only from the Yuze worktree. It does not change the
team's IDL, existing live preview, original workspace or robot code.

| Topic | Type | Contents |
| --- | --- | --- |
| `/mtc/perception/top_hole_candidates` | `std_msgs/msg/String` (JSON schema `mtc.top_hole_candidates.v1`) | Four colour slots, green top XYZ, red CAD hole XYZ, raw capture time and SDK frame IDs, candidate/missing state and geometry diagnostics |
| `/mtc/perception/batteries` | `mtc_interfaces/msg/BatteryDetectionArray` | Available candidates only; hole XYZ in `ring_pose.pose.position`, `ring_pose_valid=false`; unknown orientation and body centre are NaN |

Positions are metres in `camera_color_optical` (X right, Y down, Z forward).
Green points are top centres, not battery body centres. Red points
are CAD-derived hole centres, not detected hole surfaces. Following the user's
2026-10-14 confirmation of physical heights, the automatic inference runner
constrains primary points to top=80 mm and hole=60 mm above the current table
plane. Raw observations remain separately available in `raw_*` fields.
Historical replay rows retain their own original coordinate variant.
Geometry warnings remain separate from full-pose validity.
Candidate availability and full grasp-pose validity are separate.

The JSON includes `source_frame_id`, `capture_timestamp_s`,
`capture_timestamp_source`, `timestamps` (raw color/depth SDK time, timestamp
domains, frame numbers and host receipt UTC), and `ros_dispatch_stamp`.
The dispatch stamp is not the image capture time. Since camera-to-ROS clock
alignment has not been independently verified, PoseStamped headers retain zero
capture stamps and `ros_capture_clock_verified=false`. Consumers must use the
raw source data and complete clock mapping before TF-based motion integration.
Recorded replay keeps the original acquisition time, never stamps old frames
as newly captured. Its `source_mode=replay`, `replay=true` distinguish it from
live input. Fresh offline inference is `source_mode=offline`, also non-live.

The JSON has exactly four objects ordered red/blue/yellow/green, with colour
codes 1/2/3/4. On failure, `candidate_available=false`, `status=MISSING`, both
coordinates and heights are null. No previous point is substituted. An
unqualified RGB-D pair removes all coordinate candidates. Duplicates, invalid
units, NaN/Inf coordinates and contradictory source timestamps are rejected
before either topic is published. Identity is a session-scoped unique-colour
slot, not a guarantee of physical identity if a battery is replaced.
SAM confidence is not exported as localization accuracy; the existing array's
confidence is 0 because no calibrated confidence is available.

Both publishers use RELIABLE / KEEP_LAST(5) / VOLATILE. Data are correlated by
source frame and packet sequence; ROS does not atomically deliver two topics.
If both points are needed, use the complete JSON packet. A missing detection
produces a null slot in JSON and is omitted from the battery array; receivers
must replace their current snapshot rather than retain omitted targets.

## Local verification / replay (no camera)

From the Yuze root, source `log/yuze_env.bash`. It deliberately limits discovery
to localhost and ROS domain 215; do not change another person's ROS environment.
For a receiver use RELIABLE QoS:

```bash
ros2 topic echo /mtc/perception/top_hole_candidates std_msgs/msg/String --qos-reliability reliable
ros2 topic echo /mtc/perception/batteries mtc_interfaces/msg/BatteryDetectionArray --qos-reliability reliable
```

Then in a separate sourced terminal:

```bash
ros2 run mtc_vision_adapter automatic_top_coordinates_replay \
  --rows log/yuze_reposition_20261013_164751/automatic_result/rows.json \
  --wait-subscriber-s 10
```

No subscribers means publication alone does not prove receipt. A positive
`--wait-subscriber-s` requires compatible subscribers on both topics. Shutdown
waits for reliable acknowledgement, releases the ROS context and exits.

## Run current automatic inference and publish its results

Source `/opt/ros/jazzy/setup.bash` and `install/yuze/setup.bash`, keep an isolated
local ROS domain for tests, then use the separate SAM CPU environment:

```bash
build/sam21_cpu/bin/python -I -B vision_core/scripts/run_automatic_top_coordinates.py \
  --input log/yuze_reposition_20261013_164751 --frames 6 \
  --geometry /home/shrmeituan/private_data/v44g_cad/geometry.json \
  --checkpoint log/sam21_experiment/sam2.1_hiera_tiny.pt \
  --wait-subscriber-s 10
```

`--live` replaces `--input` only when the user authorizes camera use and the
camera is free. It never stops another process. The default run is bounded to
six frames; `--frames 0` runs until Ctrl+C. Live use requires pyrealsense2 in the
SAM environment. Camera cleanup is registered before starting the device and
runs on normal exit, interruption or failure. Current CPU inference takes
seconds per frame; this task adds coordinate output, not a real-time image
stream or a faster detector. No camera or robot is used in offline tests.

Cross-host reception, robot FSM consumption and capture-clock alignment remain
separate integration checks. The team main currently uses MockPerception in
its C++ pick/place server; publishing does not connect it automatically.

## Verification performed on 2026-10-14 (Asia/Shanghai)

- Yuze adapter built successfully into `build/yuze` and `install/yuze`.
- New coordinate tests: 12 passed, including an installed publisher running in
  a separate process and a real ROS subscriber, missing-target clearing,
  invalid unit/NaN/duplicate rejection and unchanged source timestamps.
- Actual SAM inference over six previously acquired real RGB-D frames:
  six arrays and six JSON packets received, 24 green points and 24 red points;
  hole XYZ matched the ROS message values exactly. Publisher exited normally.
  Isolated domain 217, localhost only; no camera or robot started.
- Vision regression: 239 passed, 10 skipped.
- Existing native publisher tests: 10 passed, one existing UI-launcher test
  failed because the unchanged launcher sources `install/setup.bash`, whereas
  this isolated worktree uses `install/yuze/setup.bash`. Its bytes match HEAD.
  The new coordinate entry uses the Yuze installation and passed end-to-end.
- Raw capture-clock information is carried intact. Cross-host receipt,
  ROS-to-camera clock mapping and robot consumption were not validated here.

Evidence: `log/yuze_coordinate_delivery/coord_tests.xml`, `vision_tests.xml`,
`existing_publisher_tests.xml`, and
`recorded_inference/{summary.json,received_packets.json,publisher.log}`.

## JPEG video delivery

Add `--send-images` to `run_automatic_top_coordinates.py` to enable:

| Topic | Type / QoS | Contents |
| --- | --- | --- |
| `/mtc/perception/rgb/compressed` | `sensor_msgs/msg/CompressedImage`, BEST_EFFORT KEEP_LAST(1) | Current raw RGB JPEG frames |
| `/mtc/perception/rgb_metadata` | `std_msgs/msg/String`, RELIABLE KEEP_LAST(5) | Source frame ID, capture timestamp/domain, SDK frame numbers, image size and matching image header stamp |
| `/mtc/perception/top_hole_overlay/compressed` | `sensor_msgs/msg/CompressedImage`, BEST_EFFORT KEEP_LAST(1) | Green/red markers on the exact RGB frame used by inference |
| `/mtc/perception/top_hole_overlay_metadata` | `std_msgs/msg/String`, RELIABLE KEEP_LAST(5) | Source identity and capture times for each annotated result image |

Live mode has one camera owner: CaptureWorker reads frames, publishes current
RGB, and replaces a one-frame slot. Slow inference takes the latest qualified
RGB-D from that slot. Capture/video continue while the geometry inference is
busy. `--image-fps 10` is the default video rate cap (maximum 30); actual rate
depends on the camera, CPU and network. `--jpeg-quality 80` is the default.
Coordinate computation remains slow on the current CPU. Result-source overlays
therefore update at the geometry rate, not the raw video rate. Old results are
never drawn onto unrelated current video frames.

Image Header stamps explicitly mean ROS dispatch time, not capture time. Match
JPEG to its metadata using `image_header_stamp`; match images to coordinate
packets by `source_frame_id` and SDK timestamps. Read actual capture time from
metadata. Do not use image dispatch stamps for historical TF. Images can drop
on the video topic; metadata and image publication are not atomic.

For authorized live use when the camera is free, replace `--input ...` in the
inference example with `--live`, add `--send-images --frames 0`, and retain the
geometry/checkpoint arguments. Only that explicit live switch opens the camera;
it is released on exit. With a positive subscriber wait, both coordinate
subscribers and the raw image/metadata subscribers must exist before camera
start. The consumer's image subscription must use BEST_EFFORT-compatible QoS
and decode JPEG bytes. Coordinate-only operation remains the default.

Verification on 2026-10-14: 18 coordinate/video tests passed. The actual DDS
receiver decoded six raw and six annotated 1280x720 JPEG images from new SAM
inference over recorded real data; source frames and timestamps matched the
24 green and 24 red coordinate values. A separate test verified continuous
video updates during slow geometry processing with a synthetic camera source.
No real camera was started and cross-host transport was not tested. Evidence:
`log/yuze_video_delivery/tests.xml` and `recorded_inference/summary.json`.

## T0 desired hole centre at placement

The automatic inference runner now adds `placement_targets` to the coordinate
JSON. It detects a unique current 200x100 mm rectangular border on the current
support plane; no fixed pixel region or previous target is used. It fits four
straight edges excluding rounded corners, intersects camera rays with the
same-frame support plane, and estimates the rectangle's table centre. A missing
or ambiguous border produces a null target. P1/P2/P3 are not extrapolated from
the other PDF page when that layout is absent from the real scene.

`placement_targets[0].target_id="T0"`;
`target_hole_center_camera_m` is the desired battery aperture centre after
placing the battery upright with its base centred in T0. It is **not** the
printed table point, nor a robot TCP goal. The CAD places the hole centre above
the body centre by 35 mm; body half-height is 25 mm. Thus the target is
`table_center_camera_m + 0.060 * upward_normal_camera_xyz`. The CAD horizontal
offset must be zero; unknown off-axis offsets are rejected. No camera/robot
transform, tool compensation or extra height correction is applied.

For the recorded frame `color_66_depth_64`, target hole XYZ is
`[-0.1463289102, 0.0411890348, 0.5485161603]` metres in camera_color_optical.
The table centre is `[-0.1467766248, 0.0758087246, 0.5975189830]` metres.
Six frames produced six T0 candidates; maximum deviation from their median was
0.311 mm. This is apparent repeatability, not absolute accuracy. CAD height,
table-plane identity and target identity remain assumptions/diagnostics.

Verification: 23 placement/coordinate/video tests passed. A separate actual
ROS receiver received six T0 hole targets together with 24 battery top points,
24 battery hole points and six raw/six annotated frames from actual SAM
inference over previously recorded RGB-D. No camera or robot started.
Evidence: `log/yuze_t0_target/{summary.json,tests.xml,rows_with_targets.json}`
and `full_chain/{summary.json,received_packets.json,publisher.log}`.

## Physical-height constraint enabled by default

The user confirmed on 2026-10-14 that actual battery top and hole-centre heights
are 80 mm and 60 mm above the table. `run_automatic_top_coordinates.py` now
applies these physical constraints before publishing and drawing overlays.
For upward unit normal `n`, point `p`, and current table point `p0`:

`h_raw = (p - p0) dot n`

`p_effective = p + (h_physical - h_raw) * n`

This preserves the point's in-plane position. It changes camera X/Y/Z according
to camera/table orientation; it is not a fixed offset on camera Z. Camera pose
and table plane must be re-estimated from the current frame, as before.

Default green/red XYZ fields and the existing BatteryDetection ring XYZ now
contain effective height-constrained points. Each JSON object includes:

- `coordinate_variant="HEIGHT_CONSTRAINED"` and `height_constraint.applied=true`;
- `raw_green_point_camera_m`, `raw_red_point_camera_m` and raw heights;
- `height_constraint.top_correction_m`, `hole_correction_m` and the upward normal;
- effective `green_height_m=0.080`, `red_height_m=0.060`.

Geometry residuals and `geometry_consistent` still describe the raw fitting
evidence. Exact effective heights are imposed, not measured accuracy. Full
pose validity and `motion_usable` remain false. The height prior does not remove
in-plane centre bias or errors in table-plane estimation.

Missing candidates are never corrected or filled from previous frames. A stale
plane/frame reference, malformed coordinates, invalid normal, mismatched CAD
top-to-hole offset or correction beyond the configured limit is rejected.
By default the maximum correction is 10 mm, an anomaly filter rather than an
absolute accuracy tolerance. Rejected effective points are null while eligible
raw observations remain diagnostic. No silent fallback to a raw movement point.

`--disable-height-constraint` restores raw point output. Physical values are
configurable with `--top-height-mm`, `--hole-height-mm`, and
`--max-height-correction-mm`. T0 uses the same confirmed physical hole height
when the constraint is enabled; its existing 60 mm target position is unchanged.

Verification: 33 geometry/coordinate/video tests passed. Actual SAM inference
over six recorded real RGB-D frames delivered 24 height-constrained battery
points with originals retained, six T0 targets and six raw/six annotated JPEGs
to an independent ROS subscriber. No real camera or robot was started.
Median upward corrections (mm): green 2.05, blue 2.03, yellow 2.07, red 3.13.
These vary per frame, not fixed per-colour offsets. Evidence is under
`log/yuze_height_constraint/{tests.xml,correction_summary.json,full_chain/}`.

## Final PR verification (2026-10-14, Asia/Shanghai)

Full core/adapter regression: 413 passed, 10 skipped after correcting isolated
install resolution in the legacy launcher and real-process tests. A bounded
three-result camera rerun received all four colours and T0 on each result, 217
RGB frames at 7.50 FPS, maximum video gap 0.166 s, and coordinate processing
8.93-10.02 s. Camera was released on normal exit. The earlier camera run had
red/yellow missing in two results; this intermittent behavior remains disclosed
rather than treated as a sustained robustness pass. See automatic_top_setup.md.
