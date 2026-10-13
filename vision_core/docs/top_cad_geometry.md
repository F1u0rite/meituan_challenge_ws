# V4.5-D: observed crossbar top + measured CAD

The simplified pipeline now estimates each observed battery independently. It
keeps RGB identity, local geometric support, pose validation and motion usability
separate. The V4.4 legacy pipeline is unchanged.

## Measured geometry

The supplied battery CAD's highest rectangular planar face is face 27. Its
bounds in CAD millimetres are `[-33,-11,-30] .. [33,11,-30]`, area 1452 mm²,
centre `(0,0,-30)` and outward normal `(0,0,-1)`. Area equals the bounding
rectangle's area, so this cap's geometric centre is its rectangle centre; a
nonrectangular/ambiguous cap is rejected. The existing CAD aperture sections
place the hole centre at `(0,0,-10)` with insertion axis CAD `±Y`.

Thus `offset_ring_CAD = (0,0,+20) mm`, and the body bounding-box origin offset
is `(0,0,+55) mm`. Units are converted to metres before transformation:

```text
P_ring_camera = P_top_camera + R_camera_from_CAD @ offset_ring_CAD
```

No constant is applied to a fixed camera axis. The long cap edge, normal and
cross product form proper rotation candidates. A 180-degree in-plane rotation
leaves the axial centre offsets unchanged but reverses the insertion direction.
There is no verified station-to-camera transform to choose the insertion side.

## Current-frame observations and quality

`perception/top_bar_pose.py` uses the existing RANSAC and local planar seeds over
the current mask. Window scale follows the measured cap thickness, intrinsics
and observed range; there is no fixed pixel ROI, image-up/camera-angle assumption
or inter-object depth sharing.

Acceptance requires:

- measured cap extent (66×22 mm), with declared tolerance for sampled boundaries;
- a rectangle centre obtained from observed geometric extent, not the visible
  point-cloud mean;
- all four sides supported by current RGB edges/silhouette (at least 50% per side);
- at least 100 valid holdout pixels, existing 50% depth availability, at least
  70% support within the existing 3 mm plane tolerance, and P95 residual ≤3 mm;
- the rest of the current object lying below the outward cap plane and a
  CAD-consistent observed height;
- one distinct supported cap; substantially different supported planes remain
  ambiguous.

Plane fitting uses one checkerboard pixel subset; the other is evaluated inside
the proposed polygon without first selecting plane inliers. Spatial holdout is
not an independent accuracy measurement. RGB edge extraction includes outside
pixels around the target bbox so tight cropping cannot destroy silhouette
edges. Boundary checks account for the existing mask erosion and raster rounding.

A partial/unsupported boundary does not get nominal CAD edges silently added.
There is no partial-face completion path. Size/edge matching still needs real
occlusion, lighting and accuracy validation; it is not a formal identity or
millimetre precision guarantee.

## Diagnostics, UI and ROS

Each object has `top_geometry`: source frame/timestamps, observed polygon,
centre/normal/long-axis candidates, local quality, measured CAD reference,
rotations, CAD-derived ring/body centres, both insertion directions and reasons.
The prior body calculation remains in `mask_depth_body_geometry`.

`top_geometry_valid` and `ring_center_geometry_valid` denote internal current
frame geometric support. `ring_geometry_valid` stays false without a unique
entry side. **`body_pose_valid`, `ring_pose_valid` and `motion_usable` remain false.**
A CAD-derived ring centre is labelled `CAD_DERIVED_FROM_OBSERVED_TOP`, with
`ring_observed=false`; it is never labelled a directly observed aperture.
Direct aperture observations retain their source; disagreement is diagnostic.

The existing fixed-size UI shows the automatic polygon, top centre, derived ring
centre, ± direction and local support/residual. Its history/current-frame
provenance rules still apply. These are actual offline renders, not a claim that
an SSH graphics window was opened.

`/mtc/perception/batteries` (`mtc_interfaces/msg/BatteryDetectionArray`) and
`/mtc/perception/native_diagnostics` (`std_msgs/msg/String`) retain reliable,
keep-last-5, volatile QoS and unchanged IDL. The former retains invalid NaN poses;
the latter carries the additional diagnostic fields. ROS acquisition-clock
mapping is unverified: headers remain zero, original SDK timestamps are retained
in diagnostics, and replay frames are not rebased into fresh observations.

The old `ObservationMonitor` still expects stable/full-CAD observations and can
show `OBSERVATION_TIMEOUT` in simplified mode. It does not admit/filter identity
or this current-frame top calculation. No time-system or observation-state
redesign is included here.

The observed-only 2×2 diagnostic reports available object IDs/colours and
rectangle topology. It never assigns colours to fixed slots, fabricates a
missing battery or makes one failed battery block another. Real four-colour
2×2 top/ring geometry has not yet been tested.

## Offline results

12 original manual RGB-D frames were replayed with their true timestamps. The
recorded hand-save gaps reset RGB association in several frames, so replay
confirmation counts must not be presented as the original continuous LIVE count.

- Red object observations: 12; replay identity confirmations: 5. One extra
  unconfirmed yellow/wood-region candidate appeared; existing HSV needs continued
  negative-scene testing. The top/CAD change does not alter RGB admission.
- Supported automatic top and CAD-derived ring centre: **1/12** (`preview_0007`);
  11 frames fail boundary, local plane support/dimensions or uniqueness checks.
- Supported frame top XYZ: `(-31.55,-54.04,397.85) mm` in `camera_color_optical`.
  Derived ring XYZ: `(-31.94,-44.38,415.35) mm`; local support 96.22%, P95 2.64 mm.
- Signed entry/Body/Ring valid poses: 0. No ground-truth accuracy claim.
- Combined pipeline mean/P95: 382.19/721.10 ms; RGB: 40.26/45.47 ms;
  top route: 290.18/611.91 ms. These are the final measured offline run, not camera/UI
  FPS. No live throughput claim is made.
- UI renders all remain 1600×900; all pairs qualified; 108 input file hashes unchanged.
- Targeted tests: 21 passed. Relevant comprehensive suite: 325 passed, 10 skipped,
  1 known legacy baseline failure (`test_native_real_offline_ros_process` expects
  a synthetic legacy red candidate despite the loaded CAD quality gates).
- `colcon build --packages-select mtc_vision_adapter --symlink-install`: one
  package succeeded. Actual offline DDS subscriber received 12 arrays and 12
  diagnostics, including the supported top. Historical arrays correctly remain
  empty; no acquisition timestamps or valid poses were fabricated.

## Manual desktop test and rollback

From the VMware Ubuntu graphical terminal, after connecting the D435i to Ubuntu:

```bash
cd ~/meituan_vision_ws
bash vision_core/scripts/start_simplified_live_ros_ui.sh
```

The existing script sources ROS Jazzy/workspace and uses the installed vision
venv. It starts one camera shared by preview, perception and publishing. `s`
saves a diagnostic snapshot, `r` starts/stops bounded recording, and `q`/Escape
exits and releases the camera. Do not independently start another camera publisher.
Start with one battery and a view exposing the whole top cap; then rotate/move
the camera and test all four colours in 2×2 without a fixed colour-slot order.
Include partial cap occlusion, background lookalikes and repeated/unsynced frames.

To retain V4.5-B/C geometry on this branch, set `enable_top_cad: false` in
`config/simplified_rgbd.yaml`. Legacy remains selectable with
`--perception-pipeline legacy` in the existing native entry. The original V4.5-B
branch/commit remains available; the four V4.5-C diagnostic lines are preserved
in this delivery. No reset, merge or push is required for comparison.


### V4.5-E local plane consistency

Seed windows and the global residual band propose an observed pixel rectangle;
they do not supply the final normal. The final plane is robustly refit using
range-valid even-parity pixels inside that unchanged automatic rectangle and
current object mask. Seed depth median/MAD statistics also use training pixels
only. Odd-parity pixels are held out before fitting: all range-valid holdout
pixels count, including non-inliers, with the existing 70% support and 3 mm P95
requirements. The holdout area is not eroded, cropped to inliers, or selected by
residual. Adjacent pixels share sensor correlations; this split is independent
of fitting membership, not proof of statistical independence or true accuracy.

Observed corner rays are intersected with the refitted plane. All four measured
sides must still match the CAD 66 x 22 mm cap with the existing dimensional
tolerances; RGB boundaries, depth coverage, object extremality and uniqueness
remain required. Diagnostics expose local training count, inlier count, normal
change and holdout population. No temporal state, fixed camera ROI, pose validity
or execution authorization is added. Ring centers remain CAD-derived candidates.
