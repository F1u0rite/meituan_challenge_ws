# V4.5-B current-frame body depth diagnostics

The body center bottleneck was not failed depth-to-XYZ conversion. In 353 saved
H4 pairs, all 343 detection-valid observations had valid depth, a cloud and a
visible-surface XYZ. Only 16 passed the whole-mask single-plane condition. All
343 passed the old span prior; 327 failed the 3 mm whole-cloud P95 plane gate.
The median whole-cloud P95 residual was 26.32 mm. The protruding ring and body
expose multiple surfaces, so whole-object planarity does not test one body face.

## Minimal implementation

`BodyPoseEstimation.estimate` still uses the current mask, SDK-aligned Z16,
original scale and intrinsics. It removes a two-pixel mask border, keeps both
original-mask and interior-mask valid-depth ratios >=0.50, at least 100 points,
the existing depth range/MAD rule, and the cap of 1000 sampled cloud points.
It records pixel counts, depth quantiles, rejection fraction and cloud reference.

The existing seeded `fit_plane_ransac` isolates a current observed face. A model
origin hypothesis needs >=70% support among filtered sampled points, >=100 face
points, face P95 residual <=3 mm and unchanged CAD span scales 0.70..1.25.
This does not loosen the distance threshold: it applies it to a supported face
rather than mixing the entire multi-face battery into one plane.

Definitions remain distinct:

- `visible_surface_center_xyz`: robust sample median of observed interior cloud,
  not the body CAD origin.
- `quality.face_center_camera_m`: observed in-plane quantile-bounds center,
  not proof that the full CAD face was visible.
- `body_xyz_candidate`: TOP-face-conditioned bounds center offset by CAD half
  height, explicitly model-conditional, not a uniquely established body origin.
- `body_origin_candidates`: all compatible TOP/LONG_SIDE/SHORT_SIDE center
  hypotheses, with face assumption and coordinate definition.

Every H4 recovered model candidate still has multiple compatible face origins.
Body/Ring Pose Valid remain false. Surface completeness, face identity, yaw
symmetry, full orientation and XYZ metrology are not established. No candidate
is encoded as an executable Pose; ROS invalid-Pose encoding/header mapping stay
unchanged. The old 3D values were already retained as surface XYZ, not lost by
the final interface state conversion. Never relabel that surface as a body origin.

## Actual paired comparison

Use **detection-valid observations** for all rows below:

| Metric | H3 old -> new | H4 old -> new |
|---|---:|---:|
| Original pair count | 117 -> 117 | 353 -> 353 |
| Detection Valid | 105 -> 105 | 343 -> 343 |
| Qualified depth mask / cloud / surface XYZ | 105 -> 105 | 343 -> 343 |
| Conditional body origin candidate | 12 -> 14 | 16 -> 46 |
| Body Yaw candidate | 105 -> 105 | 343 -> 343 |
| Body/Ring Valid | 0/0 -> 0/0 | 0/0 -> 0/0 |
| Ring Center candidate | 0 -> 0 | 1 -> 1 |
| Mean full pipeline time | 65.32 -> 85.12 ms | 38.59 -> 58.19 ms |
| P95 full pipeline time | 80.29 -> 105.42 ms | 49.40 -> 73.33 ms |

No old body candidate was lost. H4 retains its 16 and gains 30 current-face
hypotheses; 297 still lack enough supported face evidence. H3 retains 12 and gains
2; 91 still fail. No candidate passed with bad sync or a depth ratio below the
required minimum. These are gate checks, not an independently measured false-pose
rate. More hypotheses are not proof of improved accuracy.

Counting every object, including unconfirmed identity observations, gives H3
XYZ 13->15 and Yaw 115->115; H4 XYZ 16->46 and Yaw 344->344. The original V4.5
headline counts used that broader scope. H5 audited the same H3/H4 inputs, so
does not add independent frames. All tests use the same recorded pair IDs/times.

New plus existing targeted tests: 11 passed. Full regression: 315 passed, 10
skipped, 1 known legacy baseline failure (`test_native_real_offline_ros_process`,
synthetic red fixture expects a candidate under the original CAD-aware path).
Do not claim the full suite is green. ROS adapter colcon build: 1 package passed.
Actual independent ROS subscriber received 20 arrays and 20 diagnostics, 19 RGB
identity observations; historical timestamps preserved, targets filtered as stale,
zero executable poses. Camera never started. No IDL/FSM/MoveIt/TF/hand-eye changes.

Real-data visuals and PLYs stay in private validation storage. Where Matplotlib
3D extension is unavailable, PNGs use actual cloud XY with optical Z color-coded;
PLY contains original XYZ vertices, not invented or registered geometry.
No ground-truth position accuracy or millimetre pose accuracy is claimed.

## Manual verification and rollback

From Ubuntu VMware graphical desktop, with the existing environment:

```bash
cd "$HOME/meituan_vision_ws"
bash vision_core/scripts/start_simplified_live_ros_ui.sh
```

q/Esc/Ctrl+C exits/releases; s saves, r records. No second camera publisher.
Verify an unobstructed known CAD face, then tilted/side/occluded counterexamples.
Record face support, original/interior depth quality, source frame, alternatives
and measured camera/UI/Fast FPS. Independent body-origin distance/face identity
and orientation evidence are needed before any valid pose. The new face/cloud
reference may help delimit local depth/ROI, but the ring module still uses direct
local RGB-D; do not derive ring position from the assumed body center.

On this branch, `--perception-pipeline legacy` still selects V4.4. To restore
the exact original V4.5 implementation, save work then:

```bash
git switch codex/v45-modular-perception
```

Its commit remains d5a77bceb4bdf1d1b74c5a2b707741ef18eb9f27. Switch back with
`git switch codex/v45b-body-depth`. No reset, merge or push is required.
