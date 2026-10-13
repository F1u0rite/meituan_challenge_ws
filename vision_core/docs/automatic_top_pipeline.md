# Automatic top prompts and segmented centres

This opt-in offline pipeline runs current-frame RGB masks -> automatic support
plane and cap seed -> SAM 2.1 -> segmented rectangle centre -> axial CAD hole
centre. No manual pixel coordinates, fixed camera pose, colour slots, history
reuse are used by the raw evaluator. The separate ROS runner now adds user-confirmed
80/60 mm physical-height constraints; see automatic_top_coordinates.md and
automatic_top_setup.md. Existing legacy LIVE UI pipeline selection remains unchanged.

The physical assumption is upright batteries on a common support, with an
approximately 80 mm top height. A low-chroma background proposes the dominant
support plane. Every object must have depth support near that expected height
and a cap-plane seed approximately parallel to the current support plane.
The positive click is inside the observed seed; a lower body region supplies
a negative click. Background clutter, tilted batteries and unknown surfaces
can invalidate this hypothesis; missing candidates remain explicit.

SAM masks must cover the current seed, stay mainly inside the current object
and have bounded area. A duplicate eligible colour is marked ambiguous rather
than choosing an arbitrary object. RGB confirmation remains separate.
The current support-plane normal stabilizes the centre calculation under the
upright assumption. Its frame number and SDK capture time must exactly match.
The cap offset is measured from current seed depth, not fixed at 80 mm.
70x25 mm is a separate user-provided size reference; original CAD is unmodified.
The hole offset stays 20 mm, no hole detection is performed.

Dependencies: a separate SAM environment and official checkpoint. CPU prototype
takes seconds per frame. SAM confidence, geometry checks and temporal variation
are diagnostic, not absolute accuracy or valid robot poses.

Example from the Yuze workspace:
build/sam21_cpu/bin/python -I -B vision_core/scripts/evaluate_automatic_top.py   --input log/yuze_common_top_20261013_150253   --output log/NEW_AUTOMATIC_TOP_RUN   --geometry /home/shrmeituan/private_data/v44g_cad/geometry.json   --checkpoint log/sam21_experiment/sam2.1_hiera_tiny.pt

Outputs: per-frame mask PNGs, overlay PNGs, rows.json and summary.json.
For each colour, green_point_camera_m is observed top centre and
red_point_camera_m is CAD-derived hole centre. No missing coordinate is filled
from a previous frame; raw XYZ remains in camera_color_optical.
Pose-valid flags and motion_usable remain false. No camera or robot is started.

Validation: see log/yuze_automatic_top_validation and auto_top regression tests.
