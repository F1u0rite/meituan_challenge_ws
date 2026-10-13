# Yuze: external CAD and one-way insertion

The permitted physical direction is from the robot side into the aperture.
The observed axis still has to be estimated; the sign constraint is separate.
No source thresholds, top-plane fit, ROS IDL or motion permission were changed.

## Read-only CAD

SimplifiedPerception(geometry_path=...) reads external geometry.json without
copying CAD or changing that file. An explicit missing file raises an error.
The result records geometry-file SHA256 and declared battery STEP SHA256; the
declared STEP hash is provenance, not an independent STEP verification.
The existing preview/replay entry forwards --cad-geometry PATH in simplified mode.
Omitting it preserves the existing local-cache fallback.

## Current-frame reference API

Call process(frame, source_frame_id, insertion_reference=reference). The reference
has coordinate_frame=camera_color_optical, the identical source_frame_id and
capture_timestamp, a nonzero finite robot_to_battery_camera vector pointing from
robot side toward the battery, and a source description.

The vector must come from a current-frame station/robot transform or an explicit
current-frame observation. It is never cached, inferred from fixed camera -Z,
or generated from an unsigned axis as evidence. Candidate alignment must exceed
0.5; this is a sign-selection diagnostic gate, not a calibrated angular tolerance.

insertion_direction_selected chooses an existing observed-axis candidate.
insertion_direction_unique describes this conditional sign selection only.
Both original candidates remain available for diagnostics. Missing, stale,
wrong-coordinate, degenerate or poorly aligned references stay unresolved.
No currently supplied dataset contains a verified robot-side reference in camera
coordinates, so real replay must report unresolved signs rather than fabricate one.

Body/ring pose validity and motion_usable remain false. The 20 mm top-to-hole
offset and geometric gates are unchanged. Absolute <=3 mm accuracy is unmeasured.
