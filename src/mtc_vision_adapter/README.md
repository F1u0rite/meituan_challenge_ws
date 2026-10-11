See ../../vision_core/README.md for installation and pending Jazzy checks.
Topic /mtc/perception/batteries; RELIABLE KEEP_LAST(5) VOLATILE.
UNKNOWN=0 RED=1 BLUE=2 YELLOW=3 GREEN=4.
Replay only; invalid poses JSON null -> ROS NaN; ring_pose_valid=false; no motion.

Motion safety contract:
`motion_rejection(..., mode="live")` rejects by default. A caller must supply
`trusted_frames`, a trusted `target_frame`, and a callable
`tf_validator(source_frame, target_frame, capture_stamp)`. The validator must
return exactly `True` only after resolving a usable transform at the capture
stamp (not latest TF) into the trusted motion frame. Lookup failures, timeouts,
and missing/uncertain results reject. Frame names alone never establish trust.
This is an independent safety helper; it does not wire or validate the C++ FSM
ROS adapter. Synthetic/replay publication remains allowed and cannot be motion
input. The publisher forces `ring_pose_valid=false`.

Jazzy validation: source `/opt/ros/jazzy/setup.bash`, build this package and
`mtc_interfaces`, then run `python3 -m pytest -q src/mtc_vision_adapter/test`
with the workspace overlay sourced. TF tests use a local buffer, without a
network listener or motion execution.

Process validation: run python3 src/mtc_vision_adapter/test/ros_replay_validation.py
with the overlay sourced and an isolated local ROS domain. Artifacts default to
/tmp/mtc_vision_validation; override with MTC_VALIDATION_OUT.
MTC_REPLAY_FIXTURE can select a replay JSONL fixture without changing capture stamps.

QoS validation checks the profile on the created publisher and discovered
endpoint compatibility. On this Jazzy RMW, discovery reports history UNKNOWN
and depth 0; those fields do not verify publisher queue configuration.
The process test waits for a matched subscriber before starting its timer.
Replay validation may reuse synthetic records with mode=replay solely to test
the replay path; this does not constitute a real camera recording.
