# Reject wrongly labelled scene reference frames — 2026-10-14

The automatic top-coordinate ROS boundary accepted a scene reference whose
`coordinate_frame` was `base_link`, `camera_color_optical`, empty or absent. It
then published this reference alongside `scene_coordinates.coordinate_frame=t0_scene`.
The original camera-space BatteryDetectionArray remained invalid for motion, but
the diagnostic JSON had contradictory destination-frame labels.

The boundary now requires `scene_reference.coordinate_frame == 't0_scene'` before
either publication. Valid current-scene references, explicitly unready references
and legacy input without a scene reference retain their previous behavior. The
eight added tests cover four wrong/missing labels with both ready and unready
references. No geometry, transform computation, IDL, motion-validity flags,
FSM/Action/Cancel or Motion Execution semantics are changed.

The monitored source was `5efbd2d4f9e7d7d3a47ce91f199cbd1520f95c4c`. PR17 was already squash-merged by upstream;
repair base `4e7ddd4794c28fdd423aaf67477fee5b8f40d2a0` on `feature/perception-adapter` has an identical Git tree.
This branch targets that feature branch, not main. It does not advance any formal pin.

| Isolated check | Result |
| --- | --- |
| Original installed replay → real DDS receiver | All 4 invalid inputs wrongly published both topics; probe FAIL retained |
| Rebuilt patched adapter → same DDS receiver | All 4 rejected, exit1 and zero publications; 3 compatibility cases pass |
| Full adapter pytest, installed patched module | 168 passed, 0 failed/errors/skips, including 8 new cases |
| Source/installed module identity | Matching patched SHA; 132 vision-core Python files unchanged |
| Exit cleanup | Per-case child ended; receiver destroyed; only sleep keeper before container stop; stopped/Pid0/ExecIDs null |

The environment was Ubuntu24.04/Jazzy, Python3.12.3, pytest8.3.5, 2 CPU/4 GiB,
network none, no devices, host mounts or published ports. Dependencies came from
an existing offline cache. The adapter was built into `/fix-install`, using
unchanged interfaces from `/candidate-install`. Replay was a separate installed
process and two actual DDS subscriptions, not a mocked publisher call.

The first regression invocation used a package directory without the repository's
`src/` level. Two unrelated launcher tests consequently resolved their repository
root as `/`; 166 passed and 2 failed. Both original log/XML are included. Running
the identical tests in the full `/fix-review/src/mtc_vision_adapter` layout passed
all168. No assertion was weakened. The original branch still lacks pytest colcon
registration; this repair uses explicit pytest and does not claim that separate
packaging issue is solved.

To repeat, prepare the exact base and unchanged interfaces in an equivalent
isolated Jazzy workspace, apply only this patch, build the adapter using the
recorded build command, and run the recorded pytest command. For the before/after
DDS probe, source each installation in a separate invocation, set ROS_DOMAIN_ID=218
and run `boundary-probe.py --tag before|after --install-base <that installation>`.
The probe generates its synthetic JSON inputs and writes all received data/logs
under `/review-evidence/boundary-<tag>`. Use fresh output directories. Preserve
the repository layout and set both MTC installation variables shown in the script.

The independently observed manual-direction/T0-loss/movement issue remains open
for the vision owner. This guard does not validate scene-frame estimation,
absolute accuracy, live SAM/camera inference, robot transforms or full FSM use.
Existing Python Action and MoveIt shutdown failures remain outside this fix.

Focused [evidence index](evidence/scene_reference_frame_20261014/index.json)
SHA-256: `829b6550715025f4c21b737a7abad3b5e24b44231b10ed8543e24841bf42bd49`. Raw outputs, cached dependencies, local configuration and
unfinished features are excluded. The full event archive remains in the local
integration workspace. OpenAI Codex assisted with diagnosis, the boundary patch,
regression and documentation; upstream license declarations remain unchanged.
