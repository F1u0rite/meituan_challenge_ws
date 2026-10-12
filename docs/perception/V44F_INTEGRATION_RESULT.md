# V4.4-F Ubuntu visual semantics integration

Date: 2026-10-12. Workspace: /home/shrmeituan/meituan_vision_ws.
Branch: codex/v44c-native-runtime. Parent: d5f9fc9ccfe30ac4b16a2bacfaa763ad80e28bc3.

## Package and integration

Package SHA256 verified:
6c59fb85fd27499d836952182e6404d8752ea24407252bf9f201a8362948265f.
ZIP staged privately, not extracted onto the workspace. Absolute/traversal/drive paths,
duplicate entries and symlinks rejected; total uncompressed size bounded.
All 14 manifest files and listed SHA256SUMS verified. Every declared target matched
its payload-relative path under vision_core. Six existing files matched e09d00e text
exactly; Windows CRLF baseline hashes were independently verified. Eight paths were new.
Original files backed up privately before applying the reviewed increment with LF endings.

Fourteen payload paths integrated:

- vision_core/perception/dual_rate_rgbd.py
- vision_core/perception/vision_interface.py
- vision_core/perception/vision_ui.py
- vision_core/perception/live_vision_protocol.py
- vision_core/perception/tcp_vision_sender.py
- vision_core/config/fast_confirmation.yaml
- vision_core/config/tcp_sender.yaml
- vision_core/scripts/dual_rate_runner.py
- vision_core/scripts/live_rgbd_preview.py
- vision_core/scripts/send_vision_tcp.py
- vision_core/docs/v44b_protocol_compatibility.md
- vision_core/tests/test_detection_pose_semantics.py
- vision_core/tests/test_dual_rate_rgbd.py
- vision_core/tests/test_tcp_sender.py

Ubuntu additions:

- src/mtc_vision_adapter/mtc_vision_adapter/native_rgbd.py: separated detection freshness
  and ambiguity mapping; current confirmed detection is not hidden by old/ambiguous CAD.
  Expired/unknown detection freshness remains rejected; legacy mapping preserved.
- src/mtc_vision_adapter/test/test_native_detection_semantics.py: native mapping,
  invalid ROS pose and unchanged Bridge interoperability regressions.
- docs/perception/V44F_INTEGRATION_RESULT.md: this result record.

No unresolved overlap conflicts. The existing Ubuntu live_tcp_bridge/live_protocol,
ROS IDL, FSM, MoveIt and RealSense source were not overwritten. TCP sender is opt-in
and loopback-only. No dependency installation, camera start, robot control, push,
merge or reset occurred. No private data or CAD assets are included in the commit.

Private Windows-dependent tests retained with explicit skipif availability checks;
the original assertions are unchanged. One delivered UI trailing-space/extra-EOF
formatting issue was removed. No unrelated algorithm changes were applied.

## Actual validation

- Directed semantics/confirmation/interface/TCP/dual-rate/native compatibility:
  70 passed, 9 skipped, 2.07 s.
- Combined vision_core + ROS adapter tests after colcon build:
  257 passed, 10 skipped, 41.81 s.
  Nine skips require unprovided Windows private recordings; the remaining skip is
  the existing private-evidence skip. Missing evidence is not counted as a pass.
- colcon build --packages-select mtc_interfaces mtc_vision_adapter:
  two packages finished, 8.14 s.
- git diff --check passed after whitespace cleanup.
- All 1520 original V4.4-D array/metadata hashes matched the V4.4-E manifest.

## Real RGB-D offline results

All 331 independent approved original pairs were processed in order with the
updated NativeProcessor and its real FastPerception path. Frame IDs, capture
timestamps and raw arrays were not rebased, reassociated or modified. Only approved
pairs were used; no SDK device was constructed. Initial representative frames
completed before expanding the run to all approved pairs.

Full STEP-derived CAD pipeline could not be instantiated: genuine private STEP
assets, geometry.json and cad_meshes.npz are absent from the workspace and inspected
private directories. No substitute geometry, guessed dimensions or synthetic cache
was used. This run therefore validates the native fast fallback, not full CAD.

- Raw red HSV proposals: 766 observations across 331 frames (0-10/frame).
- Surviving red diagnostic object observations: 211; all colors: 369.
- Object statuses: CONFIRMED_OBJECT=0, AMBIGUOUS=369, REJECTED object records=0.
  Rejected raw proposals are logged separately: 8775 across all colors;
  red rejected proposals=555 (500 LOW_VALID_DEPTH_OR_POINTS, 55 OUTSIDE_FAST_CAD_SIZE_PRIOR).
  Observations are not counts of unique physical batteries or accuracy measurements.
- 298 frames contain diagnostic objects; 33 do not.
- 108 red observations have centers in the manually selected target-neighborhood ROI;
  261 all-color diagnostic observations lie outside it. These are background-suspect
  observations, not annotated false positives or a measured false-positive rate.
- Confirmation reasons: NO_FRESH_STABLE_TABLE_EVIDENCE=369,
  SHARED_CAD_DIMENSIONS_UNAVAILABLE=369, INCOMPLETE_CONTOUR_SUPPORT=285,
  ASSOCIATION_NOT_ESTABLISHED=34. Reasons may overlap per object.
- Detection ambiguity=true for all 369 objects. Detection stale=true for all 369:
  original historical host capture time was compared to actual processing wall time.
  This expected historical age is not evidence that newly captured live images are stale.
- pose_ambiguity=false for 369 fast objects, while pose validation remains independently
  unavailable. No CAD body/ring candidate was produced; CAD ran on zero frames.
- Native processor wall duration mean 178.60 ms, P50 170.68 ms, P95 287.72 ms,
  max 349.51 ms, excluding array loading/saving. Existing fast timing aliases report
  the whole fast pass as detection, not isolated detector latency. CAD duration is
  NOT RUN, not a validated zero-cost CAD measurement.

## Actual native ROS2 subscriber

The rebuilt installed native_rgbd_publisher ran offline on five selected real
approved original pairs. An independent rclpy subscriber on localhost ROS domain
219 received five BatteryDetectionArray messages plus five diagnostic messages.
Diagnostic object counts: 1,1,1,2,2. Original timestamps matched exactly; exit 0.
Detection arrays were all empty because historical stale/unconfirmed objects are
excluded. This is successful transport and filtering, not nonempty confirmed-real
object publication. Nonempty confirmed mapping and NaN body/ring components were
separately verified with native ROS message classes in the directed fixture tests.

body_pose_valid=false, ring_pose_valid=false, motion_usable=false. ROS header uses
the existing zero unmapped-clock stamp. Invalid XYZ/quaternion are not fabricated.
No actual C++ FSM or mechanical execution-chain acceptance is claimed.

## Remaining gates and V4.4-G

Separated visual semantics, offline UI-facing results, Bridge compatibility and
native offline transport are ready for bounded offline UI integration. Live UI,
continuous qualified acquisition, full CAD and confirmed-real target recognition
are not accepted yet. Before claiming those, supply genuine private CAD/cache and
validate table/outline confirmation, background rejection, synchronization/dedup,
processing freshness and throughput. Prior V4.4-E synchronization issues remain;
this semantic integration does not fix the SDK acquisition path.

## Host-local private evidence

/home/shrmeituan/private_data/v44f_handoff/

- staged_verified/, preintegration_backup/, integration_inventory.json
- directed_pytest.log, comprehensive_postbuild_pytest.log, colcon.log
- real_replay.log, real_replay/{summary.json,detections.jsonl,ros_samples/}
- real_ros_validation.log, real_ros_acceptance/{results.json,arrays.json,diagnostics.json,publisher.log}
- input_integrity_result.json

No private evidence files above are committed. Local commit contains only the
reviewed source/config/test increment and this report. Task stops after committing.
