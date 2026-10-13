# Live Vision Protocol V1 — V4.3-A Ubuntu diagnostic receiver

Status: protocol_version=1, diagnostic-only. This is the sole sender/receiver
contract for this stage. No live camera, trusted clock mapping, TF or robot
acceptance is claimed. Required source checkout: feature/perception-adapter.

## Transport, size, decoding and failure behavior

- TCP byte stream; server fixed at IPv4 127.0.0.1:8765 on Ubuntu.
  listen_port is configurable; 0 requests an ephemeral port for tests.
  There is no host/interface parameter and no public bind.
- Each frame is 4 bytes unsigned big-endian payload length N, followed by
  exactly N bytes of UTF-8 JSON. N counts encoded bytes, excluding the prefix.
  1 <= N <= 262144 (256 KiB). No newline terminator, BOM or compression.
  TCP reads may split or combine frames; boundaries come only from the prefix.
- Length 0, oversize length, truncated frame or partial-read timeout closes
  the client connection. Oversize payload is not allocated or drained.
  Once any prefix byte arrives, prefix completion has a 2 s deadline;
  once the complete prefix arrives, payload completion has its own 2 s
  deadline. An entirely idle connected client may wait indefinitely.
- Exactly one client is processed at a time (listen backlog 1).
  No multi-camera multiplexing or commands exist in V1.
- Illegal UTF-8/JSON, duplicate JSON keys, unsupported versions, wrong types,
  extra/missing fields, duplicate object IDs or non-finite values reject the
  entire frame. Valid length frames with invalid payload are counted and
  discarded; the connection remains usable. No exceptions execute user data.
- JSON NaN/Infinity/-Infinity and numeric overflow to infinity are forbidden.
  A JSON boolean is never accepted as an integer or numeric measurement.
- No receiver ACK is sent. sendall completion is not proof of ROS publication;
  use Ubuntu diagnostics/subscriber evidence. EOF means reconnect is needed.
- No disk paths, executable names, TF/control/motion commands, filenames or
  extension fields are accepted. Only the exact schema below is legal.
- One latest-frame slot replaces older pending frames; superseded count is
  reported. ROS QoS queues are depth 5. Sender must drop its own unsent older
  frame rather than accumulate a historical queue. Reconnection is not a
  replay/flush operation. Send the newest observation with a fresh session.

## Schema and exact types

ALL top-level fields and ALL object fields listed here MUST be present.
“nullable” means JSON null is permitted; missing keys are never a substitute.
Extra keys are rejected. There are at most 128 objects per frame.

| Top-level key | Exact type / meaning |
| --- | --- |
| protocol_version | integer exactly 1 |
| sequence_id | integer 0..9007199254740991, never bool; session rules below |
| mode | string: live or test; replay/synthetic are not V1 live transport modes |
| camera_frame_id | nonempty frame identifier of the actual capture camera |
| capture_timestamp_raw | nullable finite number >=0, in milliseconds exactly as returned by the camera SDK; null only with domain unknown |
| timestamp_domain | string: hardware_clock, system_time, global_time, simulated, unknown |
| windows_capture_wall_time | nullable object with EXACT keys sec and nanosec; Windows UTC Unix host wall clock sampled for the captured frame |
| source_frame_id | frame identifier in which ALL spatial candidates are expressed; also ROS header.frame_id |
| objects | array of object records, including unconfirmed diagnostic candidates; empty array allowed |

Frame identifiers match [A-Za-z][A-Za-z0-9_/]*, maximum 128 characters.
A name is a declaration, not TF verification. Standard camera optical axes
are +X right, +Y down, +Z forward. Ordinarily both frame IDs are the sender's
color optical frame. Any other source_frame_id still confers no TF trust.
Spatial candidates require a defined source frame; send null when unknown.

windows_capture_wall_time is {sec: integer 1..2147483647,
nanosec: integer 0..999999999}. It preserves integer precision. sec is UTC
Unix seconds, not local-time text, milliseconds, monotonic ticks or ROS time.
Sample a Windows host wall clock when acquiring the corresponding frame;
this does not establish its mapping to the sensor's acquisition clock.
Do not replace historical observations with a new wall timestamp.

| Object key | Exact type / meaning |
| --- | --- |
| object_id | string matching [A-Za-z0-9][A-Za-z0-9_.:-]{0,95}; unique per frame and stable for the tracked target |
| color | string: UNKNOWN, RED, BLUE, YELLOW, GREEN |
| confidence | finite number 0..1 |
| detection_status | string: CONFIRMED, RAW_CANDIDATE, AMBIGUOUS, REJECTED |
| surface_depth_m | nullable finite number >0; measured visible-surface depth in m, not inferred battery-center Z |
| body_position_candidate_m | nullable array [x,y,z], all finite numbers, m in source_frame_id; candidate body center |
| body_orientation_candidate | nullable array [qx,qy,qz,qw], all finite, squared norm within 0.001 of 1; body-frame-to-source rotation candidate |
| body_yaw_candidate_rad | nullable finite number in [-pi,pi], radians; candidate right-hand yaw about source +Z; not a grasp angle |
| ring_position_candidate_m | nullable array [x,y,z], finite, m in source_frame_id; candidate ring-frame origin |
| ring_orientation_candidate | nullable [qx,qy,qz,qw], same norm/rotation convention, ring-frame-to-source candidate |
| insertion_direction_candidates | nullable array of 0..4 vectors [x,y,z]; each finite, squared norm within 0.001 of 1, expressed in source_frame_id; unitless diagnostic directions only |
| ambiguity | boolean; true means candidate is unresolved regardless of detection_status |
| stale | boolean; true means object observation is already stale at sender |
| body_pose_valid | boolean exactly false in this stage; true rejects the entire frame |
| ring_pose_valid | boolean exactly false in this stage; true rejects the entire frame |

An unavailable scalar/vector/quaternion is null. Within a provided vector
all components must be finite measured/candidate values; partial null vectors
are illegal. Never substitute [0,0,0], [0,0,0,1], guessed yaw or invented depth
for an unmeasured quantity. Measured/CAD candidates may be transmitted for
diagnostics, but never assert validity. Direction [] means there are zero
candidates; null means not measured/computed.

CONFIRMED must be emitted only after the existing sender vision confirmation
gate; it is not merely a raw model candidate. This contract does not add or
change the Windows vision algorithm. The receiver cannot independently verify
that a sender's CONFIRMED assertion represents a real object.

## Sessions, order and reconnect

The first schema-valid frame on every TCP connection must have sequence_id=0.
Subsequent schema-valid sequences must strictly increase; gaps are allowed
(latest-frame operation), duplicates/regressions are dropped.
Reconnect resets the sequence/capture-order state; sender resets to 0.
Object identities should use a sender boot/session namespace so a restarted
tracker does not silently reuse unrelated physical identities.

mode, camera_frame_id, source_frame_id and timestamp_domain must remain
constant per connection; changed sources are dropped. Reconnect to change
sources or reset a hardware timestamp. capture_timestamp_raw must strictly
increase within a session when non-null. Raw regression/duplicates drop frames.
Sequence is consumed for schema-valid frames before freshness/source checks,
even if those later checks drop it. Schema-invalid payloads do not consume it.
Do not resend a rejected old sequence; emit a newer fresh observation or reconnect.

## Time provenance and conservative degradation

No trustworthy Windows-to-Ubuntu or sensor-to-host clock mapping exists here.

1. Keep capture_timestamp_raw and timestamp_domain unchanged in diagnostics.
   hardware_clock is device time, system_time/global_time are SDK domain labels,
   simulated is allowed only with mode=test. None establish receiver trust.
2. Keep windows_capture_wall_time unchanged; accepted ROS Header stamp uses
   THIS claimed Windows capture host wall time, not raw device ms or receive time.
   Integer sec/nanosec are copied exactly. ROS header.frame_id is source_frame_id.
3. Ubuntu records receive_wall_ns (local time.time_ns) and
   receive_monotonic_ns when the complete payload is read. The diagnostic
   ubuntu_ros_publish_stamp_ns is ROS time at dispatch, not capture/receive time.
4. timestamp_domain=unknown, absent raw timestamp or null Windows wall stamp
   causes TIME_UNKNOWN: diagnostics only, no BatteryDetectionArray.
5. Default host-wall sanity window: receive_wall_time minus the claimed Windows
   wall time must be between -1 and +5 s. Outside this range drops with
   WALL_FUTURE/WALL_OLD. This compares UNTRUSTED different host wall clocks;
   it may reject honest messages when clocks drift. It does not measure true
   acquisition latency or prove motion freshness. Synchronize host clocks for
   diagnostics, but do not treat that as a sensor/ROS clock calibration.
   Configurable positive finite max_wall_age_s=5, max_wall_future_s=1.
6. Pending data older than max_queue_age_s=0.5 by Ubuntu monotonic time is
   dropped as QUEUE_OLD. This bounds LOCAL dispatch lag only.
7. These checks never enable motion. All published Pose values remain invalid,
   even for live mode, finite candidates and plausible wall times.

## ROS output and diagnostics

live_tcp_bridge publishes both topics itself; no separate publisher is needed.

- /mtc/perception/batteries: mtc_interfaces/msg/BatteryDetectionArray,
  RELIABLE + KEEP_LAST(5) + VOLATILE.
- Only objects with detection_status=CONFIRMED, ambiguity=false, stale=false,
  known color and confidence>=0.2 enter the array. All others are filtered,
  counted and remain in diagnostic metadata. A legal frame with no eligible
  objects still publishes an empty array.
- Color mapping: UNKNOWN=0 (filtered), RED=1, BLUE=2, YELLOW=3, GREEN=4.
  IDs are unchanged. confidence converts to ROS float32 (normal rounding).
- Every body_pose and ring_pose position/orientation component is NaN.
  Every ring_pose_valid is false. Pose headers equal the array header.
  Candidate XYZ/quaternions are NOT used in ROS Pose.
- Existing IDL has no body_pose_valid field, so non-finite body Pose is the
  existing rejection signal. No IDL, C++ FSM, TF or motion code is modified.
- /mtc/perception/live_diagnostics: std_msgs/msg/String containing strict JSON:
  message (original schema-valid frame, including candidates), reason (null or
  drop code), disposition (published/dropped), receive_wall_ns,
  receive_monotonic_ns, ubuntu_ros_publish_stamp_ns,
  ros_header_source=untrusted_windows_capture_wall_time,
  clock_mapping_verified=false, motion_usable=false, body_pose_valid=false,
  ring_pose_valid=false.
  Same bounded ROS QoS. Schema-invalid payloads appear only in error counters;
  no unvalidated payload is echoed or executed.
- LIVE_TCP_ROS_MATCHED logs when both output topics first have an actual
  matched subscriber. Process tests wait for this before sending; VOLATILE
  does not promise delivery to a subscriber that joins after publication.
- LIVE_TCP_STATS logs every 5 s: counts of received frames, validated frames,
  published arrays/objects, filtered objects, superseded queued frames,
  invalid_<reason>, dropped_<reason>, framing_<reason>, connections/disconnects,
  and lifetime average received_hz. Final counts are logged on shutdown.
- Disconnection never replays cached data; next session waits normally.
  This node emits no grasp TF/control command. Every downstream consumer must
  honor invalid poses; C++ FSM safety acceptance is NOT VERIFIED here.

## Complete one-red-battery example

Populate timestamps with the real corresponding host/device values. This
illustration uses explicitly simulated time and is not a real capture:

```json
{
  "protocol_version": 1,
  "sequence_id": 0,
  "mode": "test",
  "camera_frame_id": "test_color_optical_frame",
  "capture_timestamp_raw": 0.0,
  "timestamp_domain": "simulated",
  "windows_capture_wall_time": {"sec": 1791740000, "nanosec": 123456789},
  "source_frame_id": "test_color_optical_frame",
  "objects": [{
    "object_id": "testboot-red-000001",
    "color": "RED",
    "confidence": 0.9,
    "detection_status": "CONFIRMED",
    "surface_depth_m": null,
    "body_position_candidate_m": null,
    "body_orientation_candidate": null,
    "body_yaw_candidate_rad": null,
    "ring_position_candidate_m": null,
    "ring_orientation_candidate": null,
    "insertion_direction_candidates": null,
    "ambiguity": false,
    "stale": false,
    "body_pose_valid": false,
    "ring_pose_valid": false
  }]
}
```

The example's historical wall time is intentionally not refreshed and will be
dropped by the wall sanity window if sent now. Sender serializes one strict
UTF-8 JSON object, obtains its BYTE length, prefixes that length in network
byte order, then sends the prefix and payload completely. Start sequence at
0, increment for each observation and preserve all capture metadata. No
delimiter or additional handshake exists.

## Startup and Yuze's Windows connection contract

On Ubuntu from /home/shrmeituan/meituan_vision_ws:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mtc_interfaces mtc_vision_adapter
source install/setup.bash
ros2 run mtc_vision_adapter live_tcp_bridge --ros-args -p listen_port:=8765
```

Windows PowerShell SSH tunnel (keep the terminal running):

```powershell
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8765:127.0.0.1:8765 meituan-ubuntu
```

Use the already-configured SSH identity, or add
-i "$env:USERPROFILE/.ssh/meituan_codex" if needed.
Yuze Sender connects its TCP socket to WINDOWS 127.0.0.1:8765;
SSH carries it to UBUNTU 127.0.0.1:8765. Do not connect directly to the VM's
network interface or expose a firewall/public listener. A different Windows
local port is allowed if the sender's destination matches it.

The sender owns: exact V1 schema, capture provenance, per-session sequences,
confirmed-target assertion and dropping unsent history. The Ubuntu bridge
owns: framing/type/order checks, diagnostic time sanity, bounded latest-frame
dispatch and invalid-Pose ROS publication. No Windows sender is implemented
in this task. NUC migration can reuse live_protocol.diagnostic_array and
core.to_ros with a new local input transport; no algorithm rewrite is required.

## Local no-camera acceptance

From a sourced, built workspace:

```sh
MTC_LIVE_VALIDATION_OUT=/tmp/mtc_v43a_validation_20261012 \
  python3 -m pytest -q src/mtc_vision_adapter/test
```

Tests use localhost and isolated ROS domains. Required live cases include red,
four colors, same-color IDs, empty scene, UNKNOWN and unconfirmed filtering,
finite candidates remaining NaN in ROS, old/future/unmapped timestamps,
sequence/raw regressions, source-domain change, reset on reconnect, invalid
JSON/UTF-8/versions/types/duplicate IDs/validity flags, zero/oversize prefixes,
fragmented reads, truncation/timeout, bounded latest-frame buffering and SIGINT
with a connected socket. The integration test launches the installed receiver
and subscribes to real native BatteryDetectionArray plus diagnostic String.

Runtime evidence is host-local and not committed:
- /tmp/mtc_v43a_colcon.log
- /tmp/mtc_v43a_live_pytest.log
- /tmp/mtc_v43a_pytest.log
- /tmp/mtc_v43a_validation_20261012/live_bridge.log
- /tmp/mtc_v43a_validation_20261012/live_results.json
- /tmp/mtc_v43a_validation_20261012/ros_received.json
- /tmp/mtc_v43a_validation_20261012/diagnostics_received.json

Real D435i, actual Windows SSH-tunnel traffic, true end-to-end acquisition
latency, clock calibration, C++ FSM consumption, XYZ/Yaw/ring precision and
grasp safety remain NOT RUN / NOT VERIFIED.

## Actual V4.3-A acceptance — 2026-10-12

Ubuntu 24.04 / ROS2 Jazzy formal workspace acceptance:
- colcon: mtc_interfaces and mtc_vision_adapter, 2 packages finished.
- All adapter directed pytest: 114 passed in 23.63 s.
- Installed bridge process: 11 native ROS arrays received and field-checked.
  Red, four colors, same-color separate IDs, empty/filter-only arrays and live
  hardware-clock-labelled provenance cases passed; all poses remained NaN.
- 29 length-valid payloads received; 11 arrays published, 11 objects published,
  5 objects filtered. Invalid JSON/schema, old/future/unknown time and ordering
  rejection evidence is in live_results.json and diagnostics_received.json.
- 8 connections; reconnect sequence reset, zero/oversize prefix closure,
  truncated payload and partial-read timeout all passed without node crash.
- SIGINT with an idle connected client: process exit 0, no residual node;
  server port could be rebound after shutdown.
- Burst test: 100 accepted packets retained only sequence 99 in one pending
  slot, with 99 superseded packets counted; worker shut down cleanly.
- Existing replay natural-exit/SIGINT and trusted-TF rejection regressions
  remain passing. No camera or Windows sender was started.

These numbers describe only the Ubuntu local simulation. Real Windows
SSH-forwarded traffic, D435i, C++ FSM, trusted clock mapping and grasp accuracy
remain NOT RUN / NOT VERIFIED.
