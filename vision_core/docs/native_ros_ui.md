# Native H1 UI with H2 ROS2 result dispatch

The opt-in `--ros-publish` sink consumes the already computed H1 dual-rate
result. It never constructs a camera source or reruns Fast/CAD processing.
The original standalone native_rgbd_publisher, TCP bridge and offline playback
remain available. Do not run their camera input concurrently with the LIVE UI.

## Manual desktop start

First exit any previous LIVE UI with q/ESC and wait for camera release. In the
VMware Ubuntu graphical desktop Terminal, run:

```bash
bash /path/to/workspace/vision_core/scripts/start_native_live_ros_ui.sh
```

This sources installed ROS2 Jazzy and the workspace overlay, then delegates to
the H1 launcher (existing venv, preferred RGB 1280x720 BGR8 / Depth 640x480 Z16,
15 FPS). It starts exactly one camera pipeline. Use s to save, q/ESC or Ctrl+C
to quit; the ROS node and SDK pipeline are cleaned up. Missing DISPLAY fails
before starting either node or camera. No FSM/MoveIt/robot node is started.

In a second Terminal, source ROS2 and the same overlay and inspect the topics:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/workspace/install/setup.bash
ros2 topic echo /mtc/perception/batteries --qos-reliability reliable
ros2 topic echo /mtc/perception/native_diagnostics --qos-reliability reliable
```

Topic types and QoS are unchanged: BatteryDetectionArray and std_msgs/String,
RELIABLE / KEEP_LAST(5) / VOLATILE. An empty detections list is expected if all
targets are unconfirmed, stale or ambiguous. Check diagnostic filter_report
before interpreting an empty array as a transport failure.

## Safety and metadata

native_array still admits only known-color CONFIRMED_OBJECT detections, using
detection ambiguity/freshness separately from CAD uncertainty. Every ROS body
and ring pose uses NaN position/orientation, ring_pose_valid=false. The unchanged
IDL has no body_pose_valid field; diagnostics explicitly report it as false.
No identity, quality or confirmation threshold is relaxed.

The unchanged IDL also has no detection status or RGB/Depth frame-number fields.
native_diagnostics retains the entire original result, SDK IDs/domains/times,
object ID, color, confidence, detection status, confirmation evidence, filtering
reasons and CAD provenance. ros2_dispatch in local results adds current-frame
and cumulative candidate/published/filtered counts. No unknown sensor/ROS clock
mapping is invented: message header stamp remains zero, frame_id retains the
camera coordinate frame, original times remain in diagnostics. Offline results
are marked offline, never current camera observations.

## Distinct counters

pipeline_diagnostics distinguishes SDK-observed frame-number gaps and repeats,
SDK queue draining, qualified capture-slot replacements, sync rejection by
reason, CAD-slot replacements, completed CAD jobs and historical stale CAD.
Queue replacement is intentional latest-frame backpressure. These counters
must not be summed as USB packet loss; USB packet loss is unmeasured/null.
UI counts labeled CUM accumulate SDK/queue/sync events; FRAME CAD stale and ROS
target admission describe the displayed observation. CAD age is also shown.

## Offline regression

Use only approved original pairs, with their integrity manifest:

```bash
python vision_core/scripts/live_rgbd_preview.py --full-vision --dual-rate \
  --headless --replay /path/to/private/recording \
  --pair-manifest /path/to/private/audit/independent_pair_manifest.json \
  --max-frames 20 --run-output /path/to/new/private/output --ros-publish \
  --ros-wait-subscriber-s 15
```

Start the subscriber first when using the wait option. Waiting happens before
camera startup in LIVE mode too. Omit the wait option for ordinary manual UI
startup. Physical D435i synchronization, rates and end-to-end LIVE delivery
require the user's subsequent manual test; offline DDS tests certify neither
USB stability nor three-dimensional accuracy.
