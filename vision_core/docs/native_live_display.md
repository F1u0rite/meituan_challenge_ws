# H4 native preview scheduling and table lifecycle

Manual Ubuntu desktop launch (close the previous camera UI first):

```bash
bash /path/to/workspace/vision_core/scripts/start_native_live_ros_ui.sh
```

The existing entry/venv/ROS environment and single D435i source are reused.
q/ESC or Ctrl+C exits and releases the source; s saves a qualified current
preview with explicit annotation provenance; r retains bounded RGB-D recording.
SPACE pauses the image while perception continues. No robot nodes are started.

Qt lives in the main thread and the existing Fast/CAD/ROS pipeline runs in one
processing thread plus its existing capture/CAD workers. Each SDK observation
can feed a bounded latest-RGB preview slot, including rejected RGB-D pairs.
Only admitted pairs enter the existing Fast slot, geometry and recorder.
No second device is opened. The window is created/resized once at startup;
the 1600x900 canvas and panel rectangles remain unchanged.

New RGB is displayed without waiting for Fast or CAD. Actual camera unique-ID,
UI display-call, Fast and CAD rates are separate; configuration FPS is not a
measured rate. Rendering CPU cost can still limit UI throughput. Rejection
does not replace RGB with black, and rejected depth is marked DISPLAY ONLY.

Current-frame annotations require matching RGB/Depth IDs, qualified pairing
and the existing age limit. Otherwise old 2D boxes are dashed gray and explicitly
HISTORY with source ID/age, and disappear after the age limit. Masks and CAD
geometry are never transferred to another frame. Historical values remain
diagnostic text. A held CURRENT image also expires; BODY/RING always INVALID.
The ROS sink receives only original processing results, never preview copies.
Team IDL, topics, QoS and detection/pose admission policy are unchanged.

Outputs in the normal private run directory now also include:

- display_frames.jsonl: preview/annotation source, age, sync admission, canvas
  shape and per-display camera/UI/Fast/CAD rate samples.
- display_summary.json: measured UI call FPS, interval mean/P95/max, window
  initialization/resize count, preview replacements and rejected-pair previews.
- cad_lifecycle.jsonl: TABLE_READY and full CAD events, original capture/SDK
  timestamps, submit/start/plane-generation/completion host times.
- detections.jsonl table_hint_evidence: cache update/consume/source/expiry times,
  motion history reasons and capture age. 2.5-second expiry stays unchanged.

The existing plane fitter performs its original quality checks. Its approved
plane hypothesis is now sent through a bounded early-table message slot before
the remaining CAD fit finishes. It does not declare a verified pose or bypass
motion-transfer checks. Completed evidence is consumed before Fast's gate,
rather than one Fast cycle later. Full completion updates provenance without
resetting the original capture time or extending expiry. Generation-to-cache
time uses the host clock, not an invented sensor/ROS mapping.

Motion diagnostics add feature count, matched/inlier displacement and background
gray-level delta proxies. Existing stability decisions still use the original
inlier/flow/rotation conditions. Gray change is not proof of exposure-only error;
physical camera movement needs independent scene/TF evidence.

Offline acceptance used 117 original recorded pairs, unchanged pixels and SDK
timestamps, paced at a controlled 12.9Hz with SDK and Qt explicitly substituted.
This tests scheduling and real algorithm dispatch, not physical camera FPS or
visible VMware flicker. Old recordings lack plane-generation timestamps; none
were invented. Real low/high image-displacement snippets are not calibrated
physical static/moving camera ground truth. Manual retests must verify both.
