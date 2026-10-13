# Ubuntu native diagnostic UI

Use the existing vision_core UI and dual-rate pipeline. Camera acquisition and
CAD processing run separately; the UI does not wait for CAD in normal playback.
All displayed poses remain candidates with BODY/RING INVALID. Confirmed identity,
CAD geometry support and independently verified pose accuracy are distinct fields.

## Offline original-array replay

In the workspace, activate the existing Python/ROS environment. Use a private
recording root and its independent_pair_manifest.json. The loader checks the
adjacent original_sha256.json and admits only ordered, independently qualified
original RGB/Depth pairs. No conversion, timestamp rewriting or camera fallback.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /path/to/existing/venv/bin/activate
python vision_core/scripts/live_rgbd_preview.py --full-vision --dual-rate \
  --replay /path/to/private/recording \
  --pair-manifest /path/to/private/audit/independent_pair_manifest.json \
  --max-frames 331 --replay-output /path/to/new/private/ui-output
```

Run GUI commands inside the VMware Ubuntu desktop Terminal. If SSH has no DISPLAY,
the command fails before camera or window creation. Add --headless only for actual
offscreen OpenCV rendering; this does not certify desktop-window or keyboard operation.
Do not guess another session's DISPLAY value. No Web frontend or new GUI dependency.

Controls: q/ESC exits, Ctrl+C also cleans up; SPACE pauses/resumes, n advances a
paused offline frame, s saves the current original RGB-D and rendered diagnostic
to a new user_saves/sample_NNNN directory. Repeated saves do not overwrite.
c toggles raw candidates, d depth, m masks, f foreground, p cloud, b raw RGB,
o CAD overlay, i details, 1-9 selection. Offline pause can request same-frame CAD
asynchronously for inspection. r retains the existing bounded live recording
control; it does not start recording in offline mode.

## Display and time

RGB and aligned Depth use equal side-by-side panels. Four color boxes, IDs,
confidence, Body XYZ/Yaw, Ring center/orientation (candidate quaternion xyzw),
opposite insertion unit vectors, confirmation, CAD fit, ambiguity, stale/source,
capture UTC and SDK frame IDs are exposed. Stale CAD may be shown as historical
text; it is never projected onto the current RGB image.

Depth display range is 0-1 m. Values above 1 m are saturated but still valid;
only zero Z16 is black/invalid. This display range is not a processing gate.

Camera FPS is N/A in offline mode. Fast and CAD FPS are rolling wall-time event
rates, not the configured stream FPS. Summary timing excludes acquisition in
offline replay and distinguishes CAD jobs from matched object observations.

Historical detection stale is evaluated using original host capture and actual
processing wall time. Offline CAD source association uses the recorded capture
timeline; this does not promote a historical observation to current live data.
Image display metadata sidecars preserve original timestamps and IDs.

## Manual Ubuntu native LIVE input

In the VMware Ubuntu graphical desktop Terminal, run the existing venv launcher:

```bash
bash /path/to/workspace/vision_core/scripts/start_native_live_ui.sh
```

Executing this launcher authorizes opening the real D435i. It selects `--live
--preview-authorized`, which reuses full-vision dual-rate processing, with RGB
1280x720 BGR8 and Depth 640x480 Z16 at preferred 15 FPS. Actual SDK stream
selection is printed; nearest supported fallback is reported by the existing
source. SDK alignment, intrinsics, depth scale and original timestamps are used.
No ROS2 Publisher or TCP sender starts by default.

The launcher activates `$HOME/.venvs/mtc-vision-jazzy-py312`; override with
`MTC_VISION_VENV` if the installed environment has another location. It resolves
and enters the workspace automatically. No installation is performed.
Outputs default to `$HOME/private_data/mtc_live_ui/live_<timestamp>`; override
the root with `MTC_VISION_OUTPUT_ROOT`. `s` saves original RGB/raw Z16/aligned
depth, intrinsics, timestamps, candidates and the displayed image below
`user_saves/sample_NNNN`. `q`, ESC or Ctrl+C stops and releases the camera.
`r` optionally starts/stops the existing bounded recorder. SPACE pauses display;
acquisition continues so unpausing uses the latest qualified pair.

LIVE Camera RGB/Depth rates count observed unique SDK frame IDs. Fast/CAD rates
measure their own events. The configured 15 FPS is not a measured rate. CAD
uses its existing bounded latest-job worker; preview does not wait for CAD.
Historical CAD is labeled stale and excluded from current-image overlays.
All Body/Ring candidates remain INVALID regardless of identity confirmation.
Live-device performance and ROS2 publication acceptance require later tests.

Direct equivalent (after activating the same existing venv in the workspace):

```bash
python vision_core/scripts/live_rgbd_preview.py --live --preview-authorized \
  --color-size 1280 720 --depth-size 640 480 --fps 15 \
  --run-output /path/to/new/private/live-output
```

`--run-output` also accepts the old `--replay-output` spelling. LIVE and offline
replay inputs are mutually exclusive. `--help` and launcher syntax checks do not
touch the camera. Missing DISPLAY fails before SDK startup.

Do not start a real camera until the user again confirms Ubuntu attachment and
explicitly authorizes it. Live operation still requires --preview-authorized.
Body/Ring validity stays false; no Publisher or robot output is enabled here.

Native full-vision input rejects reuse of an admitted color/depth frame,
backwards IDs/time, nonfinite timestamps, incomparable domains, and >100 ms skew.
Only matching SDK global_time/system_time labels are compared; two hardware_clock
labels alone do not establish shared sensor clocks. Rejected pairs cannot enter
Fast/CAD geometry or the qualified recorder. Counts describe SDK observations,
not USB packet loss. Thirty consecutive unqualified observations stop with a
diagnostic; existing cleanup releases the source. Live rates count delivered
unique frame IDs, not all sensor-emitted frames. Live synchronization acceptance
remains pending physical-device testing.
