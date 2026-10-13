# Automatic RGB-D coordinate and image output

Use Ubuntu 24.04, Python 3.12 and ROS 2 Jazzy. The tested implementation uses
CPU SAM 2.1 tiny, a separate environment, and read-only external geometry.
Run from an independent checkout; do not reuse another developer's camera
process, source files, ROS domain or build output.

## Build the ROS messages and adapter

From this repository root, with Jazzy/colcon dependencies installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/yuze build --build-base build/yuze \
  --install-base install/yuze --packages-up-to mtc_vision_adapter
source install/yuze/setup.bash
export MTC_VISION_INSTALL_BASE="$PWD/install/yuze"
```

The runner and actual-process tests also support a conventional `install/`
or merged install. Set `MTC_VISION_INSTALL_BASE` to the selected install base.
The manual legacy UI launcher accepts `MTC_VISION_ROS_SETUP` for its setup.bash;
it automatically falls back to `install/yuze/setup.bash` if the default setup
does not exist. No shell profile or SSH configuration is changed.

## Prepare the CPU model outside Git

The tested official [SAM 2 source](https://github.com/facebookresearch/sam2)
commit is `2b90b9f5ceec907a1c18123530e92e794ad901a4`. Reuse an existing verified
environment/checkpoint when available. The following commands prepare a fresh
checkout; inspect existing directories before running clone or installation.

```bash
python3 -m venv --system-site-packages build/sam21_cpu
build/sam21_cpu/bin/python -m pip install numpy==1.26.4 \
  opencv-python-headless==4.11.0.86 PyYAML==6.0.1 pyrealsense2==2.59.1.11053
build/sam21_cpu/bin/python -m pip install torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cpu
mkdir -p log/sam21_experiment
git clone https://github.com/facebookresearch/sam2.git log/sam21_experiment/sam2
git -C log/sam21_experiment/sam2 checkout 2b90b9f5ceec907a1c18123530e92e794ad901a4
SAM2_BUILD_CUDA=0 build/sam21_cpu/bin/python -m pip install \
  -e log/sam21_experiment/sam2
```

Download the official
[SAM 2.1 tiny checkpoint](https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt)
into `log/sam21_experiment/sam2.1_hiera_tiny.pt`; do not commit the model.
Verify SHA-256 before use:

```bash
printf '%s\n' '7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69  log/sam21_experiment/sam2.1_hiera_tiny.pt' \
  | sha256sum -c -
```

The program uses `apply_postprocessing=False` and CPU execution, so a CUDA
extension is unnecessary. `-I` avoids accidental SAM import shadowing while
the runner explicitly loads Jazzy/generated messages from the selected overlay.

## Coordinate and image output

See [the output contract](automatic_top_coordinates.md) for units, raw/effective
coordinates, clocks, missing detections and QoS. Coordinates are camera-optical
XYZ in metres; capture time is preserved separately from dispatch time.
Top=80 mm and hole=60 mm physical heights were confirmed by the user; the
height constraint is enabled by default. `--disable-height-constraint` selects
raw points. These dimensions and the 70x25 mm cap reference must be reviewed
for a different prop. All full-pose/motion validity flags remain false.

For an offline frame_* RGB-D recording, which never opens the camera:

```bash
export ROS_DOMAIN_ID=215
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$PWD/log/yuze_ros"
build/sam21_cpu/bin/python -I -B vision_core/scripts/run_automatic_top_coordinates.py \
  --input /absolute/path/to/recording --frames 6 --send-images \
  --geometry /absolute/path/to/private/geometry.json \
  --checkpoint log/sam21_experiment/sam2.1_hiera_tiny.pt
```

`--live` replaces `--input ...` only when camera use is authorized and the
device is free. Do not interrupt an existing camera owner. `--frames 3` gives
a bounded camera smoke test; `--frames 0` runs until Ctrl+C. The single capture
worker continues RGB output while slow inference consumes the latest frame.
The default image rate cap is 10 FPS, JPEG quality 80. A positive
`--wait-subscriber-s` requires subscribers on both coordinate topics and raw
image/metadata topics before camera start.

```bash
python3 -m pytest -q vision_core/tests src/mtc_vision_adapter/test
```

ROS process tests require the built overlay, rclpy, generated messages and
vision Python dependencies. In a separate vision test environment, include the
repository's `vision_core` and `src/mtc_vision_adapter` directories in PYTHONPATH.
GPU/model inference is not needed for the deterministic unit tests.

## Verification and remaining limits

2026-10-14 (Asia/Shanghai): full Python regression 413 passed, 10 skipped.
Actual recorded RGB-D inference delivered four top/hole candidates, raw and
height-constrained coordinates, T0 targets and source-matched JPEGs to a real
independent ROS receiver. No robot was operated.

A bounded three-result camera run delivered all four colours and T0 on each
result, 12 height-constrained battery candidates, and 217 RGB frames. Observed
raw video rate was 7.50 FPS, maximum gap 0.166 seconds; coordinate processing
took 8.93-10.02 seconds per result. The process exited normally and the camera
was released. An earlier three-result run missed red in the first result and
yellow in the second, then recovered. This transient/intermittent missing
output remains a limitation; a short successful rerun is not sustained
robustness acceptance. Missing coordinates are explicit and never recycled.

Absolute position error remains unmeasured. Height constraints impose known
geometry, not metrology. Camera-to-robot transforms, TCP, ROS capture-clock
mapping, cross-host DDS and robot consumption are outside this verification.
Real data, local credentials, CAD/STP/STEP/mesh assets and model weights remain
outside this PR. Runtime evidence stays in ignored local log directories.
