# Battery vision V4.0-B — source-only migration

Existing RGB/HSV four-color detection, common CAD/RGB-D body and ring candidates, pixel tracking, dual-rate UI, lossless recording/replay, GT/fixture evaluator and synthetic generator are preserved. This upload does not develop algorithms or run devices.

## Python setup (editable source layout)

From repository root:

```sh
python -m pip install -e './vision_core[test]'
python -m pip install -e ./src/mtc_vision_adapter
python -m pytest -q vision_core/tests
cd vision_core
python scripts/live_rgbd_preview.py --help
python scripts/live_realsense_handheld.py --help
```

Python >=3.10; core dependencies are declared in pyproject.toml. Optional realsense adds pyrealsense2; optional cad pins the existing cadquery-ocp 7.7.2 backend. Ubuntu/Python 3.12 CAD wheel compatibility is NOT RUN. Use a dedicated environment: existing generic package names input/perception/config are preserved. Editable installation keeps config and runtime paths relative to this checkout; a self-contained binary/wheel distribution is not claimed.

## Assets deliberately excluded

No raw images, RGB-D recordings, personal environment photos, annotations, PLY, outputs, environments, bundled runtimes or credentials are uploaded. Original STEP files are 124248/126335 bytes; public sharing permission is unconfirmed, so both STEP originals and derived caches are excluded.

Full CAD preview requires privately supplied assets/cad/电池道具.stp and assets/cad/舌规V1.STEP and the existing extraction workflow creating outputs/cad_analysis/geometry.json and cad_meshes.npz. Source hashes must match. Missing CAD resources block complete CAD preview, not RGB detection or JSON publisher replay. Do not commit the resulting data. Public tests use synthetic inputs; the absent private 22-frame evidence test is skipped, not reported passed. This subset is not the original local 215-test suite.

## ROS2 Jazzy / Ubuntu 24.04 — NOT RUN here

```sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mtc_vision_adapter
source install/setup.bash
ros2 run mtc_vision_adapter replay_publisher --ros-args -p input_jsonl:="$(pwd)/src/mtc_vision_adapter/examples/synthetic.jsonl" -p period_s:=1.0
# Another sourced terminal:
ros2 topic echo /mtc/perception/batteries --qos-reliability reliable --qos-history keep_last --qos-depth 5
ros2 topic info /mtc/perception/batteries --verbose
```

The existing adapter publishes explicit replay/synthetic records and preserves original timestamps; it is not a live camera ROS node. QoS RELIABLE + KEEP_LAST(5) + VOLATILE requires actual consumer testing. ROS2/colcon/USB/NaN-consumer tests have NOT RUN. Existing IDL, FSM and teammate code are unchanged.

## Gates and remaining work

body_pose_valid=false, ring_pose_valid=false. JSON null becomes ROS NaN for unmeasured poses. Consumers must reject invalid ring_pose, non-finite body_pose, stale/replay observations and uncalibrated frames; public IDL has no body_pose_valid field. Synthetic example timestamps/frames cannot be motion input.

Four-color/multi-target software support is synthetic tested; real four-color acceptance is NOT MEASURED. True XYZ/Yaw, aperture pose, table/tool/hand-eye calibration and actual Jazzy integration remain pending. CAD residuals are not positioning accuracy.
