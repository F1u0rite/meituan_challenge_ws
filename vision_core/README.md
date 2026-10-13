# Battery vision V4.2 — source sync and local ROS2 acceptance

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

## ROS2 Jazzy / Ubuntu 24.04 acceptance (2026-10-12)

Synced V4.2 by fast-forward to e09d00ede40b0fafad6cd5428ae1824442e74d3c,
retaining the prior SIGINT and trusted-TF safety fix e53c09f.
Acceptance used the formal workspace /home/shrmeituan/meituan_vision_ws.
Only local Ubuntu ROS2 processes ran, with localhost discovery and isolated
ROS domains 191-194. No D435i, cross-host data link or robot was started.

| Check | Actual result |
| --- | --- |
| colcon build: mtc_interfaces, mtc_vision_adapter | PASS: 2 packages finished |
| Adapter pytest (including real installed processes) | PASS: 57 passed in 16.59s |
| synthetic natural completion | PASS: 4/4 arrays, lengths 0/4/4/4, exit 0; no signal sent |
| replay natural completion | PASS: 4/4 arrays, lengths 0/4/4/4, exit 0; no signal sent |
| synthetic and replay SIGINT during replay | PASS: interrupt after 2 received arrays; both exit 0 |
| Message fields and historical capture stamps | Exact fixture matches; null poses become NaN; ring_pose_valid=false |
| QoS | Created RELIABLE KEEP_LAST(5) VOLATILE; discovered endpoints compatible |
| Cleanup | No traceback, duplicate-shutdown error or remaining replay node |
| Motion rejection helper and local TF buffer | PASS: invalid ring/poses, untrusted frames, missing or historical TF mismatch reject |
| Live D435i / USB acceptance | NOT RUN |
| Windows D435i to Ubuntu ROS2 cross-host link | NOT RUN |
| C++ FSM consumption and downstream safety acceptance | NOT RUN / NOT VERIFIED |
| True XYZ/Yaw, ring pose and grasp accuracy | NOT VERIFIED |

The replay test changes only the mode/source metadata of the synthetic
fixture; it is not evidence of a real camera recording. SIGINT tests are
mid-replay interruption checks and do not claim delivery of all four frames.
Natural-exit tests independently verify complete four-frame delivery.
The RMW discovery graph reports history UNKNOWN and depth 0; queue settings
are checked on the created publisher's profile, not inferred from discovery.
RELIABLE final-sample acknowledgment does not certify C++ FSM acceptance.

Reproduce from the workspace root after installing ROS2 Jazzy dependencies:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mtc_interfaces mtc_vision_adapter
source install/setup.bash
MTC_VALIDATION_OUT=/tmp/mtc_v42_validation_20261012 \
  python3 -m pytest -q src/mtc_vision_adapter/test
```

Host-local evidence (runtime data is excluded from Git):
- /tmp/mtc_v42_colcon.log
- /tmp/mtc_v42_pytest.log
- /tmp/mtc_v42_validation_20261012/{synthetic,replay}-{natural,sigint}/ros_results.json
- The same case directories contain ros_publisher.log, ros_received.json and ros_logs/.

The adapter is an offline replay publisher, not a live D435i ROS node.
The next stage may prepare live input integration, but cross-host transport,
capture-clock alignment, real TF/calibration and actual downstream rejection
must be verified before any motion use.


## Gates and remaining work

body_pose_valid=false, ring_pose_valid=false. JSON null becomes ROS NaN for unmeasured poses. Consumers must reject invalid ring_pose, non-finite body_pose, stale/replay observations and uncalibrated frames; public IDL has no body_pose_valid field. Synthetic example timestamps/frames cannot be motion input.

Four-color/multi-target software support is synthetic tested; real four-color acceptance is NOT MEASURED. True XYZ/Yaw, aperture pose, table/tool/hand-eye calibration, live D435i integration and C++ FSM consumption remain pending. CAD residuals are not positioning accuracy.
