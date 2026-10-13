# ROS2 battery detection offline adapter

Topic /mtc/perception/batteries; RELIABLE KEEP_LAST(5) VOLATILE.
UNKNOWN=0 RED=1 BLUE=2 YELLOW=3 GREEN=4.
Explicit replay/synthetic records remain publishable with their historical
capture timestamps. Unmeasured JSON null poses become ROS NaN.
The publisher always forces ring_pose_valid=false; no invented XYZ or
quaternion makes offline data valid for motion.

Motion safety contract:
motion_rejection(..., mode="live") rejects by default. The caller must supply
trusted_frames, a trusted target_frame and a callable
tf_validator(source_frame, target_frame, capture_stamp). It must return
exactly True after verifying usable TF at the capture stamp, not latest TF.
Missing TF, timeout and uncertain results reject. A frame name alone does
not establish trust. This helper does not wire or verify the C++ FSM.

The replay node now stops its execution loop after the last publish and
bounded RELIABLE acknowledgment, then destroys the node and calls
rclpy.try_shutdown(). Normal completion and SIGINT both exit cleanly.
For matched-subscriber acceptance, set wait_for_subscriber_s to a positive
timeout (the default 0 preserves standalone offline publication).
A subscriber discovery or acknowledgment timeout fails rather than claiming
successful delivery. No subscriber means there is no delivery guarantee.

The process tests directly launch the installed replay_publisher executable;
they do not replace ROS publishers, timers or shutdown with mocks.
For a standalone natural-exit validation, source the overlay, select a local
ROS domain and run python3 src/mtc_vision_adapter/test/ros_replay_validation.py.
Use --exit-mode sigint for an interruption test. MTC_REPLAY_FIXTURE selects
a four-frame JSONL fixture; MTC_VALIDATION_OUT selects the evidence directory.

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

## Automatic top/hole candidate and image output

The opt-in automatic runner adds top/hole candidate JSON, invalid-pose battery
arrays and JPEG streams. See [setup](../../vision_core/docs/automatic_top_setup.md)
and [message contract](../../vision_core/docs/automatic_top_coordinates.md).
Use MTC_VISION_INSTALL_BASE for an isolated/merged colcon install; process tests
continue to execute actual installed binaries. This does not connect the C++
FSM or authorize motion.

## Running the package through colcon

`setup.py` declares pytest in `tests_require`, so `colcon test` collects this
package's pytest functions and runs its installed-process checks. The previous
unittest fallback reported `Ran 0 tests` even when the package built successfully.
A successful build or an aggregate report that omits this package is not a test pass.

Use Jazzy's Python environment with the dependencies declared in
`vision_core/pyproject.toml`, pytest and the package's ROS dependencies available to
the interpreter used by colcon. Build the adapter and `mtc_interfaces` first. From
the workspace root (replace the build/install paths for an isolated build):

```sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export MTC_VISION_INSTALL_BASE="$(realpath install)"
export MTC_VISION_ROS_SETUP="$MTC_VISION_INSTALL_BASE/setup.bash"
export PYTHONPATH="$PWD/vision_core${PYTHONPATH:+:$PYTHONPATH}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
colcon test --build-base build --install-base install \
  --packages-select mtc_vision_adapter --executor sequential \
  --return-code-on-test-failure
colcon test-result --test-result-base build/mtc_vision_adapter --all
```

`MTC_VISION_INSTALL_BASE` selects the installed process executables;
`MTC_VISION_ROS_SETUP` separately selects the launcher's sourced ROS overlay.
The checks use synthetic data and local ROS/TCP processes, without camera access.
They do not validate private recordings, SAM inference, physical coordinates or
C++ FSM consumption. The separate `vision_core/tests` suite remains outside this
ROS package's colcon test selection.
