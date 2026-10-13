# Vision adapter colcon test registration — 2026-10-14

`mtc_vision_adapter` built successfully but `colcon test` selected unittest and
reported `Ran 0 tests` / exit 5. Its tests are pytest functions; `setup.py` omitted
`tests_require=['pytest']`. The same failure was reproduced with the vision and
pytest dependencies already available, separating test registration from missing
dependencies.

This change adds that metadata declaration and documents test/environment commands.
No runtime module, ROS IDL, algorithm, test expectation, FSM or Action implementation
changes. The selected source is `180acb8830106ac7a76ce63b844c579c8d919ee2`
on `codex/yuze-parallel` (feature PR #17); this fix targets that branch. It does
not advance the project's formal pin or approve the unfinished feature PR.

| Isolated validation | Result |
| --- | --- |
| Baseline, dependencies available | unittest, 0 tests, colcon exit 5 |
| Rebuild adapter with the one-line patch | PASS |
| Fresh colcon pytest gate | 159 passed; 0 errors/failures/skips; exit 0 |
| Installed executable/module provenance | Recorded in module-identity.json |
| Cleanup | Original and supplement stopped; only sleep keeper before stop |

Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3, pytest 8.3.5 and setuptools 68.1.2;
2 CPU / 4 GiB, no network, mapped device, host mount or published port.
`mtc_interfaces` came from the unchanged `/candidate-install` overlay; the patched
adapter was rebuilt into `/fix-install`. The fresh result XML belongs to this
package, not the older workspace aggregate.

The 159 tests include local TCP→ROS, installed replay/coordinate subprocesses,
synthetic RGB-D and JPEG delivery, invalid-pose filtering and cleanup. The launcher's
separate `MTC_VISION_ROS_SETUP` was set for the custom installation path. No camera,
private recording, SAM checkpoint or robot was used. Separate vision-core tests,
full C++ FSM consumption and hardware acceptance are outside this fix's result.
Unrelated historical Python Action and MoveIt shutdown failures are not closed.

Focused original logs, commands, environment and XML are frozen under
[evidence/vision_colcon_pytest_20261014](evidence/vision_colcon_pytest_20261014/index.json).
Index SHA-256: `de7dfcd9980bb9c715595357aad1de8ac7cda80cb8b62c3517610820d63856fa`. Baseline/fixed setup hashes are in inputs.json.
Raw monitoring outputs, caches, machine configuration and incomplete features are
excluded. The full monitoring archive remains local to the integration workspace.

OpenAI Codex assisted with diagnosis, the packaging edit, regression and documentation.
Upstream repository MIT / package Apache-2.0 declarations remain unchanged.
