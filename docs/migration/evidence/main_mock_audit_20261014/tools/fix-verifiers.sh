set -e
cp -a /candidate /review-fix
cp /review-fix-code/tests/*.py /review-fix/tests/
cp /review-fix-code/src/mtc_ros2_action_cpp/test/verify_results.py /review-fix/src/mtc_ros2_action_cpp/test/verify_results.py
export GIT_CONFIG_COUNT=2 GIT_CONFIG_KEY_1=safe.directory GIT_CONFIG_VALUE_1=/review-fix
cd /review-fix
python3 tests/verify_cpp_audit_results.py --build-base /candidate-build --baseline 1d940f7c5f12561fed6641457c0b4b630a282b05 --baseline-count 318
python3 src/mtc_ros2_action_cpp/test/verify_results.py --build-base /candidate-build
python3 tests/check_cpp_audit_rejections.py --build-base /candidate-build --output-dir /review-evidence/audit-regression
