test ! -e /candidate/install
export MTC_VALIDATION_OUT=/review-evidence/before-replay
set +e
colcon --log-base /review-evidence/colcon-before test --build-base /candidate-build --install-base /candidate-install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure --pytest-args -ra --basetemp=/review-evidence/before-pytest-temp
rc=$?
set -e
cp /candidate-build/mtc_vision_adapter/pytest.xml /review-evidence/colcon-before.xml
printf '%s\n' "$rc" > /review-evidence/colcon-before.rc
test "$rc" -eq 1
