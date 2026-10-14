cd /fix-review
export PYTHONPATH=/review-venv/lib/python3.12/site-packages:/fix-review/vision_core:$PYTHONPATH
colcon --log-base /review-evidence/registration-after-build-log build --base-paths /fix-review/src/mtc_vision_adapter --build-base /registration-build --install-base /registration-install --packages-select mtc_vision_adapter --executor sequential
source /registration-install/setup.bash
export MTC_VALIDATION_OUT=/review-evidence/registration-replay
colcon --log-base /review-evidence/registration-after-colcon test --build-base /registration-build --install-base /registration-install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure --pytest-args -ra --basetemp=/review-evidence/registration-pytest-temp
colcon test-result --test-result-base /registration-build/mtc_vision_adapter --all > /review-evidence/registration-results.txt
cp /registration-build/mtc_vision_adapter/pytest.xml /review-evidence/registration-after.xml
python3 /review-tools/registration_identity.py
