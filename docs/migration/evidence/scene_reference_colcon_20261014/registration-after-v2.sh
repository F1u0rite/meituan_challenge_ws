cd /fix-review
export PYTHONPATH=/review-venv/lib/python3.12/site-packages:/fix-review/vision_core:$PYTHONPATH
export MTC_VISION_INSTALL_BASE=/registration-install MTC_VISION_ROS_SETUP=/registration-install/setup.bash
colcon --log-base /review-evidence/registration-after-v2-build-log build --base-paths /fix-review/src/mtc_vision_adapter --build-base /registration-build --install-base /registration-install --packages-select mtc_vision_adapter --executor sequential
source /registration-install/setup.bash
export MTC_VALIDATION_OUT=/review-evidence/registration-replay MTC_NATIVE_VALIDATION_OUT=/review-evidence/registration-native MTC_LIVE_VALIDATION_OUT=/review-evidence/registration-live
colcon --log-base /review-evidence/registration-after-v2-colcon test --build-base /registration-build --install-base /registration-install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure --pytest-args -ra --basetemp=/review-evidence/registration-pytest-temp
colcon test-result --test-result-base /registration-build/mtc_vision_adapter --all > /review-evidence/registration-results.txt
cp /registration-build/mtc_vision_adapter/pytest.xml /review-evidence/registration-after.xml
python3 /review-tools/registration_identity.py
