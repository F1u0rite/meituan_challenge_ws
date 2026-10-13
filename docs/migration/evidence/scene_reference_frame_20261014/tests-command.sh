source /fix-install/local_setup.bash
export PYTHONPATH=/review-venv/lib/python3.12/site-packages:/fix-review/vision_core:$PYTHONPATH
export MTC_VISION_INSTALL_BASE=/fix-install MTC_VISION_ROS_SETUP=/fix-install/setup.bash
export MTC_VALIDATION_OUT=/review-evidence/fix-final-replay-processes MTC_NATIVE_VALIDATION_OUT=/review-evidence/fix-final-native-process MTC_LIVE_VALIDATION_OUT=/review-evidence/fix-final-live-process
python3 -m pytest -q -ra /fix-review/src/mtc_vision_adapter/test --junitxml=/review-evidence/fix-tests-final.xml --basetemp=/review-evidence/fix-final-pytest-temp -o cache_dir=/review-evidence/fix-final-cache
