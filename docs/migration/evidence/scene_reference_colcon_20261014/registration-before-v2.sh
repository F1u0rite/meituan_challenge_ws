cd /fix-review
export PYTHONPATH=/review-venv/lib/python3.12/site-packages:/fix-review/vision_core:$PYTHONPATH
export MTC_VISION_INSTALL_BASE=/registration-install MTC_VISION_ROS_SETUP=/registration-install/setup.bash
colcon --log-base /review-evidence/registration-before-v2-build-log build --base-paths /fix-review/src/mtc_vision_adapter --build-base /registration-build --install-base /registration-install --packages-select mtc_vision_adapter --executor sequential
source /registration-install/setup.bash
colcon --log-base /review-evidence/registration-before-v2-colcon test --build-base /registration-build --install-base /registration-install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure
