export PYTHONPATH=/review-venv/lib/python3.12/site-packages:/candidate/vision_core:$PYTHONPATH
export MTC_VISION_INSTALL_BASE=/candidate-install
colcon --log-base /review-evidence/colcon-before test --build-base /candidate-build --install-base /candidate-install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure
