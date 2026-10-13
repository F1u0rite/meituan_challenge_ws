source /fix-install/local_setup.bash
export PYTHONPATH=/review-venv/lib/python3.12/site-packages:/candidate/vision_core:$PYTHONPATH
export ROS_DOMAIN_ID=218
python3 /review-tools/scene_boundary_audit.py --tag after --install-base /fix-install
