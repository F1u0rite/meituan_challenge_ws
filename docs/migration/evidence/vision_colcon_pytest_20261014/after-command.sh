source /fix-install/local_setup.bash
export PYTHONPATH=/review-venv/lib/python3.12/site-packages:/candidate/vision_core:$PYTHONPATH
export MTC_VISION_INSTALL_BASE=/fix-install MTC_VISION_ROS_SETUP=/fix-install/setup.bash
export MTC_VALIDATION_OUT=/review-evidence/final-fix-replay-processes MTC_NATIVE_VALIDATION_OUT=/review-evidence/final-fix-native-process MTC_LIVE_VALIDATION_OUT=/review-evidence/final-fix-live-process
colcon --log-base /review-evidence/colcon-final test --build-base /fix-build --install-base /fix-install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure --pytest-args -ra --basetemp=/review-evidence/final-fix-pytest-temp
colcon test-result --test-result-base /fix-build/mtc_vision_adapter --all > /review-evidence/colcon-final-results.txt
cp /fix-build/mtc_vision_adapter/pytest.xml /review-evidence/colcon-final.xml
python3 - <<'IN'
import sys,json,importlib,importlib.metadata as md,hashlib
from pathlib import Path
names=['mtc_vision_adapter.automatic_top_result_publisher','mtc_vision_adapter.rgb_image_publisher','mtc_interfaces','perception.height_constraint','rclpy','pytest','cv2','numpy']
r={n:{'path':importlib.import_module(n).__file__} for n in names}
for n,v in r.items():
 if v['path']:v['sha256']=hashlib.sha256(Path(v['path']).read_bytes()).hexdigest()
r['python']=sys.version
r['versions']={n:md.version(n) for n in ['numpy','scipy','matplotlib','pytest','setuptools','PyYAML','opencv-python']}
Path('/review-evidence/module-identity.json').write_text(json.dumps(r,indent=2)+'\n')
IN
