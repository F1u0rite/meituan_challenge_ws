cd /fix-review
colcon --log-base /review-evidence/build-merged build --base-paths src/mtc_vision_adapter --build-base /fix-merged-build --install-base /fix-merged-install --merge-install --executor sequential
source /fix-merged-install/setup.bash
export MTC_VALIDATION_OUT=/review-evidence/merged-replay
colcon --log-base /review-evidence/test-merged test --build-base /fix-merged-build --install-base /fix-merged-install --merge-install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure --pytest-args -ra --basetemp=/review-evidence/merged-pytest-temp
colcon --log-base /review-evidence/result-merged test-result --test-result-base /fix-merged-build/mtc_vision_adapter --all > /review-evidence/merged-results.txt
cp /fix-merged-build/mtc_vision_adapter/pytest.xml /review-evidence/merged.xml
python3 - <<'INNER'
import json,hashlib,sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import mtc_vision_adapter,mtc_vision_adapter.core,mtc_vision_adapter.publisher
modules={}
for m in [mtc_vision_adapter,mtc_vision_adapter.core,mtc_vision_adapter.publisher]:
 p=Path(m.__file__);modules[m.__name__]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
assert get_package_prefix('mtc_vision_adapter')=='/fix-merged-install'
assert all(x['path'].startswith('/fix-merged-install'+'/') for x in modules.values())
assert all(x['sha256']==hashlib.sha256((Path('/candidate/src/mtc_vision_adapter/mtc_vision_adapter') / (('__init__' if k=='mtc_vision_adapter' else k.split('.')[-1])+'.py')).read_bytes()).hexdigest() for k,x in modules.items())
Path('/review-evidence/merged-identity.json').write_text(json.dumps({'prefix':'/fix-merged-install','modules':modules,'python':sys.version},indent=2)+'\n')
INNER
