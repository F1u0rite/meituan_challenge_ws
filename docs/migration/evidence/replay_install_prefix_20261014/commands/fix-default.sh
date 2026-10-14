cd /default-review
colcon --log-base /review-evidence/build-default build --base-paths src/mtc_vision_adapter --build-base /default-review/build --install-base /default-review/install --executor sequential
source /default-review/install/setup.bash
export MTC_VALIDATION_OUT=/review-evidence/default-replay
colcon --log-base /review-evidence/test-default test --build-base /default-review/build --install-base /default-review/install --packages-select mtc_vision_adapter --executor sequential --return-code-on-test-failure --pytest-args -ra --basetemp=/review-evidence/default-pytest-temp
colcon --log-base /review-evidence/result-default test-result --test-result-base /default-review/build/mtc_vision_adapter --all > /review-evidence/default-results.txt
cp /default-review/build/mtc_vision_adapter/pytest.xml /review-evidence/default.xml
python3 - <<'INNER'
import json,hashlib,sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import mtc_vision_adapter,mtc_vision_adapter.core,mtc_vision_adapter.publisher
modules={}
for m in [mtc_vision_adapter,mtc_vision_adapter.core,mtc_vision_adapter.publisher]:
 p=Path(m.__file__);modules[m.__name__]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
assert get_package_prefix('mtc_vision_adapter')=='/default-review/install/mtc_vision_adapter'
assert all(x['path'].startswith('/default-review/install/mtc_vision_adapter'+'/') for x in modules.values())
assert all(x['sha256']==hashlib.sha256((Path('/candidate/src/mtc_vision_adapter/mtc_vision_adapter') / (('__init__' if k=='mtc_vision_adapter' else k.split('.')[-1])+'.py')).read_bytes()).hexdigest() for k,x in modules.items())
Path('/review-evidence/default-identity.json').write_text(json.dumps({'prefix':'/default-review/install/mtc_vision_adapter','modules':modules,'python':sys.version},indent=2)+'\n')
INNER
