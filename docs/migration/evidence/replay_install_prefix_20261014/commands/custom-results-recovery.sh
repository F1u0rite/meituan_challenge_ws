source /fix-custom-install/setup.bash
export MTC_VALIDATION_OUT=/review-evidence/custom-replay
colcon --log-base /review-evidence/result-custom test-result --test-result-base /fix-custom-build/mtc_vision_adapter --all > /review-evidence/custom-results.txt
cp /fix-custom-build/mtc_vision_adapter/pytest.xml /review-evidence/custom.xml
python3 - <<'INNER'
import json,hashlib,sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import mtc_vision_adapter,mtc_vision_adapter.core,mtc_vision_adapter.publisher
modules={}
for m in [mtc_vision_adapter,mtc_vision_adapter.core,mtc_vision_adapter.publisher]:
 p=Path(m.__file__);modules[m.__name__]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
assert get_package_prefix('mtc_vision_adapter')=='/fix-custom-install/mtc_vision_adapter'
assert all(x['path'].startswith('/fix-custom-install/mtc_vision_adapter'+'/') for x in modules.values())
assert all(x['sha256']==hashlib.sha256((Path('/candidate/src/mtc_vision_adapter/mtc_vision_adapter') / (('__init__' if k=='mtc_vision_adapter' else k.split('.')[-1])+'.py')).read_bytes()).hexdigest() for k,x in modules.items())
Path('/review-evidence/custom-identity.json').write_text(json.dumps({'prefix':'/fix-custom-install/mtc_vision_adapter','modules':modules,'python':sys.version},indent=2)+'\n')
INNER
