"""Verify registered tests, installed module and untouched main runtime source."""
from pathlib import Path
import hashlib,json,xml.etree.ElementTree as ET
import mtc_vision_adapter
root=Path('/fix-review/src/mtc_vision_adapter')
installed=Path(mtc_vision_adapter.__file__).parent
assert str(installed).startswith('/registration-install/'),str(installed)
rows=[]
for p in sorted((root/'mtc_vision_adapter').glob('*.py')):
 q=installed/p.name
 original=Path('/candidate/src/mtc_vision_adapter/mtc_vision_adapter')/p.name
 assert p.read_bytes()==q.read_bytes()==original.read_bytes()
 rows.append(dict(file=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),installed=str(q),matches_original=True))
tests=list(ET.parse('/review-evidence/registration-after.xml').getroot().iter('testcase'))
assert len(tests)==57 and all(c.find(k) is None for c in tests for k in ['error','failure','skipped'])
result=dict(module=str(installed),runtime=rows,tests=57,failures=0,skips=0,source_tests_unchanged=True)
for p in (root/'test').glob('*.py'):assert p.read_bytes()==(Path('/candidate/src/mtc_vision_adapter/test')/p.name).read_bytes()
Path('/review-evidence/registration-module-identity.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
