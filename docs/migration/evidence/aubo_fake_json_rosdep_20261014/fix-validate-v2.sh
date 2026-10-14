set -e
mkdir -p /review-evidence/dependency-fix-v2
rosdep resolve --rosdistro jazzy --os ubuntu:noble nlohmann-json-dev > /review-evidence/dependency-fix-v2/rosdep-resolve.txt
python3 - <<'P'
import json,xml.etree.ElementTree as E
from pathlib import Path
from catkin_pkg.package import parse_package
before=Path('/candidate/src/mtc_aubo_fake_integration_cpp/package.xml')
after=Path('/fix-source/mtc_aubo_fake_integration_cpp/package.xml')
a=parse_package(str(before));b=parse_package(str(after))
assert before.read_text().replace('<depend>nlohmann_json</depend>','<depend>nlohmann-json-dev</depend>')==after.read_text()
assert b.name==a.name
print('package format parse PASS; single rosdep key substitution only')
P
colcon --log-base /review-evidence/dependency-fix-v2/build-log build --base-paths /fix-source --build-base /fix-build-v2 --install-base /fix-install-v2 --packages-select mtc_aubo_fake_integration_cpp --executor sequential --cmake-args -DCMAKE_PREFIX_PATH=/offline-json/usr
source /fix-install-v2/setup.bash
colcon --log-base /review-evidence/dependency-fix-v2/test-log test --base-paths /fix-source --build-base /fix-build-v2 --install-base /fix-install-v2 --packages-select mtc_aubo_fake_integration_cpp --executor sequential --return-code-on-test-failure
colcon test-result --test-result-base /fix-build-v2 --verbose > /review-evidence/dependency-fix-v2/test-results.txt
python3 - <<'P'
import json,hashlib,xml.etree.ElementTree as E
from pathlib import Path
r=Path('/review-evidence/dependency-fix-v2');r.joinpath('xml').mkdir()
cases=[]
for p in Path('/fix-build-v2/mtc_aubo_fake_integration_cpp/test_results').rglob('*.gtest.xml'):
 r.joinpath('xml',p.name).write_bytes(p.read_bytes())
 for c in E.parse(p).getroot().iter('testcase'):
  assert c.get('status')=='run' and c.get('result')=='completed' and not any(c.find(x) is not None for x in ['failure','error','skipped'])
  cases.append(c.get('classname')+'.'+c.get('name'))
assert len(cases)==22 and len(set(cases))==22
r.joinpath('result.json').write_text(json.dumps(dict(cases=cases,count=22,passed=True,scope='package.xml rosdep key change; no kernel changes'),indent=2))
print('DEPENDENCY FIX 22 CASES PASS')
P
