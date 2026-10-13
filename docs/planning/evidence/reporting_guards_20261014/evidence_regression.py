"""Reproduce and regress reporting defects using actual, immutable run logs."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys

R = Path('/review-evidence')
S = Path('/repair-source/src/mtc_moveit_planner/scripts')
sys.path.insert(0, str(S))
spec = importlib.util.spec_from_file_location('checker', S / 'check_planning_evidence.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)
rows = []
for run in ['planning-default', 'planning-retained']:
    for label in ['plan_fixed_pose', 'collision_probe', 'tool_collision_probe', 'plan_battery_sequence_v1', 'plan_battery_sequence_v2']:
        p = R / run / (label + '.log')
        text = p.read_text()
        passed, detail = checker.classify(text, 'collision_probe' in label)
        rows.append(dict(run=run, label=label, sha256=hashlib.sha256(p.read_bytes()).hexdigest(), passed=passed, detail=detail))
assert rows[0]['passed'] and rows[4]['passed']
assert 'collision cause UNVERIFIED' in rows[1]['detail'] and not rows[1]['passed']
assert all('NO_PLANNING_RESULT' in rows[i]['detail'] for i in [2,3,5,6,7,8,9])

# A disposable ROS-free fixture makes the delivered overwrite reproducible:
# the function shadows ros2 and aborts at the empty-domain precheck.
outcomes = []
for label, original in [('before', True), ('after', False)]:
    work = R / ('overwrite-' + label)
    work.mkdir()
    script = work / 'run.sh'
    source = Path('/candidate/src/mtc_moveit_planner/scripts/run_planning_evidence.sh') if original else S / 'run_planning_evidence.sh'
    shutil.copyfile(source, script)
    (work / 'env.sh').write_text('export ROS_DISTRO=jazzy ROS_DOMAIN_ID=221 RMW_IMPLEMENTATION=fixture\nAUBO_WS=/unused\nMTC_WS=/unused\nros2() { if [ "$1" = node ]; then echo /fixture_occupied; else echo /unused; fi; }\n')
    target = work / 'old output'; target.mkdir()
    p = target / 'SUMMARY.txt'; sentinel = b'ORIGINAL_FAILURE_MUST_SURVIVE\n'; p.write_bytes(sentinel)
    q = subprocess.run(['bash', str(script), str(target)], capture_output=True, text=True, timeout=10)
    (work / 'stdout.log').write_text(q.stdout)
    (work / 'stderr.log').write_text(q.stderr)
    outcomes.append(dict(case=label, exit_code=q.returncode, evidence_preserved=p.read_bytes()==sentinel,
                         after_sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
assert outcomes[0]['exit_code']==1 and not outcomes[0]['evidence_preserved']
assert outcomes[1]['exit_code']==2 and outcomes[1]['evidence_preserved']
(R / 'evidence-regression.json').write_text(json.dumps(dict(passed=True, actual_logs=rows, overwrite=outcomes), indent=2)+'\n')
print(json.dumps(dict(passed=True, actual_logs=len(rows), overwrite_before_after=outcomes)))
