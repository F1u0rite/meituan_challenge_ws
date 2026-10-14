"""Fail closed without a sourced install or with an incomplete sourced prefix."""
import json,os,sys,tempfile
from pathlib import Path
sys.path.insert(0,'/default-review/src/mtc_vision_adapter/test')
import ros_replay_validation as v
from ament_index_python.packages import PackageNotFoundError
# A prefix containing only the resource marker must not fall back to the
# valid /default-review/install left by the default-layout build.
assert Path('/default-review/install/mtc_vision_adapter/lib/mtc_vision_adapter/replay_publisher').is_file()
results=[]
for case in ['missing-package','missing-executable']:
 p=Path(tempfile.mkdtemp(prefix='prefix-negative-'))
 if case=='missing-executable':
  q=p/'share/ament_index/resource_index/packages/mtc_vision_adapter';q.parent.mkdir(parents=True);q.touch()
 os.environ['AMENT_PREFIX_PATH']=str(p)
 try:
  v.run_validation('/candidate/src/mtc_vision_adapter/examples/synthetic.jsonl',p/'evidence')
 except (PackageNotFoundError,AssertionError) as e:
  assert ('not found' in str(e) if case=='missing-package' else 'Installed replay_publisher missing: '+str(p) in str(e))
  assert not v.rclpy.ok()
  results.append({'case':case,'exception':type(e).__name__,'detail':str(e),'ros_context_started':False,'passed':True})
 else:raise AssertionError('Expected missing overlay/executable refusal')
Path('/review-evidence/negative-prefix.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results))
