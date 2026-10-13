from pathlib import Path
import importlib,json,hashlib
names=['mtc_vision_adapter.automatic_top_result_publisher','mtc_vision_adapter.rgb_image_publisher','mtc_interfaces','perception.height_constraint','rclpy','pytest','cv2','numpy']
r={n: {'path':importlib.import_module(n).__file__} for n in names}
for n,v in r.items():
 if v['path']: v['sha256']=hashlib.sha256(Path(v['path']).read_bytes()).hexdigest()
source=Path('/fix-review/src/mtc_vision_adapter/mtc_vision_adapter/automatic_top_result_publisher.py')
assert r['mtc_vision_adapter.automatic_top_result_publisher']['sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
assert r['mtc_vision_adapter.automatic_top_result_publisher']['path'].startswith('/registration-install/')
r['source_sha256']={str(p.relative_to('/fix-review')):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('/fix-review/src/mtc_vision_adapter').rglob('*') if p.is_file() and '.egg-info' not in str(p) and '__pycache__' not in str(p)}
Path('/review-evidence/registration-module-identity.json').write_text(json.dumps(r,indent=2)+'\n')
print('PASS installed PR19 module identity')
