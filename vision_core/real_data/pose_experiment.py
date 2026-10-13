"""GT and prediction remain separate immutable records."""
from pathlib import Path
from dataclasses import asdict
from datetime import datetime,timezone
import uuid,json,hashlib
import numpy as np
from real_data.rgbd_recording import save_rgbd_sample,_png
from evaluation.pose_accuracy import validate_gt
def now():return datetime.now(timezone.utc).isoformat()
def write_json(path,data):Path(path).write_text(json.dumps(data,indent=2,ensure_ascii=False),encoding='utf-8')
def save_accuracy_sample(root,frame,prediction,quality,gt,calibration,*,case_id,visualization,points,rgb,static_confirmed=False,capture_trigger='USER_KEY'):
    folder=Path(root)/('sample_'+uuid.uuid4().hex)
    save_rgbd_sample(folder,frame,prediction,points,rgb,visualization,scene_label='red_pose_accuracy')
    raw=getattr(frame,'raw_depth',None)
    if raw is not None:
        if raw.dtype!=np.uint16 or raw.ndim!=2:raise ValueError('Raw depth must be original Z16')
        raw_intr=getattr(frame,'raw_depth_intrinsics',None)
        if raw_intr is not None and raw.shape!=(raw_intr.height,raw_intr.width):
            raise ValueError('Raw depth / original intrinsics size mismatch')
        _png(folder/'raw_depth.png',raw)
    write_json(folder/'fixture_gt.json',gt)
    write_json(folder/'fixture_calibration.json',calibration)
    write_json(folder/'prediction.json',prediction)
    write_json(folder/'quality.json',quality)
    metadata={'schema_version':1,'sample_id':folder.name,'case_id':case_id,'timestamp':now(),
        'relative_static_confirmed':bool(static_confirmed),'capture_trigger':capture_trigger,
        'gt_status':validate_gt(gt) or 'human_measured_confirmed',
        'raw_depth_status':'AVAILABLE' if raw is not None else 'UNAVAILABLE',
        'raw_depth_intrinsics':asdict(frame.raw_depth_intrinsics) if getattr(frame,'raw_depth_intrinsics',None) is not None else 'UNAVAILABLE',
        'alignment_metadata':getattr(frame,'alignment_metadata',None) or 'UNAVAILABLE',
        'camera_configuration':getattr(frame,'camera_configuration',None) or 'UNAVAILABLE',
        'prediction_coordinate_frame':'camera_color_optical','prediction_unit':'m',
        'body_pose_valid':False,'ring_pose_valid':False,'robot_execution_allowed':False}
    write_json(folder/'experiment_metadata.json',metadata)
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
    write_json(folder/'integrity.json',hashes)
    return folder
def verify_integrity(folder):
    folder=Path(folder);hashes=json.loads((folder/'integrity.json').read_text(encoding='utf-8-sig'))
    return {'valid':all((folder/name).is_file() and hashlib.sha256((folder/name).read_bytes()).hexdigest()==sha for name,sha in hashes.items()),
            'files':len(hashes)}
