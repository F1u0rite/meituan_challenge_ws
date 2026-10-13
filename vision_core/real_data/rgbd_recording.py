"""Lossless, one-sample-per-directory RGB-D recording and replay."""
from dataclasses import asdict
from pathlib import Path
import json,hashlib
from time import perf_counter
import cv2
import numpy as np
from perception.rgbd_geometry import CameraIntrinsics, validate_frame
from input.realsense_rgbd_source import RGBDFrame


def _png(path, image, timing_ms=None):
    start=perf_counter()
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise IOError(f'Cannot encode {path}')
    encoded=perf_counter()
    data.tofile(str(path))
    if timing_ms is not None:
        timing_ms["png_encoding"]=timing_ms.get("png_encoding",0)+(encoded-start)*1000
        timing_ms["file_write"]=timing_ms.get("file_write",0)+(perf_counter()-encoded)*1000


def _read_png(path, flags):
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), flags)
    if image is None:
        raise IOError(f'Cannot decode {path}')
    return image


def save_ply(path, points_m, rgb, timing_ms=None):
    points = np.asarray(points_m)
    rgb = np.asarray(rgb)
    if points.shape != (len(rgb), 3) or rgb.shape != (len(points), 3):
        raise ValueError('PLY point and color arrays differ')
    start=perf_counter()
    header='ply\nformat ascii 1.0\n'+f'element vertex {len(points)}\n'
    header+='property float x\nproperty float y\nproperty float z\n'
    header+='property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'
    body=''.join(f'{p[0]:.8f} {p[1]:.8f} {p[2]:.8f} {int(c[0])} {int(c[1])} {int(c[2])}\n' for p,c in zip(points,rgb))
    encoded=perf_counter();Path(path).write_text(header+body,encoding='ascii')
    if timing_ms is not None:
        timing_ms['ply_encoding']=timing_ms.get('ply_encoding',0)+(encoded-start)*1000
        timing_ms['file_write']=timing_ms.get('file_write',0)+(perf_counter()-encoded)*1000


def _json(path,value,timing_ms=None,strict=False):
    start=perf_counter();encoded=json.dumps(value,indent=2,allow_nan=not strict);ready=perf_counter()
    path.write_text(encoded,encoding='utf-8')
    if timing_ms is not None:
        timing_ms['json_encoding']=timing_ms.get('json_encoding',0)+(ready-start)*1000
        timing_ms['file_write']=timing_ms.get('file_write',0)+(perf_counter()-ready)*1000


def save_rgbd_sample(folder, frame, result, points_m, rgb, visualization, *, scene_label, timing_ms=None):
    folder = Path(folder)
    if folder.exists():
        raise FileExistsError(f'RGB-D sample already exists: {folder}')
    validate_frame(frame.color_bgr, frame.aligned_depth, frame.intrinsics, frame.depth_scale)
    folder.mkdir(parents=True)
    _png(folder/'raw_color.png', frame.color_bgr,timing_ms)
    _png(folder/'aligned_depth.png', frame.aligned_depth,timing_ms)
    _png(folder/'visualization.png', visualization,timing_ms)
    _json(folder/'camera_intrinsics.json',asdict(frame.intrinsics),timing_ms)
    _json(folder/'metadata.json',{'depth_scale_m_per_unit': frame.depth_scale,
        'timestamps': frame.timestamps, 'stream_profiles': frame.stream_profiles,
        'scene_label': scene_label, 'depth_format': 'Z16_RAW_ALIGNED_TO_COLOR'},timing_ms)
    _json(folder/'detection_result.json',result,timing_ms)
    if len(points_m):
        save_ply(folder/'point_cloud.ply', points_m, rgb,timing_ms)
    return folder


def load_rgbd_sample(folder):
    folder = Path(folder)
    integrity=folder/'integrity.json'
    if integrity.exists():
        record=json.loads(integrity.read_text(encoding='utf-8'))
        if 'complete' in record:
            if record['complete'] is not True:raise IOError('RGB-D recording not complete')
            files=record['files']
        else:
            # Existing ground-truth experiment format maps filename to SHA256.
            files={name:dict(sha256=value) for name,value in record.items()}
        for name,properties in files.items():
            target=(folder/name).resolve()
            if target.parent!=folder.resolve():raise ValueError('Unsafe integrity filename')
            encoded=target.read_bytes()
            if ('bytes' in properties and len(encoded)!=properties['bytes']) or hashlib.sha256(encoded).hexdigest()!=properties['sha256']:
                raise IOError('RGB-D checksum mismatch: '+name)
    intrinsics = CameraIntrinsics(**json.loads((folder/'camera_intrinsics.json').read_text(encoding='utf-8')))
    metadata = json.loads((folder/'metadata.json').read_text(encoding='utf-8'))
    frame = RGBDFrame(_read_png(folder/'raw_color.png', cv2.IMREAD_COLOR),
                      _read_png(folder/'aligned_depth.png', cv2.IMREAD_UNCHANGED),
                      intrinsics, metadata['depth_scale_m_per_unit'],
                      metadata['timestamps'], metadata['stream_profiles'])
    validate_frame(frame.color_bgr, frame.aligned_depth, intrinsics, frame.depth_scale)
    # Continuous recordings store these values in metadata rather than an extra file.
    if (folder/'raw_depth.png').exists():
        frame.raw_depth=_read_png(folder/'raw_depth.png',cv2.IMREAD_UNCHANGED)
    original_intr=metadata.get('raw_depth_intrinsics')
    if isinstance(original_intr,dict):frame.raw_depth_intrinsics=CameraIntrinsics(**original_intr)
    frame.alignment_metadata=metadata.get('alignment_metadata')
    frame.camera_configuration=metadata.get('camera_configuration')
    extra=folder/'experiment_metadata.json'
    if extra.exists():
        experiment=json.loads(extra.read_text(encoding='utf-8-sig'))
        if (folder/'raw_depth.png').exists():
            frame.raw_depth=_read_png(folder/'raw_depth.png',cv2.IMREAD_UNCHANGED)
        original_intr=experiment.get('raw_depth_intrinsics')
        if isinstance(original_intr,dict):frame.raw_depth_intrinsics=CameraIntrinsics(**original_intr)
        frame.alignment_metadata=experiment.get('alignment_metadata')
        frame.camera_configuration=experiment.get('camera_configuration')
    return frame


def save_vision_sample(folder,frame,result,views,visualization,*,timing_ms=None):
    """Save current unmodified input and diagnostic output. Never overwrite."""
    points=np.concatenate([p['points'] for p in views]) if views else np.empty((0,3))
    rgb=np.concatenate([p['rgb'] for p in views]) if views else np.empty((0,3),np.uint8)
    folder=save_rgbd_sample(folder,frame,result,points,rgb,visualization,
                           scene_label=result.get('mode','vision_diagnostic'),timing_ms=timing_ms)
    masks={}
    for item,payload in zip(result['objects'],views):
        name='mask_'+item['object_id']+'.png'
        _png(folder/name,np.uint8(payload['mask'])*255,timing_ms);masks[item['object_id']]=name
    raw_available=frame.raw_depth is not None
    if raw_available:_png(folder/'raw_depth.png',frame.raw_depth,timing_ms)
    metadata={'raw_depth_available':raw_available,'raw_depth_status':'RECORDED' if raw_available else 'UNAVAILABLE_FROM_SOURCE',
      'raw_depth_intrinsics':asdict(frame.raw_depth_intrinsics) if frame.raw_depth_intrinsics else None,
      'alignment_metadata':frame.alignment_metadata,'camera_configuration':frame.camera_configuration,
      'mask_paths':masks,'capture_trigger':result.get('capture_trigger','OFFLINE_PROCESSING')}
    _json(folder/'experiment_metadata.json',metadata,timing_ms)
    _json(folder/'prediction.json',result,timing_ms,strict=True)
    return folder
