"""Read approved native NPY recordings without changing original pixels or time."""
import hashlib,json
from pathlib import Path
import numpy as np
from input.realsense_rgbd_source import RGBDFrame
from input.rgbd_pair_guard import RGBDPairGuard
from perception.rgbd_geometry import CameraIntrinsics,validate_frame


def qualified_native_pairs(root, manifest_path):
    root=Path(root).resolve();manifest_path=Path(manifest_path)
    manifest=json.loads(manifest_path.read_text())
    if Path(manifest['source']).resolve()!=root:raise ValueError('Pair manifest source does not match replay root')
    indices=manifest['accepted_original_indices']
    if (not indices or any(type(i) is not int or i<0 for i in indices)
            or indices!=sorted(set(indices))):raise ValueError('Invalid original pair indices')
    integrity=manifest_path.parent/'original_sha256.json'
    if not integrity.is_file():raise FileNotFoundError('Original pair SHA256 manifest required')
    hashes=json.loads(integrity.read_text());guard=RGBDPairGuard()
    for index in indices:
        folder=root/'frames'/f'{index:06d}'
        for filename in ('metadata.json','color_bgr.npy','raw_depth_z16.npy','aligned_depth_z16.npy'):
            p=folder/filename
            if hashlib.sha256(p.read_bytes()).hexdigest()!=hashes[str(p.relative_to(root))]:
                raise IOError('Original RGB-D integrity mismatch')
        meta=json.loads((folder/'metadata.json').read_text())
        if meta['index']!=index:raise ValueError('Recorded index mismatch')
        report=guard.inspect(meta)
        if not report['qualified']:raise ValueError('Unqualified manifest pair: '+str(report))
        color=np.load(folder/'color_bgr.npy',allow_pickle=False)
        raw=np.load(folder/'raw_depth_z16.npy',allow_pickle=False)
        depth=np.load(folder/'aligned_depth_z16.npy',allow_pickle=False)
        if raw.dtype!=np.uint16 or raw.ndim!=2:raise ValueError('Raw depth must be Z16')
        frame=RGBDFrame(color,depth,CameraIntrinsics(**meta['aligned_intrinsics']),
            meta['depth_scale_m_per_unit'],meta,meta['stream_profiles'],raw,
            CameraIntrinsics(**meta['raw_depth_intrinsics']),meta['alignment_metadata'])
        validate_frame(color,depth,frame.intrinsics,frame.depth_scale)
        yield 'original_'+f'{index:06d}',frame,folder
