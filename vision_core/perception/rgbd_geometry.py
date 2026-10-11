"""Offline RGB-D geometry in the optical camera frame (metres).

The visible-point estimate is deliberately not called the battery body centre.
No table/base coordinates or ring pose can be inferred from this module.
"""
from dataclasses import dataclass
from datetime import datetime
import cv2
import numpy as np


def frame_capture_time(frame):
    """Prefer a recorded SDK Unix/system timestamp; label receipt-time fallback."""
    timestamps=frame.timestamps
    receipt=datetime.fromisoformat(timestamps['host_utc']).timestamp()
    domain=str(timestamps.get('color_timestamp_domain',''))
    sdk=timestamps.get('color_sdk_ms')
    if sdk is not None and ('system_time' in domain or 'global_time' in domain):
        stamp=float(sdk)/1000
        if np.isfinite(stamp) and stamp>1e9 and abs(stamp-receipt)<60:
            return stamp,'COLOR_SDK_SYSTEM_CAPTURE_TIME'
    return receipt,'HOST_RECEIPT_TIME_FALLBACK; SDK_CLOCK_NOT_MAPPED'


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    model: str = 'none'
    coeffs: tuple = ()

    def validate(self):
        if not np.isfinite([self.width,self.height,self.fx,self.fy,self.ppx,self.ppy,*self.coeffs]).all() or self.width <= 0 or self.height <= 0 or self.fx <= 0 or self.fy <= 0:
            raise ValueError('Invalid camera intrinsics')


def validate_frame(color_bgr, aligned_depth, intrinsics, depth_scale):
    intrinsics.validate()
    if not isinstance(color_bgr,np.ndarray) or color_bgr.size==0 or color_bgr.dtype != np.uint8 or color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
        raise ValueError('Color must be BGR uint8')
    if not isinstance(aligned_depth,np.ndarray) or aligned_depth.size==0 or aligned_depth.dtype != np.uint16 or aligned_depth.ndim != 2:
        raise ValueError('Aligned depth must be raw uint16')
    if color_bgr.shape[:2] != aligned_depth.shape or aligned_depth.shape != (intrinsics.height, intrinsics.width):
        raise ValueError('Color, aligned depth and intrinsics sizes differ')
    if not np.isfinite(depth_scale) or depth_scale <= 0:
        raise ValueError('Invalid depth scale')


def deproject_pixels(uv, depth_m, intrinsics, sdk_deproject=None):
    """Deproject pixels; SDK callback handles lens distortion when present."""
    intrinsics.validate()
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    z = np.asarray(depth_m, dtype=np.float64).reshape(-1)
    if len(uv) != len(z) or np.any(~np.isfinite(z)) or np.any(z <= 0):
        raise ValueError('Invalid pixel/depth pairs')
    if sdk_deproject is not None and any(abs(c)>1e-10 for c in intrinsics.coeffs):
        return np.asarray([sdk_deproject((float(u), float(v)), float(d))
                           for (u, v), d in zip(uv, z)], dtype=np.float64)
    # The recorded aligned-depth intrinsics declare inverse Brown but have all-zero
    # coefficients; their projection is exactly the pinhole form.
    if any(abs(c) > 1e-10 for c in intrinsics.coeffs):
        raise ValueError('Distorted intrinsics require SDK deprojection')
    return np.column_stack(((uv[:, 0]-intrinsics.ppx)*z/intrinsics.fx,
                            (uv[:, 1]-intrinsics.ppy)*z/intrinsics.fy, z))


def fit_plane_ransac(points, *, threshold_m=.008, iterations=160, min_inliers=80):
    """Fit a plane candidate; the caller must establish that its ROI is a table."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(points) < max(min_inliers, 3):
        return None
    rng = np.random.default_rng(1049)
    best = None
    # Same seeded samples and gates, evaluated in bounded vectorized batches.
    for first in range(0,iterations,16):
        triplets=np.asarray([points[rng.choice(len(points),3,replace=False)] for _ in range(min(16,iterations-first))])
        normals=np.cross(triplets[:,1]-triplets[:,0],triplets[:,2]-triplets[:,0])
        norms=np.linalg.norm(normals,axis=1);valid=norms>=1e-9
        normals=np.divide(normals,norms[:,None],out=np.zeros_like(normals),where=valid[:,None])
        offsets=np.sum(triplets[:,0]*normals,axis=1)
        residual=np.abs(points@normals.T-offsets)
        masks=(residual<=threshold_m)&valid
        counts=masks.sum(axis=0);index=int(np.argmax(counts))
        if best is None or counts[index]>best.sum():best=masks[:,index]
    if best is None or best.sum() < min_inliers:
        return None
    centre = points[best].mean(axis=0)
    _, _, vh = np.linalg.svd(points[best]-centre, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    residual = np.abs((points[best]-centre) @ normal)
    return {'normal_camera_xyz': normal.tolist(), 'point_camera_m': centre.tolist(),
            'inlier_count': int(best.sum()), 'inlier_ratio': float(best.mean()),
            'mean_residual_m': float(residual.mean()), 'candidate_only': True}


def _axis(points):
    if len(points) < 10:
        return None
    centred = points-points.mean(axis=0)
    values, vectors = np.linalg.eigh(centred.T @ centred / len(points))
    if values[-1] < 1.3*max(values[-2], 1e-12):
        return None
    vector = vectors[:, -1]
    if vector[0] < 0:
        vector = -vector
    return {'axis_camera_xyz': vector.tolist(), 'eigenvalue_ratio': float(values[-1]/max(values[-2], 1e-12)),
            'direction_ambiguity_deg': 180}


def analyze_rgbd(color_bgr, aligned_depth, mask, intrinsics, depth_scale,
                 *, min_depth_m=.15, max_depth_m=2.0, max_points=12000,
                 sdk_deproject=None, table_roi=None):
    validate_frame(color_bgr, aligned_depth, intrinsics, depth_scale)
    mask = np.asarray(mask)
    if mask.shape != aligned_depth.shape:
        raise ValueError('Battery mask size differs from aligned depth')
    mask = mask.astype(bool)
    raw = aligned_depth.astype(np.float64)
    depth_m = raw*depth_scale
    valid_range = (raw > 0) & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    initial = mask & valid_range
    ratio = float(initial.sum()/mask.sum()) if mask.any() else 0.0
    result = {'coordinate_frame': 'CAMERA_OPTICAL_M', 'depth_valid_ratio': ratio,
              'mask_pixel_count': int(mask.sum()), 'valid_depth_pixel_count': int(initial.sum()),
              'point_count': 0, 'depth_distribution_m': None,
              'visible_surface_position_m': None, 'visible_surface_valid': False,
              'body_position_m': None, 'body_pose_valid': False,
              'ring_pose_valid': False, 'ring_position_m': None,
              'table_position_m': None, 'base_position_m': None,
              'table_frame_valid': False, 'yaw_table_deg': None,
              'top_surface_axis': None, 'pca_axis': None, 'plane_candidate': None,
              'object_plane_separation_m': None, 'object_points_away_from_plane_ratio': None,
              'pose_validity_reason': 'INSUFFICIENT_VALID_DEPTH'}
    if initial.sum() < 20:
        return result, np.empty((0, 3)), np.empty((0, 3), np.uint8)
    values = depth_m[initial]
    median = np.median(values)
    mad = np.median(np.abs(values-median))
    tolerance = max(.015, 3.5*1.4826*mad)
    filtered = initial & (np.abs(depth_m-median) <= tolerance)
    if filtered.sum() < 20:
        return result, np.empty((0, 3)), np.empty((0, 3), np.uint8)
    vv, uu = np.where(filtered)
    indices = np.linspace(0, len(uu)-1, min(len(uu), max_points), dtype=int)
    vv, uu = vv[indices], uu[indices]
    points = deproject_pixels(np.column_stack((uu, vv)), depth_m[vv, uu], intrinsics, sdk_deproject)
    rgb = color_bgr[vv, uu, ::-1].copy()
    result['point_count'] = len(points)
    result['depth_distribution_m'] = {'p05': float(np.percentile(values, 5)),
                                      'median': float(median), 'p95': float(np.percentile(values, 95))}
    result['visible_surface_position_m'] = np.median(points, axis=0).tolist()
    result['visible_surface_valid'] = True
    result['pca_axis'] = _axis(points)
    # Near-depth subset is merely a top-surface hypothesis, not a measured body yaw.
    near = points[:, 2] <= np.percentile(points[:, 2], 25)
    result['top_surface_axis'] = _axis(points[near])
    if table_roi is None and mask.any():
        # Nearby background is only a table-plane hypothesis. It may include a wall.
        edge = np.uint8(mask)
        outer = cv2.dilate(edge, np.ones((41, 41), np.uint8))
        inner = cv2.dilate(edge, np.ones((9, 9), np.uint8))
        table_roi = (outer > 0) & (inner == 0)
    if table_roi is not None:
        table_roi = np.asarray(table_roi, dtype=bool)
        if table_roi.shape != mask.shape:
            raise ValueError('Table ROI size differs from depth')
        background = table_roi & ~mask & valid_range
        bv, bu = np.where(background)
        if len(bu) >= 80:
            ix = np.linspace(0, len(bu)-1, min(len(bu), max_points), dtype=int)
            bv, bu = bv[ix], bu[ix]
            bg_points = deproject_pixels(np.column_stack((bu, bv)), depth_m[bv, bu], intrinsics, sdk_deproject)
            result['plane_candidate'] = fit_plane_ransac(bg_points)
            if result['plane_candidate'] is not None:
                plane = result['plane_candidate']
                normal = np.asarray(plane['normal_camera_xyz'])
                origin = np.asarray(plane['point_camera_m'])
                gaps = np.abs((points-origin) @ normal)
                result['object_plane_separation_m'] = float(np.median(gaps))
                result['object_points_away_from_plane_ratio'] = float(np.mean(gaps > .015))
    result['pose_validity_reason'] = 'VISIBLE_SURFACE_ONLY; BODY_GEOMETRY_AND_TABLE_FRAME_UNCALIBRATED'
    return result, points, rgb


def project_points(points_m, intrinsics, sdk_project=None):
    """Camera-optical XYZ to pixels; behind-camera/invalid points stay NaN."""
    intrinsics.validate()
    points = np.asarray(points_m, dtype=float).reshape(-1, 3)
    uv = np.full((len(points), 2), np.nan)
    good = np.isfinite(points).all(axis=1) & (points[:, 2] > 0)
    if sdk_project is not None:
        uv[good] = [sdk_project(p.tolist()) for p in points[good]]
    elif any(abs(c) > 1e-10 for c in intrinsics.coeffs):
        raise ValueError('Distorted intrinsics require SDK projection')
    else:
        p = points[good]
        uv[good] = np.column_stack((p[:, 0]/p[:, 2]*intrinsics.fx+intrinsics.ppx,
                                    p[:, 1]/p[:, 2]*intrinsics.fy+intrinsics.ppy))
    return uv
