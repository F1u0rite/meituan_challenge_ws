"""Conservative RGB-D table and battery diagnostics; no validated body pose."""
from dataclasses import dataclass, asdict
from time import perf_counter
import cv2
import numpy as np
from .rgbd_geometry import deproject_pixels, fit_plane_ransac, validate_frame


@dataclass(frozen=True)
class OfflineConfig:
    min_depth_m: float = .15
    max_depth_m: float = 2.0
    fast_table_roi: bool = False
    table_ransac_iterations: int = 160
    adaptive_table_roi: bool = False
    table_inner_px: int = 18
    table_outer_px: int = 90
    table_min_points: int = 1500
    table_min_inlier_ratio: float = .75
    table_max_residual_m: float = .004
    table_min_camera_facing: float = .7
    ransac_threshold_m: float = .006
    min_foreground_height_m: float = .012
    max_foreground_height_m: float = .12
    candidate_min_valid_ratio: float = .5
    candidate_min_foreground_ratio: float = .5
    component_min_area_px: int = 500
    max_cloud_points: int = 20000
    single_red_object: bool = True
    depth_bridge_px: int = 35
    grow_x_fraction: float = .30
    grow_y_fraction: float = 1.40
    max_growth_ratio: float = 2.5
    joint_rgb_depth: bool = False
    joint_footprint_m: tuple = (.080, .070)
    joint_max_footprint_scale: float = 1.8
    min_seed_overlap_ratio: float = .5
    join_min_x_overlap_ratio: float = .6
    join_max_vertical_gap_px: int = 120
    join_min_area_px: int = 1000
    join_max_height_difference_m: float = .05


def _sample_cloud(depth_m, region, intr, limit, sdk_deproject=None):
    v, u = np.where(region)
    if len(v) == 0:
        return np.empty((0, 3)), np.empty((0, 2), np.int32)
    ix = np.linspace(0, len(v)-1, min(len(v), limit), dtype=np.int64)
    uv = np.column_stack((u[ix], v[ix]))
    return deproject_pixels(uv, depth_m[v[ix], u[ix]], intr, sdk_deproject), uv


def fit_table(depth_m, red_mask, intr, cfg, sdk_deproject=None):
    valid = np.isfinite(depth_m) & (depth_m >= cfg.min_depth_m) & (depth_m <= cfg.max_depth_m)
    if not red_mask.any():
        return None, np.zeros(valid.shape, bool), 'NO_RED_EXCLUSION_MASK'
    if cfg.fast_table_roi:
        x,y,w,h=cv2.boundingRect(np.uint8(red_mask));pad=cfg.table_outer_px+1
        x0,y0=max(0,x-pad),max(0,y-pad);x1,y1=min(valid.shape[1],x+w+pad),min(valid.shape[0],y+h+pad)
        distance=cv2.distanceTransform(np.uint8(~red_mask[y0:y1,x0:x1].astype(bool)),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
        roi=np.zeros(valid.shape,bool)
        roi[y0:y1,x0:x1]=(distance<=cfg.table_outer_px)&(distance>cfg.table_inner_px)&valid[y0:y1,x0:x1]
    else:
        outer = cv2.dilate(np.uint8(red_mask), cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2*cfg.table_outer_px+1,)*2)) > 0
        inner = cv2.dilate(np.uint8(red_mask), cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2*cfg.table_inner_px+1,)*2)) > 0
        roi = outer & ~inner & valid
    points, _ = _sample_cloud(depth_m, roi, intr, cfg.max_cloud_points, sdk_deproject)
    if len(points) < cfg.table_min_points:
        return None, roi, 'INSUFFICIENT_BACKGROUND_DEPTH'
    plane = fit_plane_ransac(points, threshold_m=cfg.ransac_threshold_m,
                             min_inliers=cfg.table_min_points,iterations=cfg.table_ransac_iterations)
    if plane is None:
        return None, roi, 'PLANE_FIT_FAILED'
    if (plane['inlier_ratio'] < cfg.table_min_inlier_ratio or
            plane['mean_residual_m'] > cfg.table_max_residual_m or
            abs(plane['normal_camera_xyz'][2]) < cfg.table_min_camera_facing):
        if cfg.adaptive_table_roi:
            seed_depth=depth_m[red_mask & valid]
            if len(seed_depth):
                near=float(np.median(seed_depth)); expanded=cv2.dilate(np.uint8(red_mask),np.ones((2*cfg.table_outer_px+1,)*2,np.uint8))>0
                retry=expanded & ~red_mask & valid & (depth_m>near+cfg.min_foreground_height_m) & (depth_m<near+cfg.max_foreground_height_m)
                retry_points,_=_sample_cloud(depth_m,retry,intr,cfg.max_cloud_points,sdk_deproject)
                other=fit_plane_ransac(retry_points,threshold_m=cfg.ransac_threshold_m,min_inliers=cfg.table_min_points,iterations=cfg.table_ransac_iterations)
                if other is not None and other['inlier_ratio']>=cfg.table_min_inlier_ratio and other['mean_residual_m']<=cfg.table_max_residual_m and abs(other['normal_camera_xyz'][2])>=cfg.table_min_camera_facing:
                    other['initial_plane_rejected']=plane
                    other['support_region']='LOCAL_BACKGROUND_DEPTH_BAND_OUTSIDE_SEED; UNVERIFIED'
                    other['status']='TABLE_CANDIDATE_NOT_EXTERNALLY_VALIDATED'
                    return other,retry,None
        return None, roi, 'PLANE_QUALITY_FAILED'
    plane['support_region'] = 'ANNULUS_OUTSIDE_RED_CANDIDATE'
    plane['status'] = 'TABLE_CANDIDATE_NOT_EXTERNALLY_VALIDATED'
    return plane, roi, None


def height_from_plane(depth_m, intr, plane, rays_xyz=None):
    """Signed distance; positive means closer to the camera than the plane."""
    n = np.asarray(plane['normal_camera_xyz'])
    o = np.asarray(plane['point_camera_m'])
    if rays_xyz is not None:
        if rays_xyz.shape != (*depth_m.shape, 3): raise ValueError("Ray grid shape mismatch")
        # Signed plane distance without allocating a full XYZ temporary.
        return float(o @ n)-depth_m*(rays_xyz @ n)
    if any(abs(c)>1e-10 for c in intr.coeffs):
        raise ValueError("Distorted intrinsics require ray grid")
    # Separable pinhole rays: only 1-D coordinate arrays plus one depth-sized result.
    u=(np.arange(depth_m.shape[1],dtype=np.float64)-intr.ppx)/intr.fx
    v=(np.arange(depth_m.shape[0],dtype=np.float64)-intr.ppy)/intr.fy
    return float(o @ n)-depth_m*(n[0]*u[None,:]+n[1]*v[:,None]+n[2])


def largest_component(mask, min_area):
    x,y,w,h=cv2.boundingRect(np.uint8(mask))
    out=np.zeros(mask.shape,bool)
    if not w or not h:return out,0
    count,labels,stats,_=cv2.connectedComponentsWithStats(np.uint8(mask[y:y+h,x:x+w]),8)
    if count<=1:return out,0
    ix=int(1+np.argmax(stats[1:,cv2.CC_STAT_AREA]));area=int(stats[ix,cv2.CC_STAT_AREA])
    if area>=min_area:out[y:y+h,x:x+w]=labels==ix
    return out,area


def grow_depth_object(seed, foreground, cfg, height_m=None):
    if not seed.any():return seed, {"accepted":False,"reason":"EMPTY_RGB_SEED","growth_ratio":None}
    x,y,w,h=cv2.boundingRect(np.uint8(seed));dx=int(w*cfg.grow_x_fraction)+cfg.depth_bridge_px;dy=int(h*cfg.grow_y_fraction)+cfg.depth_bridge_px
    x0,y0=max(0,x-dx),max(0,y-dy);x1,y1=min(seed.shape[1],x+w+dx),min(seed.shape[0],y+h+dy)
    grown,info=_grow_depth_roi(seed[y0:y1,x0:x1],foreground[y0:y1,x0:x1],cfg,None if height_m is None else height_m[y0:y1,x0:x1])
    out=np.zeros_like(seed);out[y0:y1,x0:x1]=grown
    return out,info

def _grow_depth_roi(seed, foreground, cfg, height_m=None):
    """Join split foreground bands near the RGB seed; never fill depth holes."""
    if not seed.any():
        return seed, {'accepted': False, 'reason': 'EMPTY_RGB_SEED', 'growth_ratio': None}
    v, u = np.where(seed)
    left, right = int(u.min()), int(u.max()+1)
    top, bottom = int(v.min()), int(v.max()+1)
    width, height = right-left, bottom-top
    dx = int(width*cfg.grow_x_fraction)
    dy = int(height*cfg.grow_y_fraction)
    spatial = np.zeros(seed.shape, bool)
    spatial[max(0, top-dy):min(seed.shape[0], bottom+dy),
            max(0, left-dx):min(seed.shape[1], right+dx)] = True
    close = cv2.morphologyEx(np.uint8(foreground & spatial), cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                      (cfg.depth_bridge_px, cfg.depth_bridge_px)))
    count, labels, _, _ = cv2.connectedComponentsWithStats(close, 8)
    if count <= 1:
        return seed, {'accepted': False, 'reason': 'NO_DEPTH_COMPONENT', 'growth_ratio': 1.0}
    overlaps = [int(((labels == i) & seed).sum()) for i in range(1, count)]
    chosen = 1+int(np.argmax(overlaps))
    overlap_ratio = overlaps[chosen-1]/max(int(seed.sum()), 1)
    grown = seed & foreground
    joined = 0
    count_raw, labels_raw, stats_raw, _ = cv2.connectedComponentsWithStats(
        np.uint8(foreground & spatial), 8)
    seed_height = float(np.median(height_m[seed])) if height_m is not None else None
    for index in range(1, count_raw):
        x, y, w, h, area = map(int, stats_raw[index])
        if area < cfg.join_min_area_px:
            continue
        component = labels_raw == index
        if (component & seed).any():
            grown |= component
            continue
        x_overlap = max(0, min(right, x+w)-max(left, x))/max(min(width, w), 1)
        y_gap = max(0, y-bottom, top-(y+h))
        if x_overlap < cfg.join_min_x_overlap_ratio or y_gap > cfg.join_max_vertical_gap_px:
            continue
        if height_m is not None and abs(float(np.median(height_m[component]))-seed_height) > cfg.join_max_height_difference_m:
            continue
        grown |= component
        joined += 1
    ratio = float(grown.sum()/max(seed.sum(), 1))
    if overlap_ratio < cfg.min_seed_overlap_ratio:
        return seed, {'accepted': False, 'reason': 'LOW_SEED_OVERLAP', 'growth_ratio': ratio}
    if ratio > cfg.max_growth_ratio:
        return seed, {'accepted': False, 'reason': 'EXCESSIVE_DEPTH_GROWTH', 'growth_ratio': ratio}
    return grown, {'accepted': True, 'reason': None, 'growth_ratio': ratio,
                   'seed_overlap_ratio': overlap_ratio, 'joined_disconnected_components': joined}


def body_geometry(points, plane, image_yaw=None):
    if len(points) < 20 or plane is None:
        return {'visible_surface_center_m': None, 'model_estimated_body_center_m': None,
                'body_pose_valid': False, 'reason': 'EMPTY_CLOUD_OR_NO_TABLE'}
    n = np.asarray(plane['normal_camera_xyz'], np.float64)
    o = np.asarray(plane['point_camera_m'], np.float64)
    height = (o-points) @ n
    ex = np.array([1., 0., 0.])-n*n[0]
    ex /= np.linalg.norm(ex)
    ey = np.cross(n, ex)
    ey /= np.linalg.norm(ey)
    local = np.column_stack(((points-o) @ ex, (points-o) @ ey))
    rect = cv2.minAreaRect(np.asarray(local, np.float32))
    box = cv2.boxPoints(rect)
    edges = np.roll(box, -1, axis=0)-box
    major = edges[np.argmax(np.linalg.norm(edges, axis=1))]
    major /= max(np.linalg.norm(major), 1e-12)
    obb_axis = major[0]*ex+major[1]*ey
    eig, axes = np.linalg.eigh(np.cov(local.T))
    pca_axis = axes[0, -1]*ex+axes[1, -1]*ey
    top = height >= np.percentile(height, 75)
    top_axis = None
    if top.sum() >= 20:
        top_eig, top_axes = np.linalg.eigh(np.cov(local[top].T))
        if top_eig[-1] > 1.3*max(top_eig[-2], 1e-12):
            top_axis = (top_axes[0, -1]*ex+top_axes[1, -1]*ey).tolist()
    # This assumes a simple prism and half of the observed P90 height. A tab
    # can violate it, so the candidate is never marked as a measured centre.
    observed_height = float(np.percentile(height, 90))
    model_centre = o+rect[0][0]*ex+rect[0][1]*ey-n*(observed_height/2)
    def axis_gap(a, b):
        return float(np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), 0, 1))))
    return {'visible_surface_center_m': np.median(points, axis=0).tolist(),
            'model_estimated_body_center_m': model_centre.tolist(),
            'model_assumption': 'OBB footprint and half observed P90 height; hidden body and tab not measured',
            'body_pose_valid': False, 'ring_pose_valid': False,
            'visible_height_m_p10_p50_p90': np.percentile(height, [10, 50, 90]).tolist(),
            'near_camera_band_point_count': int(top.sum()),
            'remainder_point_count': int((~top).sum()),
            'near_camera_band_axis_camera_xyz': top_axis,
            'pca_axis_camera_xyz': pca_axis.tolist(),
            'pca_eigenvalue_ratio': float(eig[-1]/max(eig[-2], 1e-12)),
            'obb_axis_camera_xyz': obb_axis.tolist(),
            'pca_obb_axis_difference_deg': axis_gap(pca_axis, obb_axis),
            'near_band_pca_axis_difference_deg': axis_gap(np.asarray(top_axis), pca_axis) if top_axis else None,
            'obb_footprint_m': [float(max(rect[1])), float(min(rect[1]))],
            'image_min_area_rect_yaw_deg': image_yaw,
            'direction_ambiguity_deg': 180, 'yaw_accuracy': 'NOT_MEASURED',
            'position_accuracy': 'NOT_MEASURED', 'top_band_is_hypothesis': True}



def rgb_supported_components(rgb_mask, foreground, height_m, depth_m, intr, plane, cfg, sdk_deproject=None):
    """Measured components inside one RGB contour, not unrestricted depth growth.

    Invalid depth and table-height pixels are never filled. Every retained pixel
    must have independent RGB and depth support; CAD footprint gates still apply.
    """
    support=np.asarray(rgb_mask,bool)&foreground
    x,y,w,h=cv2.boundingRect(np.uint8(support))
    out=np.zeros_like(support)
    info=dict(accepted=False,method='RGB_CONTOUR_TABLE_DEPTH_COMPONENT_UNION',
              raw_rgb_pixels=int(np.count_nonzero(rgb_mask)),supported_pixels=int(support.sum()))
    if not w or not h:return out,dict(info,reason='EMPTY_RGB_DEPTH_SUPPORT')
    count,labels,stats,_=cv2.connectedComponentsWithStats(np.uint8(support[y:y+h,x:x+w]),8)
    retained=[]
    for label in range(1,count):
        area=int(stats[label,cv2.CC_STAT_AREA])
        if area<cfg.component_min_area_px:continue
        part=labels==label
        values=height_m[y:y+h,x:x+w][part]
        # Bounds are inherited from the existing table/foreground gate.
        if not len(values) or not np.isfinite(values).all():continue
        out[y:y+h,x:x+w]|=part
        retained.append(dict(area_px=area,height_p10_p50_p90_m=np.percentile(values,[10,50,90]).tolist()))
    points,_=_sample_cloud(depth_m,out,intr,1500,sdk_deproject)
    geom=body_geometry(points,plane)
    span=np.asarray(geom.get('obb_footprint_m',[]))
    nominal=np.sort(np.asarray(cfg.joint_footprint_m))[::-1]
    if len(span)!=2 or (span<.020).any() or (span>nominal*cfg.joint_max_footprint_scale).any():
        return np.zeros_like(out),dict(info,reason='RGB_UNION_OUTSIDE_CAD_FOOTPRINT',footprint_m=span.tolist())
    return out,dict(info,accepted=True,reason=None,retained_components=retained,
                    retained_pixels=int(out.sum()),footprint_m=span.tolist(),
                    invalid_depth_filled=False,hidden_surfaces_filled=False)

def analyze_frame(frame, detections, cfg=OfflineConfig(), *, focus_color="red",
                  table_plane_override=None, sdk_deproject=None, rays_xyz=None, prepared=None):
    stage_start=perf_counter()
    validate_frame(frame.color_bgr, frame.aligned_depth, frame.intrinsics, frame.depth_scale)
    depth_m = prepared["depth_m"] if prepared is not None else frame.aligned_depth.astype(np.float64)*frame.depth_scale
    valid = prepared["valid"] if prepared is not None else (frame.aligned_depth > 0) & (depth_m >= cfg.min_depth_m) & (depth_m <= cfg.max_depth_m)
    reds = [d for d in detections if d.color == focus_color and d.detection_valid]
    red = max(reds, key=lambda d: d.mask.sum()) if reds else None
    exclusion = red.mask if red is not None else np.zeros(valid.shape, bool)
    if table_plane_override is None:
        plane, table_roi, failure = fit_table(depth_m, exclusion, frame.intrinsics, cfg, sdk_deproject)
    else:
        plane, table_roi, failure = table_plane_override, np.zeros(valid.shape, bool), None
    report = {'coordinate_frame': 'CAMERA_OPTICAL_M', 'camera_intrinsics': asdict(frame.intrinsics),
              'timing_ms': {'depth_filter': 0.0, 'point_extraction': 0.0, 'table_plane': (perf_counter()-stage_start)*1000},
              'depth_scale_m_per_unit': frame.depth_scale, 'table_plane': plane,
              'table_failure': failure, 'candidate_counts_before': {},
              'candidate_counts_after': {}, 'candidates': [],
              'body_pose_valid': False, 'ring_pose_valid': False,
              'table_frame_valid': False, 'yaw_accuracy': 'NOT_MEASURED',
              'position_accuracy': 'NOT_MEASURED'}
    for d in detections:
        report['candidate_counts_before'][d.color] = report['candidate_counts_before'].get(d.color, 0)+1
    empty = np.zeros(valid.shape, bool)
    if plane is None:
        report['timing_ms']['depth_filter']=(perf_counter()-stage_start)*1000
        report['body_geometry'] = body_geometry(np.empty((0, 3)), None)
        return report, {'table_roi': table_roi, 'foreground': empty, 'object': empty,
                        'top_band': empty, 'height_m': np.zeros(valid.shape)}, np.empty((0, 3)), np.empty((0, 3), np.uint8)
    foreground_start=perf_counter()
    height = prepared["height_m"] if prepared is not None else height_from_plane(depth_m, frame.intrinsics, plane, rays_xyz)
    foreground = prepared["foreground"] if prepared is not None else valid & (height >= cfg.min_foreground_height_m) & (height <= cfg.max_foreground_height_m)
    candidates = []
    for d in detections:
        mask = d.mask.astype(bool)
        valid_ratio = float((mask & valid).sum()/max(mask.sum(), 1))
        fg_pixels = mask & foreground
        fg_ratio = float(fg_pixels.sum()/max((mask & valid).sum(), 1))
        component, area = largest_component(fg_pixels, cfg.component_min_area_px)
        reason = None
        if valid_ratio < cfg.candidate_min_valid_ratio:
            reason = 'LOW_VALID_DEPTH'
        elif fg_ratio < cfg.candidate_min_foreground_ratio:
            reason = 'ON_TABLE_OR_OUTSIDE_HEIGHT_BAND'
        elif not component.any():
            reason = 'NO_LARGE_CONNECTED_FOREGROUND'
        candidates.append({'color': d.color, 'raw_area_px': int(mask.sum()),
                           'valid_ratio': valid_ratio, 'foreground_ratio': fg_ratio,
                           'component_area_px': area, 'reason': reason,
                           'mask': component, 'source': d})
    good_red = [c for c in candidates if c['color'] == focus_color and c['reason'] is None]
    winner = max(good_red, key=lambda c: c['component_area_px']) if good_red else None
    rgb_seed = winner['mask'] if winner else empty
    object_mask, growth = grow_depth_object(rgb_seed, foreground, cfg, height)
    report['depth_growth'] = growth | {'rgb_seed_pixel_count': int(rgb_seed.sum()),
                                       'grown_pixel_count': int(object_mask.sum())}
    report['segmentation_method']='LEGACY_DEPTH_REGION_GROWING'
    if cfg.joint_rgb_depth and winner is not None:
        joint, joint_info=rgb_supported_components(winner['source'].mask,foreground,height,
            depth_m,frame.intrinsics,plane,cfg,sdk_deproject)
        report['rgb_supported_segmentation']=joint_info
        if joint_info['accepted']:
            object_mask=joint
            report['segmentation_method']=joint_info['method']
    report['legacy_object_pixel_count']=int(growth.get('grown_pixel_count',report['depth_growth']['grown_pixel_count']))
    for c in candidates:
        if (cfg.single_red_object and c['reason'] is None and c['color'] == focus_color
                and c is not winner):
            c['reason'] = 'SECOND_RED_CANDIDATE_IN_SINGLE_OBJECT_SESSION'
        if c['reason'] is None and c is not winner and winner is not None:
            overlap = (c['mask'] & object_mask).sum()/max(c['mask'].sum(), 1)
            if overlap > .25:
                c['reason'] = 'OVERLAPS_SELECTED_RED'
        accepted = c['reason'] is None
        if accepted:
            report['candidate_counts_after'][c['color']] = report['candidate_counts_after'].get(c['color'], 0)+1
        report['candidates'].append({k: v for k, v in c.items() if k not in ('mask', 'source')} |
                                    {'accepted': accepted})
    report['timing_ms']['depth_filter']=(perf_counter()-stage_start)*1000
    report["timing_ms"]["foreground_segmentation"]=(perf_counter()-foreground_start)*1000
    extraction_start=perf_counter()
    points, uv = _sample_cloud(depth_m, object_mask, frame.intrinsics, cfg.max_cloud_points, sdk_deproject)
    rgb = frame.color_bgr[uv[:, 1], uv[:, 0], ::-1].copy() if len(uv) else np.empty((0, 3), np.uint8)
    report['timing_ms']['point_extraction']=(perf_counter()-extraction_start)*1000
    report['body_geometry'] = body_geometry(points, plane, winner['source'].yaw_deg if winner else None)
    report['object_point_count'] = int(len(points))
    report['object_pixel_count'] = int(object_mask.sum())
    report['object_depth_valid_ratio'] = float((object_mask & valid).sum()/max(object_mask.sum(), 1)) if object_mask.any() else 0.0
    top_band = np.zeros(valid.shape, bool)
    if len(points):
        sampled_height = (np.asarray(plane['point_camera_m'])-points) @ np.asarray(plane['normal_camera_xyz'])
        top_band = object_mask & (height >= np.percentile(sampled_height, 75))
    return report, {'table_roi': table_roi, 'foreground': foreground,
                    'object': object_mask, 'top_band': top_band, 'rgb_seed':rgb_seed,
                    'rgb_support':winner['source'].mask if winner else empty,
                    'height_m': height, 'depth_m':depth_m, 'valid':valid}, points, rgb
