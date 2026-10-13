"""Observed top crossbar + measured CAD offsets; never authorizes motion.

Plane proposals use the existing RANSAC. Acceptance additionally needs the
current RGB boundary, local holdout depth, CAD dimensions and one-sided body.
No image-top assumption, fixed ROI, camera pose or station transform is used.
"""
import cv2
import numpy as np
from .rgbd_geometry import deproject_pixels,project_points,fit_plane_ransac


def top_bar_spec(geometry):
    """Select the rectangular highest cap from measured CAD planar faces (mm)."""
    battery=(geometry or {}).get('battery',{})
    aperture=battery.get('aperture',{})
    faces=[f for f in battery.get('planar_faces',[]) if abs(f['normal'][2])>.999]
    if not faces or 'ring_center_cad_mm' not in aperture:return None
    low=min(f['bbox_mm'][2] for f in faces)
    caps=[f for f in faces if abs(f['bbox_mm'][2]-low)<1e-4 and abs(f['bbox_mm'][5]-low)<1e-4]
    if len(caps)!=1:return None
    face=caps[0];box=np.asarray(face['bbox_mm'],float);span=box[3:5]-box[:2]
    # Area equal to its bounding rectangle certifies a full rectangular cap.
    if min(span)<=0 or abs(face['area_mm2']-np.prod(span))>1e-3:return None
    center=(box[:3]+box[3:])/2000
    ring=np.asarray(aperture['ring_center_cad_mm'],float)/1000
    dims=battery.get('body_dimensions_mm',{})
    T=np.asarray(battery.get('T_battery_from_cad_m',[]),float)
    if T.shape!=(4,4) or not dims:return None
    body=-T[:3,:3].T@T[:3,3]
    return dict(face_index=face['face_index'],top_center_cad_m=center.tolist(),
        top_normal_cad=[0.,0.,-1.],top_span_cad_xy_m=(span/1000).tolist(),
        ring_center_cad_m=ring.tolist(),offset_ring_cad_m=(ring-center).tolist(),
        offset_body_cad_m=(body-center).tolist(),insertion_axis_cad=aperture['normal_cad'],
        overall_height_m=battery['overall_height_mm']/1000,source_sha256=battery['source']['sha256'])


def transform_from_top(center,normal,long_axis,spec):
    """Two proper rotations; offset is rotated, never added to a camera axis."""
    n=np.asarray(normal,float);n=n/np.linalg.norm(n)
    x=np.asarray(long_axis,float);x=x-n*(x@n);x=x/np.linalg.norm(x)
    z=-n;y=np.cross(z,x);R=np.column_stack((x,y,z))
    rotations=[R,R@np.diag([-1.,-1.,1.])]
    return [dict(rotation_camera_from_cad=r.tolist(),
        ring_center_camera_m=(np.asarray(center)+r@spec['offset_ring_cad_m']).tolist(),
        body_center_camera_m=(np.asarray(center)+r@spec['offset_body_cad_m']).tolist(),
        insertion_direction_camera=(r@spec['insertion_axis_cad']).tolist()) for r in rotations]


class TopBarPoseEstimation:
    def __init__(self,config,geometry):
        self.config=config;self.spec=top_bar_spec(geometry)
    def estimate(self,frame,mask,qualified):
        out=dict(method='OBSERVED_TOP_RECTANGLE_PLUS_MEASURED_CAD',source='CURRENT_RGBD',
            coordinate_frame='camera_color_optical',unit_position='m',top_geometry_valid=False,
            ring_center_geometry_valid=False,ring_geometry_valid=False,body_pose_valid=False,
            ring_pose_valid=False,motion_usable=False,top_center_candidate=None,
            top_normal_candidate=None,top_long_axis_candidates=[],ring_center_candidate=None,
            body_center_candidate=None,insertion_direction_candidates=[],rotation_candidates=[],
            direction_unique=False,top_polygon_px=None,quality={},candidates=[],reasons=[])
        if not qualified:out['reasons']=['RGBD_PAIR_REJECTED'];return out
        spec=self.spec
        if spec is None:out['reasons']=['MEASURED_TOP_CAD_UNAVAILABLE'];return out
        out['cad_reference']=spec
        # This route requires both origin offsets to lie on the cap normal. No
        # off-axis offset is collapsed into a unique centre under yaw ambiguity.
        if max(np.linalg.norm(np.asarray(spec[k])[:2]) for k in ['offset_ring_cad_m','offset_body_cad_m'])>1e-6:
            out['reasons']=['CAD_OFFSET_DEPENDS_ON_UNRESOLVED_YAW'];return out
        mask=np.asarray(mask,bool);z=frame.aligned_depth*frame.depth_scale
        if mask.shape!=z.shape:raise ValueError('Top mask/depth size mismatch')
        if not mask.any():out['reasons']=['NO_CURRENT_MASK'];return out
        edge=int(self.config.get('body_mask_erode_px',2))
        inside=cv2.erode(mask.astype(np.uint8),np.ones((edge*2+1,edge*2+1),np.uint8)).astype(bool)
        usable=inside&(z>=self.config['min_depth_m'])&(z<=self.config['max_depth_m'])
        if usable.sum()<self.config['min_points']:out['reasons']=['INSUFFICIENT_CURRENT_DEPTH'];return out
        v,u=np.where(usable);depth=z[v,u];med=np.median(depth);mad=np.median(abs(depth-med))
        keep=abs(depth-med)<=max(self.config['depth_outlier_floor_m'],3*1.4826*mad)
        v,u=v[keep],u[keep];points=deproject_pixels(np.column_stack((u,v)),z[v,u],frame.intrinsics)
        train=(u+v)%2==0;indices=np.flatnonzero(train)
        if len(indices)<100:out['reasons']=['INSUFFICIENT_TRAIN_DEPTH'];return out
        indices=indices[np.linspace(0,len(indices)-1,min(len(indices),1500),dtype=int)]
        seeds=points[indices];remaining=np.ones(len(seeds),bool)
        bx,by,bw,bh=cv2.boundingRect(mask.astype(np.uint8))
        if bx<=0 or by<=0 or bx+bw>=mask.shape[1] or by+bh>=mask.shape[0]:
            out['reasons']=['CURRENT_TARGET_IMAGE_TRUNCATED'];return out
        # Preserve outside RGB pixels: tight cropping destroys a horizontal
        # silhouette edge at the crop border (Canny/morphology border handling).
        padding=edge+3
        x0=max(0,bx-padding);y0=max(0,by-padding)
        right=min(mask.shape[1],bx+bw+padding);bottom=min(mask.shape[0],by+bh+padding)
        w=right-x0;h=bottom-y0
        gray=cv2.cvtColor(frame.color_bgr[y0:y0+h,x0:x0+w],cv2.COLOR_BGR2GRAY)
        observed_edges=cv2.Canny(gray,20,60)
        silhouette=mask[y0:y0+h,x0:x0+w].astype(np.uint8)
        observed_edges|=cv2.morphologyEx(silhouette*255,cv2.MORPH_GRADIENT,np.ones((3,3),np.uint8))
        # Account for the deliberate mask erosion plus raster/Canny rounding.
        # The threshold is unchanged; raw RGB edges remain independent evidence.
        boundary_radius=edge+2
        observed_edges=cv2.dilate(observed_edges,np.ones((2*boundary_radius+1,2*boundary_radius+1),np.uint8))>0
        nominal=np.sort(spec['top_span_cad_xy_m'])[::-1]
        proposals=[]
        # Local seed windows slide over the current mask at a scale derived from
        # the CAD thickness and observed range, never at a fixed image location.
        half=max(5,min(24,int(frame.intrinsics.fx*nominal[1]*.3/med)))
        for yy in range(y0+half,y0+h-half,max(half,8)):
            for xx in range(x0+half,x0+w-half,max(half,8)):
                window=train&(abs(u-xx)<=half)&(abs(v-yy)<=half)
                local=points[window]
                if len(local)<100:continue
                o=local.mean(0);_,_,a=np.linalg.svd(local-o,full_matrices=False);n=a[-1]
                error=float(np.percentile(abs((local-o)@n),95))
                if error>self.config['max_plane_residual_m']/2:continue
                if n@o>0:n=-n
                if any(abs(n@nn)>.995 and abs((o-oo)@nn)<.002 for oo,nn in proposals):continue
                proposals.append((o,n))
        # Existing global RANSAC supplies fallback proposals for fragmented data.
        for _ in range(7):
            if remaining.sum()<60:break
            plane=fit_plane_ransac(seeds[remaining],threshold_m=self.config['max_plane_residual_m'],min_inliers=60)
            if plane is None:break
            o=np.asarray(plane['point_camera_m']);n=np.asarray(plane['normal_camera_xyz'])
            selected=remaining&(abs((seeds-o)@n)<=self.config['max_plane_residual_m'])
            remaining[selected]=False
            proposals.append((o,n))
        accepted=[]
        for number,(origin,n) in enumerate(proposals):
            selected=abs((seeds-origin)@n)<=self.config['max_plane_residual_m']
            part=seeds[selected]
            if len(part)<60:continue
            _,_,axes=np.linalg.svd(part-part.mean(0),full_matrices=False)
            axis=axes[0]-n*(axes[0]@n);axis/=np.linalg.norm(axis)
            basis=np.stack((axis,np.cross(n,axis)))
            anchor=part.mean(0);anchor-=((anchor-origin)@n)*n
            xy=np.float32((part-anchor)@basis.T);rect=cv2.minAreaRect(xy)
            span=np.sort(rect[1])[::-1];center=anchor+np.asarray(rect[0])@basis
            if n@center>0:n=-n
            record=dict(proposal=number,seed_points=len(part),span_m=span.tolist(),
                visible_sample_center_camera_m=np.median(part,axis=0).tolist(),reasons=[])
            out['candidates'].append(record)
            if not np.all(abs(span-nominal)<=np.maximum(.003,nominal*.10)):
                record['reasons'].append('NOT_MEASURED_TOP_CAP_DIMENSIONS');continue
            # Corners come from observed extent, not a nominal centred box.
            corners2=cv2.boxPoints(rect);corners=anchor+corners2@basis
            poly=project_points(corners,frame.intrinsics)
            if not np.isfinite(poly).all():record['reasons'].append('INVALID_TOP_PROJECTION');continue
            a=np.roll(corners,-1,axis=0)-corners;long=a[int(np.argmax(np.linalg.norm(a,axis=1)))];long/=np.linalg.norm(long)
            region=np.zeros((h,w),np.uint8);cv2.fillConvexPoly(region,np.rint(poly-[x0,y0]).astype(np.int32),1)
            # Pixel region is fixed by proposal geometry before holdout residual
            # tests. All eligible holdout pixels count, not just plane inliers.
            rv,ru=np.where((region>0)&silhouette.astype(bool));rv+=y0;ru+=x0
            hold=(ru+rv)%2==1;rv,ru=rv[hold],ru[hold]
            all_count=len(rv);valid=(z[rv,ru]>=self.config['min_depth_m'])&(z[rv,ru]<=self.config['max_depth_m'])
            valid_ratio=float(valid.sum()/max(all_count,1));rv,ru=rv[valid],ru[valid]
            test=deproject_pixels(np.column_stack((ru,rv)),z[rv,ru],frame.intrinsics) if len(rv) else np.empty((0,3))
            residual=abs((test-origin)@n);support=float((residual<=self.config['max_plane_residual_m']).mean()) if len(test) else 0.
            p95=float(np.percentile(residual,95)) if len(test) else None
            coverage=[]
            for i in range(4):
                line=poly[i]+np.linspace(0,1,60)[:,None]*(poly[(i+1)%4]-poly[i])
                local=np.rint(line-[x0,y0]).astype(int);safe=(local[:,0]>=0)&(local[:,0]<w)&(local[:,1]>=0)&(local[:,1]<h)
                coverage.append(float(observed_edges[local[safe,1],local[safe,0]].sum()/len(line)))
            below=(points-center)@n;below_ratio=float((below<=self.config['max_plane_residual_m']).mean())
            height=float(-np.percentile(below,5))
            record.update(holdout_region_pixels=all_count,holdout_points=len(test),holdout_depth_ratio=valid_ratio,
                local_support_ratio=support,local_residual_p95_m=p95,rgb_boundary_coverage=coverage,
                object_below_ratio=below_ratio,observed_height_m=height,center_camera_m=center.tolist(),
                normal_camera_xyz=n.tolist(),polygon_px=poly.tolist(),long_axis_camera=long.tolist())
            if len(test)<self.config['min_points'] or valid_ratio<self.config['min_depth_ratio']:
                record['reasons'].append('INSUFFICIENT_LOCAL_HOLDOUT_DEPTH')
            if support<self.config.get('body_min_face_support',.70) or p95 is None or p95>self.config['max_plane_residual_m']:
                record['reasons'].append('UNSUPPORTED_LOCAL_TOP_PLANE')
            if min(coverage)<.5:record['reasons'].append('TOP_BOUNDARY_INCOMPLETE')
            if below_ratio<.95 or not .5*spec['overall_height_m']<=height<=1.25*spec['overall_height_m']:
                record['reasons'].append('NOT_EXTREMAL_TOP_OF_CURRENT_OBJECT')
            if not record['reasons']:accepted.append(record)
        unique=[]
        for candidate in sorted(accepted,key=lambda c:c['local_residual_p95_m']):
            if not any(np.linalg.norm(np.asarray(candidate['center_camera_m'])-other['center_camera_m'])<.003 and np.dot(candidate['normal_camera_xyz'],other['normal_camera_xyz'])>.995 for other in unique):
                unique.append(candidate)
        accepted=unique
        if len(accepted)!=1:
            out['reasons']=['NO_UNIQUE_OBSERVED_TOP_CAP']
            return out
        selected=accepted[0];transforms=transform_from_top(selected['center_camera_m'],selected['normal_camera_xyz'],selected['long_axis_camera'],spec)
        ring=np.asarray([t['ring_center_camera_m'] for t in transforms]);body=np.asarray([t['body_center_camera_m'] for t in transforms])
        out.update(top_geometry_valid=True,ring_center_geometry_valid=True,
            top_center_candidate=selected['center_camera_m'],top_normal_candidate=selected['normal_camera_xyz'],
            top_long_axis_candidates=[selected['long_axis_camera'],(-np.asarray(selected['long_axis_camera'])).tolist()],
            top_polygon_px=selected['polygon_px'],quality=selected,
            ring_center_candidate=ring.mean(0).tolist(),body_center_candidate=body.mean(0).tolist(),
            insertion_direction_candidates=[t['insertion_direction_camera'] for t in transforms],
            rotation_candidates=[t['rotation_camera_from_cad'] for t in transforms],
            reasons=['INSERTION_SIDE_180_DEG_AMBIGUITY','NO_STATION_TO_CAMERA_TRANSFORM','NO_INDEPENDENT_POSITION_ACCURACY_VALIDATION'])
        return out


def observed_layout(objects):
    """Observed-only 2x2 diagnostic; never makes a missing target or its pose."""
    observed=[dict(object_id=o['object_id'],color=o['color'],xyz=o['top_geometry']['top_center_candidate']) for o in objects if (o.get('top_geometry') or {}).get('top_geometry_valid')]
    out=dict(observed=observed,count=len(observed),expected_count=4,color_slot_assumption=False,
        status='PARTIAL_OBSERVATIONS',motion_usable=False)
    if len(observed)!=4:return out
    points=np.asarray([o['xyz'] for o in observed]);_,_,axes=np.linalg.svd(points-points.mean(0),full_matrices=False)
    xy=np.float32((points-points.mean(0))@axes[:2].T);hull=cv2.convexHull(xy,returnPoints=False).reshape(-1)
    if len(hull)!=4 or cv2.contourArea(xy[hull])<1e-6:
        out['status']='NOT_FOUR_DISTINCT_CORNERS';return out
    edges=np.roll(xy[hull],-1,axis=0)-xy[hull];edges/=np.linalg.norm(edges,axis=1)[:,None]
    parallel=[float(abs(edges[i]@edges[i+2])) for i in range(2)]
    out.update(status='OBSERVED_2X2_TOPOLOGY' if min(parallel)>.95 else 'LAYOUT_NOT_RECTANGULAR',
        corner_object_ids=[observed[i]['object_id'] for i in hull],opposite_edge_parallel_cosine=parallel,
        station_spacing_verified=False)
    return out
