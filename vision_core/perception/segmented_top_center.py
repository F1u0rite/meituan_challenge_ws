"""Segmented cap -> observed 3-D rectangle centre -> axial CAD hole centre.

Input segmentation/visibility declarations are external evidence. This module
does not identify a top automatically, choose an insertion side, or authorize
motion. Partial completion requires a declared visible long-axis end.
"""
import cv2
import numpy as np
from .rgbd_geometry import validate_frame,deproject_pixels,project_points,fit_plane_ransac
from .top_bar_pose import top_bar_spec,transform_from_top


class SegmentedTopCenter:
    def __init__(self,geometry,*,min_depth_m=.15,max_depth_m=1.,plane_fit_threshold_m=.003):
        self.spec=top_bar_spec(geometry)
        self.min_depth_m=min_depth_m
        self.max_depth_m=max_depth_m
        self.plane_fit_threshold_m=plane_fit_threshold_m

    def estimate(self,frame,mask,*,qualified,complete_boundary=True,visible_long_edge=None,support_plane=None,plane_seed_mask=None):
        out=dict(method='SEGMENTED_CAP_RECTANGLE_PLUS_CAD',coordinate_frame='camera_color_optical',
            unit_position='m',candidate_available=False,geometry_consistent=False,
            top_center_candidate=None,ring_center_candidate=None,top_normal_candidate=None,
            top_polygon_px=None,insertion_direction_candidates=[],direction_unique=False,
            body_pose_valid=False,ring_pose_valid=False,motion_usable=False,
            absolute_accuracy='NOT_MEASURED',cad_completed=False,quality={},reasons=[],
            source_timestamps=dict(frame.timestamps))
        if not qualified:
            out['reasons']=['RGBD_PAIR_REJECTED'];return out
        if self.spec is None:
            out['reasons']=['MEASURED_TOP_CAD_UNAVAILABLE'];return out
        if np.linalg.norm(np.asarray(self.spec['offset_ring_cad_m'])[:2])>1e-6:
            out['reasons']=['CAD_OFFSET_DEPENDS_ON_UNRESOLVED_YAW'];return out
        validate_frame(frame.color_bgr,frame.aligned_depth,frame.intrinsics,frame.depth_scale)
        mask=np.asarray(mask,bool)
        if mask.shape!=frame.aligned_depth.shape:raise ValueError('Segmented top mask shape mismatch')
        if not mask.any():out['reasons']=['EMPTY_TOP_MASK'];return out
        if mask[0].any() or mask[-1].any() or mask[:,0].any() or mask[:,-1].any():
            out['reasons']=['TOP_MASK_IMAGE_TRUNCATED'];return out
        if not complete_boundary and visible_long_edge not in ('positive','negative'):
            out['reasons']=['PARTIAL_CAP_WITHOUT_VISIBLE_END_CONSTRAINT'];return out
        z=frame.aligned_depth*frame.depth_scale
        yy,xx=np.indices(mask.shape)
        inside=cv2.erode(np.uint8(mask),np.ones((5,5),np.uint8)).astype(bool)
        valid=(z>=self.min_depth_m)&(z<=self.max_depth_m)
        tv,tu=np.where(inside&valid&((xx+yy)%2==0))
        if len(tv)<100:out['reasons']=['INSUFFICIENT_TRAIN_DEPTH'];return out
        train=deproject_pixels(np.column_stack((tu,tv)),z[tv,tu],frame.intrinsics)
        indices=np.linspace(0,len(train)-1,min(len(train),2000),dtype=int)
        reference_source='LOCAL_TRAIN_DEPTH'
        seed_count=None
        if support_plane is None:
            fitted=fit_plane_ransac(train[indices],threshold_m=self.plane_fit_threshold_m,min_inliers=80)
            if fitted is None:out['reasons']=['NO_SUPPORTED_TRAIN_PLANE'];return out
            origin=np.array(fitted['point_camera_m']);normal=np.array(fitted['normal_camera_xyz'])
            if normal@origin>0:normal=-normal
        else:
            if (not isinstance(support_plane,dict) or
                support_plane.get('color_frame_number')!=frame.timestamps.get('color_frame_number') or
                support_plane.get('color_sdk_ms')!=frame.timestamps.get('color_sdk_ms') or
                frame.timestamps.get('color_frame_number') is None):
                out['reasons']=['SUPPORT_REFERENCE_FRAME_MISMATCH'];return out
            normal=np.asarray(support_plane.get('normal_camera_xyz',[]),float)
            point=np.asarray(support_plane.get('point_camera_m',[]),float)
            if normal.shape!=(3,) or point.shape!=(3,) or not np.isfinite(np.r_[normal,point]).all() or np.linalg.norm(normal)<1e-9:
                out['reasons']=['INVALID_SUPPORT_REFERENCE'];return out
            normal=normal/np.linalg.norm(normal)
            if normal@point>0:normal=-normal
            fit_train=train
            if plane_seed_mask is not None:
                seed=np.asarray(plane_seed_mask,bool)
                if seed.shape!=mask.shape:raise ValueError('Plane seed shape mismatch')
                fit_train=train[seed[tv,tu]]
            if len(fit_train)<80:
                out['reasons']=['INSUFFICIENT_CURRENT_CAP_SEED_DEPTH'];return out
            seed_count=len(fit_train)
            origin=fit_train.mean(0)
            origin+=normal*(np.median(fit_train@normal)-origin@normal)
            reference_source='CURRENT_SUPPORT_PLANE_UPRIGHT_BATTERY_CONSTRAINT'
        # All range-valid odd pixels in the original mask are evaluated, without
        # residual filtering or trimming to the fitted inlier population.
        hv,hu=np.where(mask&((xx+yy)%2==1));population=len(hv)
        keep=valid[hv,hu];hv,hu=hv[keep],hu[keep]
        hold=deproject_pixels(np.column_stack((hu,hv)),z[hv,hu],frame.intrinsics) if len(hv) else np.empty((0,3))
        residual=abs((hold-origin)@normal)
        q=dict(train_points=len(train),train_fit_points=len(indices),
            holdout_population=population,holdout_points=len(hold),
            holdout_depth_ratio=len(hold)/max(population,1),
            holdout_p95_m=float(np.percentile(residual,95)) if len(hold) else None,
            holdout_support_ratio=float((residual<=self.plane_fit_threshold_m).mean()) if len(hold) else 0.,
            fit_threshold_m=self.plane_fit_threshold_m,normal_source=reference_source,plane_seed_points=seed_count,
            holdout_scope='ALL_RANGE_VALID_ODD_PIXELS_IN_ORIGINAL_SEGMENTATION')
        contours,_=cv2.findContours(np.uint8(mask),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours=sorted(contours,key=cv2.contourArea,reverse=True)
        if len(contours)>1 and cv2.contourArea(contours[1])>.1*cv2.contourArea(contours[0]):
            out.update(quality=q,reasons=['MULTIPLE_SEGMENTED_TOP_COMPONENTS']);return out
        rays=deproject_pixels(contours[0].reshape(-1,2),np.ones(len(contours[0])),frame.intrinsics)
        den=rays@normal
        if np.any(abs(den)<1e-6):out.update(quality=q,reasons=['DEGENERATE_PLANE_PROJECTION']);return out
        cloud=rays*((origin@normal)/den)[:,None]
        if np.any(cloud[:,2]<=0):out.update(quality=q,reasons=['TOP_BEHIND_CAMERA']);return out
        ex=np.eye(3)[np.argmin(abs(normal))];ex=ex-normal*(ex@normal);ex/=np.linalg.norm(ex)
        ey=np.cross(normal,ex);basis=np.stack((ex,ey))
        rect=cv2.minAreaRect(np.float32((cloud-origin)@basis.T))
        corners=origin+cv2.boxPoints(rect)@basis
        sides=np.roll(corners,-1,axis=0)-corners
        lengths=np.linalg.norm(sides,axis=1);long=sides[np.argmax(lengths)];long/=np.linalg.norm(long)
        # Canonical sign is a camera-coordinate convention, never insertion sign.
        if long[np.argmax(abs(long))]<0:long=-long
        short=np.cross(normal,long)
        a=(cloud-origin)@long;b=(cloud-origin)@short
        span=np.array([a.max()-a.min(),b.max()-b.min()])
        nominal=np.sort(self.spec['top_span_cad_xy_m'])[::-1]
        center=origin+((a.min()+a.max())/2)*long+((b.min()+b.max())/2)*short
        dims_ok=bool(np.all(abs(span-nominal)<=np.maximum(.003,nominal*.10)))
        observed_corners=corners.copy()
        if not complete_boundary:
            if span[0]<.5*nominal[0] or span[0]>nominal[0]+.0066 or abs(span[1]-nominal[1])>.003:
                out.update(quality=q,reasons=['PARTIAL_CAP_CONSTRAINTS_INSUFFICIENT']);return out
            long_center=a.max()-nominal[0]/2 if visible_long_edge=='positive' else a.min()+nominal[0]/2
            center=origin+long_center*long+((b.min()+b.max())/2)*short
            out['cad_completed']=True
            corners=np.array([center+sa*nominal[0]/2*long+sb*span[1]/2*short for sa,sb in [(-1,-1),(1,-1),(1,1),(-1,1)]])
            dims_ok=True
        transforms=transform_from_top(center,normal,long,self.spec)
        ring=np.array([t['ring_center_camera_m'] for t in transforms]).mean(0)
        q.update(observed_span_m=span.tolist(),nominal_span_m=nominal.tolist(),
            cad_dimensions_compatible=dims_ok,boundary_declared_complete=complete_boundary,
            visible_long_edge=visible_long_edge,plane_normal_camera=normal.tolist())
        consistent=dims_ok and len(hold)>=100 and q['holdout_depth_ratio']>=.5 and q['holdout_support_ratio']>=.7
        reasons=['TOP_IDENTITY_AND_ABSOLUTE_POSITION_UNVERIFIED','INSERTION_SIDE_UNRESOLVED']
        if not dims_ok:reasons.append('SEGMENTED_BOUNDARY_NOT_CAD_CAP_DIMENSIONS')
        if not consistent:reasons.append('GEOMETRY_CHECKS_NOT_SATISFIED')
        if out['cad_completed']:reasons.append('CAD_COMPLETED_NOT_FULLY_OBSERVED')
        out.update(candidate_available=True,geometry_consistent=consistent,
            top_center_candidate=center.tolist(),ring_center_candidate=ring.tolist(),
            top_normal_candidate=normal.tolist(),top_polygon_px=project_points(corners,frame.intrinsics).tolist(),
            insertion_direction_candidates=[t['insertion_direction_camera'] for t in transforms],
            observed_top_polygon_px=project_points(observed_corners,frame.intrinsics).tolist(),
            quality=q,reasons=reasons)
        return out
