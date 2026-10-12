"""V4.5: RGB identity, same-frame body geometry and observed local apertures.

No inferred pose is executable: metrology and directional disambiguation remain
unverified. No table cache, background history or complete CAD fit admits identity.
"""
from pathlib import Path
from time import perf_counter, time
import json
import cv2
import numpy as np
import yaml
from mtc_vision_adapter.core import Tracker
from input.rgbd_pair_guard import RGBDPairGuard
from .dual_rate_rgbd import RateMeter, capture_time
from .rgbd_geometry import deproject_pixels, validate_frame, frame_capture_time
from .unified_detector import UnifiedBatteryDetector
from .battery_only import PixelBatteryDetector
from .color_classifier import ColorClassifier
from .vision_interface import finalize_interface

ROOT=Path(__file__).resolve().parents[1]

class BatteryDetection:
    def __init__(self,config):
        self.config=config;pixels=PixelBatteryDetector()
        # Separate V4.5 chromatic support: the old permissive S ranges merged wood
        # into target contours in actual H4 frames. Do not alter V4.4 shared config.
        pixels.config['hsv_ranges']={color:[[h0,max(s0,config['min_chromatic_saturation']),v0,h1,s1,v1] for h0,s0,v0,h1,s1,v1 in ranges] for color,ranges in pixels.config['hsv_ranges'].items()}
        pixels.config['min_contour_area_px']=config['min_rgb_area_px']
        self.detector=UnifiedBatteryDetector(candidate_detector=pixels);self.classifier=ColorClassifier()
        self.tracker=Tracker(max_gap_s=1.,max_distance=80.,confirmation_hits=2,ambiguity_px=8.,allow_unknown=True)
    def detect(self,color,stamp):
        hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV);proposals=[];tracking=[]
        for d in self.detector.detect_batteries(color,hsv=hsv):
            x,y,w,h=cv2.boundingRect(d.contour)
            label=self.classifier.classify(color[y:y+h,x:x+w],d.mask[y:y+h,x:x+w],hsv=hsv[y:y+h,x:x+w])
            area=float(cv2.contourArea(d.contour));reasons=[]
            if label['color_label']=='unknown':reasons.append('RGB_COLOR_UNCERTAIN')
            if d.confidence<self.config['min_rgb_contour_fill']:reasons.append('RGB_OUTLINE_INCOMPLETE')
            if area<self.config['min_rgb_area_px'] or min(w,h)<self.config['min_rgb_short_side_px']:reasons.append('RGB_SUPPORT_TOO_SMALL')
            if max(w,h)/max(min(w,h),1)>self.config['max_rgb_aspect']:reasons.append('RGB_SHAPE_INCOMPATIBLE')
            if x<=1 or y<=1 or x+w>=color.shape[1]-1 or y+h>=color.shape[0]-1:reasons.append('RGB_TRUNCATED')
            proposals.append(dict(d=d,label=label,bbox=[x,y,w,h],reasons=reasons,object_id=None,
                color=label['color_label'],battery_mask=d.mask,confidence=d.confidence,
                confidence_semantics='CONTOUR_AREA_OVER_MIN_AREA_RECT; NOT_CALIBRATED_PROBABILITY'))
            tracking.append(dict(color=label['color_label'],confidence=d.confidence,pixel_center=list(d.pixel_center),detection_valid=not reasons))
        tracks=self.tracker.update(tracking,stamp);used=set()
        for t in tracks['observed']:
            choices=sorted((np.linalg.norm(np.asarray(p['d'].pixel_center)-t['pixel_center']),i) for i,p in enumerate(proposals) if i not in used)
            if not choices:continue
            i=choices[0][1];used.add(i);proposals[i]['track']=t;proposals[i]['object_id']=t['object_id']
            if t['tracking_status']!='confirmed':proposals[i]['reasons'].append('RGB_ASSOCIATION_NOT_ESTABLISHED')
        return proposals,tracks,hsv

class BodyPoseEstimation:
    def __init__(self,config,geometry):self.config=config;self.geometry=geometry
    def estimate(self,frame,mask,qualified):
        out=dict(body_pose_valid=False,body_xyz_candidate=None,body_yaw_candidate=None,yaw_candidates=[],visible_surface_center_xyz=None,quality={},reasons=[])
        if not qualified:out['reasons']=['RGBD_PAIR_REJECTED'];return out,np.empty((0,3)),np.empty((0,2),int)
        z=frame.aligned_depth*frame.depth_scale
        selected=mask&(z>=self.config['min_depth_m'])&(z<=self.config['max_depth_m'])
        ratio=float(selected.sum()/max(mask.sum(),1));out['quality']['valid_depth_ratio']=ratio
        if selected.sum()<self.config['min_points'] or ratio<self.config['min_depth_ratio']:
            out['reasons']=['INSUFFICIENT_CURRENT_DEPTH'];return out,np.empty((0,3)),np.empty((0,2),int)
        v,u=np.where(selected);depth=z[v,u];median=np.median(depth);mad=np.median(abs(depth-median))
        keep=abs(depth-median)<=max(self.config['depth_outlier_floor_m'],3*1.4826*mad)
        out['quality'].update(raw_points=len(v),outlier_fraction=float(1-keep.mean()),depth_mad_m=float(mad))
        uv=np.column_stack((u[keep],v[keep]));depth=depth[keep]
        if len(uv)<self.config['min_points']:out['reasons']=['DEPTH_OUTLIERS'];return out,np.empty((0,3)),uv
        pick=np.linspace(0,len(uv)-1,min(len(uv),self.config['max_points']),dtype=int);uv=uv[pick]
        points=deproject_pixels(uv,depth[pick],frame.intrinsics)
        center=np.median(points,axis=0);out['visible_surface_center_xyz']=center.tolist()
        centered=points-center;_,values,axes=np.linalg.svd(centered,full_matrices=False)
        normal=axes[-1];normal=normal if np.dot(normal,center)<0 else -normal
        residual=np.abs(centered@normal);axis=axes[0];yaw=float(np.arctan2(axis[1],axis[0])%np.pi)
        extents=np.percentile(centered@axes[:2].T,95,axis=0)-np.percentile(centered@axes[:2].T,5,axis=0)
        out.update(body_yaw_candidate=yaw,yaw_candidates=[yaw,yaw+np.pi])
        out['quality'].update(point_count=len(points),plane_residual_p95_m=float(np.percentile(residual,95)),visible_span_m=extents.tolist(),axis_ratio=float(values[0]/max(values[1],1e-9)))
        dims=(self.geometry or {}).get('battery',{}).get('body_dimensions_mm')
        if dims:
            nominal=np.sort([dims['long'],dims['width']])[::-1]/1000
            match=np.all(np.sort(extents)[::-1]>=nominal*.70)&np.all(np.sort(extents)[::-1]<=nominal*1.25)
            if match and np.percentile(residual,95)<=self.config['max_plane_residual_m']:
                out['body_xyz_candidate']=(center-normal*dims['height']/2000).tolist()
                out['quality']['center_method']='OBSERVED_PLANAR_FACE_PLUS_CAD_HALF_HEIGHT_UNVERIFIED_FACE_ID'
            else:out['reasons'].append('OBSERVED_FACE_NOT_CAD_BODY_TOP')
        else:out['reasons'].append('CAD_DIMENSIONS_UNAVAILABLE')
        out['reasons']+=['YAW_180_DEG_SYMMETRY','FACE_ID_AND_XYZ_ACCURACY_UNVERIFIED']
        return out,points,uv

class RingPoseEstimation:
    def __init__(self,config,geometry,ranges):self.config=config;self.geometry=geometry;self.ranges=ranges
    def estimate(self,frame,mask,color,qualified,hsv=None):
        out=dict(ring_pose_valid=False,center_candidate=None,direction_candidates=None,hole_candidates=[],reasons=[])
        if not qualified:out['reasons']=['RGBD_PAIR_REJECTED'];return out
        hsv=cv2.cvtColor(frame.color_bgr,cv2.COLOR_BGR2HSV) if hsv is None else hsv;colored=np.zeros(mask.shape,np.uint8)
        for a,b,c,d,e,f in self.ranges.get(color,[]):colored|=cv2.inRange(hsv,(a,b,c),(d,e,f))
        colored=np.uint8((colored>0)&mask)*255
        # Raw observed topology: do not use the detector's filled contour as a hole.
        contours,hierarchy=cv2.findContours(colored,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        aperture=(self.geometry or {}).get('battery',{}).get('aperture')
        if hierarchy is not None:
            for i,c in enumerate(contours):
                if hierarchy[0,i,3]<0 or cv2.contourArea(c)<self.config['min_hole_area_px']:continue
                moments=cv2.moments(c)
                if not moments['m00']:continue
                uv=np.array([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
                hole=np.zeros(mask.shape,np.uint8);cv2.drawContours(hole,[c],-1,255,-1)
                edge=cv2.dilate(hole,np.ones((7,7),np.uint8))>0
                rim=edge&(hole==0)&(colored>0);z=frame.aligned_depth*frame.depth_scale
                valid=rim&(z>=self.config['min_depth_m'])&(z<=self.config['max_depth_m'])
                candidate=dict(pixel_center=uv.tolist(),contour=c.reshape(-1,2).tolist(),rim_depth_points=int(valid.sum()),center_xyz_candidate=None,reasons=[])
                if valid.sum()<self.config['min_ring_points']:candidate['reasons'].append('INSUFFICIENT_OBSERVED_RING_RIM_DEPTH')
                else:
                    v,u=np.where(valid);pick=np.linspace(0,len(v)-1,min(400,len(v)),dtype=int)
                    points=deproject_pixels(np.column_stack((u[pick],v[pick])),z[v[pick],u[pick]],frame.intrinsics)
                    center=np.median(points,axis=0);_,_,axes=np.linalg.svd(points-center,full_matrices=False);normal=axes[-1]
                    residual=float(np.percentile(abs((points-center)@normal),95));candidate['rim_plane_residual_p95_m']=residual
                    ray=deproject_pixels([uv],[1.],frame.intrinsics)[0];denom=float(ray@normal)
                    if residual>self.config['max_plane_residual_m'] or abs(denom)<.1:candidate['reasons'].append('RING_PLANE_UNRELIABLE')
                    else:
                        crossing=ray*float(center@normal)/denom
                        if crossing[2]<=0:candidate['reasons'].append('RING_INTERSECTION_BEHIND_CAMERA')
                        else:
                            candidate['center_xyz_candidate']=crossing.tolist();candidate['direction_candidates']=[normal.tolist(),(-normal).tolist()]
                            # A dark/glare patch on a solid face is not a through-hole.
                            # Require independently observed background depth beyond the rim plane.
                            interior=cv2.erode(hole,np.ones((5,5),np.uint8))>0
                            observed=interior&(z>=self.config['min_depth_m'])&(z<=self.config['max_depth_m'])
                            candidate['hole_valid_depth_points']=int(observed.sum())
                            if observed.sum()<self.config['min_ring_points']:
                                candidate['reasons'].append('HOLE_VOID_DEPTH_NOT_OBSERVED')
                            else:
                                hv,hu=np.where(observed);rays=deproject_pixels(np.column_stack((hu,hv)),np.ones(len(hu)),frame.intrinsics)
                                expected_z=float(center@normal)/(rays@normal)
                                gap=float(np.median(z[hv,hu]-expected_z));candidate['hole_background_gap_m']=gap
                                if not np.isfinite(gap) or gap<self.config['min_hole_background_gap_m']:
                                    candidate['reasons'].append('HOLE_HAS_NO_OBSERVED_BACKGROUND_SEPARATION')
                            # Intersect hole outline rays with the observed rim plane, not background depth in the void.
                            boundary=deproject_pixels(c.reshape(-1,2),np.ones(len(c)),frame.intrinsics);div=boundary@normal
                            if (abs(div)<.1).any():candidate['reasons'].append('HOLE_BOUNDARY_RAYS_UNRELIABLE')
                            else:
                                points_hole=boundary*(float(center@normal)/div)[:,None]
                                dimensions=np.ptp((points_hole-crossing)@axes[:2].T,axis=0);candidate['aperture_span_m']=dimensions.tolist()
                                if aperture:
                                    expected=np.sort([aperture['width_mm'],aperture['height_mm']])/1000
                                    if (np.sort(dimensions)<expected*.7).any() or (np.sort(dimensions)>expected*1.25).any():candidate['reasons'].append('HOLE_NOT_CAD_APERTURE_SIZE')
                                else:candidate['reasons'].append('CAD_APERTURE_UNAVAILABLE')
                out['hole_candidates'].append(candidate)
        supported=[c for c in out['hole_candidates'] if c['center_xyz_candidate'] is not None and not c['reasons']]
        if len(supported)==1:out.update(center_candidate=supported[0]['center_xyz_candidate'],direction_candidates=supported[0]['direction_candidates'])
        out['reasons']=['NO_UNIQUE_OBSERVED_CAD_COMPATIBLE_APERTURE'] if len(supported)!=1 else ['ENTRY_SIGN_AND_RING_PRECISION_UNVERIFIED']
        return out

class SimplifiedPerception:
    def __init__(self,config_path=None):
        self.config=yaml.safe_load(Path(config_path or ROOT/'config/simplified_rgbd.yaml').read_text())
        file=ROOT/'outputs/cad_analysis/geometry.json';self.geometry=json.loads(file.read_text()) if file.exists() else None
        self.detection=BatteryDetection(self.config);self.body=BodyPoseEstimation(self.config,self.geometry)
        self.ring=RingPoseEstimation(self.config,self.geometry,self.detection.classifier.ranges)
        self.guard=RGBDPairGuard();self.rate=RateMeter();self.last_timestamp=None;self.table_hint=None
    def set_table_hint(self,*args,**kwargs):pass  # Compatibility only: no identity/cache dependency.
    def process(self,frame,source_frame_id):
        start=perf_counter();validate_frame(frame.color_bgr,frame.aligned_depth,frame.intrinsics,frame.depth_scale)
        stamp=capture_time(frame)
        if self.last_timestamp is not None and stamp<=self.last_timestamp:raise ValueError('Non-increasing capture time')
        self.last_timestamp=stamp;pair=self.guard.inspect(frame.timestamps)
        pair['qualified']=pair['qualified'] and frame.alignment_metadata is not None and frame.alignment_metadata.get('method')=='SDK_DEPTH_TO_COLOR'
        if not pair['qualified'] and not pair['reasons']:pair['reasons'].append('ALIGNMENT_UNVERIFIED')
        proposals,tracking,hsv=self.detection.detect(frame.color_bgr,stamp);objects=[];views=[];diagnostics=[]
        rgb_ms=(perf_counter()-start)*1000;body_ms=0.;ring_ms=0.
        for p in proposals:
            t=p.get('track');d=p['d'];diagnostics.append(dict(color=d.color,bbox=p['bbox'],state='RAW_CANDIDATE',reason=';'.join(p['reasons']),confidence=d.confidence))
            if t is None:continue
            stage=perf_counter();body,points,uv=self.body.estimate(frame,d.mask,pair['qualified']);body_ms+=(perf_counter()-stage)*1000
            stage=perf_counter();ring=self.ring.estimate(frame,d.mask,p['label']['color_label'],pair['qualified'],hsv);ring_ms+=(perf_counter()-stage)*1000
            accepted=not p['reasons'];state='CONFIRMED_OBJECT' if accepted else 'AMBIGUOUS';surface=body['visible_surface_center_xyz']
            obj=dict(object_id=t['object_id'],color=p['label']['color_label'],color_label=p['label']['color_label'],observed_color=d.color,
                confidence=d.confidence,confidence_semantics='CONTOUR_AREA_OVER_MIN_AREA_RECT; NOT_CALIBRATED_PROBABILITY',color_confidence=p['label']['color_confidence'],
                detection_valid=accepted,battery_mask_encoding='CURRENT_FRAME_BOOLEAN_IN_VIEWS',bbox=p['bbox'],contour=d.contour.reshape(-1,2).tolist(),pixel_center=list(d.pixel_center),
                candidate_state=state,tracking_status=t['tracking_status'],tracking_reliability='RGB_VISUAL_HYPOTHESIS; NOT_GRASP_VALIDATION',association_status=t['tracking_status'],
                confirmation=dict(eligible=accepted,reasons=p['reasons'],scope='RGB_COLOR_SHAPE_TRACK_HYPOTHESIS',evidence=dict(color=p['label'],contour_fill=d.confidence)),
                rejection_reason=';'.join(p['reasons']) if p['reasons'] else None,depth_valid_ratio=body['quality'].get('valid_depth_ratio',0),point_count=len(points),
                visible_surface_center_xyz=surface,surface_depth_z=None if surface is None else surface[2],camera_to_surface_distance=None if surface is None else float(np.linalg.norm(surface)),
                model_estimated_body_center_xyz=body['body_xyz_candidate'],body_center_camera_z=None if body['body_xyz_candidate'] is None else body['body_xyz_candidate'][2],camera_to_body_center_distance=None,
                body_yaw_candidate=body['body_yaw_candidate'],body_yaw_candidates=body['yaw_candidates'],ring_center_candidate=ring['center_candidate'],ring_orientation_candidate=None,
                insertion_direction_candidates=ring['direction_candidates'],hook_insertion_yaw_candidates=None,body_pose_candidate=None,ring_pose_candidate=None,ring_axis_yaw_candidate=None,
                ring_feature_status='OBSERVED_APERTURE_CANDIDATE' if ring['center_candidate'] is not None else ('UNVERIFIED_COLOR_HOLE_PROPOSAL' if ring['hole_candidates'] else 'NOT_OBSERVED'),ring_feature_observability='LOCAL_RGB_DEPTH',ring_observed=ring['center_candidate'] is not None,
                body_pose_valid=False,ring_pose_valid=False,motion_usable=False,body_geometry=body,ring_geometry=ring,
                pose_quality=body['quality'],fit_status='SAME_FRAME_MASK_DEPTH_CANDIDATES',cad_fresh=False,cad_result_timestamp=None,cad_stale=False,
                ambiguity=body['reasons']+ring['reasons'],capture_timestamp=stamp,source_frame_id=source_frame_id)
            objects.append(obj);views.append(dict(object_id=obj['object_id'],mask=d.mask,foreground=d.mask,points=points,rgb=frame.color_bgr[uv[:,1],uv[:,0],::-1] if len(uv)==len(points) else np.empty((0,3)),bbox=p['bbox']))
        elapsed=(perf_counter()-start)*1000;self.rate.add()
        result=dict(timestamp=stamp,capture_timestamp=stamp,capture_timestamp_source=frame_capture_time(frame)[1],result_timestamp=time(),source_frame_id=source_frame_id,
            timestamps=frame.timestamps,objects=objects,candidate_count=len(proposals),candidate_diagnostics=diagnostics,rejections=[],accepted_object_count=sum(o['detection_valid'] for o in objects),
            diagnostic_object_count=len(objects),camera_motion_proxy=dict(stable=None,reason='NOT_REQUIRED_FOR_RGB_IDENTITY'),background_filter_report=dict(table_failure='NOT_REQUIRED_V45',table_hint_used=False),
            tracking_status={k:tracking[k] for k in ('missed','lost','ambiguous_candidate_indices')},processing_latency_ms=elapsed,timing_ms=dict(total=elapsed,detection=rgb_ms,body=body_ms,ring=ring_ms,cad_fit=0.,tracking=0.,render=0.),
            detection_fps=self.rate.hz(),max_detection_age_s=2.5,coordinate_frame='camera_color_optical',body_pose_valid=False,ring_pose_valid=False,pipeline_version='V4.5',pair_diagnostics=pair)
        result=finalize_interface(result)
        for obj in objects:obj['geometry_source']='CURRENT_MASK_DEPTH_LOCAL_APERTURE';obj['motion_usable']=False
        return result,views

class DisabledFullCAD:
    """Runner compatibility when full CAD is explicitly disabled, not fake results."""
    def __init__(self,geometry):
        from types import SimpleNamespace
        from .dual_rate_rgbd import LatestSlot
        self.pipeline=SimpleNamespace(geometry=geometry or {},config={})
        self.slot=LatestSlot();self.rate=RateMeter();self.completed_jobs=0;self.error=None;self.thread=None
    def start(self):pass
    def drain(self):return []
    def drain_table(self):return []
    def close(self):self.slot.close()
