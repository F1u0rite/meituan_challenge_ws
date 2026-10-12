"""Current-frame RGB-D/CAD diagnostics; reuse detector, segmentation and tracker."""
from pathlib import Path, PureWindowsPath
from time import perf_counter, time
from datetime import datetime
import sys,json,hashlib
from scipy.spatial import cKDTree
from dataclasses import replace
from .vision_interface import finalize_interface
from .color_classifier import ColorClassifier
import numpy as np
import cv2,yaml
from .unified_detector import UnifiedBatteryDetector,BatteryResult
from .rgbd_offline import OfflineConfig,analyze_frame,_sample_cloud,height_from_plane,body_geometry
from .rgbd_geometry import deproject_pixels,project_points,validate_frame,fit_plane_ransac,analyze_rgbd,frame_capture_time
from .cad_body import fit_body,sample_surface,ring_surface_evidence
from cad_geometry.frames import transform_points
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'src/mtc_vision_adapter'))
from mtc_vision_adapter.core import Tracker

def verify_cad_sources(geometry, asset_dir):
    """Verify cached STEP provenance, including Windows-authored source paths."""
    for name in ('battery','hook'):
        source=geometry[name]['source']
        path=Path(asset_dir)/PureWindowsPath(source['source_path']).name
        if hashlib.sha256(path.read_bytes()).hexdigest()!=source['sha256']:
            raise ValueError('CAD cache source mismatch; rerun existing analyze_cad_rgbd.py')

class RGBDVisionPipeline:
    def __init__(self,config_path=ROOT/'config/realtime_rgbd.yaml',
                 detector=None,geometry=None,surface=None,clock=perf_counter):
        self.config=yaml.safe_load(Path(config_path).read_text(encoding='utf-8-sig'))
        self.clock=clock
        cv2.setNumThreads(int(self.config.get("opencv_threads",1)))
        if self.config['cad_interval_s']<0 or self.config['min_depth_m']<=0 or self.config['max_depth_m']<=self.config['min_depth_m']:
            raise ValueError('Invalid depth range or CAD interval')
        self.geometry=geometry or json.loads((ROOT/'outputs/cad_analysis/geometry.json').read_text(encoding='utf-8'))
        if geometry is None:
            verify_cad_sources(self.geometry,ROOT/'assets/cad')
        self.surface=surface if surface is not None else sample_surface(
            np.load(ROOT/'outputs/cad_analysis/cad_meshes.npz')['battery'],
            count=int(self.config['cad_surface_samples']))
        self.tree=cKDTree(self.surface)
        self.classifier=ColorClassifier()
        self.detector=detector or UnifiedBatteryDetector()
        self.tracker=Tracker(max_distance=self.config['tracker_max_distance_px'],
            max_gap_s=self.config['tracker_max_gap_s'],
            confirmation_hits=self.config['tracker_confirmation_hits'],
            ambiguity_px=self.config['tracker_ambiguity_px'],allow_unknown=True)
        self.last_cad={};self.last_cad_timestamp={};self.rays_cache={}
        self.cfg=OfflineConfig(min_depth_m=self.config['min_depth_m'],
            max_depth_m=self.config['max_depth_m'],single_red_object=False,fast_table_roi=True,
            max_cloud_points=6000,table_min_points=600,table_ransac_iterations=80,adaptive_table_roi=True,
            joint_rgb_depth=bool(self.config.get('joint_rgb_depth',False)),
            joint_footprint_m=tuple(self.geometry['battery']['body_dimensions_mm'][k]/1000 for k in ('long','width')))

    def _rays(self,intr,sdk_deproject):
        if not any(abs(c)>1e-10 for c in intr.coeffs):return None
        key=(intr.width,intr.height,intr.fx,intr.fy,intr.ppx,intr.ppy,intr.model,tuple(intr.coeffs))
        if key not in self.rays_cache:
            if sdk_deproject is None:raise ValueError('Distorted intrinsics require SDK deprojection')
            v,u=np.indices((intr.height,intr.width))
            uv=np.column_stack((u.ravel(),v.ravel()))
            self.rays_cache[key]=deproject_pixels(uv,np.ones(len(uv)),intr,sdk_deproject).reshape(intr.height,intr.width,3)
        return self.rays_cache[key]

    def process(self,frame,*,sdk_deproject=None,force_cad=False,collect_diagnostics=False):
        start=self.clock()
        validate_frame(frame.color_bgr,frame.aligned_depth,frame.intrinsics,frame.depth_scale)
        timestamp,timestamp_source=frame_capture_time(frame)
        pre_start=self.clock();hsv=cv2.cvtColor(frame.color_bgr,cv2.COLOR_BGR2HSV)
        preprocess_ms=(self.clock()-pre_start)*1000
        detect_start=self.clock()
        detections=self.detector.detect_batteries(frame.color_bgr,hsv=hsv) if isinstance(self.detector,UnifiedBatteryDetector) else self.detector.detect_batteries(frame.color_bgr)
        detection_ms=(self.clock()-detect_start)*1000
        rays=self._rays(frame.intrinsics,sdk_deproject)
        detections=[replace(d,detection_valid=True) if d.color=='unknown' else d for d in detections]
        good=[d for d in detections if d.detection_valid]
        depth_start=self.clock()
        depth_seed=frame.aligned_depth.astype(float)*frame.depth_scale
        depth_preprocessing_ms=(self.clock()-depth_start)*1000
        def seed_depth(d):
            values=depth_seed[d.mask & (depth_seed>=self.cfg.min_depth_m)&(depth_seed<=self.cfg.max_depth_m)]
            return float(np.median(values)) if len(values) else float('inf')
        # Select a geometrically plausible seed, not the closest tiny color patch.
        nominal_seed=np.array([self.geometry['battery']['body_dimensions_mm'][k] for k in ('long','width')])/1000
        def seed_score(d):
            cloud,_=_sample_cloud(depth_seed,d.mask & (depth_seed>=self.cfg.min_depth_m)&(depth_seed<=self.cfg.max_depth_m),frame.intrinsics,200,sdk_deproject)
            if len(cloud)<20:return (2,float('inf'))
            span=np.sort(np.percentile(cloud[:,:2],95,axis=0)-np.percentile(cloud[:,:2],5,axis=0))[::-1]
            credible=bool((span>=nominal_seed*.4).all() and (span<=nominal_seed*1.8).all())
            return (0 if credible else 1,seed_depth(d) if credible else float(np.linalg.norm(np.log(np.maximum(span,1e-5)/nominal_seed))))
        focus=min(good,key=seed_score).color if good else 'unknown'
        seg_start=self.clock()
        # When color is absent, a dominant camera-facing depth plane may propose
        # geometry-only components. This is a diagnostic table hypothesis, not TF.
        plane_override=None
        if not good:
            depth=frame.aligned_depth.astype(float)*frame.depth_scale
            valid=(depth>=self.cfg.min_depth_m)&(depth<=self.cfg.max_depth_m)&(frame.aligned_depth>0)
            sample,_=_sample_cloud(depth,valid,frame.intrinsics,self.cfg.max_cloud_points,sdk_deproject)
            plane_override=fit_plane_ransac(sample,threshold_m=self.cfg.ransac_threshold_m,min_inliers=self.cfg.table_min_points)
            if plane_override is not None and (plane_override['inlier_ratio']<self.cfg.table_min_inlier_ratio or
                    plane_override['mean_residual_m']>self.cfg.table_max_residual_m or
                    abs(plane_override['normal_camera_xyz'][2])<self.cfg.table_min_camera_facing):
                plane_override=None
            if plane_override is not None:
                plane_override['support_region']='GLOBAL_DOMINANT_DEPTH_PLANE; NOT_EXTERNALLY_VERIFIED'
                height=height_from_plane(depth,frame.intrinsics,plane_override,rays)
                fg=valid&(height>=self.cfg.min_foreground_height_m)&(height<=self.cfg.max_foreground_height_m)
                count,labels,stats,_=cv2.connectedComponentsWithStats(np.uint8(fg),8)
                for component in range(1,count):
                    if stats[component,cv2.CC_STAT_AREA]<self.cfg.component_min_area_px:continue
                    mask=labels==component;contours,_=cv2.findContours(np.uint8(mask),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    contour=max(contours,key=cv2.contourArea);rect=cv2.minAreaRect(contour)
                    detections.append(BatteryResult('geometry_seed','unknown',.5,rect[0],0.,contour,
                        cv2.boxPoints(rect),mask,True))
                good=[d for d in detections if d.detection_valid];focus='unknown'
        report,maps,cloud,rgb=analyze_frame(frame,detections,self.cfg,focus_color=focus,
            sdk_deproject=sdk_deproject,rays_xyz=rays,table_plane_override=plane_override)
        accepted=[]
        if report['table_plane'] is not None:
            accepted=[(d,r) for d,r in zip(detections,report['candidates']) if r['accepted'] and d.detection_valid]
        if report['table_plane'] is None:
            accepted=[(d,{'component_area_px':int(d.mask.sum())}) for d in good]
        supplemental={}
        # Add shape-only instances not covered by any color proposal.
        if report['table_plane'] is not None:
            count,labels,stats,_=cv2.connectedComponentsWithStats(np.uint8(maps['foreground']),8)
            covered=np.logical_or.reduce([d.mask for d in good]) if good else np.zeros(frame.aligned_depth.shape,bool)
            for component in range(1,count):
                if stats[component,cv2.CC_STAT_AREA]<self.cfg.component_min_area_px:continue
                mask=labels==component
                if (mask&covered).sum()/max(mask.sum(),1)>.35:continue
                sampled,_=_sample_cloud(frame.aligned_depth.astype(float)*frame.depth_scale,mask,
                    frame.intrinsics,1000,sdk_deproject)
                seed_geometry=body_geometry(sampled,report['table_plane'])
                dims=self.geometry['battery']['body_dimensions_mm']
                nominal=np.array([dims['long'],dims['width']])/1000
                span=np.array(seed_geometry.get('obb_footprint_m',[]))
                if len(span)!=2 or (span>nominal*self.config['max_footprint_scale']).any() or (span<self.config['min_footprint_m']).any():
                    report.setdefault('geometry_seed_rejections',[]).append('OUTSIDE_CAD_FOOTPRINT_PRIOR')
                    continue
                contours,_=cv2.findContours(np.uint8(mask),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                contour=max(contours,key=cv2.contourArea);rect=cv2.minAreaRect(contour)
                seed=BatteryResult('depth_seed','unknown',.5,rect[0],0.,contour,cv2.boxPoints(rect),mask,True)
                sub=analyze_frame(frame,[seed],self.cfg,focus_color='unknown',
                    table_plane_override=report['table_plane'],sdk_deproject=sdk_deproject,rays_xyz=rays,prepared=maps)
                if sub[0]['candidates'] and sub[0]['candidates'][0]['accepted']:
                    accepted.append((seed,sub[0]['candidates'][0]));supplemental[id(seed)]=sub
            report['geometry_only_seed_count']=len(supplemental)
        primary=max([(d,r) for d,r in accepted if d.color==focus and id(d) not in supplemental],
                    key=lambda pair:pair[1]['component_area_px'],default=None)
        candidates=[];payloads=[];rejections=[];candidate_diagnostics=[]
        for i,d in enumerate(detections if collect_diagnostics else []):
            raw_points,_=_sample_cloud(depth_seed,d.mask & (depth_seed>=self.cfg.min_depth_m)&(depth_seed<=self.cfg.max_depth_m),frame.intrinsics,200,sdk_deproject)
            rx,ry,rw,rh=cv2.boundingRect(np.uint8(d.mask))
            raw_class=self.classifier.classify(frame.color_bgr[ry:ry+rh,rx:rx+rw],d.mask[ry:ry+rh,rx:rx+rw],hsv=hsv[ry:ry+rh,rx:rx+rw])
            raw_report=report['candidates'][i] if i<len(report['candidates']) else {}
            candidate_diagnostics.append(dict(candidate_index=i,color=d.color,bbox=list(cv2.boundingRect(np.uint8(d.mask))),contour=d.contour.reshape(-1,2).tolist(),mask_area=int(d.mask.sum()),hsv_evidence=raw_class,depth_valid_ratio=raw_report.get('valid_ratio',float(((depth_seed[d.mask]>=self.cfg.min_depth_m)&(depth_seed[d.mask]<=self.cfg.max_depth_m)).mean())),table_distance_m_p05_p50_p95=np.percentile(maps['height_m'][d.mask & (depth_seed>0)],[5,50,95]).tolist() if report['table_plane'] and (d.mask & (depth_seed>0)).any() else None,extent_camera_xyz_m=np.ptp(raw_points,axis=0).tolist() if len(raw_points) else None,cad_geometry_consistency='NOT_EVALUATED',state='REJECTED' if raw_report.get('reason') else 'RAW_CANDIDATE',rejection_reason=raw_report.get('reason')))
        dimensions=self.geometry['battery']['body_dimensions_mm']
        nominal=np.array([dimensions['long'],dimensions['width']])/1000
        filter_start=self.clock()
        for d,diagnostic in sorted(accepted,key=lambda pair:-pair[1]['component_area_px']):
            if id(d) in supplemental:
                local_report,local_maps,points,colors=supplemental[id(d)]
            elif report['table_plane'] is None:
                fallback,points,colors=analyze_rgbd(frame.color_bgr,frame.aligned_depth,d.mask,frame.intrinsics,frame.depth_scale,
                    min_depth_m=self.cfg.min_depth_m,max_depth_m=self.cfg.max_depth_m,sdk_deproject=sdk_deproject,table_roi=np.zeros(d.mask.shape,bool))
                if len(points):
                    footprint=sorted(np.ptp(points[:,:2],axis=0),reverse=True)
                else:footprint=[]
                local_report={'body_geometry':{'obb_footprint_m':footprint},'object_depth_valid_ratio':fallback['depth_valid_ratio'],
                    'timing_ms':{},'fallback':'VISIBLE_SURFACE_ONLY; TABLE_UNAVAILABLE'}
                local_maps={'object':d.mask,'foreground':d.mask}
            elif primary is not None and d is primary[0]:
                local_report,local_maps,points,colors=report,maps,cloud,rgb
            else:
                local_report,local_maps,points,colors=analyze_frame(frame,[d],self.cfg,
                    focus_color=d.color,table_plane_override=report['table_plane'],
                    sdk_deproject=sdk_deproject,rays_xyz=rays,prepared=maps)
            mask=local_maps['object'];reason=None
            if len(points)<self.config['min_object_points']:reason='INSUFFICIENT_OBJECT_POINTS'
            geom=local_report['body_geometry']
            footprint=np.array(geom.get('obb_footprint_m',[]))
            heights=geom.get('visible_height_m_p10_p50_p90',[])
            if len(footprint)!=2:reason=reason or 'NO_FOOTPRINT'
            elif (footprint<self.config['min_footprint_m']).any() or (footprint>nominal*self.config['max_footprint_scale']).any():
                reason='OUTSIDE_CAD_FOOTPRINT_PRIOR'
            if report['table_plane'] is None and len(footprint)==2 and (footprint<nominal*self.config.get('min_footprint_scale',.45)).any():reason='INSUFFICIENT_CAD_FOOTPRINT_SUPPORT_WITHOUT_TABLE'
            if heights and (heights[-1]>self.config['max_object_height_m'] or heights[-1]<=0):
                reason='OUTSIDE_CAD_HEIGHT_PRIOR'
            if mask.any() and (mask[0].any() or mask[-1].any() or mask[:,0].any() or mask[:,-1].any()):
                reason=reason or 'OBJECT_CLIPPED'
            for previous in payloads:
                overlap=(mask & previous['mask']).sum()/max(min(mask.sum(),previous['mask'].sum()),1)
                if overlap>self.config['overlap_fraction']:reason='DUPLICATE_OR_OVERLAPPING_COMPONENT'
            entry=next((c for c in candidate_diagnostics if c['contour']==d.contour.reshape(-1,2).tolist()),None)
            if entry is not None:entry.update(state='REJECTED' if reason else ('GEOMETRY_SUPPORTED' if report['table_plane'] else 'AMBIGUOUS'),rejection_reason=reason or ('TABLE_UNAVAILABLE' if report['table_plane'] is None else None),cad_geometry_consistency=geom)
            if reason:
                rejections.append({'color':d.color,'reason':reason});continue
            v,u=np.where(mask)
            classification=self.classifier.classify(frame.color_bgr,mask,hsv=hsv)
            candidates.append(dict(color=classification['color_label'],confidence=d.confidence,
                pixel_center=[float(u.mean()),float(v.mean())],detection_valid=True))
            payloads.append(dict(mask=mask,points=points,rgb=colors,report=local_report,classification=classification,foreground=local_maps['foreground'],rgb_support=local_maps.get('rgb_support',d.mask),region_seed=local_maps.get('rgb_seed',mask)))
        candidate_processing_ms=(self.clock()-filter_start)*1000
        point_cloud_ms=(self.clock()-seg_start)*1000
        track_start=self.clock()
        tracking=self.tracker.update(candidates,timestamp)
        tracking_ms=(self.clock()-track_start)*1000
        objects=[];views=[];cad_ms=0.;multistart_ms=0.;ring_ms=0.
        for track in tracking['observed']:
            index=min(range(len(candidates)),key=lambda i: np.linalg.norm(np.array(candidates[i]['pixel_center'])-track['pixel_center']))
            payload=payloads[index];points=payload['points'];key=track['object_id']
            geom_span=np.array(payload['report']['body_geometry'].get('obb_footprint_m',[]))
            full_support=report['table_plane'] is not None and len(geom_span)==2 and (geom_span>=nominal*self.config.get('min_footprint_scale',.45)).all()
            item=dict(candidate_state='ACCEPTED_OBJECT' if full_support else 'AMBIGUOUS',object_id=key,color=track['color'],observed_color=candidates[index]['color'],
                confidence=track['confidence'],tracking_status=track['tracking_status'],
                pixel_center=track['pixel_center'],timestamp=timestamp,
                tracking_reliability={'confirmed':'TRACKED','tentative':'NEW','color_uncertain':'AMBIGUOUS'}[track['tracking_status']],
                color_label=candidates[index]['color'],color_confidence=payload['classification']['color_confidence'],
                color_evidence=payload['classification']['color_evidence'],
                surface_depth_z=float(np.median(points[:,2])),camera_to_surface_distance=float(np.linalg.norm(np.median(points,axis=0))),
                body_center_camera_z=None,camera_to_body_center_distance=None,
                bbox=list(map(int,cv2.boundingRect(np.uint8(payload['mask'])))),
                contour=cv2.findContours(np.uint8(payload['mask']),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0][0].reshape(-1,2).tolist(),
                visible_surface_center_xyz=np.median(points,axis=0).tolist(),
                model_estimated_body_center_xyz=None,body_yaw_candidate=None,
                ring_center_candidate=None,ring_orientation_candidate=None,
                ring_axis_yaw_candidate=None,hook_insertion_yaw_candidates=None,
                insertion_direction_candidates=None,body_pose_valid=False,ring_pose_valid=False,
                body_pose_candidate=None,ring_pose_candidate=None,ring_observed=False,
                coordinate_frame='camera_color_optical',unit_position='m',unit_angle='rad',
                point_count=len(points),depth_valid_ratio=payload['report']['object_depth_valid_ratio'],
                ambiguity=['180_DEG_DIRECTION','NO_INDEPENDENT_3D_VALIDATION'],
                rejection_reason=None,pose_quality={'table_residual_m':report['table_plane']['mean_residual_m'] if report['table_plane'] else None},
                cad_result_timestamp=None,cad_last_attempt_timestamp=self.last_cad_timestamp.get(key),
                cad_fresh=False,fit_status='CAD_NOT_SCHEDULED')
            due=force_cad or self.clock()-self.last_cad.get(key,float('-inf'))>=self.config['cad_interval_s']
            fit=None
            if report['table_plane'] is None:
                item.update(fit_status='NO_TABLE_PLANE',rejection_reason='VISIBLE_SURFACE_ONLY; NO_TABLE_CAD_CONSTRAINT')
                due=False
            if due:
                fit_start=self.clock()
                fit=fit_body(points,report['table_plane'],self.geometry,self.surface,
                    max_points=int(self.config['cad_max_points']),max_nfev=int(self.config['cad_max_nfev']),surface_tree=self.tree,analytic_jacobian=True,validation_points=int(self.config.get("cad_validation_points",1500)),use_body_boundary=bool(self.config.get("cad_body_boundary",False)),use_ring_axis=bool(self.config.get("cad_ring_surface_axis",False)))
                cad_ms+=(self.clock()-fit_start)*1000
                multistart_ms+=fit.get('timing_ms',{}).get('multistart_optimization',0.)
                ring_ms+=fit.get('timing_ms',{}).get('ring_candidate',0.)
                self.last_cad[key]=self.clock();self.last_cad_timestamp[key]=timestamp
                item.update(fit_status=fit['fit_status'],cad_result_timestamp=timestamp,CAD_result_timestamp=time(),
                            cad_last_attempt_timestamp=timestamp,cad_fresh=True)
                pose=fit.get('pose',{});T=pose.get('body_pose');ring=pose.get('ring_pose')
                item['pose_quality'].update({k:v for k,v in fit.items() if k not in ('pose','visible_surface_center_m')})
                if T is not None and ring is not None:
                    item.update(model_estimated_body_center_xyz=np.array(T)[:3,3].tolist(),
                        body_pose_candidate=T,ring_pose_candidate=ring,
                        ring_center_candidate=np.array(ring)[:3,3].tolist(),
                        body_center_camera_z=float(np.array(T)[2,3]),camera_to_body_center_distance=float(np.linalg.norm(np.array(T)[:3,3])),
                        ring_axis_direction=np.array(T)[:3,1].tolist(),
                        ring_orientation_candidate=np.array(ring)[:3,:3].tolist(),
                        body_yaw_candidate=float(np.deg2rad(pose['body_yaw'])),
                        ring_axis_yaw_candidate=float(np.deg2rad(pose['ring_axis_yaw'])),
                        hook_insertion_yaw_candidates=np.deg2rad(pose['insertion_yaw']).tolist(),
                        insertion_direction_candidates=pose['insertion_direction_candidates_camera'])
                    item['ring_surface_support']=ring_surface_evidence(points,T,self.geometry)
                    if fit.get('weak_constraint_flag',False):item['ambiguity'].append('WEAK_XY_YAW_CONSTRAINT')
                    item['rejection_reason']=pose.get('reason','NO_INDEPENDENT_3D_VALIDATION')
                else:item['rejection_reason']=fit['fit_status']
            elif report['table_plane'] is not None:item['rejection_reason']='CAD_NOT_SCHEDULED; NO_CURRENT_FRAME_POSE'
            item['pose_hypotheses']=fit.get('pose_hypotheses',[]) if fit else []
            if fit and fit.get('reasonable_pose_count',0)>1:item['ambiguity'].append('MULTIPLE_REASONABLE_CAD_POSES')
            item['ring_feature_status']='CAD_PREDICTED' if item['ring_center_candidate'] is not None else 'N/A'
            item['ring_feature_observability']='NOT_INDEPENDENTLY_OBSERVED'
            item['hook_insertion_direction_candidates']=item['insertion_direction_candidates']
            item['yaw_reference']='CAMERA_OPTICAL_XY_AXIS_PROJECTION; NOT_TABLE_YAW'
            if item['fit_status'] in ('INSUFFICIENT_BODY_BAND','CAD_OBSERVATION_MISMATCH'):item['candidate_state']='AMBIGUOUS'
            if item['candidate_state']=='AMBIGUOUS':item['tracking_reliability']='AMBIGUOUS'
            for entry in candidate_diagnostics:
                if entry['state'] in ('GEOMETRY_SUPPORTED','AMBIGUOUS') and entry['color']==candidates[index]['color']:
                    x,y,w,h=entry['bbox'];u,vv=item['pixel_center']
                    if x<=u<=x+w and y<=vv<=y+h:entry.update(state=item['candidate_state'],rejection_reason=item['rejection_reason'],object_id=key)
            objects.append(item);views.append(payload)
        live_ids=set(self.tracker.tracks)
        self.last_cad={k:v for k,v in self.last_cad.items() if k in live_ids}
        self.last_cad_timestamp={k:v for k,v in self.last_cad_timestamp.items() if k in live_ids}
        total=(self.clock()-start)*1000
        result=dict(timestamp=timestamp,capture_timestamp=timestamp,capture_timestamp_source=timestamp_source,timestamps=frame.timestamps,coordinate_frame='camera_color_optical',
            coordinate_frame_status='OPTICAL_AXIS_CONVENTION; NO_ROS_TF_CALIBRATION',
            cad_update_limit_hz=1/self.config['cad_interval_s'] if self.config['cad_interval_s'] else None,objects=objects,candidate_count=len(detections),accepted_object_count=sum(o['candidate_state']=='ACCEPTED_OBJECT' for o in objects),diagnostic_object_count=len(objects),
            background_filter_report=report,rejections=rejections,candidate_diagnostics=candidate_diagnostics,
            tracking_status=dict(missed=tracking['missed'],lost=tracking['lost'],
                                 ambiguous_candidate_indices=tracking['ambiguous_candidate_indices']),
            timing_ms=dict(depth_preprocessing=depth_preprocessing_ms,rgb_preprocessing=preprocess_ms,detection=detection_ms,segmentation=point_cloud_ms,candidate_processing=candidate_processing_ms,foreground_segmentation=report.get("timing_ms",{}).get("foreground_segmentation",0.),table_plane=report.get("timing_ms",{}).get("table_plane",0.),
                depth_filter=report.get('timing_ms',{}).get('depth_filter',0.),
                point_cloud=sum(p['report'].get('timing_ms',{}).get('point_extraction',0.) for p in views),
                cad_fit=cad_ms,multistart_optimization=multistart_ms,ring_candidate=ring_ms,tracking=tracking_ms,total=total),
            body_pose_valid=False,ring_pose_valid=False,
            position_accuracy='NOT_MEASURED',yaw_accuracy='NOT_MEASURED')
        from .dual_rate_rgbd import frame_id
        result['source_frame_id']=frame_id(frame,0)
        result['result_timestamp']=time()
        for item in objects:
            if item['cad_fresh']:
                item.update(CAD_result_source_frame=result['source_frame_id'],CAD_capture_timestamp=timestamp,
                    CAD_capture_timestamp_source=timestamp_source,CAD_result_age_ms=max(0,(result['result_timestamp']-timestamp)*1000))
        return finalize_interface(result),views

def render_vision(frame,result,views,geometry,*,sdk_project=None,display_fps=None):
    from .vision_ui import VisionUI
    return VisionUI().render(frame,result,views,geometry,sdk_project,display_fps)
