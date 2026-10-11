"""Bounded synchronized capture and dual-rate diagnostic perception."""
from collections import deque
from datetime import datetime
from threading import Condition, Event, Lock, Thread
from time import perf_counter, monotonic, time
import cv2
import numpy as np
from .rgbd_geometry import validate_frame, deproject_pixels, frame_capture_time
from .unified_detector import UnifiedBatteryDetector
from .color_classifier import ColorClassifier
from .realtime_rgbd import RGBDVisionPipeline
from mtc_vision_adapter.core import Tracker


def capture_time(frame):return frame_capture_time(frame)[0]
def validate_sync(frame):
    validate_frame(frame.color_bgr,frame.aligned_depth,frame.intrinsics,frame.depth_scale)
    alignment=frame.alignment_metadata
    if alignment is not None and alignment.get('method')!='SDK_DEPTH_TO_COLOR':
        raise ValueError('RGB-D frame is not SDK depth-to-color aligned')
    t=frame.timestamps
    if all(k in t for k in ('color_sdk_ms','depth_sdk_ms','color_timestamp_domain','depth_timestamp_domain')):
        if t['color_timestamp_domain']==t['depth_timestamp_domain'] and abs(t['color_sdk_ms']-t['depth_sdk_ms'])>100:
            raise ValueError('RGB/Depth capture timestamps are too far apart')

def frame_id(frame,fallback):
    t=frame.timestamps
    return f"color_{t['color_frame_number']}_depth_{t['depth_frame_number']}" if 'color_frame_number' in t and 'depth_frame_number' in t else f'replay_{fallback:06d}'

class LatestSlot:
    def __init__(self):self.cv=Condition();self.item=None;self.closed=False;self.dropped=0
    def put(self,item):
        with self.cv:
            if self.closed:return False
            if self.item is not None:self.dropped+=1
            self.item=item;self.cv.notify_all();return True
    def get(self,timeout=None):
        with self.cv:
            if self.item is None and not self.closed:self.cv.wait(timeout)
            value,self.item=self.item,None;return value
    def close(self):
        with self.cv:self.closed=True;self.cv.notify_all()

class RateMeter:
    def __init__(self,window=5.):
        self.events=deque();self.window=window;self.lock=Lock()
    def add(self):
        now=monotonic()
        with self.lock:
            self.events.append(now)
            while self.events and now-self.events[0]>self.window:self.events.popleft()
    def hz(self):
        with self.lock:
            return (len(self.events)-1)/max(self.events[-1]-self.events[0],1e-6) if len(self.events)>=2 else None

class CaptureWorker:
    def __init__(self,source,on_frame=None):
        self.source=source;self.on_frame=on_frame;self.slot=LatestSlot();self.stop_event=Event()
        self.thread=None;self.error=None;self.frames=0;self.sdk_dropped=0;self.rate=RateMeter()
    def start(self):
        self.thread=Thread(target=self._run,name='rgbd-capture');self.thread.start()
    def _run(self):
        while not self.stop_event.is_set():
            try:frame=self.source.read(latest=True)
            except Exception as exc:
                if not self.stop_event.is_set():self.error=f'{type(exc).__name__}: {exc}'
                break
            self.frames+=1;self.sdk_dropped+=frame.timestamps.get('queued_frames_discarded',0)
            identity=frame_id(frame,self.frames)
            if self.on_frame:self.on_frame(identity,frame)
            self.slot.put((identity,frame));self.rate.add()
        self.slot.close()
    def close(self,timeout=5):
        self.stop_event.set();self.slot.close();self.source.stop()
        if self.thread:
            self.thread.join(timeout)
            if self.thread.is_alive():raise RuntimeError('Capture worker did not stop')
    @property
    def dropped(self):return self.sdk_dropped+self.slot.dropped

class FastPerception:
    """Color/outline/robust visible depth with the existing pixel tracker."""
    def __init__(self):
        self.detector=UnifiedBatteryDetector();self.classifier=ColorClassifier()
        self.tracker=Tracker(max_gap_s=1.,max_distance=80.,confirmation_hits=2,ambiguity_px=8.,allow_unknown=True)
        self.last_timestamp=None;self.rate=RateMeter();self.previous_background=None;self.table_hint=None;self.motion_history=deque(maxlen=64)
    def set_table_hint(self,plane,capture_timestamp,source_frame_id):
        if plane is None:return
        if self.table_hint is None or capture_timestamp>self.table_hint['capture_timestamp']:
            self.table_hint=dict(plane=plane,capture_timestamp=capture_timestamp,source_frame_id=source_frame_id)

    def _hint_usable(self,stamp,motion,max_age_s=2.5):
        hint=self.table_hint
        if hint is None or motion.get('stable') is not True:return False
        age=stamp-hint['capture_timestamp']
        return 0<=age<=max_age_s and all(stable for t,stable in self.motion_history if t>=hint['capture_timestamp'])

    def _motion_proxy(self,frame,masks):
        gray=cv2.resize(cv2.cvtColor(frame.color_bgr,cv2.COLOR_BGR2GRAY),(320,180))
        excluded=np.logical_or.reduce(masks) if masks else np.zeros(frame.aligned_depth.shape,bool)
        excluded=cv2.resize(np.uint8(excluded)*255,(320,180),interpolation=cv2.INTER_NEAREST)
        old=self.previous_background;self.previous_background=(gray,excluded)
        if old is None:return dict(stable=None,reason='FIRST_FRAME',reference='VISUAL_BACKGROUND_PROXY_WITHOUT_TF')
        features=cv2.goodFeaturesToTrack(old[0],maxCorners=150,qualityLevel=.01,minDistance=7,mask=255-old[1])
        if features is None or len(features)<25:return dict(stable=None,reason='INSUFFICIENT_BACKGROUND_FEATURES')
        new,status,_=cv2.calcOpticalFlowPyrLK(old[0],gray,features,None)
        if new is None:return dict(stable=None,reason='BACKGROUND_FLOW_FAILED')
        a=features.reshape(-1,2);b=new.reshape(-1,2);ok=status.ravel().astype(bool)&np.isfinite(b).all(axis=1)
        inside=(b[:,0]>=0)&(b[:,0]<320)&(b[:,1]>=0)&(b[:,1]<180)
        ix=np.where(ok&inside)[0]
        ix=ix[excluded[np.floor(b[ix,1]).astype(int),np.floor(b[ix,0]).astype(int)]==0]
        if len(ix)<25:return dict(stable=None,reason='INSUFFICIENT_BACKGROUND_FLOW_MATCHES')
        transform,inliers=cv2.estimateAffinePartial2D(a[ix],b[ix],method=cv2.RANSAC,ransacReprojThreshold=1.5)
        if transform is None:return dict(stable=None,reason='BACKGROUND_MOTION_MODEL_FAILED')
        ratio=float(inliers.mean());shift=float(np.median(np.linalg.norm(b[ix]-a[ix],axis=1))*frame.intrinsics.width/320)
        rotation=float(abs(np.rad2deg(np.arctan2(transform[1,0],transform[0,0]))))
        return dict(stable=bool(ratio>=.7 and shift<=2. and rotation<=.25),
                    median_flow_px=shift,rotation_deg=rotation,inlier_ratio=ratio,
                    reference='VISUAL_BACKGROUND_PROXY_WITHOUT_TF; OBJECT_MOTION_NOT_COMPENSATED')

    def process(self,frame,source_frame_id):
        start=perf_counter();validate_sync(frame)
        stamp=capture_time(frame)
        if self.last_timestamp is not None and stamp<=self.last_timestamp:raise ValueError('Non-increasing capture time')
        self.last_timestamp=stamp
        hsv=cv2.cvtColor(frame.color_bgr,cv2.COLOR_BGR2HSV)
        raw=self.detector.detect_batteries(frame.color_bgr,hsv=hsv)
        candidates=[];payloads=[];rejections=[]
        for d in raw:
            x,y,w,h=cv2.boundingRect(d.contour)
            if not w or not h:continue
            mask=d.mask[y:y+h,x:x+w];depth=frame.aligned_depth[y:y+h,x:x+w]
            valid=mask&(depth>0)&(depth*frame.depth_scale>=.15)&(depth*frame.depth_scale<=1.)
            ratio=float(valid.sum()/max(mask.sum(),1))
            if valid.sum()<100 or ratio<.5:
                rejections.append(dict(color=d.color,bbox=[x,y,w,h],reason='LOW_VALID_DEPTH_OR_POINTS'));continue
            vv,uu=np.where(valid)
            approx_z=float(np.median(depth[vv,uu])*frame.depth_scale)
            physical=(w*approx_z/frame.intrinsics.fx,h*approx_z/frame.intrinsics.fy)
            if min(physical)<.025 or max(physical)>.16:
                rejections.append(dict(color=d.color,bbox=[x,y,w,h],reason='OUTSIDE_FAST_CAD_SIZE_PRIOR',projected_size_m=physical));continue
            # Cheap local height check. A flat woodgrain patch has approximately the
            # same depth as its surrounding table; a mixed wall/table patch has a
            # broad depth distribution. This is only a candidate filter, not a plane fit.
            if x<=2 or y<=2 or x+w>=frame.color_bgr.shape[1]-2 or y+h>=frame.color_bgr.shape[0]-2:
                rejections.append(dict(color=d.color,bbox=[x,y,w,h],reason='TRUNCATED_AT_IMAGE_EDGE'));continue
            pad=20;left=max(0,x-pad);top=max(0,y-pad)
            right=min(frame.color_bgr.shape[1],x+w+pad);bottom=min(frame.color_bgr.shape[0],y+h+pad)
            rim=frame.aligned_depth[top:bottom,left:right].copy()
            rim[y-top:y+h-top,x-left:x+w-left]=0
            rim=rim[(rim>0)&(rim*frame.depth_scale>=.15)&(rim*frame.depth_scale<=1.)]
            candidate_depth=depth[vv,uu]*frame.depth_scale
            spread=float(np.percentile(candidate_depth,90)-np.percentile(candidate_depth,10))
            height=float(np.median(rim)*frame.depth_scale-np.percentile(candidate_depth,10)) if len(rim)>=100 else None
            if spread>.15 or height is None or height<.018:
                rejections.append(dict(color=d.color,bbox=[x,y,w,h],reason='NO_LOCAL_DEPTH_ELEVATION_OR_MIXED_BACKGROUND',
                    depth_spread_m=spread,local_height_m=height));continue
            pick=np.linspace(0,len(vv)-1,min(300,len(vv)),dtype=int)
            uv=np.column_stack((uu[pick]+x,vv[pick]+y));z=depth[vv[pick],uu[pick]]*frame.depth_scale
            points=deproject_pixels(uv,z,frame.intrinsics);xyz=np.median(points,axis=0)
            color=self.classifier.classify(frame.color_bgr[y:y+h,x:x+w],mask,hsv=hsv[y:y+h,x:x+w])
            candidates.append(dict(color=color['color_label'],confidence=d.confidence,
                                   pixel_center=[float(np.mean(uu+x)),float(np.mean(vv+y))],detection_valid=True))
            payloads.append(dict(mask=d.mask,foreground=d.mask,points=points,
                                 rgb=frame.color_bgr[uv[:,1],uv[:,0],::-1].copy(),
                                 bbox=[x,y,w,h],contour=d.contour.reshape(-1,2).tolist(),
                                 depth_valid_ratio=ratio,color=color,xyz=xyz.tolist()))
        motion=self._motion_proxy(frame,[p['mask'] for p in payloads])
        self.motion_history.append((stamp,motion.get('stable') is True))
        hint_used=self._hint_usable(stamp,motion)
        if hint_used:
            n=np.asarray(self.table_hint['plane']['normal_camera_xyz']);o=np.asarray(self.table_hint['plane']['point_camera_m'])
            filtered=[];kept=[]
            for candidate,payload in zip(candidates,payloads):
                height=(o-payload['points'])@n;support=(height>=.012)&(height<=.110)
                if support.mean()<.5 or support.sum()<50:
                    rejections.append(dict(color=candidate['color'],bbox=payload['bbox'],reason='OUTSIDE_FRESH_STABLE_TABLE_FOREGROUND',
                        table_source_frame_id=self.table_hint['source_frame_id'],table_age_ms=(stamp-self.table_hint['capture_timestamp'])*1000))
                    continue
                payload['points']=payload['points'][support];payload['rgb']=payload['rgb'][support]
                payload['xyz']=np.median(payload['points'],axis=0).tolist()
                filtered.append(candidate);kept.append(payload)
            candidates,payloads=filtered,kept
        tracking=self.tracker.update(candidates,stamp)
        objects=[];views=[];used=set()
        for t in tracking['observed']:
            ordered=sorted((float(np.linalg.norm(np.asarray(c['pixel_center'])-t['pixel_center'])),i)
                           for i,c in enumerate(candidates) if i not in used)
            if not ordered:continue
            i=ordered[0][1];used.add(i);p=payloads[i];xyz=p['xyz']
            objects.append(dict(object_id=t['object_id'],color=t['color'],observed_color=candidates[i]['color'],
                color_label=candidates[i]['color'],color_confidence=p['color']['color_confidence'],
                confidence=t['confidence'],tracking_status=t['tracking_status'],tracking_reliability=t['tracking_status'].upper(),
                pixel_center=t['pixel_center'],bbox=p['bbox'],contour=p['contour'],surface_depth_z=xyz[2],
                visible_surface_center_xyz=xyz,camera_to_surface_distance=float(np.linalg.norm(xyz)),
                depth_valid_ratio=p['depth_valid_ratio'],point_count=len(p['points']),candidate_state='AMBIGUOUS',
                model_estimated_body_center_xyz=None,body_center_camera_z=None,camera_to_body_center_distance=None,
                body_yaw_candidate=None,ring_center_candidate=None,ring_orientation_candidate=None,
                hook_insertion_yaw_candidates=None,insertion_direction_candidates=None,
                body_pose_candidate=None,ring_pose_candidate=None,ring_axis_yaw_candidate=None,
                ring_feature_status='N/A',ring_feature_observability='NOT_OBSERVED',
                body_pose_valid=False,ring_pose_valid=False,ring_observed=False,
                pose_quality={},fit_status='CAD_WORKER_PENDING',cad_fresh=False,cad_result_timestamp=None,
                ambiguity=['FAST_COLOR_DEPTH_CANDIDATE','NO_INDEPENDENT_3D_VALIDATION'],
                rejection_reason='FAST_ONLY; TABLE/CAD_NOT_CURRENTLY_VERIFIED',
                capture_timestamp=stamp,source_frame_id=source_frame_id))
            views.append(p)
        elapsed=(perf_counter()-start)*1000;end=time();self.rate.add()
        result=dict(timestamp=stamp,capture_timestamp=stamp,capture_timestamp_source=frame_capture_time(frame)[1],result_timestamp=end,source_frame_id=source_frame_id,
            result_age_ms=max(0,(end-stamp)*1000),processing_latency_ms=elapsed,timestamps=frame.timestamps,
            coordinate_frame='camera_color_optical',camera_motion_proxy=motion,objects=objects,candidate_count=len(raw),
            accepted_object_count=0,diagnostic_object_count=len(objects),candidate_diagnostics=[],rejections=rejections,
            background_filter_report={'table_failure':'CACHED_DIAGNOSTIC_PLANE' if hint_used else 'NOT_RUN_FAST_PATH',
                'table_hint_used':hint_used,'table_hint_age_ms':(stamp-self.table_hint['capture_timestamp'])*1000 if hint_used else None,
                'table_hint_source_frame_id':self.table_hint['source_frame_id'] if hint_used else None},
            tracking_status={k:tracking[k] for k in ('missed','lost','ambiguous_candidate_indices')},
            timing_ms=dict(detection=elapsed,total=elapsed,cad_fit=0.,tracking=0.),
            detection_fps=self.rate.hz(),body_pose_valid=False,ring_pose_valid=False,
            position_accuracy='NOT_MEASURED',yaw_accuracy='NOT_MEASURED')
        return result,views

def associate_cad(fast,cad,max_distance=80.,ambiguity=8.):
    edges=[]
    for i,c in enumerate(cad):
        choices=sorted((float(np.linalg.norm(np.asarray(c['pixel_center'])-f['pixel_center'])),j)
                       for j,f in enumerate(fast) if c['color_label'] in ('unknown',f['color_label'])
                       or f['color_label']=='unknown')
        choices=[(d,j) for d,j in choices if d<=max_distance]
        if len(choices)>1 and choices[1][0]-choices[0][0]<ambiguity:continue
        edges.extend((d,i,j) for d,j in choices)
    output={};used_i=set();used_j=set()
    for _,i,j in sorted(edges):
        if i in used_i or j in used_j:continue
        used_i.add(i);used_j.add(j);output[fast[j]['object_id']]=cad[i]
    return output

class CADWorker:
    def __init__(self,pipeline=None):
        self.pipeline=pipeline or RGBDVisionPipeline();self.slot=LatestSlot()
        self.results=deque();self.lock=Lock();self.thread=None;self.error=None;self.rate=RateMeter()
    def start(self):
        self.thread=Thread(target=self._run,name='cad-pose-worker');self.thread.start()
    def submit(self,frame,source_frame_id,fast_objects):
        return self.slot.put((frame,source_frame_id,[dict(o) for o in fast_objects]))
    def _run(self):
        while True:
            job=self.slot.get(.1)
            if job is None:
                if self.slot.closed:return
                continue
            frame,identity,fast=job
            try:
                result,_=self.pipeline.process(frame,force_cad=True)
                message=dict(source_frame_id=identity,capture_timestamp=capture_time(frame),result_timestamp=time(),
                             matches=associate_cad(fast,result['objects']),processing_ms=result['timing_ms']['total'],
                             table_plane=result.get('background_filter_report',{}).get('table_plane'))
                with self.lock:
                    self.results.append(message)
                    while len(self.results)>8:self.results.popleft()
                self.rate.add()
            except Exception as exc:
                with self.lock:self.error=f'{type(exc).__name__}: {exc}'
    def drain(self):
        with self.lock:
            rows=list(self.results);self.results.clear();return rows
    def close(self,timeout=10):
        self.slot.close()
        if self.thread:
            self.thread.join(timeout)
            if self.thread.is_alive():raise RuntimeError('CAD worker did not stop')

def cad_for_display(result,current_frame_id,history,now=None):
    """Historic camera XYZ is labeled; never drawn as current-frame geometry."""
    now=time() if now is None else now
    for item in result['objects']:
        prior=history.get(item['object_id'])
        if prior is None:continue
        old=prior['object'];same=prior['source_frame_id']==current_frame_id
        item['historical_cad']=dict(source_frame_id=prior['source_frame_id'],
            capture_timestamp=prior['capture_timestamp'],result_timestamp=prior['result_timestamp'],
            result_age_ms=max(0,(now-prior['capture_timestamp'])*1000),
            fit_status=old['fit_status'],body_xyz=old.get('model_estimated_body_center_xyz'),
            body_yaw_candidate=old.get('body_yaw_candidate'),ring_xyz=old.get('ring_center_candidate'),
            insertion_directions=old.get('insertion_direction_candidates'),
            camera_motion_status='UNKNOWN_WITHOUT_TF',status='CURRENT_FRAME' if same else 'STALE')
        if same:
            for key in ('body_pose_candidate','ring_pose_candidate','model_estimated_body_center_xyz',
                        'body_yaw_candidate','ring_center_candidate','ring_orientation_candidate',
                        'insertion_direction_candidates','hook_insertion_yaw_candidates','ring_axis_yaw_candidate',
                        'body_center_camera_z','camera_to_body_center_distance','pose_quality','pose_hypotheses',
                        'ring_feature_status','ring_feature_observability','ring_surface_support','ring_axis_direction',
                        'ambiguity','rejection_reason','candidate_state'):
                item[key]=old.get(key)
            item['body_pose_valid']=False;item['ring_pose_valid']=False
            item['fit_status']=old['fit_status'];item['cad_fresh']=True
            item['cad_result_timestamp']=prior['capture_timestamp']
        else:item['fit_status']='CAD_STALE_DIAGNOSTIC_ONLY'
    return result
