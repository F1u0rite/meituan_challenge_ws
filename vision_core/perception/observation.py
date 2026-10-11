"""Vision-only stopped-observation diagnostics. Never authorizes robot motion."""
from collections import deque
import numpy as np

class ObservationMonitor:
    STATES=('OBSERVING','TARGET_DETECTED','POSE_AMBIGUOUS','POSE_UNVERIFIED','OBSERVATION_TIMEOUT')
    def __init__(self,timeout_s=10.,stable_seconds=1.,min_observations=3,max_age_s=3.,min_spacing_s=.5):
        self.timeout_s=timeout_s;self.stable_seconds=stable_seconds;self.min_observations=min_observations
        self.max_age_s=max_age_s;self.min_spacing_s=min_spacing_s
        self.target_id=None;self.started=None;self.stable_since=None;self.observations=deque(maxlen=20)
        self.seen=set();self.rejected_duplicates=0;self.reason='WAITING_FOR_STABLE_VIEW';self.state='OBSERVING';self.previous_target=None;self.last_ambiguous=None
    def update(self,fast_result,cad_messages):
        now=float(fast_result['capture_timestamp']);objects=fast_result['objects']
        if self.started is None:self.started=now
        target=max(objects,key=lambda o:o['bbox'][2]*o['bbox'][3],default=None)
        if target is None:
            self.stable_since=None;self.observations.clear();self.target_id=None;self.previous_target=None;self.last_ambiguous=None
            self.state='OBSERVATION_TIMEOUT' if now-self.started>=self.timeout_s else 'OBSERVING'
            self.reason='TARGET_NOT_OBSERVED';return self.result()
        if target['object_id']!=self.target_id:
            self.target_id=target['object_id'];self.started=now;self.stable_since=None
            self.observations.clear();self.seen.clear();self.previous_target=None;self.last_ambiguous=None
        pixel=np.asarray(target.get('pixel_center',[0,0]),float);depth=target.get('surface_depth_z')
        moved=False
        if self.previous_target is not None:
            moved=bool(np.linalg.norm(pixel-self.previous_target[0])>3. or (depth is not None and self.previous_target[1] is not None and abs(depth-self.previous_target[1])>.003))
        self.previous_target=(pixel.copy(),depth)
        stable=fast_result.get('camera_motion_proxy',{}).get('stable') is True and not moved
        if not stable:
            self.stable_since=None;self.observations.clear()
            self.last_ambiguous=None
            self.reason='TARGET_RELATIVE_MOTION' if moved else 'CAMERA_MOTION_OR_UNKNOWN_WITHOUT_TF';self.state='TARGET_DETECTED'
            if now-self.started>=self.timeout_s:self.state='OBSERVATION_TIMEOUT'
            return self.result()
        if self.stable_since is None:self.stable_since=now
        self.state='TARGET_DETECTED';self.reason='WAITING_FOR_DISTINCT_CAPTURED_CAD_OBSERVATIONS'
        if self.last_ambiguous is not None and now-self.last_ambiguous['timestamp']<=self.max_age_s:
            self.state='POSE_AMBIGUOUS';self.reason=self.last_ambiguous['reason']
        for message in cad_messages:
            identity=message['source_frame_id'];stamp=float(message['capture_timestamp'])
            obj=message['matches'].get(self.target_id)
            if obj is None:continue
            if identity in self.seen:self.rejected_duplicates+=1;continue
            self.seen.add(identity)
            if now-stamp>self.max_age_s or stamp>now or stamp<self.stable_since:continue
            if now-self.stable_since<self.stable_seconds:continue
            if obj.get('fit_status')!='GEOMETRIC_SUPPORT_ONLY' or obj.get('model_estimated_body_center_xyz') is None:
                self.state='POSE_AMBIGUOUS';self.reason=obj.get('fit_status','NO_BODY_SUPPORT')
                self.last_ambiguous=dict(timestamp=stamp,reason=self.reason);continue
            if obj.get('pose_quality',{}).get('reasonable_pose_count',0)>1:
                self.state='POSE_AMBIGUOUS';self.reason='MULTIPLE_REASONABLE_CAD_POSES'
                self.last_ambiguous=dict(timestamp=stamp,reason=self.reason);continue
            if self.observations and stamp-self.observations[-1]['timestamp']<self.min_spacing_s:continue
            self.last_ambiguous=None
            self.observations.append(dict(source_frame_id=identity,timestamp=stamp,
                xyz=obj['model_estimated_body_center_xyz'],yaw=obj.get('body_yaw_candidate')))
        if len(self.observations)>=self.min_observations:
            self.state='POSE_UNVERIFIED';self.reason='DISTINCT_STABLE_OBSERVATIONS; NO_INDEPENDENT_GROUND_TRUTH'
        elif now-self.started>=self.timeout_s and self.state!='POSE_AMBIGUOUS':
            self.state='OBSERVATION_TIMEOUT'
        return self.result()
    def result(self):
        samples=list(self.observations);xyz=[s['xyz'] for s in samples];yaw=[s['yaw'] for s in samples if s['yaw'] is not None]
        circular=None
        if len(yaw)>=2:
            resultant=abs(np.mean(np.exp(2j*np.asarray(yaw))))
            circular=float(np.sqrt(max(0,-2*np.log(max(resultant,1e-12))))/2)
        return dict(state=self.state,target_object_id=self.target_id,reason=self.reason,
            distinct_capture_observations=len(samples),source_frame_ids=[s['source_frame_id'] for s in samples],
            xyz_std_m=np.std(xyz,axis=0).tolist() if len(xyz)>=2 else None,yaw_circular_std_rad=circular,
            duplicate_cad_rejected=self.rejected_duplicates,
            camera_stability='VISUAL_BACKGROUND_PROXY; NOT_ROBOT_TF',
            statistical_independence='NOT_ESTABLISHED: distinct video captures may be correlated',
            body_pose_valid=False,ring_pose_valid=False,position_accuracy='NOT_MEASURED',yaw_accuracy='NOT_MEASURED')
