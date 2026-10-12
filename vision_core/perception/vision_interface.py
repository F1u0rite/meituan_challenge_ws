"""Explicit non-ROS diagnostic aliases; no pose inference or validity promotion."""
import numpy as np
from datetime import datetime, timezone
import math

def finalize_interface(result):
    result['detection_result_timestamp']=result.get('result_timestamp')
    result['clock_domains']={'capture':result.get('capture_timestamp_source','UNAVAILABLE'),
        'completion':'SYSTEM_UNIX_SECONDS','sdk':'SEE_TIMESTAMPS_WITH_DOMAIN', 'ros':'NOT_MAPPED'}
    for item in result['objects']:
        state=item.get('candidate_state','AMBIGUOUS')
        item['detection_status']=state if state in ('RAW_CANDIDATE','AMBIGUOUS','CONFIRMED_OBJECT','REJECTED') else 'AMBIGUOUS'
        legacy=item.get('ambiguity',[])
        legacy=legacy if isinstance(legacy,list) else (['LEGACY_UNRESOLVED'] if legacy else [])
        pose_flags=[v for v in legacy if v not in ('FAST_COLOR_DEPTH_CANDIDATE','NO_INDEPENDENT_3D_VALIDATION')]
        if item.get('historical_cad'):
            pose_flags+=item['historical_cad'].get('pose_ambiguity_reasons',[])
        item['pose_ambiguity_reasons']=list(dict.fromkeys(pose_flags))
        item['pose_ambiguity']=bool(item['pose_ambiguity_reasons'])
        item['pose_validation_reasons']=['NO_INDEPENDENT_3D_VALIDATION']
        item['detection_ambiguity']=item['detection_status']!='CONFIRMED_OBJECT'
        item['detection_ambiguity_reasons']=list(item.get('confirmation',{}).get('reasons',[]))
        if item['detection_ambiguity'] and not item['detection_ambiguity_reasons']:
            item['detection_ambiguity_reasons']=['TARGET_NOT_CONFIRMED']
        times=result.get('timestamps',{})
        captured=None
        try:
            if type(times.get('host_wall_ns')) is int:
                captured=times['host_wall_ns']/1e9
            elif times.get('host_utc'):
                instant=datetime.fromisoformat(times['host_utc'].replace('Z','+00:00'))
                if instant.tzinfo is not None:captured=instant.timestamp()
            elif result.get('capture_timestamp_source')=='HOST_UTC':
                captured=result.get('capture_timestamp')
        except (ValueError,TypeError):pass
        finished=result.get('result_timestamp')
        known=type(captured) in (int,float) and type(finished) in (int,float) and math.isfinite(captured) and math.isfinite(finished)
        age=finished-captured if known else None
        item['detection_age_ms']=age*1000 if age is not None else None
        item['detection_stale']=(age<0 or age>result.get('max_detection_age_s',2.5)) if age is not None else None
        item['detection_freshness_source']='CAPTURE_HOST_WALL_TO_RESULT_SYSTEM_WALL' if known else 'UNKNOWN'
        item['cad_stale']=item.get('cad_stale',item.get('stale_status')=='STALE')
        item['frame_id']=item.get('coordinate_frame',result.get('coordinate_frame','camera_color_optical'))
        item['coordinate_frame_status']='OPTICAL_AXIS_CONVENTION; NO_TF_CALIBRATION'
        item['unit_position']='m';item['unit_angle']='rad'
        item['capture_timestamp']=result.get('capture_timestamp')
        item['capture_timestamp_source']=result.get('capture_timestamp_source','UNAVAILABLE')
        item['source_frame_id']=result.get('source_frame_id')
        item['detection_result_timestamp']=result.get('detection_result_timestamp')
        item['body_center_candidate_xyz']=item.get('model_estimated_body_center_xyz')
        pose=item.get('body_pose_candidate')
        item['body_orientation_candidate']=None
        if pose is not None:
            matrix=np.asarray(pose,dtype=float)
            if matrix.shape==(4,4) and np.isfinite(matrix).all():
                rotation=matrix[:3,:3]
                if np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) and np.isclose(np.linalg.det(rotation),1):
                    item['body_orientation_candidate']=rotation.tolist()
        item['body_orientation_representation']='ROTATION_MATRIX' if item['body_orientation_candidate'] is not None else None
        item['hook_insertion_direction_candidates']=item.get('insertion_direction_candidates')
        item.setdefault('ring_axis_direction',None)
        item['geometry_source']='CAD_PREDICTED' if item.get('ring_center_candidate') is not None else None
        item.setdefault('stale_status','NO_CURRENT_CAD' if not item.get('cad_fresh') else 'CURRENT_FRAME')
        item['body_pose_valid']=False;item['ring_pose_valid']=False
    result['confirmed_objects']=[o for o in result['objects'] if o['detection_status']=='CONFIRMED_OBJECT']
    result['body_pose_valid']=False;result['ring_pose_valid']=False
    return result
