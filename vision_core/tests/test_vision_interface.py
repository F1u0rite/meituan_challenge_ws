import json
import numpy as np
from perception.vision_interface import finalize_interface
from perception.dual_rate_rgbd import cad_for_display

def result(item,stamp=1):
    return dict(objects=[item],capture_timestamp=stamp,source_frame_id='frame',
                result_timestamp=2,capture_timestamp_source='HOST_UTC')

def test_null_pose_and_confirmed_output_are_distinct():
    r=finalize_interface(result(dict(candidate_state='AMBIGUOUS',visible_surface_center_xyz=[0,0,.4])))
    o=r['objects'][0]
    assert o['body_center_candidate_xyz'] is None and o['body_orientation_candidate'] is None
    assert not r['confirmed_objects'] and not o['body_pose_valid'] and not o['ring_pose_valid']
    assert o['unit_position']=='m' and o['unit_angle']=='rad'
    assert o['capture_timestamp']==1 and o['detection_result_timestamp']==2
    json.dumps(r,allow_nan=False)

def test_complete_rotation_and_bidirectional_cad_candidates():
    t=np.eye(4);t[:3,3]=[.1,.2,.4]
    r=finalize_interface(result(dict(candidate_state='CONFIRMED_OBJECT',body_pose_candidate=t.tolist(),
        model_estimated_body_center_xyz=[.1,.2,.4],ring_center_candidate=[.1,.2,.435],
        insertion_direction_candidates=[[1,0,0],[-1,0,0]])))
    o=r['objects'][0]
    assert o['body_orientation_candidate']==np.eye(3).tolist()
    assert o['geometry_source']=='CAD_PREDICTED' and len(o['hook_insertion_direction_candidates'])==2
    assert len(r['confirmed_objects'])==1 and not o['ring_pose_valid']
    json.dumps(r,allow_nan=False)

def test_invalid_rotation_is_not_fabricated():
    r=finalize_interface(result(dict(body_pose_candidate=np.zeros((4,4)).tolist())))
    assert r['objects'][0]['body_orientation_candidate'] is None

def test_expired_same_frame_clears_current_candidate_and_keeps_history():
    o=dict(object_id='A',body_pose_candidate=np.eye(4).tolist(),model_estimated_body_center_xyz=[0,0,.4],
           ring_center_candidate=[0,0,.435],cad_fresh=True)
    prior=dict(source_frame_id='frame',capture_timestamp=1,result_timestamp=2,
               object=dict(fit_status='WEAK',model_estimated_body_center_xyz=[0,0,.4]))
    r=cad_for_display(result(o),'frame',{'A':prior},now=5)
    assert o['stale_status']=='STALE' and not o['cad_fresh']
    assert o['body_center_candidate_xyz'] is None and o['ring_center_candidate'] is None
    assert o['historical_cad']['body_xyz']==[0,0,.4]
    assert o['CAD_result_source_frame']=='frame' and o['CAD_result_timestamp']==2
    assert o['CAD_result_age_ms']==4000
    json.dumps(r,allow_nan=False)

def test_reused_frame_id_with_different_capture_is_stale():
    o=dict(object_id='A');p=dict(source_frame_id='frame',capture_timestamp=1,result_timestamp=2,
                              object=dict(fit_status='WEAK'))
    cad_for_display(result(o,stamp=1.1),'frame',{'A':p},now=2)
    assert o['stale_status']=='STALE'
