import numpy as np
import pytest
from perception.vision_interface import finalize_interface
from perception.dual_rate_rgbd import FastPerception,cad_for_display
from perception.live_vision_protocol import vision_message,validate_message

def record(ambiguity):
    return dict(result_timestamp=10.1,capture_timestamp=10,source_frame_id='current',capture_timestamp_source='HOST_UTC',
      timestamps=dict(host_wall_ns=10000000000,color_sdk_ms=100.,color_timestamp_domain='hardware_clock'),
      objects=[dict(object_id='A',color_label='red',confidence=.9,candidate_state='CONFIRMED_OBJECT',
        confirmation=dict(reasons=[]),ambiguity=ambiguity,body_pose_candidate=np.eye(4).tolist(),
        model_estimated_body_center_xyz=[.1,.2,.4],ring_center_candidate=[.1,.2,.435],
        ring_orientation_candidate=np.eye(3).tolist(),insertion_direction_candidates=[[1,0,0],[-1,0,0]])])

def wire(r):
    return vision_message(r,camera_frame_id='camera_color_optical',namespace='test',mode='test')['objects'][0]

def test_unverified_pose_does_not_mean_unconfirmed_detection():
    r=finalize_interface(record(['FAST_COLOR_DEPTH_CANDIDATE','NO_INDEPENDENT_3D_VALIDATION']))
    o=r['objects'][0];w=wire(r)
    assert not o['detection_ambiguity'] and not o['pose_ambiguity']
    assert o['pose_validation_reasons']==['NO_INDEPENDENT_3D_VALIDATION']
    assert w['detection_status']=='CONFIRMED' and not w['ambiguity'] and not w['stale']
    assert not w['body_pose_valid'] and not w['ring_pose_valid']

def test_unresolved_pose_candidates_are_withheld_without_hiding_directions_locally():
    r=finalize_interface(record(['180_DEG_DIRECTION','NO_INDEPENDENT_3D_VALIDATION']))
    w=wire(r)
    assert not w['ambiguity'] and w['body_position_candidate_m'] is None
    assert w['ring_orientation_candidate'] is None and w['insertion_direction_candidates'] is None
    assert len(r['objects'][0]['hook_insertion_direction_candidates'])==2
    assert r['objects'][0]['pose_ambiguity']

def test_current_detection_with_old_cad_is_not_stale_object():
    r=record(['NO_INDEPENDENT_3D_VALIDATION'])
    old=dict(source_frame_id='old',capture_timestamp=9,result_timestamp=9.5,
        object=dict(fit_status='WEAK',ambiguity=['180_DEG_DIRECTION'],model_estimated_body_center_xyz=[1,2,3]))
    cad_for_display(r,'current',{'A':old},now=10.2)
    o=r['objects'][0];w=wire(r)
    assert o['cad_stale'] and not o['detection_stale']
    assert o['pose_ambiguity'] and not o['detection_ambiguity']
    assert not w['stale'] and w['body_position_candidate_m'] is None

def test_detection_expiry_and_missing_capture_clock_remain_conservative():
    r=record([]);r['result_timestamp']=13
    assert wire(finalize_interface(r))['stale']
    r=record([]);r.pop('timestamps');r.pop('capture_timestamp_source')
    o=finalize_interface(r)['objects'][0]
    assert o['detection_stale'] is None and wire(r)['stale']

@pytest.mark.parametrize('color',['red','blue','yellow','green'])
def test_current_plane_no_transfer_still_requires_shape_color_and_association(color):
    f=FastPerception();f.body_dimensions=dict(long=80,width=70,height=50)
    f.set_table_hint(dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5]),1,'current')
    evidence=f.table_hint_evidence(1,dict(stable=None,reason='FIRST_FRAME'),'current')
    assert evidence['usable'] and evidence['transfer']=='NOT_REQUIRED_CURRENT_FRAME'
    assert not f.table_hint_evidence(1,dict(stable=False),'different')['usable']
    x,y=np.meshgrid(np.linspace(-.04,.04,20),np.linspace(-.035,.035,20))
    p=dict(points=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,.45)]),
        color=dict(color_label=color),contour_fill=.95,depth_valid_ratio=1)
    assert f.confirmation_evidence(p,True)['eligible']
    p['contour_fill']=.4
    assert 'INCOMPLETE_CONTOUR_SUPPORT' in f.confirmation_evidence(p,True)['reasons']
    r=record([]);r['objects'][0]['candidate_state']='AMBIGUOUS'
    r['objects'][0]['confirmation']['reasons']=['ASSOCIATION_NOT_ESTABLISHED']
    assert wire(finalize_interface(r))['ambiguity']

def test_cross_frame_table_motion_and_age_gates_unchanged_with_detailed_reasons():
    f=FastPerception();f.set_table_hint(dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5]),1,'old')
    f.motion_history.extend([(1.1,False),(1.2,True)])
    e=f.table_hint_evidence(1.2,dict(stable=True),'new')
    assert not e['usable'] and 'BACKGROUND_NOT_STABLE_SINCE_TABLE_CAPTURE' in e['reasons']
    assert 'TABLE_HINT_EXPIRED' in f.table_hint_evidence(4,dict(stable=True),'new')['reasons']
    assert 'TABLE_HINT_FROM_FUTURE' in f.table_hint_evidence(.5,dict(stable=True),'old')['reasons']

def test_legacy_records_keep_old_protocol_mapping():
    r=record(['NO_INDEPENDENT_3D_VALIDATION']);r['objects'][0]['stale_status']='STALE'
    w=wire(r)
    assert w['ambiguity'] and w['stale']
