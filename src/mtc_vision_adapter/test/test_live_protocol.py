import copy
import json
import math
import time
import pytest
from mtc_vision_adapter.live_protocol import (
    decode, validate, ProtocolError, Session, diagnostic_array, MAX_BYTES)
from live_test_support import frame, battery


def test_diagnostic_conversion_never_promotes_candidates():
    message=frame(objects=[battery(body_position_candidate_m=[.1,.2,.3],
                                   body_orientation_candidate=[0.,0.,0.,1.],
                                   ring_position_candidate_m=[.2,.3,.4],
                                   ring_orientation_candidate=[0.,0.,0.,1.])])
    data=diagnostic_array(validate(message))
    assert data['header']['stamp']==message['windows_capture_wall_time']
    d=data['detections'][0]
    assert d['ring_pose_valid'] is False
    for name in ('body_pose','ring_pose'):
        assert all(v is None for group in d[name]['pose'].values() for v in group.values())


@pytest.mark.parametrize('changes',[
    {'protocol_version':2},{'protocol_version':True},
    {'sequence_id':True},{'sequence_id':-1},{'sequence_id':1.2},{'sequence_id':9007199254740992},
    {'mode':'replay'},{'mode':[]},{'camera_frame_id':'../bad'},{'source_frame_id':None},
    {'timestamp_domain':'device_unknown'},{'capture_timestamp_raw':True},
    {'capture_timestamp_raw':float('inf')},{'objects':{}},
    {'windows_capture_wall_time':{'sec':1,'nanosec':1000000000}},
    {'windows_capture_wall_time':{'sec':True,'nanosec':0}},
    {'command':'move'},{'mode':'live','timestamp_domain':'simulated'},
])
def test_invalid_frames_reject(changes):
    message=frame();message.update(changes)
    with pytest.raises(ProtocolError):
        decode(json.dumps(message).encode())


@pytest.mark.parametrize('changes',[
    {'object_id':''},{'color':'PURPLE'},{'color':1},
    {'confidence':True},{'confidence':-1},{'confidence':1.1},
    {'detection_status':'confirmed'},{'ambiguity':1},{'stale':'false'},
    {'body_pose_valid':True},{'ring_pose_valid':True},
    {'surface_depth_m':0},{'body_yaw_candidate_rad':4.},
    {'body_position_candidate_m':[1,2]},{'body_position_candidate_m':[1,2,None]},
    {'body_orientation_candidate':[0,0,0,0]},
    {'ring_orientation_candidate':[0,0,float('nan'),1]},
    {'insertion_direction_candidates':[[0,0,0]]},
    {'insertion_direction_candidates':[None]},{'control':{}},
])
def test_invalid_objects_reject(changes):
    with pytest.raises(ProtocolError):
        decode(json.dumps(frame(objects=[battery(**changes)])).encode())


@pytest.mark.parametrize('payload',[
    b'{',b'\xff',b'{"protocol_version":1,"protocol_version":1}',
    b'[]',b'null',b'NaN',b'Infinity',b'-Infinity',
    b'['*2000+b'0'+b']'*2000, b' '* (MAX_BYTES+1),
])
def test_bad_json_and_lengths_do_not_escape(payload):
    with pytest.raises(ProtocolError):decode(payload)


def test_duplicate_and_too_many_objects():
    for objects in ([battery(),battery()], [battery(str(i)) for i in range(129)]):
        with pytest.raises(ProtocolError):validate(frame(objects=objects))


def test_required_fields_are_exact():
    message=frame()
    for key in list(message):
        candidate=copy.deepcopy(message);del candidate[key]
        with pytest.raises(ProtocolError):validate(candidate)
    obj=battery()
    for key in list(obj):
        candidate=copy.deepcopy(obj);del candidate[key]
        with pytest.raises(ProtocolError):validate(frame(objects=[candidate]))


def test_filtered_targets_stay_diagnostic():
    objects=[battery('ok'), battery('unknown','UNKNOWN'),
             battery('amb',detection_status='AMBIGUOUS'),battery('raw',detection_status='RAW_CANDIDATE'),
             battery('reject',detection_status='REJECTED'),battery('stale',stale=True),
             battery('uncertain',ambiguity=True),battery('low',confidence=.1)]
    assert [d['object_id'] for d in diagnostic_array(validate(frame(objects=objects)))['detections']]==['ok']


def test_session_order_reconnect_unknown_and_untrusted_age():
    now=time.time_ns()
    assert Session().observe(frame(1),now)=='SEQUENCE_START'
    session=Session()
    m=frame(0)
    assert session.observe(m,time.time_ns()) is None
    assert session.observe(m,time.time_ns())=='SEQUENCE_ORDER'
    assert session.observe(frame(2,capture_timestamp_raw=0.),time.time_ns())=='CAPTURE_ORDER'
    assert Session().observe(frame(0),time.time_ns()) is None
    assert Session().observe(frame(0,timestamp_domain='unknown',capture_timestamp_raw=None),now)=='TIME_UNKNOWN'
    m=frame();m['windows_capture_wall_time']['sec']-=60
    assert Session().observe(m,time.time_ns())=='WALL_OLD'
    m=frame();m['windows_capture_wall_time']['sec']+=60
    assert Session().observe(m,time.time_ns())=='WALL_FUTURE'


def test_epoch_bounds_and_huge_integer_reject():
    m=frame(capture_timestamp_raw=10**400)
    with pytest.raises(ProtocolError):decode(json.dumps(m).encode())
    m=frame(windows_capture_wall_time={'sec':2147483648,'nanosec':0})
    with pytest.raises(ProtocolError):validate(m)
