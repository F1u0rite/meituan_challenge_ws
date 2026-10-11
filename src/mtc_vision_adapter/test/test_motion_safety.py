import copy
import json
from pathlib import Path
import pytest
from mtc_vision_adapter.core import motion_rejection, to_ros

NOW=1700000000.25
TRUST={'camera_optical_frame','base_link'}

def message():
    header={'frame_id':'camera_optical_frame','stamp':{'sec':1700000000,'nanosec':250000000}}
    pose={'header':copy.deepcopy(header),'pose':{'position':{'x':.1,'y':.2,'z':.3},'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}}
    return {'header':header,'detections':[{'object_id':'battery-1','color':1,'confidence':.9,'ring_pose_valid':True,'body_pose':copy.deepcopy(pose),'ring_pose':copy.deepcopy(pose)}]}

def check(msg,**kw):
    options=dict(mode='live',trusted_frames=TRUST,target_frame='base_link',tf_validator=lambda *args:True)
    options.update(kw)
    return motion_rejection(msg,NOW,**options)

def test_default_live_configuration_rejects():
    assert motion_rejection(message(),NOW,mode='live')=='UNTRUSTED_FRAME'

@pytest.mark.parametrize('frame',['unregistered_frame','synthetic_unregistered_color_optical_frame','',None])
def test_unknown_frame_rejects_before_tf(frame):
    m=message();m['header']['frame_id']=frame
    for name in ('body_pose','ring_pose'):m['detections'][0][name]['header']['frame_id']=frame
    assert check(m,tf_validator=lambda *args:pytest.fail('unknown frame reached TF'))=='UNTRUSTED_FRAME'

@pytest.mark.parametrize('target',[None,'unknown_target',''])
def test_untrusted_target_rejects(target):
    assert check(message(),target_frame=target)=='UNTRUSTED_FRAME'

@pytest.mark.parametrize('validator',[None,False])
def test_missing_tf_rejects(validator):
    assert check(message(),tf_validator=validator)=='TF_UNVERIFIED'

@pytest.mark.parametrize('result',[False,None,1,'verified',object()])
def test_unverified_tf_rejects(result):
    assert check(message(),tf_validator=lambda *args:result)=='TF_UNVERIFIED'

@pytest.mark.parametrize('error',[LookupError,TimeoutError,RuntimeError])
def test_tf_exception_rejects(error):
    def verify(*args):raise error('TF unavailable')
    assert check(message(),tf_validator=verify)=='TF_UNVERIFIED'

def test_verified_tf_receives_capture_timestamp():
    calls=[]
    def verify(*args):calls.append(args);return True
    m=message()
    assert check(m,tf_validator=verify)=='NO_REJECTION'
    assert calls==[('camera_optical_frame','base_link',m['header']['stamp'])]

def test_same_frame_still_requires_verification():
    assert check(message(),target_frame='camera_optical_frame',tf_validator=None)=='TF_UNVERIFIED'

@pytest.mark.parametrize('mode',['replay','synthetic'])
def test_offline_not_motion_input(mode):
    assert check(message(),mode=mode)=='REPLAY_NOT_MOTION_INPUT'

@pytest.mark.parametrize('value',[None,float('nan'),float('inf'),-float('inf')])
@pytest.mark.parametrize('pose',['body_pose','ring_pose'])
def test_invalid_pose_values_reject(value,pose):
    m=message();m['detections'][0][pose]['pose']['position']['x']=value
    assert check(m)=='POSE_INVALID'

@pytest.mark.parametrize('flag',[False,None,'true',1])
def test_ring_flag_requires_boolean_true(flag):
    m=message();m['detections'][0]['ring_pose_valid']=flag
    assert check(m)=='RING_INVALID'

@pytest.mark.parametrize('sec',[1699999999,1700000001])
def test_stale_future_reject(sec):
    m=message();m['header']['stamp']['sec']=sec
    assert check(m)=='STALE_OR_FUTURE'

def test_malformed_pose_rejects():
    m=message();del m['detections'][0]['body_pose']['pose']['position']['z']
    assert check(m)=='MALFORMED_INPUT'

@pytest.mark.parametrize('nanosec',[-1,1000000000,float('nan')])
def test_malformed_stamp_rejects(nanosec):
    m=message();m['header']['stamp']['nanosec']=nanosec
    assert check(m)=='MALFORMED_INPUT'

def test_nan_age_limit_rejects():
    assert check(message(),max_age_s=float('nan'))=='MALFORMED_INPUT'

def test_pose_header_mismatch():
    m=message();m['detections'][0]['body_pose']['header']['frame_id']='other'
    assert check(m)=='POSE_HEADER_MISMATCH'

def test_duplicate_id():
    m=message();m['detections'].append(copy.deepcopy(m['detections'][0]))
    assert check(m)=='DUPLICATE_ID'

def test_zero_quaternion():
    m=message();m['detections'][0]['ring_pose']['pose']['orientation']['w']=0.
    assert check(m)=='QUATERNION_INVALID'

def test_original_synthetic_records_rejected():
    path=Path(__file__).parents[1]/'examples/synthetic.jsonl'
    for line in path.read_text().splitlines():
        r=json.loads(line)
        assert r['body_pose_valid'] is False
        assert motion_rejection(r['message'],NOW)=='REPLAY_NOT_MOTION_INPUT'
