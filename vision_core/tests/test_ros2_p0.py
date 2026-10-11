import sys,math,json
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'src/mtc_vision_adapter'))
from mtc_vision_adapter.core import *

def c(x=0,color='red'):
    return dict(pixel_center=[x,20],color=color,confidence=.9,detection_valid=True)
def confirmed():
    t=Tracker(prefix='test');t.update([c()],100)
    return make_array(t.update([c()],101),'camera_color_optical_frame',101)
def h():return NS(frame_id='',stamp=NS(sec=0,nanosec=0))
def p():return NS(header=h(),pose=NS(position=NS(x=0,y=0,z=0),orientation=NS(x=0,y=0,z=0,w=1)))
def d():return NS(body_pose=p(),ring_pose=p())
def arr():return NS(header=h(),detections=[])

def test_single_miss_reappearance():
    t=Tracker(prefix='t');a=t.update([c()],100)['observed'][0]
    assert t.update([],100.2)['missed'][0]['object_id']==a['object_id']
    assert t.update([c(2)],100.4)['observed'][0]['object_id']==a['object_id']
def test_long_loss_new_identity():
    t=Tracker(prefix='t');a=t.update([c()],100)['observed'][0]
    result=t.update([c()],102)
    assert result['lost'][0]['object_id']==a['object_id']
    assert result['observed'][0]['object_id']!=a['object_id']
def test_multiple_same_color_and_order():
    t=Tracker(prefix='t');a=t.update([c(),c(200)],100)['observed']
    b=t.update([c(201),c(1)],100.1)['observed']
    assert a[0]['object_id']==b[1]['object_id'] and a[1]['object_id']==b[0]['object_id']
def test_color_jump_suppressed():
    t=Tracker(prefix='t');t.update([c()],100);t.update([c()],100.1)
    result=t.update([c(color='blue')],100.2)
    assert result['observed'][0]['tracking_status']=='color_uncertain'
    assert not make_array(result,'camera',100.2)['detections']
def test_ambiguity_suppressed():
    t=Tracker(prefix='t');t.update([c(0),c(10)],100)
    result=t.update([c(5)],100.1)
    assert result['ambiguous_candidate_indices']==[0] and not result['observed']
def test_duplicate_candidate_does_not_get_two_ids():
    t=Tracker(prefix='t');t.update([c()],100)
    assert not t.update([c(-1),c(1)],100.1)['observed']
@pytest.mark.parametrize('color,enum',[('red',1),('blue',2),('yellow',3),('green',4)])
def test_color_map(color,enum):
    t=Tracker(prefix='t',confirmation_hits=1)
    assert make_array(t.update([c(color=color)],100),'camera',100)['detections'][0]['color']==enum
@pytest.mark.parametrize('frame',['','/camera','IMAGE PIXEL',None])
def test_invalid_frame(frame):
    with pytest.raises(ValueError):header(frame,100)
@pytest.mark.parametrize('ts',[0,-1,float('nan'),float('inf')])
def test_invalid_timestamp(ts):
    with pytest.raises(ValueError):header('camera',ts)
def test_monotonic_tracker():
    t=Tracker();t.update([],100)
    with pytest.raises(ValueError):t.update([],100)
def test_duplicate_ids():
    tr={'observed':[dict(c(),object_id='a',tracking_status='confirmed')]*2}
    with pytest.raises(ValueError):make_array(tr,'camera',100)
def test_empty():
    assert make_array({'observed':[]},'camera',100)['detections']==[]
def test_exact_ros_conversion_and_nan():
    msg=confirmed();native=to_ros(msg,arr,d)
    assert native.header.stamp.sec==101 and native.detections[0].color==1
    assert not native.detections[0].ring_pose_valid
    assert math.isnan(native.detections[0].body_pose.pose.position.x)
    assert math.isnan(native.detections[0].ring_pose.pose.orientation.w)
    assert native.detections[0].body_pose.header.frame_id=='camera_color_optical_frame'
def test_ring_reject():
    assert motion_rejection(confirmed(),101,mode='live')=='RING_INVALID'
def test_stale_reject():
    assert motion_rejection(confirmed(),102,mode='live')=='STALE_OR_FUTURE'
def test_replay_reject_and_json_units():
    msg=confirmed()
    assert motion_rejection(msg,101)=='REPLAY_NOT_MOTION_INPUT'
    assert json.loads(json.dumps(msg,allow_nan=False))==msg
    assert all(v is None for v in msg['detections'][0]['body_pose']['pose']['position'].values())
def test_no_groundtruth_or_pixel_as_pose():
    assert confirmed()['detections'][0]['body_pose']['pose']['position']['x'] is None
def test_invalid_detection_not_tracked():
    t=Tracker();bad=c();bad['detection_valid']=False
    assert not t.update([bad],100)['observed']
def test_saved_offline_evidence():
    root=ROOT/'outputs/p0_ros2'
    if not (root/'replay.jsonl').exists():pytest.skip('run replay first')
    real=[json.loads(line) for line in (root/'replay.jsonl').read_text().splitlines()]
    syn=[json.loads(line) for line in (root/'synthetic.jsonl').read_text().splitlines()]
    assert len(real)==22 and all(r['mode']=='replay' for r in real)
    assert {d['color'] for r in syn for d in r['message']['detections']}=={1,2,3,4}
    assert all(not d['ring_pose_valid'] for r in real+syn for d in r['message']['detections'])
