"""UI refresh must not discard CAD table hints or relax their transfer gates."""
from types import SimpleNamespace
from perception.dual_rate_rgbd import FastPerception
from scripts.dual_rate_runner import consume_cad_results


def test_refresh_consumption_preserves_same_frame_table_and_all_messages():
    fast=FastPerception()
    plane=dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5])
    message=dict(table_plane=plane,capture_timestamp=10.,source_frame_id='rgbd10')
    # Previously refresh only drained the message; fast.table_hint stayed None.
    assert fast.table_hint_evidence(10.,dict(stable=None),'rgbd10')['reasons']==['TABLE_HINT_MISSING']
    assert consume_cad_results(fast,SimpleNamespace(drain=lambda:[message]))==[message]
    evidence=fast.table_hint_evidence(10.,dict(stable=None),'rgbd10')
    assert evidence['usable'] and evidence['transfer']=='NOT_REQUIRED_CURRENT_FRAME'
    assert fast.table_hint['plane']==plane
    # Evidence from another frame still needs stable observed motion.
    assert not fast.table_hint_evidence(10.1,dict(stable=None),'rgbd11')['usable']
    assert not fast.table_hint_evidence(13.,dict(stable=True),'rgbd13')['usable']


def test_missing_or_out_of_order_plane_cannot_erase_new_table():
    fast=FastPerception();new=dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5])
    fast.set_table_hint(new,10.,'new')
    messages=[dict(table_plane=None,capture_timestamp=11.,source_frame_id='absent'),
        dict(table_plane={'old':True},capture_timestamp=9.,source_frame_id='old')]
    consume_cad_results(fast,SimpleNamespace(drain=lambda:messages))
    assert fast.table_hint['source_frame_id']=='new' and fast.table_hint['plane']==new
