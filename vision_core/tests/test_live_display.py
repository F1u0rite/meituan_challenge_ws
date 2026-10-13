"""Offline display admission, rejected RGB preview and table lifecycle tests."""
from types import SimpleNamespace
import numpy as np
from scripts.live_display import display_snapshot
from perception.dual_rate_rgbd import CaptureWorker,FastPerception
from threading import Event


def test_rejected_pair_still_reaches_preview_never_fast_or_recorder():
    stamp=dict(color_frame_number=1,depth_frame_number=1,color_sdk_ms=10.,depth_sdk_ms=1000.,
        color_timestamp_domain='global_time',depth_timestamp_domain='global_time')
    values=iter([SimpleNamespace(timestamps=stamp)])
    source=SimpleNamespace(read=lambda **kw:next(values),stop=lambda:None)
    offered=[];preview=[]
    worker=CaptureWorker(source,on_frame=lambda *a:offered.append(a),on_preview=lambda *a:preview.append(a))
    worker.start();worker.thread.join(2);worker.close()
    assert len(preview)==1 and not preview[0][2]['qualified']
    assert offered==[] and worker.slot.get(.01) is None


def test_historical_annotation_never_transfers_geometry_or_current_status():
    frame=SimpleNamespace(timestamps={'host_utc':'1970-01-01T00:00:10+00:00'})
    obj=dict(candidate_state='CONFIRMED_OBJECT',body_pose_candidate=np.eye(4).tolist(),ring_pose_candidate=None)
    result=dict(source_frame_id='old',capture_timestamp=10.,objects=[obj])
    shown,views=display_snapshot(frame,'new',dict(qualified=True),(result,[{'mask':1}]),now=10.2)
    assert views==[] and not shown['annotation_is_current']
    assert shown['annotation_source_frame_id']=='old' and abs(shown['annotation_age_ms']-200)<1e-6
    assert shown['objects'][0]['candidate_state'].startswith('HISTORY')
    assert shown['objects'][0]['body_pose_candidate'] is None
    assert result['objects'][0]['body_pose_candidate'] is not None
    rejected,_=display_snapshot(frame,'old',dict(qualified=False),(result,[]),now=10.2)
    assert not rejected['annotation_is_current']


def test_current_source_still_requires_age_and_sync_admission():
    frame=SimpleNamespace(timestamps={'host_utc':'1970-01-01T00:00:10+00:00'})
    result=dict(source_frame_id='same',capture_timestamp=10.,objects=[])
    assert display_snapshot(frame,'same',dict(qualified=True),(result,[]),now=10.1)[0]['annotation_is_current']
    assert not display_snapshot(frame,'same',dict(qualified=True),(result,[]),now=13.)[0]['annotation_is_current']


def test_table_lifecycle_keeps_original_expiry_and_motion_checks():
    fast=FastPerception();fast.set_table_hint(dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5]),10.,'source',
        plane_generated_host_s=10.2,cad_completed_host_s=11.5)
    fast.motion_history.extend([(10.1,True),(10.2,True)])
    stable=fast.table_hint_evidence(10.2,dict(stable=True),'next')
    assert stable['usable'] and stable['expiry_capture_timestamp']==12.5
    assert stable['plane_generated_host_s']==10.2 and stable['cad_completed_host_s']==11.5
    assert not fast.table_hint_evidence(12.6,dict(stable=True),'later')['usable']
    fast.motion_history.append((10.3,False))
    assert not fast.table_hint_evidence(10.3,dict(stable=True),'moved')['usable']


def test_table_is_available_before_slow_cad_finishes():
    from perception.dual_rate_rgbd import CADWorker
    from input.realsense_rgbd_source import RGBDFrame
    from perception.rgbd_geometry import CameraIntrinsics
    ready=Event();release=Event()
    plane=dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5])
    class Pipeline:
        def process(self,frame,force_cad=True):
            import time
            self.on_table_plane(plane,time.time());ready.set();assert release.wait(3)
            return dict(objects=[],timing_ms={'total':500.},background_filter_report={'table_plane':plane}),[]
    frame=RGBDFrame(np.zeros((8,8,3),np.uint8),np.full((8,8),500,np.uint16),
        CameraIntrinsics(8,8,10,10,4,4),.001,{'host_utc':'2026-10-12T12:00:00+00:00'}, {})
    worker=CADWorker(Pipeline());worker.start()
    try:
        worker.submit(frame,'fixture',[]);assert ready.wait(2)
        assert worker.drain()==[]
        tables=worker.drain_table();assert len(tables)==1 and tables[0]['kind']=='TABLE_READY'
        assert tables[0]['table_plane']==plane and tables[0]['matches']=={}
    finally:release.set();worker.close()
    assert worker.completed_jobs==1 and worker.error is None
