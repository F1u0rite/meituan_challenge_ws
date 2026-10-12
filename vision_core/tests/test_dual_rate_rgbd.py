"""Concurrency, timestamps and safety tests for the opt-in dual-rate path."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from perception.dual_rate_rgbd import (LatestSlot, CaptureWorker, FastPerception,
    CADWorker, cad_for_display, validate_sync, associate_cad)
from scripts.dual_rate_runner import merge_cad_history
from real_data.continuous_rgbd import ContinuousRGBDRecorder
from real_data.rgbd_recording import load_rgbd_sample

ROOT=Path(__file__).resolve().parents[1]
SAMPLE=ROOT/'outputs/live_rgbd_integration/run_1be68a65/user_saves/sample_0001'


def test_latest_slot_replaces_and_counts():
    slot=LatestSlot()
    assert slot.put(('rgb1','depth1'))
    assert slot.put(('rgb2','depth2'))
    assert slot.get(.01)==('rgb2','depth2')
    assert slot.dropped==1
    slot.close()
    assert not slot.put(3)


@pytest.mark.skipif(not SAMPLE.is_dir(), reason='Private Windows RGB-D fixture unavailable on Ubuntu')
def test_sync_rejects_misalignment_and_large_skew():
    f=load_rgbd_sample(SAMPLE)
    validate_sync(f)
    bad=SimpleNamespace(**vars(f))
    bad.aligned_depth=f.aligned_depth[:20]
    with pytest.raises(ValueError):validate_sync(bad)
    bad=SimpleNamespace(**vars(f))
    bad.timestamps=dict(f.timestamps,depth_sdk_ms=f.timestamps['color_sdk_ms']+500)
    with pytest.raises(ValueError):validate_sync(bad)


@pytest.mark.skipif(not SAMPLE.is_dir(), reason='Private Windows RGB-D fixture unavailable on Ubuntu')
def test_fast_red_and_capture_identity_on_real_frame():
    f=load_rgbd_sample(SAMPLE)
    fast=FastPerception()
    result,views=fast.process(f,'color_468_depth_465')
    assert result['source_frame_id']=='color_468_depth_465'
    assert len([o for o in result['objects'] if o['color_label']=='red'])==1
    assert len(result['objects'])==len(views)
    assert all(not o['body_pose_valid'] and not o['ring_pose_valid'] for o in result['objects'])


@pytest.mark.skipif(not SAMPLE.is_dir(), reason='Private Windows RGB-D fixture unavailable on Ubuntu')
def test_tracker_continuity_on_continuous_capture_times():
    f=load_rgbd_sample(SAMPLE)
    fast=FastPerception();a,_=fast.process(f,'frame_a')
    t=dict(f.timestamps);t['host_utc']='2026-10-11T11:57:17.518923+00:00';t['color_sdk_ms']+=100;t['depth_sdk_ms']+=100
    g=SimpleNamespace(**vars(f));g.timestamps=t
    b,_=fast.process(g,'frame_b')
    red=lambda r:next(o['object_id'] for o in r['objects'] if o['color_label']=='red')
    assert red(a)==red(b)


def test_cad_association_one_to_one_and_color():
    fast=[dict(object_id='A',color_label='red',pixel_center=[10,10]),
          dict(object_id='B',color_label='blue',pixel_center=[100,100])]
    cad=[dict(color_label='blue',pixel_center=[99,100]),
         dict(color_label='red',pixel_center=[12,10])]
    assert set(associate_cad(fast,cad))=={'A','B'}
    assert associate_cad(fast,[dict(color_label='green',pixel_center=[10,10])])=={}


def test_out_of_order_cad_result_cannot_replace_newer():
    history={}
    def msg(timestamp,source):
        return dict(capture_timestamp=timestamp,result_timestamp=timestamp+1,
                    source_frame_id=source,processing_ms=10,
                    matches={'A':dict(fit_status='WEAK_XY_YAW_CONSTRAINT')})
    assert merge_cad_history(history,[msg(3,'new')])[1]==0
    assert merge_cad_history(history,[msg(2,'old')])[1]==1
    assert history['A']['source_frame_id']=='new'


def test_stale_camera_pose_is_not_transplanted_without_tf():
    obj=dict(object_id='A',body_pose_candidate=None,ring_pose_candidate=None,
             model_estimated_body_center_xyz=None,body_yaw_candidate=None,
             ring_center_candidate=None,body_pose_valid=False,ring_pose_valid=False,
             fit_status='CAD_WORKER_PENDING')
    result=dict(objects=[obj])
    old=dict(source_frame_id='old',capture_timestamp=1,result_timestamp=2,
             object=dict(fit_status='WEAK_XY_YAW_CONSTRAINT',
                         model_estimated_body_center_xyz=[.1,.2,.3],
                         body_yaw_candidate=.4,ring_center_candidate=[.1,.2,.35],
                         insertion_direction_candidates=[[1,0,0],[-1,0,0]]))
    cad_for_display(result,'new',{'A':old},now=3)
    assert obj['historical_cad']['status']=='STALE'
    assert obj['historical_cad']['camera_motion_status']=='UNKNOWN_WITHOUT_TF'
    assert obj['body_pose_candidate'] is None and obj['ring_pose_candidate'] is None
    assert not obj['body_pose_valid'] and not obj['ring_pose_valid']


@pytest.mark.skipif(not SAMPLE.is_dir(), reason='Private Windows RGB-D fixture unavailable on Ubuntu')
def test_bounded_recorder_keeps_rgbd_pair_and_stops(tmp_path):
    f=load_rgbd_sample(SAMPLE)
    rec=ContinuousRGBDRecorder(tmp_path/'session',max_frames=2,queue_capacity=2)
    rec.start()
    assert rec.offer('pair1',f)
    assert rec.offer('pair2',f)
    assert not rec.offer('pair3',f)
    rec.add_detection('pair1',dict(capture_timestamp=1,result_timestamp=2,processing_latency_ms=3,
        objects=[],tracking_status={}))
    summary=rec.close()
    assert summary['saved']==2 and summary['offered']==2
    assert not (rec.folder/'frame_0003').exists()
    assert json.loads((rec.folder/'frame_0001/metadata.json').read_text())['source_frame_id']=='pair1'
    assert (rec.folder/'frame_0001/aligned_depth.png').exists()
    assert len((rec.folder/'detections.jsonl').read_text().splitlines())==1


@pytest.mark.skipif(not SAMPLE.is_dir(), reason='Private Windows RGB-D fixture unavailable on Ubuntu')
def test_capture_worker_safe_shutdown_with_complete_packets():
    f=load_rgbd_sample(SAMPLE)
    class Source:
        def __init__(self):self.count=0;self.stopped=False
        def read(self,latest=True):
            assert latest
            self.count+=1
            if self.count>3:raise EOFError('finite source')
            return f
        def stop(self):self.stopped=True
    source=Source();worker=CaptureWorker(source);worker.start();worker.thread.join(3)
    assert worker.frames==3
    identity,pair=worker.slot.get(.1)
    assert identity.startswith('color_') and pair is f
    assert worker.slot.dropped==2
    worker.close()
    assert source.stopped


@pytest.mark.skipif(not SAMPLE.is_dir(), reason='Private Windows RGB-D fixture unavailable on Ubuntu')
def test_cad_worker_latest_job_and_thread_shutdown():
    from threading import Event
    f=load_rgbd_sample(SAMPLE);entered=Event();release=Event()
    class SlowCAD:
        def process(self,frame,force_cad=True):
            assert force_cad
            entered.set();assert release.wait(3)
            return dict(objects=[],timing_ms=dict(total=10.)),[]
    worker=CADWorker(SlowCAD());worker.start()
    assert worker.submit(f,'first',[])
    assert entered.wait(3)
    assert worker.submit(f,'old_queued',[])
    assert worker.submit(f,'latest_queued',[])
    assert worker.slot.dropped==1
    release.set();worker.close()
    assert [m['source_frame_id'] for m in worker.drain()]==['first','latest_queued']


@pytest.mark.skipif(not SAMPLE.is_dir(), reason='Private Windows RGB-D fixture unavailable on Ubuntu')
def test_tracker_miss_reacquire_and_eventual_loss():
    from datetime import datetime,timedelta
    f=load_rgbd_sample(SAMPLE);fast=FastPerception()
    first,_=fast.process(f,'first')
    red=lambda r:next(o['object_id'] for o in r['objects'] if o['color_label']=='red')
    old_id=red(first);base=datetime.fromisoformat(f.timestamps['host_utc'])
    def frame(seconds):
        x=SimpleNamespace(**vars(f));x.timestamps=dict(f.timestamps,
            host_utc=(base+timedelta(seconds=seconds)).isoformat(),
            color_sdk_ms=f.timestamps['color_sdk_ms']+seconds*1000,depth_sdk_ms=f.timestamps['depth_sdk_ms']+seconds*1000)
        return x
    detect=fast.detector.detect_batteries
    fast.detector.detect_batteries=lambda *a,**k:[]
    missing,_=fast.process(frame(.1),'miss')
    assert not missing['objects'] and missing['tracking_status']['missed']
    fast.detector.detect_batteries=detect
    recovered,_=fast.process(frame(.2),'back')
    assert red(recovered)==old_id
    fast.detector.detect_batteries=lambda *a,**k:[]
    fast.process(frame(2.),'lost')
    fast.detector.detect_batteries=detect
    reacquired,_=fast.process(frame(2.1),'reacquired')
    assert red(reacquired)!=old_id
