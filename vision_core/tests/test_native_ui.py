"""Temporal admission and existing OpenCV interaction safety; no camera needed."""
from types import SimpleNamespace
import numpy as np
import pytest
from input.rgbd_pair_guard import RGBDPairGuard
from perception.dual_rate_rgbd import CaptureWorker
from perception.vision_ui import VisionUI
from scripts.dual_rate_runner import interactive_frame


def stamp(color,depth=None,color_ms=None,depth_ms=None,domain='global_time'):
    depth=color if depth is None else depth
    return dict(color_frame_number=color,depth_frame_number=depth,
        color_sdk_ms=color*100 if color_ms is None else color_ms,
        depth_sdk_ms=depth*100 if depth_ms is None else depth_ms,
        color_timestamp_domain=domain,depth_timestamp_domain=domain)


def test_bad_pair_does_not_consume_good_depth_frame():
    gate=RGBDPairGuard()
    assert gate.inspect(stamp(1))['qualified']
    assert not gate.inspect(stamp(1,3))['qualified']
    assert gate.inspect(stamp(3,3))['qualified']
    assert gate.snapshot()['accepted']==2 and gate.snapshot()['rejected']==1


@pytest.mark.parametrize('changes',[
    dict(depth_timestamp_domain='hardware_clock'),dict(color_sdk_ms=float('nan')),
    dict(color_frame_number=True),dict(depth_sdk_ms=400),
])
def test_incomparable_nonfinite_or_large_skew_rejects(changes):
    t=stamp(1);t.update(changes)
    assert not RGBDPairGuard().inspect(t)['qualified']


def test_same_hardware_clock_label_does_not_prove_common_sensor_clock():
    assert not RGBDPairGuard().inspect(stamp(1,domain='hardware_clock'))['qualified']


def test_reused_depth_and_backwards_time_reject():
    guard=RGBDPairGuard();assert guard.inspect(stamp(3))['qualified']
    assert not guard.inspect(stamp(4,3,400,300))['qualified']
    assert not guard.inspect(stamp(5,5,299,299))['qualified']


def test_capture_worker_never_offers_unqualified_pair_to_cad_or_recorder():
    class Source:
        def __init__(self):self.values=iter([stamp(1),stamp(1,3),stamp(3)]);self.stopped=False
        def read(self,latest=True):return SimpleNamespace(timestamps=next(self.values))
        def stop(self):self.stopped=True
    offered=[];source=Source();worker=CaptureWorker(source,on_frame=lambda _,f:offered.append(f.timestamps))
    worker.start();worker.thread.join(3);worker.close()
    assert not worker.thread.is_alive() and source.stopped
    assert worker.frames==2 and worker.observed_frames==3
    assert worker.pair_guard.rejected==1
    assert [x['color_frame_number'] for x in offered]==[1,3]


def test_permanently_incomparable_input_stops_with_diagnostic():
    class Source:
        def read(self,latest=True):return SimpleNamespace(timestamps=stamp(1,domain='hardware_clock'))
        def stop(self):pass
    worker=CaptureWorker(Source());worker.start();worker.thread.join(3);worker.close()
    assert not worker.thread.is_alive() and worker.frames==0
    assert worker.observed_frames==30 and 'No qualified RGB-D' in worker.error


def test_pause_step_and_two_manual_saves_are_distinct_actions(monkeypatch):
    import scripts.dual_rate_runner as app
    keys=iter([ord(' '),ord('s'),ord('s'),ord('n')])
    monkeypatch.setattr(app.cv2,'imshow',lambda *a:None)
    monkeypatch.setattr(app.cv2,'waitKey',lambda *a:next(keys))
    ui=VisionUI();saved=[]
    quit_requested,_=interactive_frame(ui,'test',np.zeros((8,8,3),np.uint8),lambda image:saved.append(image.copy()))
    assert not quit_requested and ui.paused and len(saved)==2


def test_quit_while_paused_exits(monkeypatch):
    import scripts.dual_rate_runner as app
    monkeypatch.setattr(app.cv2,'imshow',lambda *a:None)
    monkeypatch.setattr(app.cv2,'waitKey',lambda *a:ord('q'))
    ui=VisionUI();ui.paused=True
    quit_requested,_=interactive_frame(ui,'test',np.zeros((8,8,3),np.uint8),lambda _:pytest.fail('unexpected save'))
    assert quit_requested


def test_missing_display_fails_before_camera_or_gui(monkeypatch,capsys):
    from scripts import live_rgbd_preview as app
    monkeypatch.delenv('DISPLAY',raising=False)
    if app.os.name=='nt':pytest.skip('Linux DISPLAY rule')
    monkeypatch.setattr(app.RealSenseRGBDSource,'start',lambda *a:pytest.fail('camera started'))
    with pytest.raises(SystemExit) as e:app.main(['--replay','missing','--full-vision','--dual-rate'])
    assert e.value.code==2 and 'No DISPLAY' in capsys.readouterr().err


@pytest.mark.parametrize('color',['red','blue','yellow','green'])
def test_actual_renderer_keeps_stale_candidates_text_only_and_pose_invalid(color):
    from input.realsense_rgbd_source import RGBDFrame
    from perception.rgbd_geometry import CameraIntrinsics
    intr=CameraIntrinsics(320,180,200,200,160,90)
    frame=RGBDFrame(np.zeros((180,320,3),np.uint8),np.full((180,320),600,np.uint16),
        intr,.001,dict(stamp(1),host_utc='2020-01-01T00:00:00+00:00'),{})
    mask=np.zeros((180,320),bool);mask[50:100,100:180]=True
    transform=np.eye(4);transform[2,3]=.6
    item=dict(object_id='fixture-'+color,color_label=color,color=color,
        candidate_state='AMBIGUOUS',detection_status='AMBIGUOUS',color_confidence=.9,
        surface_depth_z=.6,pixel_center=[140,75],bbox=[100,50,80,50],
        visible_surface_center_xyz=[0,0,.6],camera_to_surface_distance=.6,
        body_pose_candidate=transform.tolist(),ring_center_candidate=[0,0,.64],
        ring_orientation_candidate=np.eye(3).tolist(),insertion_direction_candidates=[[1,0,0],[-1,0,0]],
        hook_insertion_yaw_candidates=[0,3.14],model_estimated_body_center_xyz=[0,0,.6],
        body_yaw_candidate=0.,body_center_camera_z=.6,camera_to_body_center_distance=.6,
        tracking_reliability='AMBIGUOUS',fit_status='WEAK_XY_YAW_CONSTRAINT',
        cad_fresh=False,cad_stale=True,cad_result_timestamp=0.,depth_valid_ratio=1.,
        ring_feature_status='CAD_PREDICTED',pose_quality={},rejection_reason='UNIT_TEST_STALE',
        detection_ambiguity=True,pose_ambiguity=True,detection_stale=True)
    result=dict(mode='OFFLINE_UNIT_FIXTURE',objects=[item],timing_ms=dict(total=1.),timestamp=1.,source_frame_id='fixture')
    ui=VisionUI()
    image=ui.render(frame,result,[dict(mask=mask,foreground=mask)],{})
    assert image.shape==(900,1600,3) and image.dtype==np.uint8
    assert ui.last_display_data['cad_overlay_object_ids']==[]
    assert ui.last_display_data['pose_label']=='BODY/RING INVALID'
    assert ui.last_display_data['capture_timestamps']==frame.timestamps
    assert ui.last_display_data['depth_above_display_max_still_valid'] is True
    assert color.upper() in ui.last_display_data['labels'][0]
