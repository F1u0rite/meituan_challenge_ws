"""Explicit synthetic unit fixtures; never substitute these for real-camera evidence."""
from datetime import datetime,timezone
from time import time
import cv2,numpy as np,pytest
from input.realsense_rgbd_source import RGBDFrame
from perception.rgbd_geometry import CameraIntrinsics
from perception.simplified_rgbd import SimplifiedPerception,RingPoseEstimation
from perception.vision_ui import VisionUI
from mtc_vision_adapter.native_rgbd import native_array

def frame(number=1,depth=True):
    image=np.zeros((720,1280,3),np.uint8)
    for i,color in enumerate([(0,0,255),(255,0,0),(0,255,255),(0,255,0)]):
        cv2.rectangle(image,(40+i*300,250),(200+i*300,390),color,-1)
    stamp=time();timestamps=dict(host_utc=datetime.fromtimestamp(stamp,timezone.utc).isoformat(),color_sdk_ms=stamp*1000,depth_sdk_ms=stamp*1000,color_timestamp_domain='timestamp_domain.global_time',depth_timestamp_domain='timestamp_domain.global_time',color_frame_number=number,depth_frame_number=number)
    return RGBDFrame(image,np.full((720,1280),300 if depth else 0,np.uint16),CameraIntrinsics(1280,720,600,600,640,360),.001,timestamps,{},alignment_metadata=dict(method='SDK_DEPTH_TO_COLOR'))

def test_four_color_identity_without_depth_or_table():
    p=SimplifiedPerception();p.process(frame(depth=False),'one');r,views=p.process(frame(2,False),'two')
    assert {o['color'] for o in r['confirmed_objects']}=={'red','blue','yellow','green'}
    assert [v['object_id'] for v in views]==[o['object_id'] for o in r['objects']]
    assert all(o['detection_valid'] and not o['body_pose_valid'] and not o['ring_pose_valid'] for o in r['objects'])
    assert all(o['visible_surface_center_xyz'] is None for o in r['objects'])
    assert len(native_array(r)['detections'])==4
    assert VisionUI().render(frame(3,False),r,views,{}).shape==(900,1600,3)

def test_wrong_sync_keeps_rgb_identity_but_rejects_geometry():
    p=SimplifiedPerception();p.process(frame(),'one');f=frame(2);f.timestamps['depth_sdk_ms']-=500
    r,_=p.process(f,'two')
    assert len(r['confirmed_objects'])==4
    assert not r['pair_diagnostics']['qualified']
    assert all(o['body_geometry']['reasons']==['RGBD_PAIR_REJECTED'] and not o['ring_pose_valid'] for o in r['objects'])

def test_reused_depth_not_used_and_old_capture_not_ros_admitted():
    p=SimplifiedPerception();p.process(frame(),'one');f=frame(2);f.timestamps['depth_frame_number']=1
    r,_=p.process(f,'two');assert not r['pair_diagnostics']['qualified']
    f=frame(3);f.timestamps['host_utc']=datetime.fromtimestamp(time()-10,timezone.utc).isoformat()
    r,_=p.process(f,'old');assert len(r['confirmed_objects'])==4 and not native_array(r)['detections']
    with pytest.raises(ValueError):p.process(f,'same')

def test_body_candidates_are_not_unique_valid_poses():
    p=SimplifiedPerception();p.process(frame(),'one');r,_=p.process(frame(2),'two')
    assert all(o['body_yaw_candidate'] is not None and len(o['body_yaw_candidates'])==2 for o in r['objects'])
    assert all(not o['body_pose_valid'] and not o['motion_usable'] for o in r['objects'])

def test_local_observed_aperture_and_missing_hole():
    p=SimplifiedPerception();f=frame();mask=np.zeros((720,1280),bool);mask[200:401,300:501]=True
    p.ring.geometry={'battery':{'aperture':{'width_mm':50.,'height_mm':20.}}}  # Explicit unit fixture, no private assets required.
    f.color_bgr[:]=0;f.color_bgr[mask]=(0,0,255)
    absent=p.ring.estimate(f,mask,'red',True);assert not absent['hole_candidates'] and absent['center_candidate'] is None
    cv2.rectangle(f.color_bgr,(350,280),(450,320),(0,0,0),-1)
    solid=p.ring.estimate(f,mask,'red',True)
    assert solid['center_candidate'] is None  # Dark paint on a solid face is not a ring.
    f.aligned_depth[280:321,350:451]=400
    out=p.ring.estimate(f,mask,'red',True)
    assert out['hole_candidates'] and out['center_candidate'] is not None
    assert np.allclose(out['center_candidate'][2],.3,atol=.001)
    assert len(out['direction_candidates'])==2 and not out['ring_pose_valid']
    f.aligned_depth[:]=0;out=p.ring.estimate(f,mask,'red',True)
    assert out['center_candidate'] is None and not out['ring_pose_valid']

def test_weak_wood_chroma_is_not_a_battery_and_border_occlusion_is_ambiguous():
    p=SimplifiedPerception();f=frame();f.color_bgr[:]=(80,90,120)
    r,_=p.process(f,'wood');assert not r['objects']
    f=frame(2);f.color_bgr[:]=0;cv2.rectangle(f.color_bgr,(0,250),(160,390),(0,0,255),-1)
    p.process(f,'edge');f=frame(3);f.color_bgr[:]=0;cv2.rectangle(f.color_bgr,(0,250),(160,390),(0,0,255),-1)
    r,_=p.process(f,'edge2');assert not r['confirmed_objects']


@pytest.mark.parametrize('red_hue',[1,175])
def test_red_wrap_and_blue_survive_touching_wood_without_valid_depth(red_hue):
    p=SimplifiedPerception()
    results=[]
    for i in [1,2]:
        f=frame(i,False)
        hsv=np.zeros(f.color_bgr.shape,np.uint8)
        # Measured wood H=17 attaches directly to both synthetic target bases.
        hsv[390:720,:]=(17,150,180)
        hsv[250:391,40:201]=(red_hue,220,220)
        hsv[250:391,340:501]=(110,220,220)
        f.color_bgr=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        result,views=p.process(f,'wood-touch-'+str(i));results.append(result)
    assert {o['color'] for o in results[-1]['confirmed_objects']}=={'red','blue'}
    red=next(o for o in results[-1]['objects'] if o['color']=='red')
    assert red['bbox']==[40,250,161,141]  # Wood must not extend the red contour.
    assert not red['body_pose_valid'] and not red['ring_pose_valid']
    from perception.battery_only import PixelBatteryDetector
    # The shared Legacy HSV configuration remains available for rollback.
    assert PixelBatteryDetector().config['hsv_ranges']['red'][0][3]==18


@pytest.mark.parametrize('green_hue',[34,40])
def test_touching_four_colors_do_not_merge_at_hue_boundaries(green_hue):
    p=SimplifiedPerception()
    for number in [1,2]:
        f=frame(number,False);hsv=np.zeros(f.color_bgr.shape,np.uint8)
        hsv[250:391,40:201]=(green_hue,220,220)
        hsv[250:391,202:363]=(90,220,220)  # Old green/blue overlap.
        hsv[392:533,40:201]=(24,220,220)
        hsv[392:533,202:363]=(1,220,220)
        f.color_bgr=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        result,views=p.process(f,'touching-four-'+str(number))
    assert {o['color'] for o in result['confirmed_objects']}=={'green','blue','yellow','red'}
    assert len(views)==4 and all(not o['body_pose_valid'] and not o['ring_pose_valid'] for o in result['objects'])
    green=next(o for o in result['objects'] if o['color']=='green')
    assert green['bbox']==[40,250,161,141]
