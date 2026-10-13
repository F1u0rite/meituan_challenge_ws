"""Offline automatic prompting tests; SAM predictor is injected, no weights."""
import time
from datetime import datetime,timezone
import cv2,numpy as np,pytest,yaml
from pathlib import Path
from perception.automatic_top_prompt import AutomaticTopPrompt
from perception.automatic_segmented_top import AutomaticSegmentedTop
from perception.segmented_top_center import SegmentedTopCenter
from perception.rgbd_geometry import CameraIntrinsics
from input.realsense_rgbd_source import RGBDFrame
from test_top_bar_pose import geometry,rendered
ROOT=Path(__file__).resolve().parents[1]


def scene(tilt=0.):
    image=np.full((480,640,3),180,np.uint8)
    yy,xx=np.indices(image.shape[:2])
    rays=np.stack(((xx-320)/600,(yy-240)/600,np.ones(xx.shape)),axis=-1)
    angle=np.deg2rad(tilt);normal=np.array([0.,-np.sin(angle),-np.cos(angle)])
    table_offset=np.array([0.,0.,.6])@normal
    table_z=table_offset/(rays@normal)
    body_z=(table_offset+.05)/(rays@normal)
    cap_z=(table_offset+.08)/(rays@normal)
    depth=np.uint16(np.rint(table_z/.001));masks={}
    for color,x,bgr in [('red',170,(0,0,255)),('blue',370,(255,0,0))]:
        mask=np.zeros(depth.shape,bool);mask[170:290,x:x+81]=True
        image[mask]=bgr;depth[mask]=np.uint16(np.rint(body_z[mask]/.001));depth[170:200,x:x+81]=np.uint16(np.rint(cap_z[170:200,x:x+81]/.001))
        masks[color]=mask
    stamp=time.time()
    ts=dict(host_utc=datetime.fromtimestamp(stamp,timezone.utc).isoformat(),
        color_sdk_ms=stamp*1000,depth_sdk_ms=stamp*1000,color_frame_number=1,depth_frame_number=1,
        color_timestamp_domain='global_time',depth_timestamp_domain='global_time')
    return RGBDFrame(image,depth,CameraIntrinsics(640,480,600,600,320,240),.001,ts,{},alignment_metadata={'method':'SDK_DEPTH_TO_COLOR'}),masks


class Predictor:
    def __init__(self):self.calls=0
    def set_image(self,image):self.calls+=1;self.shape=image.shape[:2]
    def predict(self,box,**kw):
        mask=np.zeros(self.shape,bool)
        x0,y0,x1,y1=np.rint(box).astype(int);mask[y0:y1+1,x0:x1+1]=True
        # The highest-score whole-image mask must be rejected as leakage.
        return np.array([np.ones(self.shape,bool),mask,mask]),np.array([.99,.3,.2]),None


@pytest.mark.parametrize('rotate',[False,True])
@pytest.mark.parametrize('tilt',[0.,25.,-20.])
def test_prompts_follow_current_geometry_without_image_top_assumption(rotate,tilt):
    f,masks=scene(tilt)
    if rotate:
        f.color_bgr=np.rot90(f.color_bgr).copy();f.aligned_depth=np.rot90(f.aligned_depth).copy()
        f.intrinsics=CameraIntrinsics(480,640,600,600,240,319)
        masks={c:np.rot90(m).copy() for c,m in masks.items()}
    r=AutomaticTopPrompt().propose(f,masks,qualified=True)
    assert not r['manual_prompts'] and r['support_plane']
    for c in masks:
        p=r['objects'][c];assert p['available'],p['reasons']
        assert abs(p['observed_height_median_m']-.08)<.001
        x,y=p['point_coords_px'][0];assert p['seed_mask'][y,x]
        assert p['point_labels']==[1,0]


def test_missing_top_depth_has_no_old_or_nominal_seed():
    f,masks=scene();f.aligned_depth[170:200,170:251]=0
    r=AutomaticTopPrompt().propose(f,masks,qualified=True)
    assert not r['objects']['red']['available']
    assert r['objects']['blue']['available']


def test_pipeline_automatically_outputs_current_centers_and_rejects_leakage():
    f,masks=scene();pred=Predictor()
    pipe=AutomaticSegmentedTop(yaml.safe_load((ROOT/'config/simplified_rgbd.yaml').read_text()),geometry(),predictor=pred)
    r=pipe.process(f,'one')
    valid=[o for o in r['objects'] if o['candidate_available']]
    assert {o['color'] for o in valid}=={'red','blue'}
    assert pred.calls==1 and not r['manual_prompts']
    for o in valid:
        assert o['sam_selected_candidate']==1
        assert np.isclose(np.linalg.norm(np.array(o['red_point_camera_m'])-o['green_point_camera_m']),.020)
        assert o['geometry']['quality']['normal_source']=='CURRENT_SUPPORT_PLANE_UPRIGHT_BATTERY_CONSTRAINT'
    assert {'green','yellow'}<=set(r['missing_colors'])
    assert not r['motion_usable']


def test_unsynced_pipeline_never_runs_sam_or_outputs_coords():
    f,masks=scene();f.timestamps['depth_sdk_ms']-=500
    pred=Predictor();pipe=AutomaticSegmentedTop(yaml.safe_load((ROOT/'config/simplified_rgbd.yaml').read_text()),geometry(),predictor=pred)
    r=pipe.process(f,'one')
    assert not r['pair_qualified'] and pred.calls==0
    assert all(not o['candidate_available'] for o in r['objects'])


def test_missing_support_plane_never_runs_sam():
    f,masks=scene();union=np.logical_or.reduce(list(masks.values()));f.aligned_depth[~union]=0
    pred=Predictor();pipe=AutomaticSegmentedTop(yaml.safe_load((ROOT/'config/simplified_rgbd.yaml').read_text()),geometry(),predictor=pred)
    r=pipe.process(f,'one')
    assert pred.calls==0 and all(not o['candidate_available'] for o in r['objects'])


def test_stale_support_normal_is_rejected():
    f,masks=scene()
    hint=dict(normal_camera_xyz=[0,0,-1],point_camera_m=[0,0,.6],
        color_frame_number=0,color_sdk_ms=f.timestamps['color_sdk_ms'])
    r=SegmentedTopCenter(geometry()).estimate(f,masks['red'],qualified=True,support_plane=hint)
    assert not r['candidate_available'] and r['reasons']==['SUPPORT_REFERENCE_FRAME_MISMATCH']


def test_duplicate_color_is_not_arbitrarily_selected():
    f,masks=scene()
    f.color_bgr[170:290,50:131]=(0,0,255)
    f.aligned_depth[170:290,50:131]=550
    f.aligned_depth[170:200,50:131]=520
    pred=Predictor()
    pipe=AutomaticSegmentedTop(yaml.safe_load((ROOT/'config/simplified_rgbd.yaml').read_text()),geometry(),predictor=pred)
    r=pipe.process(f,'one')
    red=next(o for o in r['objects'] if o['color']=='red')
    assert not red['candidate_available']
    assert red['reasons']==['AMBIGUOUS_MULTIPLE_OBJECTS_OF_SAME_COLOR']
