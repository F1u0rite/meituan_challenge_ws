"""Analytic camera/CAD fixtures, never evidence of real-camera accuracy."""
from datetime import datetime,timezone
from time import time
import cv2,numpy as np,pytest
from perception.top_bar_pose import top_bar_spec,transform_from_top,TopBarPoseEstimation,observed_layout
from perception.simplified_rgbd import SimplifiedPerception
from perception.rgbd_geometry import CameraIntrinsics,project_points
from input.realsense_rgbd_source import RGBDFrame


def geometry():
    return {'battery':{'source':{'sha256':'synthetic-test-only'},
        'planar_faces':[{'face_index':27,'normal':[0,0,-1],'bbox_mm':[-33,-11,-30,33,11,-30],'area_mm2':1452}],
        'aperture':{'ring_center_cad_mm':[0,0,-10],'normal_cad':[0,1,0]},
        'T_battery_from_cad_m':[[0,1,0,0],[1,0,0,0],[0,0,-1,.025],[0,0,0,1]],
        'body_dimensions_mm':{'long':80.,'width':70.,'height':50.},'overall_height_mm':80.}}


def rendered(theta_deg=33.):
    K=CameraIntrinsics(640,480,900,900,320,240);image=np.zeros((480,640,3),np.uint8);depth=np.zeros((480,640),float)
    theta=np.deg2rad(theta_deg);R=np.array([[1,0,0],[0,np.cos(theta),np.sin(theta)],[0,-np.sin(theta),np.cos(theta)]])
    translation=np.array([0,0,.4])-R@np.array([0,0,-.03])
    # Exact physical top cap and two adjacent front faces; different visible
    # brightness is a test fixture, not an assumed real battery texture.
    quads=[(np.array([[-.033,-.011,-.03],[.033,-.011,-.03],[.033,.011,-.03],[-.033,.011,-.03]]),(0,0,240)),
        (np.array([[-.033,.013,-.028],[.033,.013,-.028],[.033,.013,0],[-.033,.013,0]]),(0,0,140)),
        (np.array([[-.035,.04,0],[.035,.04,0],[.035,.04,.05],[-.035,.04,.05]]),(0,0,180))]
    vv,uu=np.indices(depth.shape);rays=np.stack(((uu-K.ppx)/K.fx,(vv-K.ppy)/K.fy,np.ones(depth.shape)),axis=-1)
    for q,color in quads:
        world=q@R.T+translation;region=np.zeros(depth.shape,np.uint8);cv2.fillConvexPoly(region,np.rint(project_points(world,K)).astype(np.int32),1)
        n=np.cross(world[1]-world[0],world[2]-world[0]);zz=(world[0]@n)/(rays@n)
        use=(region>0)&(zz>0)&((depth==0)|(zz<depth));depth[use]=zz[use];image[use]=color
    mask=depth>0
    return RGBDFrame(image,np.uint16(np.rint(depth/.001)),K,.001,{}, {},alignment_metadata={'method':'SDK_DEPTH_TO_COLOR'}),mask


def test_measured_cap_identity_and_offsets():
    spec=top_bar_spec(geometry())
    assert spec['face_index']==27 and np.allclose(spec['top_span_cad_xy_m'],[.066,.022])
    assert np.allclose(spec['offset_ring_cad_m'],[0,0,.02])
    assert np.allclose(spec['offset_body_cad_m'],[0,0,.055])
    bad=geometry();bad['battery']['planar_faces'][0]['area_mm2']=1000
    assert top_bar_spec(bad) is None  # Bbox centre is not enough for a cut-out cap.


@pytest.mark.parametrize('angles',[(0,0,0),(.4,-.7,.3),(-.8,.5,1.4)])
def test_rotated_offset_and_entry_symmetry(angles):
    from scipy.spatial.transform import Rotation
    R=Rotation.from_euler('xyz',angles).as_matrix();spec=top_bar_spec(geometry());origin=np.array([.12,-.03,.45])
    top=origin+R@spec['top_center_cad_m'];out=transform_from_top(top,-R[:,2],R[:,0],spec)
    assert len(out)==2
    for result in out:
        matrix=np.asarray(result['rotation_camera_from_cad'])
        assert np.allclose(matrix.T@matrix,np.eye(3)) and np.isclose(np.linalg.det(matrix),1)
        assert np.allclose(result['ring_center_camera_m'],origin+R@spec['ring_center_cad_m'])
        assert np.allclose(result['body_center_camera_m'],origin+R@np.array([0,0,.025]))
    assert np.allclose(out[0]['insertion_direction_camera'],-np.asarray(out[1]['insertion_direction_camera']))


@pytest.mark.parametrize('theta_deg',[40.,50.,60.])
def test_automatic_top_not_manual_roi_and_unverified_pose(theta_deg):
    f,mask=rendered(theta_deg);p=SimplifiedPerception();top=TopBarPoseEstimation(p.config,geometry())
    out=top.estimate(f,mask,True)
    assert out['top_geometry_valid'],out['candidates']
    assert out['ring_center_geometry_valid'] and np.allclose(out['top_center_candidate'],[0,0,.4],atol=.003)
    assert not out['ring_geometry_valid'] and not out['ring_pose_valid'] and not out['body_pose_valid']
    assert not out['direction_unique'] and not out['motion_usable']
    assert out['quality']['local_support_ratio']>=.7
    # Full geometry follows translation in the image, not an absolute pixel ROI.
    shift=30;f.color_bgr=np.roll(f.color_bgr,shift,axis=1);f.aligned_depth=np.roll(f.aligned_depth,shift,axis=1);mask=np.roll(mask,shift,axis=1)
    moved=top.estimate(f,mask,True)
    assert moved['top_geometry_valid']
    assert moved['top_center_candidate'][0]>out['top_center_candidate'][0]+.008


def test_missing_unsynced_and_occluded_never_gains_geometry():
    f,mask=rendered();p=SimplifiedPerception();top=TopBarPoseEstimation(p.config,geometry())
    assert top.estimate(f,mask,False)['reasons']==['RGBD_PAIR_REJECTED']
    f.aligned_depth[:]=0
    assert not top.estimate(f,mask,True)['top_geometry_valid']
    f,mask=rendered();f.color_bgr[:,:310]=0;f.aligned_depth[:,:310]=0;mask[:,:310]=False
    assert not top.estimate(f,mask,True)['top_geometry_valid']  # Half cap has no complete CAD extent.


def test_layout_is_observed_only_without_color_slot_order():
    xyz=[[-.1,-.1,.4],[.1,-.1,.4],[.1,.1,.4],[-.1,.1,.4]]
    objects=[dict(object_id=str(i),color=c,top_geometry=dict(top_geometry_valid=True,top_center_candidate=x)) for i,(c,x) in enumerate(zip(['blue','red','green','yellow'],xyz))]
    out=observed_layout(objects[::-1]);assert out['status']=='OBSERVED_2X2_TOPOLOGY'
    assert set(out['corner_object_ids'])=={'0','1','2','3'} and not out['color_slot_assumption']
    assert observed_layout(objects[:3])['count']==3
    assert not out['motion_usable']


def test_multiple_supported_plane_orientations_are_not_chosen_arbitrarily():
    f,mask=rendered(33.);p=SimplifiedPerception();top=TopBarPoseEstimation(p.config,geometry())
    out=top.estimate(f,mask,True)
    assert any(not c['reasons'] for c in out['candidates'])
    assert not out['top_geometry_valid'] and out['ring_center_candidate'] is None
    assert out['reasons']==['NO_UNIQUE_OBSERVED_TOP_CAP']
