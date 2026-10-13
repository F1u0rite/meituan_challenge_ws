"""Synthetic geometry regressions; these do not certify real battery pose accuracy."""
import cv2,numpy as np,pytest
from perception.simplified_rgbd import SimplifiedPerception
from perception.rgbd_geometry import CameraIntrinsics
from input.realsense_rgbd_source import RGBDFrame

def sample():
    mask=np.zeros((480,640),bool);mask[170:311,220:381]=True
    frame=RGBDFrame(np.zeros((480,640,3),np.uint8),np.full((480,640),300,np.uint16),CameraIntrinsics(640,480,600,600,320,240),.001,{}, {},alignment_metadata={'method':'SDK_DEPTH_TO_COLOR'})
    p=SimplifiedPerception();p.body.geometry={'battery':{'body_dimensions_mm':{'long':80.,'width':70.,'height':50.}}}
    return p.body,frame,mask

def test_multiple_surfaces_can_support_current_face_but_not_a_valid_origin():
    body,f,m=sample();f.aligned_depth[215:250,250:351]=290
    m[301:311]=False  # Partial extent is compatible with multiple CAD face identities.
    out,points,uv=body.estimate(f,m,True);q=out['quality']
    assert q['plane_residual_p95_m']>.003
    assert q['face_quality_pass'] and q['face_support_ratio']>=.7 and q['face_residual_p95_m']<=.003
    assert out['body_xyz_candidate'] is not None
    assert len(out['body_origin_candidates'])>1
    assert 'NOT_UNIQUE' in out['body_xyz_definition'] and not out['body_pose_valid']
    assert np.allclose(out['body_xyz_candidate'][2],.325,atol=.002)
    assert len(out['yaw_candidates'])==2

def test_edges_are_removed_without_relabeling_surface_as_body_origin():
    body,f,m=sample();inner=cv2.erode(m.astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)
    f.aligned_depth[m&~inner]=310
    out,points,_=body.estimate(f,m,True)
    assert out['quality']['edge_pixels_removed']>0 and np.allclose(points[:,2],.3)
    assert 'NOT_BODY_ORIGIN' in out['quality']['position_reference']
    assert out['visible_surface_center_xyz']!=out['body_xyz_candidate']
    assert not out['body_pose_valid']

def test_missing_depth_or_bad_pair_never_gets_xyz():
    body,f,m=sample()
    for bad in (0,100,2000):
        f.aligned_depth[:]=bad;out,points,_=body.estimate(f,m,True)
        assert out['visible_surface_center_xyz'] is None and out['body_xyz_candidate'] is None and not len(points)
    out,points,_=body.estimate(f,m,False)
    assert out['reasons']==['RGBD_PAIR_REJECTED'] and not len(points)

def test_erosion_cannot_bypass_original_low_depth_ratio():
    body,f,m=sample();f.aligned_depth[:]=0;f.aligned_depth[202:279,228:372]=300
    out,_,_=body.estimate(f,m,True)
    assert out['quality']['original_valid_depth_ratio']<.5
    assert out['quality']['valid_depth_ratio']>.5
    assert out['body_xyz_candidate'] is None and out['visible_surface_center_xyz'] is None

def test_no_dominant_face_and_misaligned_mask_are_rejected():
    body,f,m=sample()
    f.aligned_depth[170:217,220:381]=275;f.aligned_depth[217:264,220:381]=300;f.aligned_depth[264:311,220:381]=325
    out,points,_=body.estimate(f,m,True)
    assert len(points)>=100 and out['visible_surface_center_xyz'] is not None
    assert out['body_xyz_candidate'] is None and not out['body_pose_valid']
    with pytest.raises(ValueError):body.estimate(f,m[:-1],True)
