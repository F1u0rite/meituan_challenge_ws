"""Analytic centre/offset truth; no assertion of real-camera accuracy."""
import cv2,numpy as np,pytest
from test_top_bar_pose import geometry,rendered
from perception.segmented_top_center import SegmentedTopCenter
from perception.rgbd_geometry import project_points


def cap(theta=50.):
    f,_=rendered(theta)
    t=np.deg2rad(theta);R=np.array([[1,0,0],[0,np.cos(t),np.sin(t)],[0,-np.sin(t),np.cos(t)]])
    q=np.array([[-.033,-.011,0],[.033,-.011,0],[.033,.011,0],[-.033,.011,0]])
    q=q@R.T+[0,0,.4]
    mask=np.zeros(f.aligned_depth.shape,np.uint8)
    cv2.fillConvexPoly(mask,np.rint(project_points(q,f.intrinsics)).astype(np.int32),1)
    return f,mask.astype(bool),-R[:,2]


@pytest.mark.parametrize('theta',[33.,40.,50.,60.])
def test_observed_center_and_rotated_cad_offset(theta):
    f,m,n=cap(theta)
    r=SegmentedTopCenter(geometry()).estimate(f,m,qualified=True)
    assert r['candidate_available'] and r['geometry_consistent']
    assert np.allclose(r['top_center_candidate'],[0,0,.4],atol=.0015)
    assert np.allclose(r['ring_center_candidate'],np.array([0,0,.4])-.020*n,atol=.0015)
    assert not r['ring_pose_valid'] and not r['motion_usable']


def test_center_pixel_depth_hole_does_not_destroy_plane_center():
    f,m,n=cap();f.aligned_depth[235:246,315:326]=0
    r=SegmentedTopCenter(geometry()).estimate(f,m,qualified=True)
    assert r['candidate_available']
    assert np.allclose(r['top_center_candidate'],[0,0,.4],atol=.0015)


def test_partial_cap_without_constraint_is_not_completed():
    f,m,n=cap();x=np.where(m)[1];m[:,:int(x.min()+.25*(x.max()-x.min()))]=False
    r=SegmentedTopCenter(geometry()).estimate(f,m,qualified=True,complete_boundary=False)
    assert not r['candidate_available']


def test_declared_remaining_long_end_recovers_partial_center():
    f,m,n=cap();x=np.where(m)[1];m[:,:int(x.min()+.25*(x.max()-x.min()))]=False
    r=SegmentedTopCenter(geometry()).estimate(f,m,qualified=True,complete_boundary=False,visible_long_edge='positive')
    assert r['candidate_available'] and r['cad_completed']
    assert np.allclose(r['top_center_candidate'],[0,0,.4],atol=.0015)
    assert 'CAD_COMPLETED_NOT_FULLY_OBSERVED' in r['reasons']


def test_unsynced_empty_missing_depth_and_wrong_mask_rejected():
    f,m,n=cap();engine=SegmentedTopCenter(geometry())
    assert not engine.estimate(f,m,qualified=False)['candidate_available']
    assert not engine.estimate(f,np.zeros_like(m),qualified=True)['candidate_available']
    with pytest.raises(ValueError):engine.estimate(f,m[:10],qualified=True)
    f.aligned_depth[:]=0
    assert not engine.estimate(f,m,qualified=True)['candidate_available']


def test_heldout_depth_damage_is_not_hidden_by_train_inliers():
    f,m,n=cap();yy,xx=np.indices(m.shape)
    f.aligned_depth[m&((xx+yy)%2==1)]+=30
    r=SegmentedTopCenter(geometry()).estimate(f,m,qualified=True)
    assert r['candidate_available'] and not r['geometry_consistent']
    # 30 mm axial depth corruption projects to about 19 mm normal distance
    # at this 50-degree tilt, rather than a 30 mm plane-normal displacement.
    assert r['quality']['holdout_p95_m']>.015


def test_translation_in_image_is_not_fixed_roi():
    f,m,n=cap();engine=SegmentedTopCenter(geometry())
    r=engine.estimate(f,m,qualified=True)
    f.color_bgr=np.roll(f.color_bgr,40,axis=1);f.aligned_depth=np.roll(f.aligned_depth,40,axis=1);m=np.roll(m,40,axis=1)
    moved=engine.estimate(f,m,qualified=True)
    assert moved['candidate_available']
    assert moved['top_center_candidate'][0]-r['top_center_candidate'][0]>.015
