"""Known synthetic geometry, ambiguity and missing/current-frame target behavior."""
import cv2
import numpy as np
import pytest
from input.realsense_rgbd_source import RGBDFrame
from perception.rgbd_geometry import CameraIntrinsics, frame_capture_time
from perception.placement_target import T0PlacementTarget


def estimator():
    return T0PlacementTarget(dict(battery=dict(body_dimensions_mm=dict(height=50.),
        T_battery_from_ring_m=[[1,0,0,0],[0,1,0,0],[0,0,1,.035],[0,0,0,1]])))


def sample(visible=True, duplicate=False):
    image = np.full((400,800,3),230,np.uint8)
    if visible:
        cv2.rectangle(image,(160,120),(360,220),(20,20,20),2)
    if duplicate:
        cv2.rectangle(image,(500,120),(700,220),(20,20,20),2)
    frame = RGBDFrame(image,np.full((400,800),600,np.uint16),
        CameraIntrinsics(800,400,600,600,400,200),.001,
        dict(host_utc='2026-10-14T00:00:00+00:00',color_frame_number=1,depth_frame_number=1),{})
    result = dict(source_frame_id='color_1_depth_1',capture_timestamp=frame_capture_time(frame)[0],
        timestamps=frame.timestamps,pair_qualified=True,
        support_plane=dict(point_camera_m=[0,0,.6],normal_camera_xyz=[0,0,-1]))
    return frame,result


def test_known_target_is_60mm_above_the_table_not_the_table_point():
    frame,result=sample()
    target=estimator().estimate(frame,result)
    assert target['candidate_available']
    assert np.linalg.norm(np.array(target['table_center_camera_m'])-[-.14,-.03,.6])<.0015
    assert np.linalg.norm(np.array(target['target_hole_center_camera_m'])-[-.14,-.03,.54])<.0015
    assert target['nominal_hole_height_above_table_m']==pytest.approx(.06)
    assert not target['ring_pose_valid'] and not target['motion_usable']


def test_missing_border_never_carries_a_previous_target():
    method=estimator()
    frame,result=sample()
    assert method.estimate(frame,result)['candidate_available']
    frame,result=sample(visible=False)
    target=method.estimate(frame,result)
    assert not target['candidate_available'] and target['target_hole_center_camera_m'] is None


def test_two_valid_borders_are_ambiguous_not_arbitrarily_selected():
    frame,result=sample(duplicate=True)
    target=estimator().estimate(frame,result)
    assert not target['candidate_available']
    assert 'AMBIGUOUS_MULTIPLE_T0_SIZED_BORDERS' in target['reasons']


def test_rejects_previous_frame_reference():
    frame,result=sample()
    result['source_frame_id']='color_2_depth_2'
    with pytest.raises(ValueError,match='source frame'):
        estimator().estimate(frame,result)


def test_unqualified_pair_has_no_target():
    frame,result=sample()
    result['pair_qualified']=False
    assert not estimator().estimate(frame,result)['candidate_available']
