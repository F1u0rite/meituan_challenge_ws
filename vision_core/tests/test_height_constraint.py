"""Known tilted-table geometry and rejection/immutability of height correction."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
from perception.height_constraint import HeightConstrainedPoints
from perception.rgbd_geometry import frame_capture_time


def sample(height=.077):
    frame=SimpleNamespace(timestamps=dict(host_utc='2026-10-14T00:00:00+00:00',
        color_frame_number=1,depth_frame_number=1))
    normal=np.array([0.,-.6,-.8])
    point=np.array([0.,0.,.6])
    base=np.array([.1,.05,.5625])
    result=dict(source_frame_id='color_1_depth_1',capture_timestamp=frame_capture_time(frame)[0],
        timestamps=frame.timestamps,pair_qualified=True,
        support_plane=dict(point_camera_m=point.tolist(),normal_camera_xyz=normal.tolist()),
        objects=[dict(color='green',candidate_available=True,geometry_consistent=False,reasons=['BOUNDARY_WARNING'],
            green_point_camera_m=(base+height*normal).tolist(),
            red_point_camera_m=(base+(height-.020)*normal).tolist())])
    return frame,result,normal,point


def test_correction_is_along_table_normal_not_camera_z_and_keeps_horizontal_position():
    frame,result,normal,point=sample()
    before=copy.deepcopy(result)
    corrected=HeightConstrainedPoints().apply(frame,result)
    obj=corrected['objects'][0]
    assert result==before
    raw=np.array(obj['raw_red_point_camera_m']);effective=np.array(obj['red_point_camera_m'])
    delta=effective-raw
    assert np.allclose(delta,.003*normal)
    assert abs(delta[1])>0  # A camera-Z-only offset would be wrong for a tilted camera.
    assert np.linalg.norm((np.eye(3)-np.outer(normal,normal))@delta)<1e-12
    assert float((effective-point)@normal)==pytest.approx(.060)
    assert obj['green_height_m']==pytest.approx(.080)
    assert obj['height_constraint']['applied']
    assert not obj['geometry_consistent'] and not obj['ring_pose_valid']


def test_depth_too_high_gets_negative_correction():
    frame,result,normal,point=sample(height=.082)
    obj=HeightConstrainedPoints().apply(frame,result)['objects'][0]
    assert obj['height_constraint']['top_correction_m']==pytest.approx(-.002)
    assert obj['red_height_m']==pytest.approx(.060)


def test_large_error_is_rejected_not_forced_onto_known_height():
    frame,result,_,_=sample(height=.040)
    obj=HeightConstrainedPoints().apply(frame,result)['objects'][0]
    assert not obj['candidate_available'] and obj['red_point_camera_m'] is None
    assert obj['raw_red_point_camera_m'] is not None and obj['raw_candidate_available']
    assert obj['height_constraint']['status']=='HEIGHT_CORRECTION_EXCEEDS_CONFIGURED_LIMIT'


def test_missing_detection_does_not_keep_old_raw_or_effective_points():
    frame,result,_,_=sample()
    result['objects'][0]['candidate_available']=False
    obj=HeightConstrainedPoints().apply(frame,result)['objects'][0]
    assert obj['red_point_camera_m'] is None and obj['raw_red_point_camera_m'] is None
    assert not obj['height_constraint']['applied']


def test_stale_frame_reference_rejected():
    frame,result,_,_=sample()
    result['source_frame_id']='color_2_depth_2'
    with pytest.raises(ValueError,match='source frame'):
        HeightConstrainedPoints().apply(frame,result)


def test_missing_plane_does_not_apply_height_constraint():
    frame,result,_,_=sample()
    result['support_plane']=None
    obj=HeightConstrainedPoints().apply(frame,result)['objects'][0]
    assert not obj['candidate_available'] and obj['red_point_camera_m'] is None
    assert not obj['height_constraint']['applied']


def test_wrong_top_to_hole_offset_is_rejected():
    frame,result,_,_=sample()
    result['objects'][0]['red_point_camera_m'][0]+=.005
    obj=HeightConstrainedPoints().apply(frame,result)['objects'][0]
    assert not obj['candidate_available']
    assert obj['height_constraint']['status']=='CAD_TOP_TO_HOLE_OFFSET_MISMATCH'


@pytest.mark.parametrize('kwargs',[dict(top_height_m=float('nan')),dict(hole_height_m=.09),dict(max_correction_m=0)])
def test_invalid_physical_inputs(kwargs):
    with pytest.raises(ValueError):HeightConstrainedPoints(**kwargs)
