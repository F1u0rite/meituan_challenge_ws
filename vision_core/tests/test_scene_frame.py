"""Scene invariance under camera motion, signed axes and stale/missing rejection."""
import copy
import json
import cv2
import numpy as np
import pytest
from input.realsense_rgbd_source import RGBDFrame
from perception.rgbd_geometry import CameraIntrinsics, frame_capture_time
from perception.scene_frame import SceneFrameInitializer, scene_coordinates


def sample(index=1, camera_translation=(0., 0., 0.), angle_deg=0.):
    theta = np.deg2rad(angle_deg)
    rotation = np.array([[np.cos(theta), -np.sin(theta), 0.],
                         [np.sin(theta), np.cos(theta), 0.], [0., 0., 1.]])
    origin = np.array([0., 0., .6]) + camera_translation
    normal = rotation @ [0., 0., -1.]
    x = rotation @ [1., 0., 0.]
    y = np.cross(normal, x)
    corners = np.array([origin + u*x + v*y for u, v in
                        ((-.1,-.05), (.1,-.05), (.1,.05), (-.1,.05))])
    k = CameraIntrinsics(800, 600, 800, 800, 400, 300)
    def project(points):
        return np.column_stack((points[:,0]/points[:,2]*k.fx+k.ppx,
                                points[:,1]/points[:,2]*k.fy+k.ppy))
    frame = RGBDFrame(np.full((600,800,3), 230, np.uint8),
        np.full((600,800), int(origin[2]*1000), np.uint16), k, .001,
        dict(host_utc=f'2026-10-14T00:00:{index:02d}+00:00',
            color_frame_number=index, depth_frame_number=index), {})
    result = dict(source_frame_id=f'color_{index}_depth_{index}', timestamps=frame.timestamps,
        capture_timestamp=frame_capture_time(frame)[0], unit_position='m',
        coordinate_frame='camera_color_optical', pair_qualified=True,
        support_plane=dict(point_camera_m=origin.tolist(), normal_camera_xyz=normal.tolist(), inlier_ratio=.9),
        placement_targets=[dict(target_id='T0', candidate_available=True,
            table_center_camera_m=origin.tolist(), border_corners_px=project(corners).tolist(),
            target_hole_center_camera_m=(origin+.06*normal).tolist(),
            coordinate_frame='camera_color_optical', unit_position='m')],
        objects=[dict(color='green', candidate_available=True,
            green_point_camera_m=(origin+.15*x+.1*y+.08*normal).tolist(),
            red_point_camera_m=(origin+.15*x+.1*y+.06*normal).tolist(),
            coordinate_variant='HEIGHT_CONSTRAINED')])
    return frame, result, project(np.array([origin+.1*x]))[0]


def ready_case(**kwargs):
    frame, result, pixel = sample(**kwargs)
    method = SceneFrameInitializer(positive_x_pixel=pixel, min_stable_frames=2)
    assert not method.process(frame, result)['ready']
    frame, result, _ = sample(index=2, **kwargs)
    return method, frame, result, method.process(frame, result)


def test_camera_motion_and_rotation_leave_scene_points_identical():
    for translation, angle in (((0.,0.,0.), 0.), ((.025,.015,.04), 32.)):
        _, _, result, reference = ready_case(camera_translation=translation, angle_deg=angle)
        assert reference['ready']
        json.dumps(reference, allow_nan=False)
        converted = scene_coordinates(result, reference)
        assert converted['objects'][0]['green_point_scene_m'] == pytest.approx([.15,.1,.08])
        assert converted['objects'][0]['red_point_scene_m'] == pytest.approx([.15,.1,.06])
        assert converted['placement_targets'][0]['target_hole_center_scene_m'] == pytest.approx([0.,0.,.06])
        assert not converted['motion_usable']


def test_no_direction_is_not_a_complete_calibration():
    frame, result, _ = sample()
    reference = SceneFrameInitializer(min_stable_frames=2).process(frame, result)
    assert reference['status'] == 'DIRECTION_REFERENCE_REQUIRED'
    assert reference['T_scene_from_camera'] is None
    assert scene_coordinates(result, reference)['objects'][0]['red_point_scene_m'] is None


def test_movement_discards_old_transform_and_requires_stability_again():
    method, _, _, reference = ready_case()
    assert reference['ready']
    frame, result, _ = sample(index=3, camera_translation=(0.,0.,.02))
    changed = method.process(frame, result)
    assert not changed['ready'] and changed['T_scene_from_camera'] is None
    assert changed['camera_moved']
    assert changed['status'] == 'POSITIVE_END_PIXEL_EXPIRED_AFTER_CAMERA_MOVEMENT'
    frame, result, _ = sample(index=4)
    assert method.process(frame, result)['status'] == 'POSITIVE_END_PIXEL_EXPIRED_AFTER_CAMERA_MOVEMENT'


def test_missing_t0_resets_stability_and_never_reuses_reference():
    method, _, _, _ = ready_case()
    frame, result, _ = sample(index=3)
    result['placement_targets'][0]['candidate_available'] = False
    assert method.process(frame, result)['status'] == 'NO_UNIQUE_CURRENT_T0'
    frame, result, _ = sample(index=4)
    assert not method.process(frame, result)['ready']


def test_cross_frame_conversion_and_duplicate_input_are_rejected():
    method, frame, result, reference = ready_case()
    with pytest.raises(ValueError, match='increasing'):
        method.process(frame, result)
    old = copy.deepcopy(result)
    old['timestamps']['color_frame_number'] = 9
    with pytest.raises(ValueError, match='source mismatch'):
        scene_coordinates(old, reference)


def test_unqualified_pair_or_wrong_table_never_produces_reference():
    for key, value, status in [('pair_qualified', False, 'RGBD_PAIR_REJECTED'),
                              ('support_plane', None, 'NO_SUPPORTED_CURRENT_TABLE')]:
        frame, result, pixel = sample()
        result[key] = value
        reference = SceneFrameInitializer(positive_x_pixel=pixel, min_stable_frames=2).process(frame, result)
        assert not reference['ready'] and reference['status'] == status


def test_missing_points_stay_missing_and_invalid_transform_rejected():
    _, _, result, reference = ready_case()
    result['objects'][0]['candidate_available'] = False
    assert scene_coordinates(result, reference)['objects'][0]['green_point_scene_m'] is None
    reference['T_scene_from_camera'][0][0] = 4.
    with pytest.raises(ValueError, match='invalid current scene transform'):
        scene_coordinates(result, reference)


def marker_sample(index=1, on_table=True, duplicate=False):
    frame, result, _ = sample(index=index)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 7, 64)
    frame.color_bgr[400:464, 360:424] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    if not on_table:
        frame.aligned_depth[400:464, 360:424] = 550
    if duplicate:
        frame.color_bgr[400:464, 500:564] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame, result


def test_real_aruco_detection_defines_signed_table_axis():
    method = SceneFrameInitializer(aruco_id=7, min_stable_frames=2)
    for i in (1,2):
        frame, result = marker_sample(i)
        reference = method.process(frame, result)
    assert reference['ready']
    assert reference['direction_reference'] == 'FIXED_TABLE_ARUCO_0_TO_1'
    assert scene_coordinates(result, reference)['objects'][0]['red_point_scene_m'] == pytest.approx([.15,.1,.06])


@pytest.mark.parametrize('kwargs,expected', [({'on_table':False}, 'ARUCO_NOT_COPLANAR_WITH_TABLE'),
    ({'duplicate':True}, 'ARUCO_MISSING_OR_DUPLICATE')])
def test_elevated_or_duplicate_marker_is_rejected(kwargs, expected):
    frame, result = marker_sample(**kwargs)
    reference = SceneFrameInitializer(aruco_id=7, min_stable_frames=2).process(frame, result)
    assert reference['status'] == expected


def test_missing_aruco_never_recovers_from_a_saved_reference():
    frame, result, _ = sample()
    reference = SceneFrameInitializer(aruco_id=7, min_stable_frames=2).process(frame, result)
    assert reference['status'] == 'ARUCO_MISSING_OR_DUPLICATE'
