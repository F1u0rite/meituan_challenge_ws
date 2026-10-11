import numpy as np
import cv2
import pytest

from perception.rgbd_geometry import (CameraIntrinsics, analyze_rgbd, deproject_pixels,
                                      fit_plane_ransac, validate_frame)
from input.realsense_rgbd_source import RGBDFrame, RealSenseRGBDSource
from real_data.rgbd_recording import load_rgbd_sample, save_rgbd_sample
from perception.unified_detector import vision_result
from scripts.live_rgbd_preview import _diagnose


def fixture_frame():
    color = np.zeros((80, 100, 3), np.uint8)
    color[25:60, 30:75] = (0, 0, 200)
    depth = np.full((80, 100), 1000, np.uint16)
    depth[25:60, 30:75] = 850
    depth[30:33, 35:38] = 0
    depth[40, 50] = 5000
    intr = CameraIntrinsics(100, 80, 100, 100, 50, 40)
    mask = np.zeros((80, 100), bool)
    mask[25:60, 30:75] = True
    return color, depth, intr, mask


def test_deprojection_and_depth_units():
    intr = CameraIntrinsics(100, 80, 100, 100, 50, 40)
    xyz = deproject_pixels([[50, 40], [60, 30]], [.8, 1.0], intr)
    assert np.allclose(xyz, [[0, 0, .8], [.1, -.1, 1.0]])
    zero_distortion = CameraIntrinsics(100, 80, 100, 100, 50, 40,
                                       'inverse_brown_conrady', (0., 0., 0., 0., 0.))
    assert np.allclose(deproject_pixels([[50, 40]], [.8], zero_distortion), [[0, 0, .8]])
    with pytest.raises(ValueError, match='SDK'):
        deproject_pixels([[50, 40]], [.8], CameraIntrinsics(100, 80, 100, 100, 50, 40,
                         'brown_conrady', (0.1,)))


def test_alignment_validation_empty_mask_and_holes():
    color, depth, intr, mask = fixture_frame()
    validate_frame(color, depth, intr, .001)
    with pytest.raises(ValueError, match='sizes differ'):
        validate_frame(color, depth[:, :-1], intr, .001)
    with pytest.raises(ValueError, match='depth scale'):
        validate_frame(color, depth, intr, 0)
    result, points, _ = analyze_rgbd(color, depth, np.zeros_like(mask), intr, .001)
    assert not result['body_pose_valid'] and not result['ring_pose_valid'] and len(points) == 0
    result, points, _ = analyze_rgbd(color, depth, mask, intr, .001)
    assert 0.98 < result['depth_valid_ratio'] < 1
    assert result['visible_surface_valid'] and not result['body_pose_valid']
    assert abs(result['visible_surface_position_m'][2]-.85) < .005
    assert np.max(points[:, 2]) < 1.0  # rejects hole and 5 m outlier
    assert result['table_position_m'] is None and result['base_position_m'] is None


def test_plane_fit_and_candidate_not_table_pose():
    color, depth, intr, mask = fixture_frame()
    table = ~mask
    result, _, _ = analyze_rgbd(color, depth, mask, intr, .001, table_roi=table)
    assert result['plane_candidate']['inlier_count'] > 100
    assert result['plane_candidate']['mean_residual_m'] < .001
    assert result['plane_candidate']['candidate_only']
    assert not result['table_frame_valid'] and not result['body_pose_valid']
    assert fit_plane_ransac(np.zeros((2, 3))) is None


def test_lossless_record_replay_and_legacy_rgb_only(tmp_path):
    color, depth, intr, mask = fixture_frame()
    result, points, rgb = analyze_rgbd(color, depth, mask, intr, .001)
    frame = RGBDFrame(color, depth, intr, .001, {'color_sdk_ms': 12.0},
                      {'color': {'format': 'BGR8'}, 'depth': {'format': 'Z16'}})
    folder = save_rgbd_sample(tmp_path/'sample_001', frame, result, points, rgb, color,
                              scene_label='red_top')
    replay = load_rgbd_sample(folder)
    assert np.array_equal(replay.color_bgr, color)
    assert np.array_equal(replay.aligned_depth, depth)
    assert replay.aligned_depth.dtype == np.uint16
    assert (folder/'point_cloud.ply').exists()
    with pytest.raises(FileExistsError):
        save_rgbd_sample(folder, frame, result, points, rgb, color, scene_label='red_top')
    old = vision_result(color)
    assert old.coordinate_frame == 'IMAGE_PIXEL'
    assert all(not b.body_pose_valid and not b.ring_pose_valid for b in old.batteries)


def test_stream_selection_prefers_supported_combination():
    options = [(640, 480, 30), (640, 480, 15), (1280, 720, 15)]
    assert RealSenseRGBDSource._choose(options, (1280, 720), 15) == (1280, 720, 15)


def test_preview_handles_no_red_candidate():
    color, depth, intr, _ = fixture_frame()
    frame = RGBDFrame(color, depth, intr, .001, {}, {})
    class EmptyDetector:
        def detect_batteries(self, _):
            return []
    class FakeSource:
        deproject = None
    geometry, points, _, visualization = _diagnose(frame, EmptyDetector(), FakeSource(), .15, 2.0)
    assert geometry['candidate_count'] == 0 and not geometry['body_pose_valid']
    assert len(points) == 0 and visualization.shape == (80, 200, 3)
