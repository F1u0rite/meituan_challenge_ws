import numpy as np
import pytest
from input.realsense_rgbd_source import RGBDFrame
from perception.rgbd_geometry import CameraIntrinsics
from perception.rgbd_offline import OfflineConfig, analyze_frame, fit_table, largest_component, grow_depth_object


class Candidate:
    def __init__(self, color, mask):
        self.color = color
        self.mask = mask
        self.detection_valid = True
        self.yaw_deg = 90.0


def scene():
    image = np.full((120, 160, 3), 90, np.uint8)
    depth = np.full((120, 160), 1000, np.uint16)
    red = np.zeros(depth.shape, bool)
    red[35:85, 50:110] = True
    image[red] = (0, 0, 220)
    depth[red] = 920
    depth[40:43, 55:58] = 0
    depth[50, 60] = 1800  # outlier
    yellow = np.zeros(depth.shape, bool)
    yellow[5:25, 5:40] = True  # table-colored false candidate
    intr = CameraIntrinsics(160, 120, 180, 180, 80, 60)
    return RGBDFrame(image, depth, intr, .001, {}, {}), red, yellow


def config():
    return OfflineConfig(table_inner_px=5, table_outer_px=30, table_min_points=200,
                         component_min_area_px=100, max_cloud_points=2000)


def test_table_foreground_rejects_background_and_preserves_invalid_body():
    frame, red, yellow = scene()
    report, masks, points, _ = analyze_frame(frame, [Candidate('red', red),
                                                     Candidate('yellow', yellow)], config())
    assert report['table_plane'] is not None
    assert report['table_plane']['inlier_ratio'] > .95
    assert report['table_plane']['mean_residual_m'] < .001
    assert report['candidate_counts_before'] == {'red': 1, 'yellow': 1}
    assert report['candidate_counts_after'] == {'red': 1}
    assert report['object_point_count'] > 100
    assert np.median(points[:, 2]) == pytest.approx(.92, abs=.005)
    assert np.max(points[:, 2]) < 1.0
    assert masks['object'][red].sum() > 100
    assert not report['body_pose_valid'] and not report['ring_pose_valid']
    assert not report['body_geometry']['body_pose_valid']
    assert report['body_geometry']['model_estimated_body_center_m'] is not None
    assert report['body_geometry']['visible_surface_center_m'] != report['body_geometry']['model_estimated_body_center_m']
    assert 0 <= report['body_geometry']['pca_obb_axis_difference_deg'] <= 90
    assert report['position_accuracy'] == 'NOT_MEASURED'


def test_no_table_or_empty_cloud_refuses_pose():
    frame, red, _ = scene()
    report, _, points, _ = analyze_frame(frame, [], config())
    assert report['table_failure'] == 'NO_RED_EXCLUSION_MASK'
    assert len(points) == 0 and not report['body_pose_valid']
    frame.aligned_depth[:] = 0
    report, _, points, _ = analyze_frame(frame, [Candidate('red', red)], config())
    assert report['table_failure'] == 'INSUFFICIENT_BACKGROUND_DEPTH'
    assert len(points) == 0


def test_alignment_and_components():
    frame, red, _ = scene()
    frame.aligned_depth = frame.aligned_depth[:, :-1]
    with pytest.raises(ValueError, match='sizes differ'):
        analyze_frame(frame, [Candidate('red', red)], config())
    mask, area = largest_component(np.zeros((10, 10), bool), 3)
    assert area == 0 and not mask.any()


def test_single_red_session_suppresses_second_candidate():
    frame, red, _ = scene()
    second = red.copy()
    second[:, 80:] = False
    report, _, _, _ = analyze_frame(frame, [Candidate('red', red), Candidate('red', second)], config())
    assert report['candidate_counts_after'] == {'red': 1}
    assert any(c['reason'] == 'SECOND_RED_CANDIDATE_IN_SINGLE_OBJECT_SESSION'
               for c in report['candidates'])


def test_depth_growth_rejoins_split_bands_without_inventing_hole_pixels():
    seed = np.zeros((100, 120), bool)
    seed[25:45, 35:85] = True
    foreground = seed.copy()
    foreground[55:75, 35:85] = True
    foreground[25:75, 105:115] = True  # unrelated object beyond spatial gate
    cfg = OfflineConfig(depth_bridge_px=21, grow_y_fraction=2.0,
                        grow_x_fraction=.1, max_growth_ratio=2.5)
    grown, info = grow_depth_object(seed, foreground, cfg)
    assert info['accepted'] and grown[60, 50]
    assert not grown[50, 50]  # absent depth remains absent
    assert not grown[40, 110]


def test_depth_growth_rejects_implausible_height_component():
    seed = np.zeros((100, 120), bool)
    seed[20:40, 30:70] = True
    foreground = seed.copy()
    foreground[50:70, 30:70] = True
    heights = np.zeros(seed.shape)
    heights[seed] = .05
    heights[50:70, 30:70] = .115
    grown, info = grow_depth_object(seed, foreground,
        OfflineConfig(depth_bridge_px=11, grow_y_fraction=2.0), heights)
    assert info['accepted'] and not grown[60, 50]


def test_plane_height_separable_and_ray_projection_equivalence():
    from perception.rgbd_offline import height_from_plane
    from perception.rgbd_geometry import CameraIntrinsics
    intr=CameraIntrinsics(50,40,90,95,23,18)
    depth=np.random.default_rng(3).uniform(.2,.8,(40,50));depth[0,0]=0
    v,u=np.indices(depth.shape)
    rays=np.stack([(u-intr.ppx)/intr.fx,(v-intr.ppy)/intr.fy,np.ones(depth.shape)],axis=-1)
    n=np.array([.08,-.12,.98954535]);o=np.array([.01,-.03,.5])
    plane=dict(normal_camera_xyz=n.tolist(),point_camera_m=o.tolist())
    expected=(o-rays*depth[...,None])@n
    assert np.allclose(height_from_plane(depth,intr,plane),expected,atol=2e-15,rtol=0)
    assert np.allclose(height_from_plane(depth,intr,plane,rays),expected,atol=2e-15,rtol=0)
