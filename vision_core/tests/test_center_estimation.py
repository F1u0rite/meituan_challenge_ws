import cv2
import numpy as np
import pytest

from perception.center_estimation import (contour_centroid, mask_centroid,
    min_area_rectangle, robust_rectangle_fit, yaw_delta_180,
    estimate_visible_geometry)


def largest_contour(mask):
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    return max(contours,key=cv2.contourArea)


def test_center_definitions_on_simple_rectangle():
    mask=np.zeros((140,180),np.uint8)
    cv2.rectangle(mask,(30,20),(130,100),255,-1)
    contour=largest_contour(mask)
    for center in (mask_centroid(mask),contour_centroid(contour),
                   min_area_rectangle(contour).center,
                   robust_rectangle_fit(contour).center):
        assert np.allclose(center,(80,60),atol=1)


def test_mask_hole_changes_color_centroid_but_not_external_contour():
    mask=np.zeros((140,180),np.uint8)
    cv2.rectangle(mask,(30,20),(130,100),255,-1)
    cv2.rectangle(mask,(90,30),(125,55),0,-1)
    contour=largest_contour(mask)
    assert mask_centroid(mask)[0] < contour_centroid(contour)[0]


def test_robust_fit_reduces_small_protrusion_leverage():
    mask=np.zeros((180,240),np.uint8)
    cv2.rectangle(mask,(30,50),(130,130),255,-1)
    cv2.rectangle(mask,(130,85),(195,90),255,-1)
    contour=largest_contour(mask)
    baseline=min_area_rectangle(contour)
    robust=robust_rectangle_fit(contour)
    assert abs(robust.center[0]-80) < abs(baseline.center[0]-80)
    assert robust.corners.shape==(4,2)


def test_yaw_periodicity_and_empty_masks():
    assert yaw_delta_180(179,1)==2
    assert yaw_delta_180(0,180)==0
    with pytest.raises(ValueError):
        mask_centroid(np.zeros((5,5),np.uint8))
    with pytest.raises(ValueError):
        robust_rectangle_fit(np.array([[[2,2]]],np.int32))


def test_visible_geometry_yaw_gate_and_no_3d_pose():
    tall=np.zeros((180,180),np.uint8)
    cv2.rectangle(tall,(60,20),(120,150),255,-1)
    result=estimate_visible_geometry(largest_contour(tall))
    assert result.yaw_valid and result.center_method=='detected_contour_centroid'
    assert not result.body_pose_valid and not result.ring_pose_valid
    square=np.zeros((180,180),np.uint8)
    cv2.rectangle(square,(40,40),(140,140),255,-1)
    assert estimate_visible_geometry(largest_contour(square)).yaw_reason=='NEAR_SQUARE_SILHOUETTE'
