import pytest
from synthetic.geometry import mm_to_px,px_to_mm,normalize_yaw,yaw_error


def test_conversion():
    assert px_to_mm(mm_to_px((123.4,567.8),2),2)==pytest.approx((123.4,567.8))


def test_yaw():
    assert normalize_yaw(-1)==179
    assert normalize_yaw(181)==1
    assert yaw_error(179,1)==2
    assert yaw_error(0,90)==90
