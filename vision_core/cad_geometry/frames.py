"""Proper SE(3) transforms in metres. No implicit camera/base calibration."""
import numpy as np
def validate_transform(T):
    T=np.asarray(T,float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(T[:3,:3]),1):
        raise ValueError('Expected proper right-handed rigid transform')
    return T
def transform_points(points,T):
    T=validate_transform(T)
    return np.asarray(points)@T[:3,:3].T+T[:3,3]
def calibrated_pose(T_camera_body,T_base_camera=None):
    validate_transform(T_camera_body)
    if T_base_camera is None:raise ValueError('CAMERA_TO_BASE_NOT_CALIBRATED')
    return validate_transform(T_base_camera)@T_camera_body
def axis_yaw(axis):
    return float(np.degrees(np.arctan2(axis[1],axis[0]))%180)
def periodic_error(a,b):
    return float((a-b+90)%180-90)
