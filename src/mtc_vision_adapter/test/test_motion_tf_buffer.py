"""Local TF buffer checks; no camera, network listener or motion execution."""
import math
import pytest
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer
from geometry_msgs.msg import TransformStamped
from mtc_vision_adapter.core import motion_rejection

STAMP={'sec':1700000000,'nanosec':250000000}

def message():
    h={'frame_id':'camera_optical_frame','stamp':dict(STAMP)}
    def pose():
        return {'header':{'frame_id':h['frame_id'],'stamp':dict(STAMP)},'pose':{'position':dict(x=.1,y=.2,z=.3),'orientation':dict(x=0.,y=0.,z=0.,w=1.)}}
    return {'header':h,'detections':[dict(object_id='battery-1',color=1,confidence=1.,ring_pose_valid=True,body_pose=pose(),ring_pose=pose())]}

def validator(buffer):
    def verify(source,target,stamp):
        capture=Time(seconds=stamp['sec'],nanoseconds=stamp['nanosec'])
        t=buffer.lookup_transform(target,source,capture,timeout=Duration())
        q=t.transform.rotation;p=t.transform.translation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        return all(math.isfinite(v) for v in values) and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-3
    return verify

def check(buffer):
    return motion_rejection(message(),1700000000.25,mode='live',trusted_frames={'camera_optical_frame','base_link'},target_frame='base_link',tf_validator=validator(buffer))

def transform(nanosec):
    t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='camera_optical_frame'
    t.header.stamp.sec=STAMP['sec'];t.header.stamp.nanosec=nanosec;t.transform.rotation.w=1.
    return t

def test_missing_tf_rejects():
    assert check(Buffer())=='TF_UNVERIFIED'

def test_transform_at_capture_time_verified():
    b=Buffer();b.set_transform(transform(STAMP['nanosec']),'acceptance_fixture')
    assert check(b)=='NO_REJECTION'

def test_latest_tf_does_not_cover_historical_capture():
    b=Buffer();b.set_transform(transform(750000000),'acceptance_fixture')
    assert check(b)=='TF_UNVERIFIED'
