"""Actual DDS JPEG delivery while the geometry worker is deliberately slow."""
import json
import threading
import time

import cv2
import numpy as np
import pytest

from mtc_vision_adapter.rgb_image_publisher import RGBImagePublisher, source_frame_id
from input.realsense_rgbd_source import RGBDFrame
from perception.rgbd_geometry import CameraIntrinsics


def frame(number=1):
    stamp = 1791910073. + number / 30
    return RGBDFrame(np.full((120, 160, 3), [20, 60, 120], np.uint8),
        np.full((120, 160), 550, np.uint16), CameraIntrinsics(160, 120, 100, 100, 80, 60),
        .001, dict(host_utc='2026-10-13T16:47:53+00:00', color_frame_number=number,
            depth_frame_number=number, color_sdk_ms=stamp*1000, depth_sdk_ms=stamp*1000,
            color_timestamp_domain='timestamp_domain.global_time',
            depth_timestamp_domain='timestamp_domain.global_time'), {},
        alignment_metadata=dict(method='SDK_DEPTH_TO_COLOR'))


@pytest.mark.parametrize('changes', [dict(fps=0), dict(fps=float('nan')), dict(fps=31), dict(quality=0)])
def test_image_options_fail_before_creating_ros_context(changes):
    with pytest.raises(ValueError):
        RGBImagePublisher(**changes)


def test_stale_overlay_is_rejected_before_encoding():
    sink = RGBImagePublisher.__new__(RGBImagePublisher)
    f = frame()
    with pytest.raises(ValueError, match='source frame'):
        sink.publish_overlay(f, dict(source_frame_id='color_999_depth_999'), mode='replay')


def test_rgb_updates_independently_of_slow_geometry_and_jpeg_matches_metadata(monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from perception.dual_rate_rgbd import CaptureWorker
    monkeypatch.setenv('ROS_DOMAIN_ID', '218')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    monkeypatch.delenv('ROS_STATIC_PEERS', raising=False)
    context = Context()
    rclpy.init(context=context)
    node = Node('yuze_rgb_test_receiver', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    video_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
    info_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
    images, infos = [], []
    node.create_subscription(CompressedImage, '/mtc/perception/rgb/compressed', images.append, video_qos)
    node.create_subscription(String, '/mtc/perception/rgb_metadata',
        lambda msg: infos.append(json.loads(msg.data)), info_qos)
    class RecordedFakeSource:
        def __init__(self): self.stopped=False; self.count=0
        def read(self, **kwargs):
            time.sleep(1/30)
            if self.stopped: raise RuntimeError('fake source stopped')
            self.count += 1
            return frame(self.count)
        def stop(self): self.stopped=True
    source = RecordedFakeSource()
    capture = sink = None
    completed = []
    slow_worker = None
    try:
        sink = RGBImagePublisher(fps=10, wait_for_subscriber_s=10)
        def preview(identity, f, report):
            sink.publish_frame(f, source_frame_id(f), mode='replay', pair_qualified=report['qualified'])
        capture = CaptureWorker(source, on_preview=preview)
        capture.start()
        def slow_geometry():
            for _ in range(2):
                current = capture.slot.get(timeout=2)
                assert current is not None
                time.sleep(.65)
                completed.append(current[1].timestamps['color_frame_number'])
        slow_worker = threading.Thread(target=slow_geometry)
        slow_worker.start()
        deadline = time.monotonic()+5
        while (slow_worker.is_alive() or len(images)<8 or len(infos)<8) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.03)
        slow_worker.join(timeout=1)
        assert len(completed)==2 and len(images)>=8 and len(infos)>=8
        assert capture.error is None
        by_stamp={(info['image_header_stamp']['sec'], info['image_header_stamp']['nanosec']):info for info in infos}
        matched=[(image, by_stamp[(image.header.stamp.sec,image.header.stamp.nanosec)]) for image in images
            if (image.header.stamp.sec,image.header.stamp.nanosec) in by_stamp]
        assert len(matched)>=8
        for image, info in matched:
            decoded=cv2.imdecode(np.frombuffer(image.data,dtype=np.uint8),cv2.IMREAD_COLOR)
            assert decoded.shape==(120,160,3)
            assert np.mean(np.abs(decoded.astype(float)-[20,60,120]))<5
            assert info['source_mode']=='replay' and info['replay'] is True
            assert info['image_header_policy']=='ROS_DISPATCH_TIME_NOT_CAPTURE_TIME'
            assert info['image_semantics']=='RAW_CURRENT_RGB'
            assert image.header.frame_id==info['coordinate_frame']=='camera_color_optical'
        ids=[info['timestamps']['color_frame_number'] for info in infos]
        assert len(ids)==len(set(ids))
        assert ids[-1]>completed[-1]  # Fresh video continued during the second slow geometry job.
    finally:
        if capture: capture.close()
        if slow_worker: slow_worker.join(timeout=2)
        if sink: sink.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown(context=context)
