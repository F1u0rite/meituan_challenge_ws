"""JPEG ROS video frames with explicit dispatch/capture clock separation."""
import json
import math
import threading
import time

IMAGE_TOPIC = '/mtc/perception/rgb/compressed'
IMAGE_METADATA_TOPIC = '/mtc/perception/rgb_metadata'
OVERLAY_TOPIC = '/mtc/perception/top_hole_overlay/compressed'
OVERLAY_METADATA_TOPIC = '/mtc/perception/top_hole_overlay_metadata'


class RGBImagePublisher:
    def __init__(self, *, fps=10., quality=80, wait_for_subscriber_s=0.):
        if not math.isfinite(fps) or not 0 < fps <= 30:
            raise ValueError('image fps must be between 0 and 30')
        if type(quality) is not int or not 1 <= quality <= 100:
            raise ValueError('JPEG quality must be 1..100')
        if not math.isfinite(wait_for_subscriber_s) or wait_for_subscriber_s < 0:
            raise ValueError('invalid subscriber wait')
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        self.rclpy, self.Image, self.String = rclpy, CompressedImage, String
        self.context = Context()
        self.node = self.executor = None
        self.lock = threading.Lock()
        self.closed = False
        self.interval, self.quality = 1. / fps, quality
        self.last_sent = None
        self.last_identity = None
        self.sequence = 0
        rclpy.init(context=self.context)
        try:
            self.node = Node('yuze_rgb_image_stream', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            # Old video is discarded instead of blocking new camera frames on network retries.
            video_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            metadata_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            self.image = self.node.create_publisher(CompressedImage, IMAGE_TOPIC, video_qos)
            self.metadata = self.node.create_publisher(String, IMAGE_METADATA_TOPIC, metadata_qos)
            self.overlay = self.node.create_publisher(CompressedImage, OVERLAY_TOPIC, video_qos)
            self.overlay_metadata = self.node.create_publisher(String, OVERLAY_METADATA_TOPIC, metadata_qos)
            if wait_for_subscriber_s:
                deadline = time.monotonic() + wait_for_subscriber_s
                while not (self.image.get_subscription_count() and self.metadata.get_subscription_count()):
                    if time.monotonic() >= deadline:
                        raise TimeoutError('RGB image and metadata subscribers must match')
                    self.executor.spin_once(timeout_sec=.05)
        except BaseException:
            self._destroy()
            raise

    def publish_frame(self, frame, source_frame_id, *, mode, pair_qualified=None):
        with self.lock:
            now = time.monotonic()
            if self.closed:
                raise RuntimeError('image publisher closed')
            if source_frame_id == self.last_identity or (self.last_sent is not None and now - self.last_sent < self.interval):
                return None
            metadata = self._publish(frame.color_bgr, frame, source_frame_id, mode,
                self.image, self.metadata, pair_qualified=pair_qualified,
                image_semantics='RAW_CURRENT_RGB')
            self.last_sent, self.last_identity = now, source_frame_id
            return metadata

    def publish_overlay(self, frame, result, *, mode):
        import cv2
        import numpy as np
        from perception.rgbd_geometry import project_points
        if result.get('source_frame_id') != source_frame_id(frame):
            raise ValueError('overlay/source frame mismatch')
        if result.get('timestamps') != frame.timestamps:
            raise ValueError('overlay/source capture timestamps mismatch')
        image = frame.color_bgr.copy()
        for obj in result['objects']:
            if not obj.get('candidate_available'):
                continue
            for key, color in [('green_point_camera_m', (0, 255, 0)),
                               ('red_point_camera_m', (0, 0, 255))]:
                points = np.asarray([obj[key]], dtype=float)
                if points.shape != (1, 3) or not np.isfinite(points).all() or points[0, 2] <= 0:
                    raise ValueError('invalid overlay coordinates')
                x, y = project_points(points, frame.intrinsics)[0]
                if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                    cv2.drawMarker(image, (round(x), round(y)), color, cv2.MARKER_CROSS, 14, 2)
        cv2.putText(image, 'TOP / CAD HOLE | RESULT SOURCE FRAME', (12, 25),
            cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
        for target in result.get('placement_targets', []):
            if target.get('candidate_available'):
                pixel = project_points([target['target_hole_center_camera_m']], frame.intrinsics)[0]
                x, y = np.rint(pixel).astype(int)
                if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                    cv2.drawMarker(image, (x,y), (255,0,255), cv2.MARKER_CROSS, 18, 2)
                    cv2.putText(image, 'T0 target hole', (x+10,y-8), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,0,255), 2)
        with self.lock:
            if self.closed:
                raise RuntimeError('image publisher closed')
            return self._publish(image, frame, result['source_frame_id'], mode,
                self.overlay, self.overlay_metadata,
                pair_qualified=result.get('pair_qualified'),
                image_semantics='ANNOTATED_INFERENCE_SOURCE_FRAME')

    def _publish(self, image, frame, sid, mode, image_pub, metadata_pub, **extra):
        import cv2
        from perception.rgbd_geometry import frame_capture_time
        if mode not in ('live', 'offline', 'replay'):
            raise ValueError('invalid source mode')
        if sid != source_frame_id(frame):
            raise ValueError('image/source frame mismatch')
        if image is None or image.ndim != 3 or image.shape[2] != 3 or str(image.dtype) != 'uint8':
            raise ValueError('image must be BGR uint8')
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        capture, capture_source = frame_capture_time(frame)
        message = self.Image()
        message.header.frame_id = 'camera_color_optical'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.format = 'bgr8; jpeg compressed bgr8'
        message.data = jpeg.tobytes()
        metadata = dict(schema='mtc.rgb_frame.v1', sequence=self.sequence + 1,
            source_frame_id=sid, source_mode=mode, replay=mode != 'live',
            coordinate_frame='camera_color_optical', capture_timestamp_s=capture,
            capture_timestamp_source=capture_source, timestamps=dict(frame.timestamps),
            image_header_stamp=dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
            image_header_policy='ROS_DISPATCH_TIME_NOT_CAPTURE_TIME',
            ros_capture_clock_verified=False, width=image.shape[1], height=image.shape[0],
            jpeg_bytes=len(message.data), **extra)
        encoded = json.dumps(metadata, allow_nan=False, separators=(',', ':'))
        image_pub.publish(message)
        metadata_pub.publish(self.String(data=encoded))
        self.sequence += 1
        return metadata

    def _destroy(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        self.rclpy.try_shutdown(context=self.context)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            from rclpy.duration import Duration
            try:
                acked = [p.wait_for_all_acked(Duration(seconds=2.))
                    for p in (self.metadata, self.overlay_metadata)]
                if not all(acked):
                    raise TimeoutError('image metadata not acknowledged')
            finally:
                self._destroy()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def source_frame_id(frame):
    return f"color_{frame.timestamps['color_frame_number']}_depth_{frame.timestamps['depth_frame_number']}"
