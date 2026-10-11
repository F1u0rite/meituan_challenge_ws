"""Diagnostic-only TCP to BatteryDetectionArray bridge; no TF or motion."""
import json
import math
import time
from .core import to_ros
from .live_protocol import diagnostic_array
from .live_transport import TCPReceiver


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from mtc_interfaces.msg import BatteryDetectionArray, BatteryDetection
    from std_msgs.msg import String

    rclpy.init()
    node, receiver = None, None
    started = time.monotonic()
    try:
        node = Node('live_tcp_bridge')
        for name, default in (('listen_port', 8765), ('max_wall_age_s', 5.),
                              ('max_wall_future_s', 1.), ('max_queue_age_s', .5)):
            node.declare_parameter(name, default)
        port = node.get_parameter('listen_port').value
        age = node.get_parameter('max_wall_age_s').value
        future = node.get_parameter('max_wall_future_s').value
        queue_age = node.get_parameter('max_queue_age_s').value
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError('invalid listen_port')
        if not all(type(v) in (int,float) and math.isfinite(v) and v > 0
                   for v in (age,future,queue_age)):
            raise ValueError('positive finite timeout parameters required')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        publisher = node.create_publisher(BatteryDetectionArray, '/mtc/perception/batteries', qos)
        diagnostics = node.create_publisher(String, '/mtc/perception/live_diagnostics', qos)
        receiver = TCPReceiver(port, age, future)
        q=publisher.qos_profile
        node.get_logger().info(f'LIVE_TCP_QOS {q.reliability.name} {q.history.name}({q.depth}) {q.durability.name}')
        node.get_logger().info(f'LIVE_TCP_READY host=127.0.0.1 port={receiver.port}')
        node.get_logger().warning('DIAGNOSTIC ONLY: no clock mapping; all body/ring poses invalid')

        ros_matched=False
        def tick():
            nonlocal ros_matched
            if (not ros_matched and publisher.get_subscription_count()>0
                    and diagnostics.get_subscription_count()>0):
                ros_matched=True
                node.get_logger().info('LIVE_TCP_ROS_MATCHED batteries and diagnostics')
            item = receiver.latest()
            if item is None:
                return
            message = item['message']
            reason = item['reason']
            if reason is None and (time.monotonic_ns()-item['receive_monotonic_ns'])*1e-9 > queue_age:
                reason = 'QUEUE_OLD'
                receiver.count('dropped_QUEUE_OLD')
            if reason is None:
                data = diagnostic_array(message)
                publisher.publish(to_ros(data, BatteryDetectionArray, BatteryDetection))
                receiver.count('published')
                receiver.count('published_objects', len(data['detections']))
                receiver.count('filtered_objects', len(message['objects'])-len(data['detections']))
            diagnostic = dict(item, reason=reason, disposition='dropped' if reason else 'published',
                              ubuntu_ros_publish_stamp_ns=node.get_clock().now().nanoseconds,
                              ros_header_source='untrusted_windows_capture_wall_time',
                              clock_mapping_verified=False, motion_usable=False,
                              body_pose_valid=False, ring_pose_valid=False)
            diagnostics.publish(String(data=json.dumps(diagnostic, allow_nan=False,
                                                       separators=(',', ':'))))

        def report():
            elapsed = max(time.monotonic()-started, .001)
            stats = receiver.stats()
            stats['received_hz'] = round(stats.get('received',0)/elapsed, 3)
            node.get_logger().info('LIVE_TCP_STATS '+json.dumps(stats, sort_keys=True))

        node.create_timer(.01, tick)
        node.create_timer(5., report)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if receiver is not None:
                receiver.close()
                if node is not None:
                    node.get_logger().info('LIVE_TCP_STATS '+json.dumps(receiver.stats(), sort_keys=True))
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                rclpy.try_shutdown()
