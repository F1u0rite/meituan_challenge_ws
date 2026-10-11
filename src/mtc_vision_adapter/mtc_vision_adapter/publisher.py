"""Explicit offline replay publisher. No camera code, no timestamp rewriting."""
import json
from pathlib import Path
from .core import to_ros

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from mtc_interfaces.msg import BatteryDetectionArray, BatteryDetection
    rclpy.init()
    node=None
    try:
        node=Node('vision_offline_replay')
        node.declare_parameter('input_jsonl','')
        node.declare_parameter('period_s',1.0)
        path=Path(node.get_parameter('input_jsonl').value)
        records=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line]
        if not records or any(r.get('mode') not in ('replay','synthetic') for r in records):
            raise ValueError('explicit replay/synthetic records required')
        qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
        pub=node.create_publisher(BatteryDetectionArray,'/mtc/perception/batteries',qos)
        node.get_logger().warning('OFFLINE REPLAY: original timestamps; invalid poses; never motion input')
        index=0
        def tick():
            nonlocal index
            if index>=len(records):
                timer.cancel(); node.get_logger().info('replay complete'); return
            record=records[index]
            pub.publish(to_ros(record['message'],BatteryDetectionArray,BatteryDetection))
            node.get_logger().info(f"frame={index} mode={record['mode']} source={record['timestamp_source']}")
            index+=1
        timer=node.create_timer(float(node.get_parameter('period_s').value),tick)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            # SIGINT may already have shut down the context in Jazzy.
            rclpy.try_shutdown()
