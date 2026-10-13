"""Explicit offline replay publisher. No camera code, no timestamp rewriting."""
import json
import time
from pathlib import Path
from .core import to_ros

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from mtc_interfaces.msg import BatteryDetectionArray, BatteryDetection
    rclpy.init()
    node=None
    try:
        node=Node('vision_offline_replay')
        node.declare_parameter('input_jsonl','')
        node.declare_parameter('period_s',1.0)
        node.declare_parameter('wait_for_subscriber_s',0.0)
        path=Path(node.get_parameter('input_jsonl').value)
        records=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line]
        if not records or any(r.get('mode') not in ('replay','synthetic') for r in records):
            raise ValueError('explicit replay/synthetic records required')
        qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
        pub=node.create_publisher(BatteryDetectionArray,'/mtc/perception/batteries',qos)
        node.get_logger().warning('OFFLINE REPLAY: original timestamps; invalid poses; never motion input')
        index=0
        complete=False
        wait_s=float(node.get_parameter('wait_for_subscriber_s').value)
        discovery_deadline=time.monotonic()+wait_s
        q=pub.qos_profile
        node.get_logger().info(f'created QoS: {q.reliability.name} {q.history.name}({q.depth}) {q.durability.name}')
        def tick():
            nonlocal index,complete
            # Optional bounded discovery wait keeps the first offline frame
            # until an acceptance subscriber has actually matched.
            if index==0 and wait_s>0 and pub.get_subscription_count()==0:
                if time.monotonic()>=discovery_deadline:
                    raise TimeoutError('no matched replay subscriber')
                return
            record=records[index]
            pub.publish(to_ros(record['message'],BatteryDetectionArray,BatteryDetection))
            node.get_logger().info(f"frame={index} mode={record['mode']} source={record['timestamp_source']}")
            index+=1
            if index==len(records):
                timer.cancel()
                # Do not destroy a RELIABLE publisher with its final sample
                # still awaiting acknowledgment. No motion consumer is assumed.
                if not pub.wait_for_all_acked(Duration(seconds=2.0)):
                    raise TimeoutError('final replay samples not acknowledged')
                complete=True
                node.get_logger().info('replay complete')
        timer=node.create_timer(float(node.get_parameter('period_s').value),tick)
        while rclpy.ok() and not complete:
            rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            # SIGINT may already have shut down the context in Jazzy.
            rclpy.try_shutdown()
