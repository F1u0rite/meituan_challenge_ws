"""Publish an existing perception result. No camera or perception execution here."""
from collections import Counter
import json
import time
from .native_rgbd import native_array, native_filter_reasons
from .core import to_ros


class NativeResultPublisher:
    """Opt-in ROS2 sink for the single H1 pipeline, including empty safe arrays."""
    def __init__(self,wait_for_subscriber_s=0.):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
        from mtc_interfaces.msg import BatteryDetectionArray,BatteryDetection
        from std_msgs.msg import String
        self.rclpy=rclpy;self.context=Context();self.node=None;self.executor=None;self.closed=False
        self.Array=BatteryDetectionArray;self.Detection=BatteryDetection;self.String=String
        self.messages=0;self.candidates=0;self.published=0;self.filtered=0;self.reasons=Counter()
        rclpy.init(context=self.context)
        try:
            self.node=Node('native_rgbd_result_publisher',context=self.context)
            self.executor=SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
            self.pub=self.node.create_publisher(BatteryDetectionArray,'/mtc/perception/batteries',qos)
            self.diag=self.node.create_publisher(String,'/mtc/perception/native_diagnostics',qos)
            deadline=time.monotonic()+wait_for_subscriber_s
            if wait_for_subscriber_s>0:
                while self.pub.get_subscription_count()==0 or self.diag.get_subscription_count()==0:
                    if time.monotonic()>=deadline:raise TimeoutError('native result subscribers did not match')
                    self.executor.spin_once(timeout_sec=.05)
        except BaseException:
            if self.executor is not None:self.executor.shutdown()
            if self.node is not None:self.node.destroy_node()
            rclpy.try_shutdown(context=self.context)
            raise

    def snapshot(self):
        return dict(messages=self.messages,candidates=self.candidates,published_targets=self.published,
            filtered_targets=self.filtered,filter_reason_counts=dict(self.reasons),
            header_policy='ZERO_UNMAPPED_CAMERA_CLOCK',body_pose_valid=False,ring_pose_valid=False,
            camera_started_by_ros_adapter=False)

    def publish_result(self,result,mode,pipeline_diagnostics):
        if self.closed:raise RuntimeError('native result publisher closed')
        if result.get('body_pose_valid') or result.get('ring_pose_valid') or any(
                obj.get('body_pose_valid') or obj.get('ring_pose_valid') for obj in result['objects']):
            raise ValueError('H2 accepts INVALID candidate poses only')
        data=native_array(result)
        filters=[]
        for obj in result['objects']:
            reasons=native_filter_reasons(obj)
            filters.append(dict(object_id=obj.get('object_id'),color=obj.get('color'),
                confidence=obj.get('confidence'),detection_status=obj.get('detection_status'),
                detection_ambiguity=obj.get('detection_ambiguity'),detection_stale=obj.get('detection_stale'),
                published=not reasons,filter_reasons=reasons,
                confirmation=obj.get('confirmation'),rejection_reason=obj.get('rejection_reason')))
        counts=dict(candidates=len(filters),published=len(data['detections']),
            filtered=sum(not f['published'] for f in filters))
        # Serialize before publishing either topic, so invalid JSON cannot half-dispatch.
        payload=dict(mode=mode,actual_camera_started=mode=='live',result=result,
            source_frame_id=result.get('source_frame_id'),timestamps=result.get('timestamps'),
            filter_report=filters,frame_counts=counts,pipeline_diagnostics=pipeline_diagnostics,
            body_pose_valid=False,ring_pose_valid=False,camera_to_ros_clock_verified=False,
            motion_usable=False,ros_header_source='ZERO_UNMAPPED_CAMERA_CLOCK',
            ubuntu_ros_dispatch_ns=self.node.get_clock().now().nanoseconds,
            camera_started_by_ros_adapter=False)
        encoded=json.dumps(payload,allow_nan=False,separators=(',',':'))
        self.pub.publish(to_ros(data,self.Array,self.Detection))
        self.diag.publish(self.String(data=encoded))
        self.executor.spin_once(timeout_sec=0.)
        self.messages+=1;self.candidates+=counts['candidates'];self.published+=counts['published']
        self.filtered+=counts['filtered']
        for f in filters:self.reasons.update(f['filter_reasons'])
        return dict(counts,filter_report=filters,cumulative=self.snapshot())

    def close(self):
        if self.closed:return
        self.closed=True
        from rclpy.duration import Duration
        try:
            if not self.pub.wait_for_all_acked(Duration(seconds=2.)) or not self.diag.wait_for_all_acked(Duration(seconds=2.)):
                raise TimeoutError('native result final samples not acknowledged')
        finally:
            try:
                self.executor.shutdown()
                self.node.destroy_node()
            finally:self.rclpy.try_shutdown(context=self.context)
