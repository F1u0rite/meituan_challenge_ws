"""Native RGBDFrame -> existing vision_core -> diagnostic-only native ROS data."""
import importlib
import json
import re
from pathlib import Path
from .core import COLORS


def load_core(core_path=''):
    if core_path:
        import sys
        root=Path(core_path).expanduser().resolve()
        if not (root/'perception/dual_rate_rgbd.py').is_file():
            raise ValueError('vision_core_path must point to existing vision_core source')
        sys.path.insert(0,str(root))
    return importlib.import_module('perception.dual_rate_rgbd')


def native_filter_reasons(obj):
    """Existing detection admission policy, separate from CAD pose uncertainty."""
    reasons=[]
    if obj.get('detection_status')!='CONFIRMED_OBJECT':reasons.append('DETECTION_NOT_CONFIRMED')
    if obj.get('color') not in COLORS or obj.get('color')=='unknown':reasons.append('COLOR_UNKNOWN')
    if obj.get('detection_ambiguity') is True:reasons.append('DETECTION_AMBIGUOUS')
    stale=(obj.get('detection_stale') is not False if 'detection_stale' in obj
           else obj.get('stale_status')=='STALE')
    if stale:reasons.append('DETECTION_STALE_OR_UNKNOWN')
    return reasons


def native_array(result):
    """Unknown camera/ROS mapping: zero stamp, NaN poses; never invent a map."""
    frame=result.get('coordinate_frame','camera_color_optical')
    if not isinstance(frame,str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*',frame):
        raise ValueError('invalid native coordinate frame')
    h={'frame_id':frame,'stamp':{'sec':0,'nanosec':0}}
    detections=[];seen=set()
    for obj in result['objects']:
        if native_filter_reasons(obj):
            continue
        identity=obj['object_id']
        if not identity or identity in seen:
            raise ValueError('duplicate/empty native object ID')
        seen.add(identity)
        invalid={'header':h,'pose':{'position':dict.fromkeys(('x','y','z')),
                                   'orientation':dict.fromkeys(('x','y','z','w'))}}
        detections.append(dict(object_id=identity,color=COLORS[obj['color']],
                               confidence=float(obj['confidence']),body_pose=invalid,
                               ring_pose=invalid,ring_pose_valid=False))
    return {'header':h,'detections':detections}


class NativeProcessor:
    def __init__(self,core_path=''):
        self.core=load_core(core_path)
        self.fast=self.core.FastPerception()
        root=Path(importlib.import_module('perception.realtime_rgbd').ROOT)
        self.cad=None
        self.cad_status='PRIVATE_CAD_ASSETS_OR_CACHE_MISSING'
        needed=(root/'outputs/cad_analysis/geometry.json',
                root/'outputs/cad_analysis/cad_meshes.npz',
                root/'assets/cad/电池道具.stp',root/'assets/cad/舌规V1.STEP')
        if all(p.is_file() for p in needed):
            try:
                self.cad=self.core.RGBDVisionPipeline()
                self.cad_status='CACHE_SOURCE_HASH_CHECKED; CANDIDATES_ONLY'
            except (OSError,ValueError,RuntimeError,KeyError) as error:
                self.cad_status='CAD_UNAVAILABLE: '+str(error)

    def process(self,frame,identity,sdk_deproject=None):
        cad_result=None
        if self.cad is not None:
            cad_result,_=self.cad.process(frame,sdk_deproject=sdk_deproject)
            plane=cad_result.get('background_filter_report',{}).get('table_plane')
            self.fast.set_table_hint(plane,self.core.capture_time(frame),identity)
        result,_=self.fast.process(frame,identity)
        if cad_result is not None:
            matches=self.core.associate_cad(result['objects'],cad_result['objects'])
            history={key:dict(object=value,source_frame_id=identity,
                              capture_timestamp=result['capture_timestamp'],
                              capture_timestamp_source=result['capture_timestamp_source'],
                              result_timestamp=cad_result['result_timestamp'])
                     for key,value in matches.items()}
            result=self.core.cad_for_display(result,identity,history)
        result.update(native_cad_status=self.cad_status,
                      ros_header_source='ZERO_UNMAPPED_CAMERA_CLOCK',
                      camera_to_ros_clock_verified=False,motion_usable=False,
                      body_pose_valid=False,ring_pose_valid=False)
        return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    from mtc_interfaces.msg import BatteryDetectionArray,BatteryDetection
    from std_msgs.msg import String
    import time
    rclpy.init()
    node,source=None,None
    try:
        node=Node('native_rgbd_publisher')
        for name,default in (('input_mode','offline'),('offline_dir',''),
                             ('vision_core_path',''),('camera_authorized',False),
                             ('period_s',.5),('wait_for_subscriber_s',0.)):
            node.declare_parameter(name,default)
        mode=node.get_parameter('input_mode').value
        if mode not in ('offline','d435i'):
            raise ValueError('input_mode must be offline or d435i')
        if mode=='d435i' and node.get_parameter('camera_authorized').value is not True:
            raise PermissionError('Human confirmation required: 准备好了，启动预览')
        processor=NativeProcessor(node.get_parameter('vision_core_path').value)
        qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
        pub=node.create_publisher(BatteryDetectionArray,'/mtc/perception/batteries',qos)
        diag=node.create_publisher(String,'/mtc/perception/native_diagnostics',qos)
        loader=importlib.import_module('real_data.rgbd_recording').load_rgbd_sample
        if mode=='offline':
            folder=node.get_parameter('offline_dir').value
            if not folder:raise ValueError('offline_dir required; no implicit camera fallback')
            paths=sorted(Path(folder).glob('sample_*'))
            if not paths:raise FileNotFoundError('no offline RGB-D samples')
            samples=iter(paths)
            def next_frame():
                p=next(samples)
                return p.name,loader(p)
        else:
            source=importlib.import_module('input.realsense_rgbd_source').RealSenseRGBDSource()
            source.start()
            count=0
            def next_frame():
                nonlocal count
                count+=1
                return 'native_'+str(count),source.read(latest=True)
        done=False
        wait_s=float(node.get_parameter('wait_for_subscriber_s').value)
        deadline=time.monotonic()+wait_s
        node.get_logger().info('NATIVE_READY mode='+mode+' cad='+processor.cad_status)
        def tick():
            nonlocal done
            if wait_s>0 and (pub.get_subscription_count()==0 or diag.get_subscription_count()==0):
                if time.monotonic()>deadline:raise TimeoutError('native subscribers did not match')
                return
            try:identity,frame=next_frame()
            except StopIteration:
                if not pub.wait_for_all_acked(Duration(seconds=2.)) or not diag.wait_for_all_acked(Duration(seconds=2.)):
                    raise TimeoutError('native final samples not acknowledged')
                done=True
                return
            result=processor.process(frame,identity,source.deproject if source else None)
            data=native_array(result)
            pub.publish(to_ros(data,BatteryDetectionArray,BatteryDetection))
            diag.publish(String(data=json.dumps(dict(mode=mode,actual_camera_started=source is not None,
                result=result,ubuntu_ros_dispatch_ns=node.get_clock().now().nanoseconds),
                allow_nan=False,separators=(',',':'))))
            node.get_logger().info(f'native frame={identity} candidates={len(result["objects"])} confirmed={len(data["detections"])}')
        from .core import to_ros
        node.create_timer(float(node.get_parameter('period_s').value),tick)
        while rclpy.ok() and not done:rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        try:
            if source is not None:source.stop()
        finally:
            try:
                if node is not None:node.destroy_node()
            finally:rclpy.try_shutdown()
