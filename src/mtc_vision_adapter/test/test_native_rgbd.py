"""Native vision preparation: synthetic RGB-D through real core and ROS2, no device."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime,timezone

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy,LivelinessPolicy)
from mtc_interfaces.msg import BatteryDetectionArray,BatteryDetection
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict
from mtc_vision_adapter.native_rgbd import native_array
from mtc_vision_adapter.core import to_ros


def test_native_confirmed_mapping_still_has_no_motion_pose():
    result=dict(coordinate_frame='camera_color_optical',objects=[
        dict(object_id='red',color='red',confidence=.9,detection_status='CONFIRMED_OBJECT',
             body_pose_candidate=np.eye(4).tolist(),ring_pose_valid=True),
        dict(object_id='raw',color='blue',confidence=.8,detection_status='RAW_CANDIDATE'),
        dict(object_id='unknown',color='unknown',confidence=.8,detection_status='CONFIRMED_OBJECT'),
        dict(object_id='stale',color='yellow',confidence=.8,detection_status='CONFIRMED_OBJECT',stale_status='STALE')])
    message=to_ros(native_array(result),BatteryDetectionArray,BatteryDetection)
    assert len(message.detections)==1 and message.detections[0].color==1
    assert message.detections[0].object_id=='red'
    assert message.header.stamp.sec==0 and message.header.stamp.nanosec==0
    d=message.detections[0]
    assert not d.ring_pose_valid
    for pose in (d.body_pose,d.ring_pose):
        assert all(math.isnan(v) for v in (pose.pose.position.x,pose.pose.position.y,
            pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,
            pose.pose.orientation.z,pose.pose.orientation.w))


def test_native_duplicate_confirmed_id_rejects():
    item=dict(object_id='same',color='red',confidence=.9,detection_status='CONFIRMED_OBJECT')
    with pytest.raises(ValueError,match='duplicate'):
        native_array(dict(objects=[item,item]))


def test_native_real_offline_ros_process(tmp_path,monkeypatch):
    root=Path(__file__).resolve().parents[3]
    core=root/'vision_core'
    from mtc_vision_adapter.native_rgbd import load_core
    load_core(str(core))
    from input.realsense_rgbd_source import RGBDFrame
    from perception.rgbd_geometry import CameraIntrinsics
    from real_data.rgbd_recording import save_rgbd_sample
    out=Path(os.environ.get('MTC_NATIVE_VALIDATION_OUT',str(tmp_path/'evidence')))
    out.mkdir(parents=True,exist_ok=True)
    dataset=out/'synthetic_rgbd'
    if dataset.exists():
        # Repeated tests use unique directories, preserving older evidence.
        dataset=out/('synthetic_rgbd_'+str(time.time_ns()))
    timestamps=[]
    for i in range(4):
        color=np.full((480,640,3),90,np.uint8)
        color[180:290,270:370]=(0,0,220)
        depth=np.full((480,640),800,np.uint16)
        depth[180:290,270:370]=600
        stamp={'host_utc':datetime.fromtimestamp(1700000000+i*.25,timezone.utc).isoformat(),
               'color_sdk_ms':1000.+i*250.,'depth_sdk_ms':1000.+i*250.,
               'color_timestamp_domain':'hardware_clock','depth_timestamp_domain':'hardware_clock',
               'color_frame_number':i,'depth_frame_number':i}
        timestamps.append(stamp)
        frame=RGBDFrame(color,depth,CameraIntrinsics(640,480,750,750,320,240),.001,
                        stamp,dict(color=dict(format='BGR8'),depth=dict(format='Z16')))
        save_rgbd_sample(dataset/f'sample_{i:04d}',frame,{},np.empty((0,3)),
                         np.empty((0,3),np.uint8),color,scene_label='SIMULATED_NATIVE_TEST_ONLY')
    monkeypatch.setenv('ROS_DOMAIN_ID','211')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER',raising=False)
    monkeypatch.delenv('ROS_STATIC_PEERS',raising=False)
    executable=root/'install/mtc_vision_adapter/lib/mtc_vision_adapter/native_rgbd_publisher'
    assert executable.is_file()
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE,
                   liveliness=LivelinessPolicy.AUTOMATIC)
    rclpy.init();node=Node('native_offline_acceptance')
    arrays,diagnostics=[],[]
    node.create_subscription(BatteryDetectionArray,'/mtc/perception/batteries',
                             lambda msg:arrays.append(message_to_ordereddict(msg)),qos)
    node.create_subscription(String,'/mtc/perception/native_diagnostics',
                             lambda msg:diagnostics.append(json.loads(msg.data)),qos)
    log=open(out/'native_publisher.log','w')
    proc=subprocess.Popen([str(executable),'--ros-args','-p','offline_dir:='+str(dataset),
                           '-p','vision_core_path:='+str(core),'-p','period_s:=0.25',
                           '-p','wait_for_subscriber_s:=15.0'],
                           stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                           env=dict(os.environ,ROS_LOG_DIR=str(out/'ros_logs')))
    result={'passed':False,'actual_camera_started':False}
    try:
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if proc.poll() is not None:
                end=time.monotonic()+.5
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
                break
        assert proc.poll()==0,(proc.poll(),(out/'native_publisher.log').read_text())
        assert len(arrays)==4 and len(diagnostics)==4
        objects=[]
        for i,(array,diagnostic) in enumerate(zip(arrays,diagnostics)):
            assert diagnostic['actual_camera_started'] is False
            row=diagnostic['result']
            assert row['timestamps']==timestamps[i]
            assert row['camera_to_ros_clock_verified'] is False and row['motion_usable'] is False
            assert row['body_pose_valid'] is False and row['ring_pose_valid'] is False
            assert row['capture_timestamp_source'].startswith('HOST_RECEIPT_TIME_FALLBACK')
            assert row['objects'],'actual core should detect synthetic red RGB-D candidate'
            assert all(obj['color']=='red' and obj['surface_depth_z']==pytest.approx(.6)
                       for obj in row['objects'])
            assert all(obj['detection_status']!='CONFIRMED_OBJECT' for obj in row['objects'])
            # Missing private CAD/table evidence is not bypassed to fill ROS arrays.
            assert array['detections']==[]
            assert array['header']['stamp']=={'sec':0,'nanosec':0}
            objects.append(len(row['objects']))
        assert 'Traceback' not in (out/'native_publisher.log').read_text()
        deadline=time.monotonic()+10
        while any(name=='native_rgbd_publisher' for name,_ in node.get_node_names_and_namespaces()):
            assert time.monotonic()<deadline
            rclpy.spin_once(node,timeout_sec=.1)
        result.update(passed=True,received_arrays=len(arrays),received_diagnostics=len(diagnostics),
                      core_red_candidates_per_frame=objects,
                      published_confirmed_objects=0,exit_code=proc.returncode,
                      private_cad='NOT AVAILABLE',remaining_nodes=[])
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close();node.destroy_node();rclpy.try_shutdown()
        (out/'native_results.json').write_text(json.dumps(result,indent=2))
        (out/'native_diagnostics.json').write_text(json.dumps(diagnostics,indent=2))
        (out/'native_received.json').write_text(json.dumps(arrays,indent=2))


def test_native_camera_mode_refuses_without_authorization(tmp_path):
    root=Path(__file__).resolve().parents[3]
    executable=root/'install/mtc_vision_adapter/lib/mtc_vision_adapter/native_rgbd_publisher'
    result=subprocess.run([str(executable),'--ros-args','-p','input_mode:=d435i'],
                           capture_output=True,text=True,timeout=10,
                           env=dict(os.environ,ROS_DOMAIN_ID='212',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'))
    assert result.returncode!=0
    assert 'Human confirmation required' in result.stderr
    assert 'pyrealsense2' not in result.stderr
