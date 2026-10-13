"""Ubuntu local TCP sender -> installed bridge -> real ROS2 subscriptions."""
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import struct
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
    LivelinessPolicy, qos_check_compatible)
from mtc_interfaces.msg import BatteryDetectionArray
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict
from mtc_vision_adapter.live_protocol import MAX_BYTES, diagnostic_array
from live_test_support import frame, battery, installed_executable


def test_real_live_tcp_ros_bridge(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','205')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER',raising=False)
    monkeypatch.delenv('ROS_STATIC_PEERS',raising=False)
    root=Path(__file__).resolve().parents[3]
    out=Path(os.environ.get('MTC_LIVE_VALIDATION_OUT',str(tmp_path/'evidence')))
    out.mkdir(parents=True,exist_ok=True)
    log_path=out/'live_bridge.log'
    executable=installed_executable(root, 'live_tcp_bridge')
    assert executable.is_file(),'build and source overlay first'
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE,
                   liveliness=LivelinessPolicy.AUTOMATIC)
    rclpy.init()
    node=Node('live_bridge_acceptance')
    arrays,diagnostics,cases=[],[],[]
    node.create_subscription(BatteryDetectionArray,'/mtc/perception/batteries',
                             lambda msg:arrays.append(message_to_ordereddict(msg)),qos)
    node.create_subscription(String,'/mtc/perception/live_diagnostics',
                             lambda msg:diagnostics.append(json.loads(msg.data)),qos)
    proc=None;connection=None
    result={'passed':False,'scope':'Ubuntu local simulated sender only; no D435i/Windows link/FSM'}
    log=open(log_path,'w')

    def wait(predicate,seconds=5.):
        deadline=time.monotonic()+seconds
        while not predicate():
            assert time.monotonic()<deadline,'timed out: '+log_path.read_text()[-2000:]
            assert proc.poll() is None,'bridge crashed: '+log_path.read_text()[-2000:]
            rclpy.spin_once(node,timeout_sec=.03)

    def send_payload(payload):
        connection.sendall(struct.pack('!I',len(payload))+payload)

    def send_checked(message,name,drop=None,fragment=False):
        before_arrays,before_diag=len(arrays),len(diagnostics)
        payload=json.dumps(message,allow_nan=False,separators=(',',':')).encode()
        packet=struct.pack('!I',len(payload))+payload
        if fragment:
            for chunk in (packet[:1],packet[1:3],packet[3:11],packet[11:]):
                connection.sendall(chunk)
        else:
            connection.sendall(packet)
        wait(lambda:len(diagnostics)>before_diag)
        diagnostic=diagnostics[-1]
        assert diagnostic['message']==message
        assert diagnostic['reason']==drop
        assert diagnostic['clock_mapping_verified'] is False and diagnostic['motion_usable'] is False
        assert diagnostic['receive_wall_ns']>0 and diagnostic['receive_monotonic_ns']>0
        assert diagnostic['ros_header_source']=='untrusted_windows_capture_wall_time'
        if drop:
            deadline=time.monotonic()+.12
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
            assert len(arrays)==before_arrays
        else:
            wait(lambda:len(arrays)>before_arrays)
            actual=arrays[-1];expected=diagnostic_array(message)
            assert actual['header']==expected['header']
            assert len(actual['detections'])==len(expected['detections'])
            for observed,target in zip(actual['detections'],expected['detections']):
                assert observed['object_id']==target['object_id']
                assert observed['color']==target['color']
                assert observed['confidence']==pytest.approx(target['confidence'],abs=1e-6)
                assert observed['ring_pose_valid'] is False
                for key in ('body_pose','ring_pose'):
                    assert observed[key]['header']==actual['header']
                    assert all(math.isnan(value) for group in observed[key]['pose'].values()
                               for value in group.values())
        cases.append({'case':name,'reason':drop,'ros_arrays':len(arrays)-before_arrays,
                      'object_ids':[] if drop else [o['object_id'] for o in arrays[-1]['detections']]})

    try:
        proc=subprocess.Popen([str(executable),'--ros-args','-p','listen_port:=0'],
                              stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,
                              start_new_session=True,
                              env=dict(os.environ,ROS_LOG_DIR=str(out/'ros_logs')))
        wait(lambda:re.search(r'LIVE_TCP_READY host=127.0.0.1 port=(\d+)',log_path.read_text()) is not None)
        port=int(re.search(r'port=(\d+)',log_path.read_text()).group(1))
        wait(lambda:bool(node.get_publishers_info_by_topic('/mtc/perception/batteries')) and
                    bool(node.get_publishers_info_by_topic('/mtc/perception/live_diagnostics')))
        wait(lambda:'LIVE_TCP_ROS_MATCHED' in log_path.read_text())
        assert 'LIVE_TCP_QOS RELIABLE KEEP_LAST(5) VOLATILE' in log_path.read_text()
        endpoints=node.get_publishers_info_by_topic('/mtc/perception/batteries')
        assert all(qos_check_compatible(ep.qos_profile,qos)[0].name=='OK' and
                   ep.qos_profile.reliability==ReliabilityPolicy.RELIABLE and
                   ep.qos_profile.durability==DurabilityPolicy.VOLATILE for ep in endpoints)
        connection=socket.create_connection(('127.0.0.1',port),timeout=3.)
        send_checked(frame(0),'single red',fragment=True)
        send_checked(frame(1,[battery('red','RED'),battery('yellow','YELLOW'),
                              battery('blue','BLUE'),battery('green','GREEN')]),'four colors')
        send_checked(frame(2,[battery('red-1'),battery('red-2')]),'same color multiple IDs')
        send_checked(frame(3,[]),'empty scene')
        send_checked(frame(4,[battery('unknown','UNKNOWN')]),'UNKNOWN filtered')
        send_checked(frame(5,[battery('amb',detection_status='AMBIGUOUS'),
                              battery('raw',detection_status='RAW_CANDIDATE'),
                              battery('reject',detection_status='REJECTED')]),'unconfirmed filtered')
        send_checked(frame(6,[battery('candidate',surface_depth_m=.6,
                                      body_position_candidate_m=[.1,.2,.6],
                                      body_orientation_candidate=[0.,0.,0.,1.],
                                      body_yaw_candidate_rad=.2,
                                      ring_position_candidate_m=[.1,.2,.65],
                                      ring_orientation_candidate=[0.,0.,0.,1.],
                                      insertion_direction_candidates=[[0.,0.,1.]])]),
                     'finite candidates stay invalid')
        send_checked(frame(7,[battery('stale-object',stale=True)]),'stale object filtered')
        old=frame(8);old['windows_capture_wall_time']['sec']-=60
        send_checked(old,'old wall timestamp','WALL_OLD')
        future=frame(9);future['windows_capture_wall_time']['sec']+=60
        send_checked(future,'future wall timestamp','WALL_FUTURE')
        send_checked(frame(8),'out of order sequence','SEQUENCE_ORDER')
        send_checked(frame(10,capture_timestamp_raw=1.),'capture regression','CAPTURE_ORDER')

        invalid=[]
        invalid.append(b'{')
        invalid.append(b'\xff')
        invalid.append(b'NaN')
        invalid.append(b'{"protocol_version":1,"protocol_version":1}')
        invalid.append(json.dumps(frame(11,protocol_version=2)).encode())
        invalid.append(json.dumps(frame(11,sequence_id=-1)).encode())
        invalid.append(json.dumps(frame(11,[battery(),battery()])).encode())
        invalid.append(json.dumps(frame(11,[battery(color='PURPLE')])).encode())
        invalid.append(json.dumps(frame(11,[battery(confidence=True)])).encode())
        invalid.append(json.dumps(frame(11,[battery(body_pose_valid=True)])).encode())
        invalid.append(json.dumps(frame(11,[battery(ring_pose_valid=True)])).encode())
        for payload in invalid:send_payload(payload)
        send_checked(frame(11),'survives eleven malformed frames')
        connection.close()
        connection=socket.create_connection(('127.0.0.1',port),timeout=3.)
        send_checked(frame(0,[battery('reconnected','BLUE')]),'reconnect resets sequence')
        connection.sendall(struct.pack('!I',MAX_BYTES+1))
        assert connection.recv(1)==b''
        cases.append({'case':'oversize prefix closes connection'})
        connection.close()
        connection=socket.create_connection(('127.0.0.1',port),timeout=3.)
        connection.sendall(struct.pack('!I',0))
        assert connection.recv(1)==b''
        connection.close()
        connection=socket.create_connection(('127.0.0.1',port),timeout=3.)
        connection.sendall(struct.pack('!I',20)+b'{')
        connection.close()
        connection=socket.create_connection(('127.0.0.1',port),timeout=4.)
        connection.sendall(struct.pack('!I',20)+b'{')
        assert connection.recv(1)==b''
        connection.close()
        connection=socket.create_connection(('127.0.0.1',port),timeout=3.)
        send_checked(frame(0,timestamp_domain='unknown',capture_timestamp_raw=None),
                     'unknown clock source','TIME_UNKNOWN')
        connection.close()
        connection=socket.create_connection(('127.0.0.1',port),timeout=3.)
        send_checked(frame(0,windows_capture_wall_time=None),'missing wall clock','TIME_UNKNOWN')
        connection.close()
        connection=socket.create_connection(('127.0.0.1',port),timeout=3.)
        send_checked(frame(0,mode='live',timestamp_domain='hardware_clock',
                           capture_timestamp_raw=3456.75),
                     'hardware time provenance diagnostic only')
        changed=frame(1,mode='live',timestamp_domain='global_time',capture_timestamp_raw=3457.)
        send_checked(changed,'domain changed without reconnect','SOURCE_CHANGED')
        # Shutdown with an idle connected socket, exercising the read unblock.
        os.killpg(proc.pid,signal.SIGINT)
        code=proc.wait(timeout=8.)
        assert code==0,log_path.read_text()
        assert 'Traceback' not in log_path.read_text()
        stats=[json.loads(line.split('LIVE_TCP_STATS ',1)[1]) for line in log_path.read_text().splitlines()
               if 'LIVE_TCP_STATS ' in line][-1]
        assert stats['published']==len(arrays)
        assert stats['framing_LENGTH']==2
        assert stats['framing_TRUNCATED']>=1 and stats['framing_PARTIAL_TIMEOUT']>=1
        for reason in ('VERSION','SEQUENCE_TYPE','DUPLICATE_ID','COLOR','CONFIDENCE','POSE_VALID_FORBIDDEN',
                       'JSON_OR_TYPE','NONFINITE','DUPLICATE_KEY'):
            assert stats.get('invalid_'+reason,0)>0,(reason,stats)
        deadline=time.monotonic()+10
        while any(name=='live_tcp_bridge' for name,_ in node.get_node_names_and_namespaces()):
            assert time.monotonic()<deadline,'residual live_tcp_bridge node'
            rclpy.spin_once(node,timeout_sec=.1)
        # Port can be rebound after exit; listener and worker sockets are gone.
        with socket.socket() as check:
            check.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            check.bind(('127.0.0.1',port))
        result.update(passed=True,port=port,received_ros_arrays=len(arrays),cases=cases,
                      stats=stats,bridge_exit=code,remaining_nodes=[],
                      created_qos='RELIABLE KEEP_LAST(5) VOLATILE')
    finally:
        if connection is not None:connection.close()
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()
        node.destroy_node();rclpy.try_shutdown()
        (out/'live_results.json').write_text(json.dumps(result,indent=2))
        (out/'ros_received.json').write_text(json.dumps(arrays,indent=2,allow_nan=True))
        (out/'diagnostics_received.json').write_text(json.dumps(diagnostics,indent=2))
