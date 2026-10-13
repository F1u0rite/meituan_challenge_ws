"""Coordinate contract failures and real ROS DDS/process roundtrip; no camera."""
import copy
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
from live_test_support import installed_executable
from mtc_vision_adapter.automatic_top_result_publisher import candidate_packet, battery_array


def fixture():
    return dict(source_frame_id='color_21_depth_19', capture_timestamp=1791910073.8,
        capture_timestamp_source='COLOR_SDK_SYSTEM_CAPTURE_TIME',
        timestamps=dict(color_frame_number=21, depth_frame_number=19, color_sdk_ms=1791910073800.),
        coordinate_frame='camera_color_optical', unit_position='m', pair_qualified=True,
        motion_usable=False, ring_pose_valid=False, body_pose_valid=False,
        objects=[dict(color=c, candidate_available=True,
            green_point_camera_m=[.1 + i*.01, -.002, .559],
            red_point_camera_m=[.1 + i*.01, .009, .576],
            geometry_consistent=False, rgb_confirmed=True, reasons=['BOUNDARY_TOO_WIDE'])
            for i, c in enumerate(('red', 'blue', 'yellow', 'green'))])


def convert(row):
    return candidate_packet(row, session_id='test', sequence=1, mode='offline')


def test_xyz_remain_metres_and_top_never_becomes_body_pose():
    row = fixture()
    packet = convert(row)
    assert packet['objects'][0]['green_point_camera_m'] == [.1, -.002, .559]
    assert packet['objects'][0]['red_point_camera_m'] == [.1, .009, .576]
    assert packet['capture_timestamp_s'] == row['capture_timestamp']
    assert packet['timestamps'] == row['timestamps']
    assert packet['objects'][0]['geometry_consistent'] is False
    data = battery_array(packet)
    assert data['detections'][0]['ring_pose']['pose']['position']['x'] == .1
    assert all(v is None for v in data['detections'][0]['body_pose']['pose']['position'].values())
    assert all(v is None for v in data['detections'][0]['ring_pose']['pose']['orientation'].values())
    assert not data['detections'][0]['ring_pose_valid']
    assert data['header']['stamp'] == dict(sec=0, nanosec=0)


def test_missing_detection_erases_coordinates_even_if_input_contains_old_points():
    row = fixture()
    first = convert(row)
    row['objects'][0]['candidate_available'] = False
    second = convert(row)
    assert first['objects'][0]['red_point_camera_m'] is not None
    assert second['objects'][0]['red_point_camera_m'] is None
    assert second['objects'][0]['green_point_camera_m'] is None
    assert second['missing_colors'] == ['red']
    assert len(battery_array(second)['detections']) == 3


def test_unqualified_pair_cannot_publish_coordinates():
    row = fixture()
    row['pair_qualified'] = False
    assert battery_array(convert(row))['detections'] == []
    assert all(o['red_point_camera_m'] is None for o in convert(row)['objects'])


@pytest.mark.parametrize('change', ['nan', 'short', 'mm', 'duplicate', 'valid', 'frame', 'time', 'mismatch'])
def test_malformed_inputs_rejected(change):
    row = fixture()
    if change == 'nan': row['objects'][0]['red_point_camera_m'][0] = float('nan')
    if change == 'short': row['objects'][0]['green_point_camera_m'] = [0, 1]
    if change == 'mm': row['unit_position'] = 'mm'
    if change == 'duplicate': row['objects'].append(copy.deepcopy(row['objects'][0]))
    if change == 'valid': row['ring_pose_valid'] = True
    if change == 'frame': row['coordinate_frame'] = '../invalid'
    if change == 'time': row['capture_timestamp'] = -1
    if change == 'mismatch': row['objects'][0]['geometry'] = dict(source_timestamps=dict(color_frame_number=999))
    with pytest.raises(ValueError):
        convert(row)


def test_separate_installed_publisher_process_delivers_four_slots_and_missing_state(tmp_path, monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from mtc_interfaces.msg import BatteryDetectionArray
    from std_msgs.msg import String
    monkeypatch.setenv('ROS_DOMAIN_ID', '217')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    monkeypatch.delenv('ROS_STATIC_PEERS', raising=False)
    row = fixture()
    row['placement_targets'] = [dict(target_id='T0', candidate_available=True,
        coordinate_frame='camera_color_optical', unit_position='m', ring_pose_valid=False,
        motion_usable=False, table_center_camera_m=[-.147,.076,.598],
        target_hole_center_camera_m=[-.146,.041,.549])]
    # Actual delivery must preserve both raw and height-constrained coordinates.
    from types import SimpleNamespace
    import numpy as np
    from perception.height_constraint import HeightConstrainedPoints
    row['timestamps'].update(host_utc='2026-10-13T16:47:53.800000+00:00',
        color_timestamp_domain='timestamp_domain.global_time')
    normal=np.array([0.,-.6,-.8])
    row['support_plane']=dict(point_camera_m=[0,0,.6],normal_camera_xyz=normal.tolist())
    for i,obj in enumerate(row['objects']):
        base=np.array([.1+i*.01,0.,.6])
        obj['green_point_camera_m']=(base+.077*normal).tolist()
        obj['red_point_camera_m']=(base+.057*normal).tolist()
    row=HeightConstrainedPoints().apply(SimpleNamespace(timestamps=row['timestamps']),row)
    # Optional scene fields must reach a separate ROS receiver with camera fields
    # unchanged. This known synthetic transform is a serialization test only.
    row['scene_reference'] = dict(schema='mtc.scene_reference.v1', ready=True,
        status='READY', source_frame_id=row['source_frame_id'],
        capture_timestamp_s=row['capture_timestamp'], timestamps=copy.deepcopy(row['timestamps']),
        input_coordinate_frame='camera_color_optical', coordinate_frame='t0_scene',
        unit_position='m', motion_usable=False, robot_transform_available=False,
        reference_validity='EXACT_SOURCE_FRAME_ONLY',
        T_scene_from_camera=[[1,0,0,0],[0,-1,0,0],[0,0,-1,.6],[0,0,0,1]])
    missing = copy.deepcopy(row)
    missing['source_frame_id'] = 'color_22_depth_20'
    missing['capture_timestamp'] += .5
    missing['objects'][0]['candidate_available'] = False
    missing['objects'][0]['coordinate_variant']='RAW_UNCORRECTED'
    missing['objects'][0]['raw_candidate_available']=False
    missing['objects'][0]['height_constraint'].update(applied=False,status='NO_CURRENT_CANDIDATE')
    missing['placement_targets'][0]['candidate_available'] = False
    missing['scene_reference'].update(ready=False, status='NO_UNIQUE_CURRENT_T0',
        source_frame_id=missing['source_frame_id'],
        capture_timestamp_s=missing['capture_timestamp'],
        T_scene_from_camera=None)
    rows = tmp_path / 'rows.json'
    rows.write_text(json.dumps([row, missing]))
    context = Context()
    rclpy.init(context=context)
    node = Node('yuze_coordinate_test_receiver', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
    arrays, packets = [], []
    node.create_subscription(BatteryDetectionArray, '/mtc/perception/batteries', arrays.append, qos)
    node.create_subscription(String, '/mtc/perception/top_hole_candidates',
        lambda message: packets.append(json.loads(message.data)), qos)
    root = Path(__file__).resolve().parents[3]
    executable = installed_executable(root, 'automatic_top_coordinates_replay')
    process = None
    try:
        with (tmp_path / 'publisher.log').open('w') as log:
            process = subprocess.Popen([str(executable), '--rows', str(rows),
                '--wait-subscriber-s', '10', '--period-s', '.2'], stdout=log, stderr=log)
            deadline = time.monotonic() + 20
            while (len(arrays) < 2 or len(packets) < 2 or process.poll() is None) and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            assert process.poll() == 0, (tmp_path / 'publisher.log').read_text()
        assert len(arrays) == len(packets) == 2
        assert [len(a.detections) for a in arrays] == [4, 3]
        assert packets[0]['source_mode'] == 'replay' and packets[0]['replay'] is True
        assert packets[0]['source_frame_id'] == row['source_frame_id']
        assert packets[0]['scene_coordinates']['objects'][0]['red_point_scene_m'] == pytest.approx([.1, .036, .048])
        assert packets[0]['objects'][0]['red_point_camera_m'] == pytest.approx([.1,-.036,.552])
        assert packets[1]['scene_coordinates']['ready'] is False
        assert all(o['red_point_scene_m'] is None for o in packets[1]['scene_coordinates']['objects'])
        assert packets[0]['capture_timestamp_s'] == row['capture_timestamp']
        assert packets[0]['timestamps'] == row['timestamps']
        assert packets[0]['ros_dispatch_stamp']['sec'] > 0
        assert packets[0]['objects'][0]['coordinate_variant']=='HEIGHT_CONSTRAINED'
        assert packets[0]['objects'][0]['height_constraint']['applied'] is True
        assert packets[0]['objects'][0]['raw_red_point_camera_m']==pytest.approx([.1,-.0342,.5544])
        assert packets[0]['objects'][0]['red_point_camera_m']==pytest.approx([.1,-.036,.552])
        assert packets[0]['objects'][0]['red_height_m']==pytest.approx(.060)
        assert packets[1]['objects'][0]['red_point_camera_m'] is None
        assert packets[1]['objects'][0]['green_point_camera_m'] is None
        assert packets[1]['missing_colors'] == ['red']
        assert packets[0]['placement_targets'][0]['target_hole_center_camera_m'] == [-.146,.041,.549]
        assert packets[1]['placement_targets'][0]['target_hole_center_camera_m'] is None
        assert packets[1]['placement_targets'][0]['table_center_camera_m'] is None
        for i, det in enumerate(arrays[0].detections):
            assert det.color == i + 1
            assert det.ring_pose.pose.position.x == pytest.approx(row['objects'][i]['red_point_camera_m'][0])
            assert not det.ring_pose_valid
            assert math.isnan(det.ring_pose.pose.orientation.w)
            assert math.isnan(det.body_pose.pose.position.x)
            assert det.ring_pose.header.frame_id == 'camera_color_optical'
            assert det.ring_pose.header.stamp.sec == det.ring_pose.header.stamp.nanosec == 0
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown(context=context)


def test_scene_reference_must_match_current_packet_and_leave_legacy_default_unchanged():
    row=fixture()
    assert 'scene_reference' not in convert(row)
    row['scene_reference']=dict(schema='mtc.scene_reference.v1',ready=True,
        source_frame_id='old_frame',capture_timestamp_s=row['capture_timestamp'],
        timestamps=row['timestamps'],input_coordinate_frame='camera_color_optical',
        coordinate_frame='t0_scene',unit_position='m',motion_usable=False,
        robot_transform_available=False,T_scene_from_camera=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    with pytest.raises(ValueError,match='source mismatch'):
        convert(row)
