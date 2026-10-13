"""Real installed replay process -> DDS; reject wrongly labelled scene frames."""
from pathlib import Path
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy
from mtc_interfaces.msg import BatteryDetectionArray
from std_msgs.msg import String
import argparse
import copy
import hashlib
import json
import os
import subprocess
import time

a = argparse.ArgumentParser()
a.add_argument('--tag', required=True)
a.add_argument('--install-base', default='/candidate-install')
args = a.parse_args()
R = Path('/review-evidence') / ('boundary-' + args.tag)
R.mkdir(exist_ok=False)
row = dict(source_frame_id='color_21_depth_19', capture_timestamp=1791910073.8,
    timestamps=dict(color_frame_number=21, depth_frame_number=19, color_sdk_ms=1791910073800.),
    coordinate_frame='camera_color_optical', unit_position='m', pair_qualified=True,
    motion_usable=False, ring_pose_valid=False, body_pose_valid=False,
    objects=[dict(color=c, candidate_available=True, green_point_camera_m=[.1,-.02,.52],
                  red_point_camera_m=[.1,-.02,.54]) for c in ['red','blue','yellow','green']])
row['scene_reference'] = dict(schema='mtc.scene_reference.v1', ready=True, status='READY',
    source_frame_id=row['source_frame_id'], capture_timestamp_s=row['capture_timestamp'],
    timestamps=copy.deepcopy(row['timestamps']), input_coordinate_frame='camera_color_optical',
    coordinate_frame='t0_scene', unit_position='m', motion_usable=False, robot_transform_available=False,
    reference_validity='EXACT_SOURCE_FRAME_ONLY', T_scene_from_camera=[[1,0,0,0],[0,-1,0,0],[0,0,-1,.6],[0,0,0,1]])
cases = [('valid', copy.deepcopy(row), False)]
not_ready = copy.deepcopy(row)
not_ready['scene_reference'].update(ready=False, status='NO_UNIQUE_CURRENT_T0', T_scene_from_camera=None)
cases.append(('not-ready', not_ready, False))
legacy = copy.deepcopy(row)
del legacy['scene_reference']
cases.append(('legacy', legacy, False))
for label, value in [('base', 'base_link'), ('camera','camera_color_optical'), ('empty',''), ('missing',None)]:
    bad = copy.deepcopy(row)
    if value is None:
        del bad['scene_reference']['coordinate_frame']
    else:
        bad['scene_reference']['coordinate_frame'] = value
    cases.append((label, bad, True))
context = Context()
rclpy.init(context=context)
node = Node('scene_boundary_receiver', context=context)
executor = SingleThreadedExecutor(context=context)
executor.add_node(node)
packets, arrays, results = [], [], []
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(String, '/mtc/perception/top_hole_candidates', lambda x: packets.append(json.loads(x.data)), qos)
node.create_subscription(BatteryDetectionArray, '/mtc/perception/batteries', lambda x: arrays.append(dict(frame=x.header.frame_id, count=len(x.detections), valid=[d.ring_pose_valid for d in x.detections], x=[d.ring_pose.pose.position.x for d in x.detections])), qos)
exe = Path(args.install_base) / 'mtc_vision_adapter/lib/mtc_vision_adapter/automatic_top_coordinates_replay'
process = None
try:
    for name, source, reject in cases:
        file = R / (name + '-input.json')
        file.write_text(json.dumps([source], indent=2) + '\n')
        before = (len(packets), len(arrays))
        argv = [str(exe), '--rows', str(file), '--period-s', '0', '--wait-subscriber-s', '8']
        started = time.time()
        with (R / (name + '.log')).open('w') as out:
            process = subprocess.Popen(argv, stdout=out, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 15
            while process.poll() is None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=4)
                raise AssertionError('process timeout: ' + name)
            end = time.monotonic() + .3
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.05)
        received = [len(packets)-before[0], len(arrays)-before[1]]
        ok = (process.returncode != 0 and received == [0,0]) if reject else (process.returncode == 0 and received == [1,1])
        if not reject and received == [1,1]:
            assert arrays[-1]['frame'] == 'camera_color_optical' and arrays[-1]['valid'] == [False]*4
            assert packets[-1]['objects'][0]['red_point_camera_m'] == [.1,-.02,.54]
            if name == 'valid':
                assert abs(packets[-1]['scene_coordinates']['objects'][0]['red_point_scene_m'][2] - .06) < 1e-9
            if name == 'legacy':
                assert 'scene_reference' not in packets[-1]
            if name == 'not-ready':
                assert packets[-1]['scene_coordinates']['objects'][0]['red_point_scene_m'] is None
        results.append(dict(name=name, expected_reject=reject, passed=ok, argv=argv, child_exit=process.returncode,
                            received=received, started_at=started, ended_at=time.time()))
        process = None
    end = time.monotonic() + 3
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.05)
    remaining = node.get_node_names_and_namespaces()
    assert remaining == [('scene_boundary_receiver', '/')], remaining
finally:
    if process is not None and process.poll() is None:
        process.terminate()
        process.wait(timeout=5)
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown(context=context)
passed = all(x['passed'] for x in results)
record = dict(passed=passed, cases=results, packets=packets, arrays=arrays, remaining_nodes=remaining,
              executable=str(exe), executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),
              scope='Actual installed separate processes, synthetic coordinates and local DDS; never motion or physical calibration')
(R / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(dict(passed=passed, failures=[x['name'] for x in results if not x['passed']])))
raise SystemExit(0 if passed else 1)
