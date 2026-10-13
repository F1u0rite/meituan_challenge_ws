"""Real installed ROS2 replay processes; no camera, listener or motion execution."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy,
    qos_check_compatible,
)
from rosidl_runtime_py.convert import message_to_ordereddict
from mtc_interfaces.msg import BatteryDetectionArray
from mtc_vision_adapter.core import motion_rejection
from live_test_support import installed_executable


def equal(actual, expected):
    if expected is None:
        return isinstance(actual, float) and math.isnan(actual)
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            equal(actual[key], value) for key, value in expected.items())
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            equal(a, b) for a, b in zip(actual, expected))
    return actual == expected


def run_validation(fixture, out, exit_mode='natural'):
    fixture, out = Path(fixture), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    workspace = Path(__file__).resolve().parents[3]
    executable = installed_executable(workspace, 'replay_publisher')
    assert executable.is_file(), 'Build and source the workspace overlay first'
    records = [json.loads(line) for line in fixture.read_text().splitlines() if line]
    assert len(records) == 4
    assert exit_mode in ('natural', 'sigint')
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE, liveliness=LivelinessPolicy.AUTOMATIC)
    received, checks, rejections, endpoints = [], [], [], []
    rclpy.init()
    node = Node('vision_process_acceptance')
    proc = None
    result = {'exit_mode': exit_mode, 'passed': False,
              'scope': 'Local Ubuntu ROS2 only; no D435i, cross-host or C++ FSM'}
    log = open(out/'ros_publisher.log', 'w')

    def callback(msg):
        data = message_to_ordereddict(msg)
        index = len(received)
        received.append(data)
        checks.append(index < len(records) and equal(data, records[index]['message']))
        stamp = data['header']['stamp']
        # Verify ring rejection at a fresh reference time; the transmitted
        # historical capture timestamp is never changed.
        capture = stamp['sec'] + stamp['nanosec']*1e-9 + .01
        offline = motion_rejection(data, capture, mode=records[index]['mode'])
        live = motion_rejection(data, capture, mode='live')
        assert offline == 'REPLAY_NOT_MOTION_INPUT'
        assert live == ('RING_INVALID' if data['detections'] else 'EMPTY')
        rejections.append({'offline': offline, 'live': live})

    node.create_subscription(BatteryDetectionArray, '/mtc/perception/batteries', callback, qos)
    try:
        env = dict(os.environ, ROS_LOG_DIR=str(out/'ros_logs'))
        proc = subprocess.Popen(
            [str(executable), '--ros-args', '-p', 'input_jsonl:='+str(fixture),
             '-p', 'period_s:=0.5', '-p', 'wait_for_subscriber_s:=15.0'],
            stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            start_new_session=True, env=env)
        expected = 4 if exit_mode == 'natural' else 2
        signaled = False
        deadline = time.monotonic()+35
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not endpoints:
                for ep in node.get_publishers_info_by_topic('/mtc/perception/batteries'):
                    compatibility, reason = qos_check_compatible(ep.qos_profile, qos)
                    endpoints.append(dict(
                        node=ep.node_name, type=ep.topic_type,
                        reliability=ep.qos_profile.reliability.name,
                        durability=ep.qos_profile.durability.name,
                        history=ep.qos_profile.history.name, depth=ep.qos_profile.depth,
                        compatibility=compatibility.name, reason=reason))
            if exit_mode == 'sigint' and len(received) >= expected and not signaled:
                os.killpg(proc.pid, signal.SIGINT)
                signaled = True
            if proc.poll() is not None:
                # DDS delivery and subscriber dispatch can finish after the
                # publisher has acknowledged its final reliable sample.
                drain_until = time.monotonic()+.5
                while time.monotonic() < drain_until:
                    rclpy.spin_once(node, timeout_sec=.05)
                break
        assert proc.poll() is not None, 'publisher did not exit automatically'
        code = proc.wait(timeout=2)
        assert code == 0, (code, (out/'ros_publisher.log').read_text())
        assert len(received) == expected and all(checks), (len(received), checks)
        assert endpoints and all(ep['compatibility'] == 'OK' and
            ep['reliability'] == 'RELIABLE' and ep['durability'] == 'VOLATILE'
            for ep in endpoints), endpoints
        text = (out/'ros_publisher.log').read_text()
        assert 'created QoS: RELIABLE KEEP_LAST(5) VOLATILE' in text
        assert 'Traceback' not in text and 'already shutdown' not in text
        if exit_mode == 'natural':
            assert not signaled and 'replay complete' in text
        else:
            assert signaled and 'replay complete' not in text
        deadline = time.monotonic()+10
        while any(name == 'vision_offline_replay'
                  for name, _ in node.get_node_names_and_namespaces()):
            assert time.monotonic() < deadline, 'replay node remains in ROS graph'
            rclpy.spin_once(node, timeout_sec=.1)
        result.update(
            passed=True, received_frames=len(received),
            array_lengths=[len(msg['detections']) for msg in received],
            exact_fixture_matches=checks, motion_rejections=rejections,
            discovered_qos=endpoints,
            created_qos='RELIABLE KEEP_LAST(5) VOLATILE',
            publisher_exit=code, signal_sent=signaled,
            replay_nodes_remaining=[])
        return result
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()
        result['subscriber_cleanup_complete'] = True
        (out/'ros_results.json').write_text(json.dumps(result, indent=2))
        (out/'ros_received.json').write_text(json.dumps(received, indent=2, allow_nan=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exit-mode', choices=('natural', 'sigint'), default='natural')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[3]
    fixture = os.environ.get('MTC_REPLAY_FIXTURE',
                             str(workspace/'src/mtc_vision_adapter/examples/synthetic.jsonl'))
    out = os.environ.get('MTC_VALIDATION_OUT', '/tmp/mtc_vision_validation')
    print(json.dumps(run_validation(fixture, out, args.exit_mode), indent=2))


if __name__ == '__main__':
    main()
