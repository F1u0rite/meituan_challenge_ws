#!/usr/bin/env python3
"""Controlled GenericSystem gate: functional and actual child teardown results stay separate."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


BLOCKED_KEYS = {'ROS_SUPER_CLIENT', 'ROS2_EASY_MODE', 'ROS_STATIC_PEERS', 'ROS_LOCALHOST_ONLY',
                'CYCLONEDDS_URI', 'SKIP_DEFAULT_XML_FILE', 'SKIP_DEFAULT_XML'}
BLOCKED_PREFIXES = ('FASTDDS_', 'FASTRTPS_', 'RMW_FASTRTPS_', 'ROS_DISCOVERY_', 'ROS_SECURITY_')


def validate_environment(environment, directory):
    if environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST' or \
            environment.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp':
        raise ValueError('explicit LOCALHOST and validated rmw_fastrtps_cpp required')
    domain = environment.get('ROS_DOMAIN_ID', '')
    if not domain.isascii() or not domain.isdigit() or len(domain) > 3 or not 1 <= int(domain) <= 232:
        raise ValueError('explicit isolated domain 1..232 required')
    for key in environment:
        if key in BLOCKED_KEYS or key.startswith(BLOCKED_PREFIXES):
            raise ValueError(f'unreviewed discovery override: {key}')
    if any((directory / name).exists() for name in
           ['DEFAULT_FASTDDS_PROFILES.xml', 'DEFAULT_FASTRTPS_PROFILES.xml']):
        raise ValueError('implicit local DDS profile prevents controlled start')


def reserve_evidence(path):
    # Atomic exclusive creation rejects existing empty directories and symlinks too.
    path.mkdir(parents=True, exist_ok=False)


def read_child_events(path):
    events = [json.loads(line) for line in path.read_text().splitlines() if line]
    started, exited = {}, {}
    for event in events:
        key = (event['pid'], event['name'])
        if event['event'] == 'start':
            if key in started:
                raise ValueError('duplicate child start event')
            started[key] = event
        elif event['event'] == 'exit':
            if key in exited or not isinstance(event['returncode'], int):
                raise ValueError('invalid child exit event')
            exited[key] = event
        else:
            raise ValueError('unknown child event')
    if not started or started.keys() != exited.keys():
        raise ValueError('missing actual child exit evidence')
    return events, all(event['returncode'] == 0 for event in exited.values())


def overall_result(functional, teardown):
    if functional == 'PASS' and teardown == 'PASS':
        return 'PASS', 0
    if functional == 'BLOCKED' and teardown == 'NOT RUN':
        return 'BLOCKED', 2
    return 'FAIL', 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['execute', 'plan', 'mismatch', 'expired', 'foreign',
                                          'cancel', 'fault', 'silence'], required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    # Freeze the validated environment passed to every owned child; do not silently sanitize overrides.
    environment = os.environ.copy()
    try:
        validate_environment(environment, Path.cwd())
        reserve_evidence(args.evidence)
    except (ValueError, OSError) as error:
        print(f'BLOCKED: {error}', file=sys.stderr)
        return 2
    result = {'mode': args.mode, 'domain': int(environment['ROS_DOMAIN_ID']),
              'rmw': environment['RMW_IMPLEMENTATION'], 'functional': 'BLOCKED',
              'teardown': 'NOT RUN', 'overall': 'BLOCKED', 'processes': []}
    children, handles = [], []
    recorder = Path(__file__).resolve().with_name('launch_recorder.py')

    def start(name, package, launch_file, arguments):
        log = (args.evidence / f'{name}.log').open('x')
        handles.append(log)
        record_path = args.evidence / f'{name}.processes.jsonl'
        command = [sys.executable, str(recorder), '--records', str(record_path),
                   package, launch_file, *arguments]
        proc = subprocess.Popen(command, env=environment, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, proc, record_path))
        return proc

    try:
        description = Path(get_package_share_directory('aubo_description'))
        if not (description / 'urdf/aubo_S3.urdf.xacro').is_file():
            raise ValueError('missing existing Xacro; reference workspace must stay read-only')
        robot = subprocess.check_output(['xacro', str(description / 'urdf/xacro/inc/aubo_ros2.xacro'),
                                         'aubo_type:=aubo_S3', 'use_fake_hardware:=true'],
                                        env=environment, text=True)
        tree = ET.fromstring(robot)
        plugins = [element.text for element in tree.findall('.//ros2_control/hardware/plugin')]
        if plugins != ['mock_components/GenericSystem']:
            raise ValueError(f'unverified hardware plugins {plugins}')
        joints = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                  'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
        reviewed = [2*math.pi, math.pi, 2.41, 2.66, 2*math.pi, 2.23]
        def bound(text):
            return {'{-2*pi}': -2*math.pi, '{2*pi}': 2*math.pi}.get(text) if text.startswith('{') else float(text)
        for name, maximum in zip(joints, reviewed):
            limit = tree.find(f"joint[@name='{name}']/limit")
            command = tree.find(f"ros2_control/joint[@name='{name}']/command_interface[@name='position']")
            if limit is None or command is None:
                raise ValueError('complete URDF/controller position limits required')
            raw = {element.attrib['name']: bound(element.text) for element in command.findall('param')}
            if raw.get('min') is None or raw.get('max') is None:
                raise ValueError('unknown controller bound expression')
            actual = (max(float(limit.attrib['lower']), raw['min']),
                      min(float(limit.attrib['upper']), raw['max']))
            if abs(actual[0]+maximum) > 1e-9 or abs(actual[1]-maximum) > 1e-9 or float(limit.attrib['velocity']) < 0.2:
                raise ValueError(f'reviewed position capability mismatch for {name}')
        (args.evidence / 'verified_fake.urdf').write_text(robot)
        owner = start('owner', 'mtc_phase2c_integration_cpp', 'planning_only.launch.py', [f'mode:={args.mode}'])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if 'COLD_START_CREDENTIAL_CONSUMED' in (args.evidence / 'owner.log').read_text():
                break
            if owner.poll() is not None:
                raise ValueError('owner failed cold-start proof; no controller started')
            time.sleep(0.05)
        else:
            raise ValueError('no consumed startup credential; no controller started')
        start('control', 'aubo_ros2_driver', 'aubo_control.launch.py',
              ['aubo_type:=aubo_S3', 'use_fake_hardware:=true',
               'initial_joint_controller:=joint_trajectory_controller', 'launch_rviz:=false'])
        start('planning_only', 'mtc_phase2c_integration_cpp', 'planning_only.launch.py', ['mode:=planner'])
        owner.wait(timeout=100)
        text = (args.evidence / 'owner.log').read_text()
        result['functional'] = 'PASS' if f'PASS {args.mode}' in text and owner.returncode == 0 else 'FAIL'
        print(text)
    except Exception as error:
        result['error'] = str(error)
        if children:
            result['functional'] = 'FAIL'
        print(f'BLOCKED/FAIL: {error}', file=sys.stderr)
    finally:
        process_results = []
        for name, proc, records in reversed(children):
            premature = name != 'owner' and proc.poll() is not None
            signals = []
            try:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    signals.append('SIGINT')
                try:
                    proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    signals.append('SIGTERM')
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        signals.append('SIGKILL')
                        proc.wait(timeout=8)
                child_events, clean = read_child_events(records)
                entry = {'name': name, 'launcher_returncode': proc.returncode,
                         'signals_requested': signals, 'premature_exit': premature,
                         'child_events': child_events, 'pass': clean and proc.returncode == 0
                         and not premature and not any(s in signals for s in ['SIGTERM', 'SIGKILL'])}
            except Exception as error:
                entry = {'name': name, 'launcher_returncode': proc.returncode,
                         'signals_requested': signals, 'pass': False, 'error': str(error)}
            process_results.append(entry)
            print(f"owned_process {name} launcher_exit={proc.returncode} teardown={'PASS' if entry['pass'] else 'FAIL'}")
        for log in handles:
            log.close()
        result['processes'] = process_results
        if children:
            result['teardown'] = 'PASS' if all(entry['pass'] for entry in process_results) else 'FAIL'
        result['overall'], code = overall_result(result['functional'], result['teardown'])
        (args.evidence / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
        print(f"Functional={result['functional']} Teardown={result['teardown']} Overall={result['overall']}")
    return code


if __name__ == '__main__':
    raise SystemExit(main())
