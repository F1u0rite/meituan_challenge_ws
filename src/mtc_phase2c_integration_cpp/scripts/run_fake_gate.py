#!/usr/bin/env python3
"""Fresh local GenericSystem deployment; never adopts an existing controller or emits a warmup Goal."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mode', choices=['execute', 'plan', 'mismatch', 'expired', 'foreign', 'cancel', 'fault', 'silence'], required=True)
    p.add_argument('--evidence', type=Path, required=True)
    args = p.parse_args()
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST' or os.environ.get('ROS_STATIC_PEERS'):
        raise SystemExit('BLOCKED: LOCALHOST with no static peers required')
    if not 1 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232:
        raise SystemExit('BLOCKED: explicit isolated non-default domain required')
    for key in ['ROS_LOCALHOST_ONLY', 'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI']:
        if os.environ.get(key):
            raise SystemExit(f'BLOCKED: uncontrolled discovery configuration {key}')
    args.evidence.mkdir(parents=True, exist_ok=True)
    description = Path(get_package_share_directory('aubo_description'))
    if not (description / 'urdf/aubo_S3.urdf.xacro').is_file():
        raise SystemExit('BLOCKED: missing existing Xacro; must not generate into reference workspace')
    robot = subprocess.check_output(['xacro', str(description / 'urdf/xacro/inc/aubo_ros2.xacro'),
                                    'aubo_type:=aubo_S3', 'use_fake_hardware:=true'], text=True)
    plugins = [x.text for x in ET.fromstring(robot).findall('.//ros2_control/hardware/plugin')]
    if plugins != ['mock_components/GenericSystem']:
        raise SystemExit(f'BLOCKED: hardware plugins {plugins}')
    tree = ET.fromstring(robot)
    joints = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
    reviewed = [2*math.pi, math.pi, 2.41, 2.66, 2*math.pi, 2.23]
    def bound(text):
        # Known literal upstream expressions only; never eval arbitrary Xacro text.
        return {'{-2*pi}': -2*math.pi, '{2*pi}': 2*math.pi}.get(text, None) if text.startswith('{') else float(text)
    for name, maximum in zip(joints, reviewed):
        limit = tree.find(f"joint[@name='{name}']/limit")
        command = tree.find(f"ros2_control/joint[@name='{name}']/command_interface[@name='position']")
        if limit is None or command is None:
            raise SystemExit('BLOCKED: complete URDF/controller position limits required')
        raw = {x.attrib['name']: bound(x.text) for x in command.findall('param')}
        if raw.get('min') is None or raw.get('max') is None:
            raise SystemExit('BLOCKED: unknown controller bound expression')
        actual = (max(float(limit.attrib['lower']), raw['min']), min(float(limit.attrib['upper']), raw['max']))
        if abs(actual[0]+maximum)>1e-9 or abs(actual[1]-maximum)>1e-9 or float(limit.attrib['velocity'])<0.2:
            raise SystemExit(f'BLOCKED: reviewed capability does not match current sources for {name}')
    (args.evidence / 'verified_fake.urdf').write_text(robot)
    children = []
    handles = []

    def start(name, command):
        log = (args.evidence / f'{name}.log').open('w')
        handles.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, proc))
        return proc

    result = 1
    try:
        owner = start('owner', ['ros2', 'launch', 'mtc_phase2c_integration_cpp', 'planning_only.launch.py', f'mode:={args.mode}'])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if 'COLD_START_CREDENTIAL_CONSUMED' in (args.evidence / 'owner.log').read_text():
                break
            if owner.poll() is not None:
                raise RuntimeError('BLOCKED: owner failed cold-start proof; no controller started')
            time.sleep(0.05)
        else:
            raise RuntimeError('BLOCKED: no consumed startup credential')
        start('control', ['ros2', 'launch', 'aubo_ros2_driver', 'aubo_control.launch.py', 'aubo_type:=aubo_S3',
                          'use_fake_hardware:=true', 'initial_joint_controller:=joint_trajectory_controller', 'launch_rviz:=false'])
        start('planning_only', ['ros2', 'launch', 'mtc_phase2c_integration_cpp', 'planning_only.launch.py', 'mode:=planner'])
        owner.wait(timeout=100)
        text = (args.evidence / 'owner.log').read_text()
        result = 0 if f'PASS {args.mode}' in text and owner.returncode == 0 else 1
        print(text)
    finally:
        for _, proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for name, proc in reversed(children):
            try:
                proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=8)
            print(f'owned_process {name} exit={proc.returncode}')
        for log in handles:
            log.close()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
