#!/usr/bin/env python3
"""Controlled LOCALHOST GenericSystem only; preserve functional and actual teardown outcomes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def prepare(output):
    description = Path(get_package_share_directory('aubo_description'))
    config = Path(get_package_share_directory('aubo_moveit_config')) / 'config'
    if not (description / 'urdf/aubo_S3.urdf.xacro').is_file():
        raise RuntimeError('existing read-only S3 Xacro missing; will not generate inside driver workspace')
    raw = subprocess.check_output(['xacro', str(description / 'urdf/xacro/inc/aubo_ros2.xacro'),
        'aubo_type:=aubo_S3', 'use_fake_hardware:=true', 'robot_ip:=DISABLED'], text=True, timeout=20)
    robot = ET.fromstring(raw)
    hardware = robot.findall('ros2_control')
    if len(hardware) != 1 or hardware[0].findtext('hardware/plugin') != 'mock_components/GenericSystem':
        raise RuntimeError('unique GenericSystem not proven; refusing to start')
    names = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
    hw_joints = hardware[0].findall('joint')
    if [j.attrib['name'] for j in hw_joints] != names:
        raise RuntimeError('reviewed six hardware joints mismatch')
    for j in hw_joints:
        if 'position' not in [i.attrib['name'] for i in j.findall('command_interface')]:
            raise RuntimeError('position command interface missing')
        if not {'position', 'velocity'} <= {i.attrib['name'] for i in j.findall('state_interface')}:
            raise RuntimeError('six position/velocity state interfaces required')
    # This is a fresh model copy, not a write to the read-only driver. No limits/initial values are patched.
    (output / 'robot.urdf').write_text(raw)
    params = dict(robot_description=raw, robot_description_semantic=(config / 'aubo_robot.srdf').read_text(),
        robot_description_kinematics=yaml.safe_load((config / 'kinematics.yaml').read_text()),
        robot_description_planning=yaml.safe_load((config / 'joint_limits.yaml').read_text()),
        planning_pipelines=['ompl'], default_planning_pipeline='ompl',
        ompl=yaml.safe_load((config / 'ompl_planning.yaml').read_text()),
        allow_trajectory_execution=False, **{'trajectory_execution.manage_controllers': False},
        moveit_controller_manager='mtc_aubo_fake/PlanningOnly', publish_robot_description_semantic=True,
        use_sim_time=False)
    controllers = Path(get_package_share_directory('aubo_ros2_driver')) / 'config/aubo_controllers.yaml'
    controller_yaml = yaml.safe_load(controllers.read_text())
    jtc = controller_yaml['joint_trajectory_controller']['ros__parameters']
    if jtc['joints'] != names or jtc['command_interfaces'] != ['position'] or jtc.get('allow_partial_joints_goal') is not False:
        raise RuntimeError('controller joint/interface configuration mismatch')
    return params, controllers, dict(plugin='mock_components/GenericSystem',
        robot_sha256=hashlib.sha256(raw.encode()).hexdigest(), source_description=str(description),
        controllers_file=str(controllers), joints=names)


def main():
    began = time.monotonic()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['plan', 'execute', 'mismatch', 'expired', 'foreign', 'cancel', 'identity', 'silence', 'fault'], default='plan')
    p.add_argument('--domain', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not 1 <= args.domain <= 232 or args.domain == 91:
        raise RuntimeError('choose isolated domain 1..232 other than existing site domain 91')
    # mkdir without exist_ok refuses all overwrites, including an empty existing directory.
    args.output.mkdir(parents=True, exist_ok=False)
    output = args.output.resolve()
    children = []
    env = os.environ.copy()
    # Do not silently remove discovery selectors supplied by the caller. Native credential validation rejects them.
    env.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=str(output / 'ros_logs'), PYTHONDONTWRITEBYTECODE='1')
    stopping = False
    def interrupted(sig, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    def spawn(name, command):
        log = (output / (name + '.log')).open('x')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        item = dict(name=name, process=process, file=log, pid=process.pid, pgid=process.pid, command=command, signals=[])
        children.append(item)
        (output / 'processes.json').write_text(json.dumps([{k:v for k,v in c.items() if k not in ('process','file')} for c in children], indent=2))
        return process
    errors = []
    try:
        params, controllers, model = prepare(output)
        (output / 'static_proof.json').write_text(json.dumps(model, indent=2))
        probe_params = dict(params, mode=args.mode, enable_fake_execution=True,
            session_id=output.name, output_dir=str(output))
        def parameter_file(name, data):
            path=output / (name + '.yaml')
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': data}}))
            return str(path)
        base_params = parameter_file('model', params)
        probe_params_file = parameter_file('probe', probe_params)
        prefix = Path(get_package_prefix('mtc_aubo_fake_integration_cpp'))
        probe = spawn('probe', [str(prefix / 'lib/mtc_aubo_fake_integration_cpp/aubo_fake_probe'), '--ros-args', '--params-file', probe_params_file])
        deadline = time.monotonic()+12
        while time.monotonic() < deadline and probe.poll() is None and not stopping:
            if 'AUBO_FAKE_COLD_START_READY' in (output / 'probe.log').read_text(): break
            time.sleep(.05)
        else: raise RuntimeError('cold-start authority was not established; driver not started')
        control = spawn('controller', [str(Path(get_package_prefix('controller_manager')) / 'lib/controller_manager/ros2_control_node'),
            '--ros-args', '--params-file', base_params, '--params-file', str(controllers)])
        spawn('state_publisher', [str(Path(get_package_prefix('robot_state_publisher')) / 'lib/robot_state_publisher/robot_state_publisher'),
            '--ros-args', '--params-file', base_params])
        spawn('world_tf', [str(Path(get_package_prefix('tf2_ros')) / 'lib/tf2_ros/static_transform_publisher'),
            '--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'base_link'])
        for name in ['joint_state_broadcaster', 'joint_trajectory_controller']:
            spawner=spawn('spawn_'+name, ['ros2', 'run', 'controller_manager', 'spawner', name, '--controller-manager-timeout', '15'])
            if spawner.wait(timeout=22) != 0: raise RuntimeError('controller activation failed: '+name)
        spawn('move_group', [str(Path(get_package_prefix('moveit_ros_move_group')) / 'lib/moveit_ros_move_group/move_group'),
            '--ros-args', '--params-file', base_params])
        deadline=time.monotonic()+95
        while time.monotonic()<deadline and probe.poll() is None and not stopping:
            if control.poll() is not None: raise RuntimeError('controlled ros2_control process exited')
            time.sleep(.1)
        if probe.poll() is None: raise RuntimeError('bounded experiment deadline/interruption before result')
        if probe.returncode != 0: errors.append('probe exit '+str(probe.returncode))
    except Exception as e:
        errors.append(str(e))
    finally:
        # SIGINT every own process group first; never stop other members' domain 91 processes.
        for child in reversed(children):
            proc=child['process']
            if proc.poll() is None:
                os.killpg(child['pgid'], signal.SIGINT)
                child['signals'].append(dict(signal='SIGINT', at=time.time()))
        for child in reversed(children):
            proc=child['process']
            try: proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child['pgid'], signal.SIGTERM)
                child['signals'].append(dict(signal='SIGTERM', at=time.time()))
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child['pgid'], signal.SIGKILL)
                    child['signals'].append(dict(signal='SIGKILL', at=time.time()));proc.wait(timeout=5)
            child['exit_code']=proc.returncode
            child['file'].close()
        teardown=[{k:v for k,v in child.items() if k not in ('process','file')} for child in children]
        (output/'teardown.json').write_text(json.dumps(teardown,indent=2))
    functional='FAIL'
    if (output/'functional.json').is_file(): functional=json.loads((output/'functional.json').read_text())['functional_result']
    # A negative exit/escalation is always a teardown failure, even if a parent Launch would exit 0.
    teardown_pass=bool(children) and all(c.get('exit_code')==0 and all(s['signal']=='SIGINT' for s in c['signals']) for c in children)
    result=dict(functional_result=functional, teardown_result='PASS' if teardown_pass else 'FAIL',
        overall_result='PASS' if functional=='PASS' and teardown_pass and not errors else 'FAIL',errors=errors,
        real_hardware='NOT RUN', stop_confirmed_from_process_exit=False,
        mode=args.mode, domain=args.domain, elapsed_seconds=time.monotonic()-began)
    (output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['overall_result']=='PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
