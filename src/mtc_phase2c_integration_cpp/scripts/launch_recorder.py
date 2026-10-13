#!/usr/bin/env python3
"""Record actual ROS Launch child exits; the launch parent's code is not sufficient."""
import argparse
import json
import os
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource


def event_record(kind, event):
    record = {'event': kind, 'name': event.process_name, 'pid': event.pid,
              'observed_steady_s': time.monotonic()}
    if kind == 'exit':
        record['returncode'] = event.returncode
    return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--records', type=Path, required=True)
    parser.add_argument('package', choices=['mtc_phase2c_integration_cpp', 'aubo_ros2_driver'])
    parser.add_argument('launch_file', choices=['planning_only.launch.py', 'aubo_control.launch.py'])
    parser.add_argument('arguments', nargs='*')
    args = parser.parse_args()
    pairs = [argument.split(':=', 1) for argument in args.arguments]
    if any(len(pair) != 2 for pair in pairs):
        raise SystemExit('invalid launch arguments')
    arguments = dict(pairs)
    from run_fake_gate import validate_environment
    validate_environment(os.environ, Path.cwd())
    if len(arguments) != len(pairs):
        raise SystemExit('duplicate launch arguments')
    if args.package == 'aubo_ros2_driver':
        expected = {'aubo_type': 'aubo_S3', 'use_fake_hardware': 'true',
                    'initial_joint_controller': 'joint_trajectory_controller', 'launch_rviz': 'false'}
        if args.launch_file != 'aubo_control.launch.py' or arguments != expected:
            raise SystemExit('only reviewed explicit S3 Fake arguments are supported')
    elif args.launch_file != 'planning_only.launch.py' or set(arguments) != {'mode'} or \
            arguments['mode'] not in {'planner', 'execute', 'plan', 'mismatch', 'expired',
                                      'foreign', 'cancel', 'fault', 'silence'}:
        raise SystemExit('unsupported integration launch')
    # No parameter values, command arguments, environment values or credentials are logged.
    with args.records.open('x') as records:
        def record(kind, event, context):
            del context
            records.write(json.dumps(event_record(kind, event)) + '\n')
            records.flush()
            return []
        description = LaunchDescription([
            RegisterEventHandler(OnProcessStart(on_start=lambda e, c: record('start', e, c))),
            RegisterEventHandler(OnProcessExit(on_exit=lambda e, c: record('exit', e, c))),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory(args.package)) / 'launch' / args.launch_file)),
                launch_arguments=arguments.items()),
        ])
        service = LaunchService()
        service.include_launch_description(description)
        return service.run()


if __name__ == '__main__':
    raise SystemExit(main())
