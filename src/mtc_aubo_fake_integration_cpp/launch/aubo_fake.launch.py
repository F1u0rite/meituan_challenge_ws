"""Independent aubo_fake mode. Starts only our controlled supervisor, never adopts a live controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
from pathlib import Path


def setup(context):
    root = Path(get_package_prefix('mtc_aubo_fake_integration_cpp'))
    process = ExecuteProcess(cmd=[str(root / 'lib/mtc_aubo_fake_integration_cpp/run_aubo_fake.py'),
        '--mode', LaunchConfiguration('mode').perform(context),
        '--domain', LaunchConfiguration('domain').perform(context),
        '--output', LaunchConfiguration('output').perform(context)], output='screen')
    def completed(event, _context):
        if event.returncode != 0:
            raise RuntimeError('aubo_fake Overall FAIL; inspect preserved summary.json and actual child exit codes')
        return []
    return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=completed)), process]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='plan'),
        DeclareLaunchArgument('domain'), DeclareLaunchArgument('output'), OpaqueFunction(function=setup)])
