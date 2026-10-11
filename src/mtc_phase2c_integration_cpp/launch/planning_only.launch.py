"""仅规划的 move_group 和受控宿主参数；不加载驱动、不发送 Goal。"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
import math


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    description = Path(get_package_share_directory('aubo_description'))
    moveit = Path(get_package_share_directory('aubo_moveit_config'))
    if not (description / 'urdf/aubo_S3.urdf.xacro').is_file():
        raise RuntimeError('existing S3 Xacro required; reference workspace is read-only')
    config = (MoveItConfigsBuilder('aubo', package_name='aubo_moveit_config')
              .robot_description(file_path=str(description / 'urdf/xacro/inc/aubo_ros2.xacro'),
                                 mappings={'aubo_type': 'aubo_S3', 'use_fake_hardware': 'true'})
              .robot_description_semantic(file_path=str(moveit / 'config/aubo_robot.srdf'))
              .robot_description_kinematics(file_path=str(moveit / 'config/kinematics.yaml'))
              .joint_limits(file_path=str(moveit / 'config/joint_limits.yaml'))
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .to_moveit_configs())
    # Model-side limits use the reviewed URDF/controller intersection. Backend
    # sees these same limits, avoiding redundant JointConstraints that select
    # a random joint sampler rather than IK for a Pose goal in this MoveIt build.
    names = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
    maxima = [2*math.pi, math.pi, 2.41, 2.66, 2*math.pi, 2.23]
    limits = config.joint_limits['robot_description_planning']['joint_limits']
    for name, maximum in zip(names, maxima):
        limits[name].update({'has_position_limits': True, 'min_position': -maximum, 'max_position': maximum})
    if mode == 'planner':
        # MoveItCpp's execution manager options must also disable controller loading.
        return [Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                     parameters=[config.to_dict(), {'allow_trajectory_execution': False,
                       'trajectory_execution.manage_controllers': False,
                       'moveit_controller_manager': 'mtc_phase2c/PlanningOnlyManager'}])]
    if mode not in ('execute', 'plan', 'mismatch', 'expired', 'foreign', 'cancel', 'fault', 'silence'):
        raise RuntimeError('unsupported controlled Fake verification mode')
    return [Node(package='mtc_phase2c_integration_cpp', executable='single_owner', output='screen',
                 parameters=[config.to_dict(), {'mode': mode}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='planner'),
                              OpaqueFunction(function=setup)])
