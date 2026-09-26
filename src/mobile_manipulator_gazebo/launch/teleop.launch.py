"""Teleop only; simulation and SLAM remain in their existing sessions."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    keyboard_only = LaunchConfiguration('keyboard_only').perform(context).lower() == 'true'
    start_joy = LaunchConfiguration('start_joy').perform(context).lower() == 'true'
    path = Path(get_package_share_directory('mobile_manipulator_gazebo')) / 'config/teleop_mapping.yaml'
    nodes = []
    if not keyboard_only:
        if not path.exists():
            raise RuntimeError('请先完成 joy_calibrate.py 的真实手柄标定；仅键盘使用 keyboard_only:=true')
        with path.open() as file:
            config = yaml.safe_load(file)
        if start_joy:
            nodes.append(Node(package='joy', executable='joy_node', name='joy_node', output='screen',
                         parameters=[{'device_name': config['device_name'], 'deadzone': config['driver_deadzone'],
                                      'autorepeat_rate': 20., 'sticky_buttons': False, 'use_sim_time': False}]))
    nodes.append(Node(package='mobile_manipulator_gazebo', executable='teleop_router.py', name='teleop_router',
                      parameters=[{'mapping_file': '' if keyboard_only else str(path), 'use_sim_time': False}], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_joy', default_value='true', description='false when joy_node is already running'),
        DeclareLaunchArgument('keyboard_only', default_value='false', description='Use keyboard without a calibrated gamepad'),
        OpaqueFunction(function=setup),
    ])
