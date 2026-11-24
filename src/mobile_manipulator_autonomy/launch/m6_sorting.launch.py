"""Start an isolated, headless three-color sorting demonstration."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or not os.environ.get('IGN_PARTITION'):
        raise ValueError('Set a dedicated ROS_DOMAIN_ID and IGN_PARTITION before starting the demo')
    package = 'mobile_manipulator_autonomy'
    share = Path(get_package_share_directory(package))
    prefix = Path(get_package_prefix(package))
    directory = Path(LaunchConfiguration('run_dir').perform(context)).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch'/'m5_scene.launch.py')),
        launch_arguments={'pick_demo': 'true', 'sorting': 'true', 'gui': 'false',
                          'seed': LaunchConfiguration('seed'), 'scene_dir': str(directory)}.items())
    sorter = ExecuteProcess(
        cmd=[str(prefix/'lib'/package/'sort_cubes.py'), '--output', str(directory/'result.json')],
        additional_env={'M6_COLORS': LaunchConfiguration('colors')}, output='screen')
    return [scene, sorter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_dir', description='New directory for scene and task records'),
        DeclareLaunchArgument('seed', default_value='51'),
        DeclareLaunchArgument('colors', default_value='',
                              description='Optional comma-separated subset in task order'),
        OpaqueFunction(function=setup),
    ])
