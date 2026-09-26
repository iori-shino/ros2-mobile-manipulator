"""Online async mapping using the existing simulation topics and TF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    toolbox = Path(get_package_share_directory('slam_toolbox'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(toolbox / 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': str(sim / 'config/slam.yaml')}.items()),
        Node(package='rviz2', executable='rviz2', name='slam_rviz',
             arguments=['-d', str(sim / 'config/slam.rviz')],
             parameters=[{'use_sim_time': True}], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
