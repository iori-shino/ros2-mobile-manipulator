"""Start the M3 locator against an existing camera, or with the separate demo table."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = 'mobile_manipulator_autonomy'
    gazebo = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    world = Path(get_package_share_directory(package)) / 'worlds' / 'vision_table.sdf'
    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='false',
                              description='Launch the standalone vision table simulation'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('table_height_m', default_value='0.086',
                              description='Tabletop height measured in base_footprint'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch' / 'sim.launch.py')),
            launch_arguments={'world': str(world), 'gui': LaunchConfiguration('gui')}.items(),
            condition=IfCondition(LaunchConfiguration('demo')),
        ),
        Node(package=package, executable='color_cube_locator.py', output='screen',
             parameters=[{'use_sim_time': True,
                          'table_height_m': ParameterValue(LaunchConfiguration('table_height_m'),
                                                           value_type=float)}]),
    ])
