"""Start the structured test world and SLAM; driving remains explicit."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    def include(name, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim / 'launch' / name)),
                                        launch_arguments=arguments.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        include('sim.launch.py', {'world': str(sim / 'worlds/slam_room.sdf'), 'gui': LaunchConfiguration('gui')}),
        include('slam.launch.py', {'rviz': LaunchConfiguration('rviz')}),
    ])
