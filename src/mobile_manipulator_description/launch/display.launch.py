"""RViz-only model inspection; uses wall time and no simulation controllers."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_dir = Path(get_package_share_directory('mobile_manipulator_description'))
    robot_description = xacro.process_file(
        str(package_dir / 'urdf' / 'robot.urdf.xacro')
    ).toxml()
    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui', output='screen',
            parameters=[str(package_dir / 'config' / 'joint_defaults.yaml'),
                        {'robot_description': robot_description, 'use_sim_time': False}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(package_dir / 'config' / 'display.rviz')],
            parameters=[{'use_sim_time': False}],
        ),
    ])
