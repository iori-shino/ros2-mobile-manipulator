"""Run Nav2 against the existing Fortress/SLAM interfaces."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = 'mobile_manipulator_autonomy'
    sim = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    default_params = Path(get_package_share_directory(package)) / 'config' / 'navigation.yaml'

    def include(filename, **arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch' / filename)),
            launch_arguments=arguments.items(),
        )

    nav2 = [
        ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer'),
        ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer'),
        ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator'),
    ]
    nav_params = [LaunchConfiguration('params_file'), {'use_sim_time': True}]
    manager = ('nav2_lifecycle_manager', 'lifecycle_manager_navigation',
               'nav2_lifecycle_manager::LifecycleManager')
    manager_params = [{'use_sim_time': True,
                       'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                       'node_names': [name for _, name, _ in nav2]}]

    def arguments(package, name):
        remap = [('cmd_vel', '/cmd_vel/nav')] if name in ('controller_server', 'behavior_server') else []
        return dict(package=package, name=name, parameters=manager_params if name == manager[1] else nav_params,
                    remappings=remap)

    use_components = LaunchConfiguration('use_components')
    servers = [Node(executable=name if name != manager[1] else 'lifecycle_manager',
                    output='screen', condition=UnlessCondition(use_components), **arguments(pkg, name))
               for pkg, name, _ in [*nav2, manager]]
    container = ComposableNodeContainer(
        package='rclcpp_components', executable='component_container_isolated',
        name='m43_nav_container', namespace='', output='screen',
        # Costmap subnodes obtain their own YAML sections from process arguments.
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(use_components),
        composable_node_descriptions=[ComposableNode(plugin=plugin, **arguments(pkg, name))
                                      for pkg, name, plugin in [*nav2, manager]])
    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('keyboard_only', default_value='true'),
        DeclareLaunchArgument('panel', default_value='false'),
        DeclareLaunchArgument('workspace', default_value=str(Path.home() / 'robot_ws')),
        DeclareLaunchArgument('use_components', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(default_params)),
        GroupAction(condition=IfCondition(LaunchConfiguration('teleop')), actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel/manual'),
            include('teleop.launch.py', keyboard_only=LaunchConfiguration('keyboard_only')),
        ]),
        Node(package='mobile_manipulator_gazebo', executable='robot_panel.py', output='screen',
             condition=IfCondition(LaunchConfiguration('panel')),
             arguments=['--workspace', LaunchConfiguration('workspace')],
             remappings=[('/cmd_vel', '/cmd_vel/manual')]),
        Node(package=package, executable='velocity_arbiter.py', output='screen'),
        *servers,
        container,
    ])
