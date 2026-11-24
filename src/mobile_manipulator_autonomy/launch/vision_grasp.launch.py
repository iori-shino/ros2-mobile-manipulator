"""Start a standalone single-cube M4 world and the existing M3 locator."""

from pathlib import Path
from tempfile import NamedTemporaryFile
from copy import deepcopy
import math
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    sim = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    autonomy = Path(get_package_share_directory('mobile_manipulator_autonomy'))
    x = float(LaunchConfiguration('cube_x').perform(context))
    y = float(LaunchConfiguration('cube_y').perform(context))
    if not (.265 <= x <= .285 and -.016 <= y <= .016):
        raise ValueError('Cube must start in the M4 test rectangle')
    carry = LaunchConfiguration('carry_demo').perform(context) == 'true'
    dock = carry or LaunchConfiguration('dock_demo').perform(context) == 'true'
    layout = yaml.safe_load((autonomy / 'config' / 'workcells.yaml').read_text())
    cell = layout['m43']
    px, py, yaw = cell['pickup_dock'] if dock else (0., 0., 0.)

    def pickup_pose(x, y, z):
        wx = px+math.cos(yaw)*x-math.sin(yaw)*y
        wy = py+math.sin(yaw)*x+math.cos(yaw)*y
        return f'{wx:.9f} {wy:.9f} {z:.9f} 0 0 {yaw:.9f}'
    tree = ET.parse(sim / 'worlds' / 'grasp_test.sdf')
    world = tree.find('world')
    vision = ET.parse(autonomy / 'worlds' / 'vision_table.sdf')
    world.insert(5, vision.find("world/plugin[@name='ignition::gazebo::systems::Sensors']"))
    support = world.find("model[@name='grasp_support']")
    top = cell['table_top_m']
    support.find('pose').text = pickup_pose(*cell['pickup_table_local_xy'], top/2)
    for size in support.findall('.//box/size'):
        size.text = ' '.join(str(value) for value in (*cell['table_size_xy'], top))
    cube = world.find("model[@name='grasp_cube']")
    cube.find('pose').text = pickup_pose(x, y, top+.035/2+.001)
    for component in cube.findall('.//material/*'):
        component.text = '0.95 0.035 0.045 1'
    if dock:
        room = ET.parse(sim / 'worlds' / 'slam_room.sdf')
        for wall in room.findall('world/model'):
            if wall.get('name').endswith('_wall'):
                world.append(wall)
    if carry:
        destination = deepcopy(support)
        destination.set('name', 'place_support')
        dx, dy = cell['drop_table_world_xy']
        heading = cell['drop_dock_world'][2]
        destination.find('pose').text = f'{dx} {dy} {top/2} 0 0 {heading}'
        world.append(destination)
        sensor = ET.SubElement(cube.find('link'), 'sensor', name='cube_contact', type='contact')
        ET.SubElement(sensor, 'update_rate').text = '50'
        contact = ET.SubElement(sensor, 'contact')
        ET.SubElement(contact, 'collision').text = 'collision'
        ET.SubElement(contact, 'topic').text = '/carry/cube_contacts'
    with NamedTemporaryFile(prefix='m4_grasp_', suffix='.sdf', delete=False) as output:
        tree.write(output, encoding='utf-8', xml_declaration=True)
    world_path = Path(output.name)

    temporary_files = [world_path]

    def cleanup(_context):
        for path in temporary_files:
            path.unlink(missing_ok=True)

    vision_nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(autonomy / 'launch' / 'vision.launch.py')),
            launch_arguments={'table_height_m': str(top-layout['base_footprint_rest_z_m'])}.items()),
    ]
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch' / 'sim.launch.py')),
            launch_arguments={'world': str(world_path), 'grasp_contacts': 'true',
                              'gui': LaunchConfiguration('gui'),
                              'configure_controller_clock': 'false' if dock else 'true',
                              'staged_startup': 'true' if carry else 'false',
                              'controller_service_timeout': '45.0' if dock else '10.0'}.items()),
    ]
    if carry:
        vision_nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                                 name='carry_cube_bridge',
                                 arguments=['/carry/cube_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts']))
    else:
        actions.extend(vision_nodes)
    if dock:
        params = yaml.safe_load((autonomy / 'config' / 'navigation.yaml').read_text())
        controller = params['controller_server']['ros__parameters']
        controller['controller_frequency'] = 20.
        controller['goal_checker'].update(xy_goal_tolerance=.003, yaw_goal_tolerance=.018,
                                          stateful=False)
        controller['FollowPath']['min_approach_linear_velocity'] = .010
        params['planner_server']['ros__parameters']['GridBased']['tolerance'] = .02
        # The room layout and odometry origin are fixed for this short docking demo.
        params['bt_navigator']['ros__parameters']['global_frame'] = 'odom'
        costmap = params['global_costmap']['global_costmap']['ros__parameters']
        costmap.update(global_frame='odom', rolling_window=True, width=3, height=3,
                       track_unknown_space=False, plugins=['obstacle_layer', 'inflation_layer'])
        del costmap['static_layer']
        if carry:
            controller['FollowPath']['rotate_to_heading_angular_vel'] = .25
            behavior = params['behavior_server']['ros__parameters']
            behavior.update(max_rotational_vel=.25, min_rotational_vel=.10)
        with NamedTemporaryFile(prefix='m42_nav_', suffix='.yaml', mode='w', delete=False) as output:
            yaml.safe_dump(params, output)
        temporary_files.append(Path(output.name))
        navigation = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(autonomy / 'launch' / 'navigation.launch.py')),
                launch_arguments={'teleop': 'false', 'panel': 'false',
                                  'params_file': output.name,
                                  'autostart': 'false' if carry else 'true',
                                  'use_components': 'true' if carry else 'false'}.items(),
            ),
        ]
        if carry:
            # The camera, both arm controllers and their clock have completed their startup.
            def start_carry_nodes(event, _context):
                if event.process_name.startswith('m43_controller_clock-') and event.returncode == 0:
                    return vision_nodes+navigation
                return []

            actions.insert(0, RegisterEventHandler(OnProcessExit(on_exit=start_carry_nodes)))
            return actions

        # Fortress's ros2_control requests the robot description during spawn.
        # Gripper activates after the arm. Wait for its controller state traffic.
        ready = ExecuteProcess(cmd=['timeout', '180', 'ros2', 'topic', 'echo',
                                    '/gripper_controller/controller_state',
                                    'control_msgs/msg/JointTrajectoryControllerState',
                                    '--once', '--no-daemon'], output='log')

        def start_navigation(event, _context):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='Grasp controller did not publish state'))]
            return navigation

        actions.extend([RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=start_navigation)),
                        ready])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cube_x', default_value='0.276'),
        DeclareLaunchArgument('cube_y', default_value='0.008'),
        DeclareLaunchArgument('carry_demo', default_value='false'),
        DeclareLaunchArgument('dock_demo', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=setup),
    ])
