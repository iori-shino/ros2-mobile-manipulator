"""Launch the seeded M5 world with the existing camera and M3 locator."""

from pathlib import Path
import subprocess
import sys
from tempfile import mkdtemp

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xml.etree.ElementTree as ET
from launch.actions import EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import yaml


def setup(context):
    package = 'mobile_manipulator_autonomy'
    autonomy = Path(get_package_share_directory(package))
    gazebo = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    config = Path(LaunchConfiguration('config').perform(context)).resolve()
    layout = yaml.safe_load(config.read_text())
    seed = LaunchConfiguration('seed').perform(context)
    saved = LaunchConfiguration('scene_dir').perform(context)
    directory = Path(saved).resolve() if saved else Path(mkdtemp(prefix='m5_scene_'))
    directory.mkdir(parents=True, exist_ok=True)
    world, manifest = directory/'world.sdf', directory/'manifest.json'
    generator = Path(get_package_prefix(package))/'lib'/package/'m5_scene.py'
    try:
        subprocess.run([sys.executable, str(generator), '--seed', seed, '--config', str(config),
                        '--sdf', str(world), '--manifest', str(manifest)], check=True)
    except subprocess.CalledProcessError:
        if not saved:
            world.unlink(missing_ok=True)
            manifest.unlink(missing_ok=True)
            directory.rmdir()
        raise

    pick = LaunchConfiguration('pick_demo').perform(context) == 'true'
    sorting = LaunchConfiguration('sorting').perform(context) == 'true'
    if sorting and not pick:
        raise ValueError('sorting requires pick_demo')
    temporary_files = [world, manifest]
    if pick:
        tree = ET.parse(world)
        ET.SubElement(tree.find('world'), 'plugin',
                      filename='ignition-gazebo-contact-system',
                      name='ignition::gazebo::systems::Contact')
        if sorting:
            for color in ('red', 'blue', 'yellow'):
                link = tree.find(f"world/model[@name='m3_{color}_cube']/link")
                sensor = ET.SubElement(link, 'sensor', name='sorting_contact', type='contact')
                ET.SubElement(sensor, 'update_rate').text = '50'
                contact = ET.SubElement(sensor, 'contact')
                ET.SubElement(contact, 'collision').text = 'collision'
                ET.SubElement(contact, 'topic').text = f'/sorting/contact/{color}'
        tree.write(world, encoding='utf-8', xml_declaration=True)

    def cleanup(_context):
        for path in (temporary_files if not saved else temporary_files[2:]):
            path.unlink(missing_ok=True)
        if not saved:
            directory.rmdir()

    table_height = layout['m5']['table_top_m'] - layout['base_footprint_rest_z_m']
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo/'launch'/'sim.launch.py')),
            launch_arguments={'world': str(world), 'gui': LaunchConfiguration('gui'),
                              'grasp_contacts': 'true' if pick else 'false',
                              'staged_startup': 'true' if pick else 'false'}.items(),
        ),
    ]
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(autonomy/'launch'/'vision.launch.py')),
        launch_arguments={'table_height_m': str(table_height)}.items())
    if not pick:
        return actions+[vision]

    params = yaml.safe_load((autonomy/'config'/'navigation.yaml').read_text())
    params['controller_server']['ros__parameters']['goal_checker'].update(
        xy_goal_tolerance=.003, yaw_goal_tolerance=.018, stateful=False)
    if sorting:
        params['behavior_server']['ros__parameters']['max_rotational_vel'] = .24
        follow = params['controller_server']['ros__parameters']['FollowPath']
        follow['rotate_to_heading_angular_vel'] = .24
    nav_yaml = directory/'.m52_navigation.yaml'
    with nav_yaml.open('x') as output:
        yaml.safe_dump(params, output)
    temporary_files.append(nav_yaml)
    color = LaunchConfiguration('target_color').perform(context)
    if color not in ('red', 'blue', 'yellow'):
        raise ValueError('target_color must be red, blue or yellow')
    pose_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                       name='m52_target_pose_bridge',
                       arguments=[f'/model/m3_{color}_cube/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
                       remappings=[(f'/model/m3_{color}_cube/pose', '/grasp/cube_pose')])
    sorting_bridges = [Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='m6_contact_bridge', arguments=[
                                f'/sorting/contact/{name}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
                                for name in ('red', 'blue', 'yellow')])] if sorting else [pose_bridge]
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo/'launch'/'slam.launch.py')),
        launch_arguments={'rviz': 'false'}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(autonomy/'launch'/'navigation.launch.py')),
        launch_arguments={'teleop': 'false', 'panel': 'false',
                          'use_components': LaunchConfiguration('use_components'),
                          'autostart': 'false', 'params_file': str(nav_yaml)}.items())
    map_ready = ExecuteProcess(cmd=['timeout', '150', 'ros2', 'topic', 'echo', '/map',
                                   'nav_msgs/msg/OccupancyGrid', '--field', 'header',
                                   '--qos-durability', 'transient_local', '--once', '--no-daemon'],
                               output='log')

    def start_navigation(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='M5.2 did not receive a SLAM map'))]
        return [navigation]

    def start_perception(event, _context):
        if event.process_name.startswith('m43_controller_clock-') and event.returncode == 0:
            return [vision, *sorting_bridges, slam, map_ready]
        return []

    return [RegisterEventHandler(OnProcessExit(on_exit=start_perception)),
            RegisterEventHandler(OnProcessExit(target_action=map_ready, on_exit=start_navigation)),
            *actions]


def generate_launch_description():
    autonomy = Path(get_package_share_directory('mobile_manipulator_autonomy'))
    return LaunchDescription([
        DeclareLaunchArgument('sorting', default_value='false'),
        DeclareLaunchArgument('use_components', default_value='true'),
        DeclareLaunchArgument('pick_demo', default_value='false'),
        DeclareLaunchArgument('target_color', default_value='red'),
        DeclareLaunchArgument('seed', default_value='51'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('scene_dir', default_value='',
                              description='Optional directory for the generated SDF and manifest'),
        DeclareLaunchArgument('config', default_value=str(autonomy/'config'/'workcells.yaml')),
        OpaqueFunction(function=setup),
    ])
