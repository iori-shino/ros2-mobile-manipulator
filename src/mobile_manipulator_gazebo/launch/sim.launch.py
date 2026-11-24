"""Fortress + physical four-wheel skid-steer. No joint_state_publisher_gui."""
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    sim=Path(get_package_share_directory('mobile_manipulator_gazebo'))
    description=Path(get_package_share_directory('mobile_manipulator_description'))
    output=Path(tempfile.mkdtemp(prefix='mobile_manipulator_fortress_'))
    staged = LaunchConfiguration('staged_startup').perform(context) == 'true'
    grasp_contacts = LaunchConfiguration('grasp_contacts').perform(context).lower() == 'true'
    model_options = ['--grasp-contacts'] if grasp_contacts else []
    subprocess.run([sys.executable,str(sim/'scripts/prepare_model.py'),
                    '--description',str(description),'--output',str(output),
                    '--controllers',str(sim/'config/arm_controllers.yaml'),
                    '--control-plugin',str(Path(get_package_prefix('gz_ros2_control'))/'lib/libgz_ros2_control-system.so'), *model_options],check=True)
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    command=['ign','gazebo','-s','-r','-v','3','--render-engine','ogre']
    world_path = LaunchConfiguration('world').perform(context)
    if staged:
        # Same initial pose as spawn_when_ready, with no Gazebo transport create round-trip.
        world = ET.parse(world_path)
        robot = ET.parse(output/'model.sdf').find('model')
        pose = robot.find('pose')
        if pose is None:
            pose = ET.SubElement(robot, 'pose')
        pose.text = '0 0 0.04 0 0 0'
        world.find('world').append(robot)
        world_path = str(output/'initial_world.sdf')
        world.write(world_path, encoding='utf-8', xml_declaration=True)
    command.append(world_path)
    # VMware's accelerated renderer can produce invalid GPU lidar ranges.
    # Mesa software rendering keeps the sensor renderer deterministic here.
    gazebo=ExecuteProcess(cmd=command,output='screen',additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'})
    spawn=ExecuteProcess(cmd=[sys.executable,str(sim/'scripts/spawn_when_ready.py'),
                              '--world','flat_world','--model',str(output/'model.sdf')],output='screen')
    # Require an acknowledged creation after the world starts ticking.
    gui_actions=[]
    def spawned(event, _context):
        if event is not None and event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Robot spawn failed; see spawn process log'))]
        controllers=Node(package='controller_manager',executable='spawner',output='screen',
                     name='m43_controller_spawner' if staged else None,
                     parameters=[{'use_sim_time':True}],
                     arguments=['arm_controller','gripper_controller','--controller-manager-timeout','120',
                                '--service-call-timeout',LaunchConfiguration('controller_service_timeout')])
        # Humble's plugin helper node ignores controller parameter-file options.
        # Set its clock once it exists; this does not alter any control gains.
        clock_setup=ExecuteProcess(cmd=['ros2','param','set','--no-daemon','--spin-time','5',
                                       '/gz_ros2_control','use_sim_time','true'],output='screen')
        if staged:
            # Use the service directly and check its response before starting the remaining ROS graph.
            clock_setup = ExecuteProcess(name='m43_controller_clock', cmd=[sys.executable, '-c',
                "import subprocess; "
                "result = subprocess.run(['ros2','service','call','/gz_ros2_control/set_parameters',"
                "'rcl_interfaces/srv/SetParameters',"
                "'{parameters: [{name: use_sim_time, value: {type: 1, bool_value: true}}]}'],"
                "capture_output=True, text=True, timeout=60, check=True); "
                "print(result.stdout, flush=True); "
                "raise SystemExit(0 if 'successful=True' in result.stdout else 1)"], output='screen')

        def clock_set(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='M4.3 controller clock setup failed'))]
            return bridges+grasp_bridges+gui_actions

        def configured(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Controller startup failed; see spawner log'))]
            if staged:
                return [RegisterEventHandler(OnProcessExit(target_action=clock_setup, on_exit=clock_set)),
                        clock_setup]
            if LaunchConfiguration('configure_controller_clock').perform(_context) == 'true':
                return [clock_setup]
            return []
        watch_config = RegisterEventHandler(OnProcessExit(target_action=controllers,on_exit=configured))
        if not staged:
            return [watch_config, controllers]+gui_actions

        # The first camera frame arrives after Fortress has initialized its renderer.
        camera_ready = ExecuteProcess(
            cmd=['timeout', '150', 'ros2', 'topic', 'echo', '/camera/image_raw',
                 'sensor_msgs/msg/Image', '--field', 'header', '--once', '--no-daemon',
                 '--qos-reliability', 'best_effort'], output='log')

        def start_controllers(frame_event, _context):
            if frame_event.returncode:
                return [EmitEvent(event=Shutdown(reason='Camera did not initialize before arm startup'))]
            return [controllers]

        return [watch_config, RegisterEventHandler(OnProcessExit(target_action=camera_ready,
                                                                  on_exit=start_controllers)),
                camera_ready]
    shutdown_on_gazebo = RegisterEventHandler(OnProcessExit(target_action=gazebo,
        on_exit=[EmitEvent(event=Shutdown(reason='Gazebo closed'))]))
    state_publisher = Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
                           parameters=[{'robot_description':(output/'robot_description.urdf').read_text(),
                                        'use_sim_time':True}])
    camera = Node(package='ros_gz_bridge',executable='parameter_bridge',name='camera_bridge',output='screen',
             arguments=['/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
             parameters=[{'use_sim_time':True,'override_frame_id':'camera_optical_frame'}])
    bridges=[
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='gazebo_bridge',output='screen',
             parameters=[{'config_file':str(sim/'config/bridge.yaml'),'use_sim_time':True}]),
        Node(package='mobile_manipulator_gazebo',executable='cmd_vel_guard.py',output='screen',parameters=[{'use_sim_time':True}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='lidar_bridge',output='screen',
             arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
             parameters=[{'use_sim_time':True,'override_frame_id':'lidar_scan_frame'}]),
        Node(package='tf2_ros',executable='static_transform_publisher',name='lidar_scan_tf',
             arguments=['--z','0.02','--frame-id','lidar_link','--child-frame-id','lidar_scan_frame'],
             parameters=[{'use_sim_time':True}]),
    ]
    grasp_bridges = []
    if grasp_contacts:
        grasp_bridges.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='grasp_bridge', output='screen',
                            parameters=[{'config_file': str(sim/'config/grasp_bridge.yaml'),
                                         'use_sim_time': True}]))
    if gui:
        # Keep the verified VMware GUI rendering separate from software sensors.
        client=ExecuteProcess(cmd=['ign','gazebo','-g','--render-engine','ogre',
                                  '--gui-config',str(sim/'config/fortress_gui.config')],output='screen')
        gui_actions.extend([RegisterEventHandler(OnProcessExit(target_action=client,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo GUI closed'))])),client])
    if staged:
        return [shutdown_on_gazebo, state_publisher, camera, gazebo,
                OpaqueFunction(function=lambda context: spawned(None, context))]
    return [shutdown_on_gazebo,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
            gazebo, state_publisher, *bridges, camera, spawn, *grasp_bridges]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('world',default_value=str(Path(get_package_share_directory('mobile_manipulator_gazebo'))/'worlds/flat_world.sdf')),
        DeclareLaunchArgument('configure_controller_clock',default_value='true'),
        DeclareLaunchArgument('staged_startup',default_value='false'),
        DeclareLaunchArgument('controller_service_timeout',default_value='10.0'),
        DeclareLaunchArgument('grasp_contacts', default_value='false',
                              description='Enable fingertip sensors and the grasp test bridge'),
        OpaqueFunction(function=setup)])
