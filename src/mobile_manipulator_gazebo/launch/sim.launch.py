"""Fortress + physical four-wheel skid-steer. No joint_state_publisher_gui."""
from pathlib import Path
import subprocess
import sys
import tempfile

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
    subprocess.run([sys.executable,str(sim/'scripts/prepare_model.py'),
                    '--description',str(description),'--output',str(output),
                    '--controllers',str(sim/'config/arm_controllers.yaml'),
                    '--control-plugin',str(Path(get_package_prefix('gz_ros2_control'))/'lib/libgz_ros2_control-system.so')],check=True)
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    command=['ign','gazebo','-s','-r','-v','3','--render-engine','ogre']
    command.append(LaunchConfiguration('world').perform(context))
    # VMware's accelerated renderer can produce invalid GPU lidar ranges.
    # Mesa software rendering keeps the sensor renderer deterministic here.
    gazebo=ExecuteProcess(cmd=command,output='screen',additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'})
    spawn=Node(package='ros_gz_sim',executable='create',name='spawn_mobile_manipulator',output='screen',parameters=[{'use_sim_time':True}],
               arguments=['-world','flat_world','-name','mobile_manipulator','-file',str(output/'model.sdf'),
                          '-allow_renaming','false','-z','0.04'])
    # Spawn uses the world create service and waits for it; no guessed launch delay.
    def spawned(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Robot spawn failed; see spawn process log'))]
        controllers=Node(package='controller_manager',executable='spawner',output='screen',
                     parameters=[{'use_sim_time':True}],
                     arguments=['arm_controller','gripper_controller','--controller-manager-timeout','120'])
        # Humble's plugin helper node ignores controller parameter-file options.
        # Set its clock once it exists; this does not alter any control gains.
        clock_setup=ExecuteProcess(cmd=['ros2','param','set','/gz_ros2_control','use_sim_time','true'],output='screen')
        return [RegisterEventHandler(OnProcessExit(target_action=controllers,on_exit=[clock_setup])),controllers]
    actions=[
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=spawned)),
        gazebo,
        Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
             parameters=[{'robot_description':(output/'robot_description.urdf').read_text(),'use_sim_time':True}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='gazebo_bridge',output='screen',
             parameters=[{'config_file':str(sim/'config/bridge.yaml'),'use_sim_time':True}]),
        Node(package='mobile_manipulator_gazebo',executable='cmd_vel_guard.py',output='screen',parameters=[{'use_sim_time':True}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='lidar_bridge',output='screen',
             arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
             parameters=[{'use_sim_time':True,'override_frame_id':'lidar_scan_frame'}]),
        Node(package='tf2_ros',executable='static_transform_publisher',name='lidar_scan_tf',
             arguments=['--z','0.02','--frame-id','lidar_link','--child-frame-id','lidar_scan_frame'],parameters=[{'use_sim_time':True}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='camera_bridge',output='screen',
             arguments=['/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
             parameters=[{'use_sim_time':True,'override_frame_id':'camera_optical_frame'}]),
        spawn,
    ]
    if gui:
        # Keep the verified VMware GUI rendering separate from software sensors.
        client=ExecuteProcess(cmd=['ign','gazebo','-g','--render-engine','ogre',
                                  '--gui-config',str(sim/'config/fortress_gui.config')],output='screen')
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=client,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo GUI closed'))])),client])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('world',default_value=str(Path(get_package_share_directory('mobile_manipulator_gazebo'))/'worlds/flat_world.sdf')),
        OpaqueFunction(function=setup)])
