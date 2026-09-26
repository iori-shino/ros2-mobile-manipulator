#!/usr/bin/env python3
"""Generate a simulation-only model without editing the validated Xacro sources.

Retains names, link/joint parent-child graph, joint types, origins and axes.
Arm / jaws use ros2_control with their original limits and axes.
Fixed joints are explicitly preserved in SDF.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import xacro


def put(parent, name, text):
    child = parent.find(name)
    if child is None:
        child = ET.SubElement(parent, name)
    child.text = str(text)
    return child


def graph(robot):
    return {j.get('name'):(j.get('type'), j.find('parent').get('link'),j.find('child').get('link'),
                          ET.tostring(j.find('origin')), ET.tostring(j.find('axis')) if j.find('axis') is not None else None)
            for j in robot.findall('joint')}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--description',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--controllers',type=Path,required=True)
    parser.add_argument('--control-plugin',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    files=sorted(args.description.glob('urdf/*.xacro'))
    before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    robot=ET.fromstring(xacro.process_file(str(args.description/'urdf/robot.urdf.xacro')).toxml())
    original_graph=graph(robot)
    controlled=['J1','J2','J3','J4','left_jaw_joint','right_jaw_joint']
    control=ET.SubElement(robot,'ros2_control',name='ArmGripperSystem',type='system')
    put(ET.SubElement(control,'hardware'),'plugin','gz_ros2_control/GazeboSimSystem')
    for name in controlled:
        entry=ET.SubElement(control,'joint',name=name)
        jaw=name.endswith('_jaw_joint')
        # Effort servo avoids DART velocity motors sticking at a prismatic stop.
        # Both jaws receive the same position trajectory; source mimic stays intact.
        command=ET.SubElement(entry,'command_interface',name='effort' if jaw else 'position')
        limit=robot.find(f"joint[@name='{name}']/limit")
        for key,attribute in [('min','lower'),('max','upper')]:
            ET.SubElement(command,'param',name=key).text=(('-' if key=='min' else '')+limit.get('effort')) if jaw else limit.get(attribute)
        position=ET.SubElement(entry,'state_interface',name='position')
        ET.SubElement(position,'param',name='initial_value').text='0.0075' if jaw else '0.0'
        ET.SubElement(entry,'state_interface',name='velocity')
    ET.ElementTree(robot).write(args.output/'robot_description.urdf',encoding='utf-8',xml_declaration=True)
    wheels={side:[f'front_{side}_wheel_joint',f'rear_{side}_wheel_joint'] for side in ['left','right']}
    # References without geometry need a small positive mass to survive URDF->SDF.
    # Both remain fixed to the same parents; source URDF stays massless for RViz/KDL.
    for name in ['base_footprint','camera_optical_frame']:
        link=robot.find(f"link[@name='{name}']")
        inertia=ET.SubElement(link,'inertial')
        ET.SubElement(inertia,'mass',value='0.001')
        ET.SubElement(inertia,'inertia',ixx='0.000001',iyy='0.000001',izz='0.000001',ixy='0',ixz='0',iyz='0')
    for joint in robot.findall('joint'):
        name=joint.get('name')
        if joint.get('type')=='fixed':
            extension=ET.SubElement(robot,'gazebo',reference=name)
            put(extension,'preserveFixedJoint','true')
        if name in controlled:
            ET.SubElement(joint,'dynamics',damping='0.2',friction='0.0')
    assert graph(robot)==original_graph, 'The validated joint graph must not change'
    for mesh in robot.findall('.//mesh'):
        prefix='package://mobile_manipulator_description/'
        uri=mesh.get('filename')
        if uri.startswith(prefix):
            mesh_path=(args.description/uri[len(prefix):]).resolve()
            assert mesh_path.is_file(),mesh_path
            mesh.set('filename',mesh_path.as_uri())
    urdf=args.output/'simulation.urdf'
    ET.ElementTree(robot).write(urdf,encoding='utf-8',xml_declaration=True)
    result=subprocess.run(['ign','sdf','-p',str(urdf)],capture_output=True,text=True,check=True)
    # ign may place non-XML diagnostic text before the document.
    start=result.stdout.find('<sdf ')
    assert start>=0, result.stdout+result.stderr
    sdf=ET.fromstring(result.stdout[start:result.stdout.rfind('</sdf>')+6])
    model=sdf.find('model')
    model.set('name','mobile_manipulator')
    model.set('canonical_link','base_link')
    put(model,'self_collide','false')
    assert {e.get('name') for e in model.findall('link')}=={e.get('name') for e in robot.findall('link')}
    assert {e.get('name') for e in model.findall('joint')}==set(original_graph)
    for joint in model.findall('joint'):
        source=original_graph[joint.get('name')]
        assert (joint.findtext('parent'),joint.findtext('child'))==source[1:3]
    for link in model.findall('link'):
        put(link,'self_collide','false')
        if link.get('name').endswith('_wheel'):
            for collision in link.findall('collision'):
                surface=ET.SubElement(collision,'surface')
                friction=ET.SubElement(surface,'friction')
                ode=ET.SubElement(friction,'ode')
                # Local collision Z is the axle, invariant under wheel rotation.
                # Follow Fortress's skid-steer example: lateral slip on direction 1,
                # rolling traction on the orthogonal tangent direction 2.
                put(ode,'mu','0.2');put(ode,'mu2','0.8')
                put(ode,'fdir1','0 0 1')
                put(ode,'slip1','0.035');put(ode,'slip2','0.0')
    # Sensor origins match the existing TF frames; no source Xacro changes.
    lidar=ET.SubElement(model.find("link[@name='lidar_link']"),'sensor',name='lidar',type='gpu_lidar')
    # Scan above the placeholder housing so its own visual cannot occlude rays.
    put(lidar,'pose','0 0 0.02 0 0 0')
    put(lidar,'topic','/scan');put(lidar,'update_rate','10');put(lidar,'always_on','true')
    ray=ET.SubElement(lidar,'lidar')
    scan=ET.SubElement(ray,'scan')
    horizontal=ET.SubElement(scan,'horizontal')
    for key,value in {'samples':360,'resolution':1,'min_angle':-3.141592653589793,'max_angle':3.141592653589793}.items():
        put(horizontal,key,value)
    vertical=ET.SubElement(scan,'vertical')
    for key,value in {'samples':1,'resolution':1,'min_angle':0,'max_angle':0}.items():
        put(vertical,key,value)
    ranges=ET.SubElement(ray,'range')
    for key,value in {'min':.08,'max':8.,'resolution':.01}.items():
        put(ranges,key,value)
    camera=ET.SubElement(model.find("link[@name='camera_link']"),'sensor',name='rgb_camera',type='camera')
    # Gazebo looks along +X. Existing camera_optical_frame is at this lens,
    # rotated to ROS optical axes (+Z forward, +X right, +Y down).
    put(camera,'pose','0.025 0 0 0 0 0')
    put(camera,'topic','/camera/image_raw');put(camera,'update_rate','10');put(camera,'always_on','true')
    optics=ET.SubElement(camera,'camera')
    put(optics,'horizontal_fov','1.0471975512')
    put(optics,'optical_frame_id','camera_optical_frame')
    put(optics,'camera_info_topic','/camera/camera_info')
    pixels=ET.SubElement(optics,'image')
    put(pixels,'width','320');put(pixels,'height','240');put(pixels,'format','R8G8B8')
    clip=ET.SubElement(optics,'clip');put(clip,'near','.02');put(clip,'far','10')
    drive=ET.SubElement(model,'plugin',filename='ignition-gazebo-diff-drive-system',name='ignition::gazebo::systems::DiffDrive')
    for side,names in wheels.items():
        for name in names:
            ET.SubElement(drive,side+'_joint').text=name
    for key,value in {'wheel_separation':.35,'wheel_radius':.05,'odom_publish_frequency':30,
                      'topic':'/model/mobile_manipulator/cmd_vel','odom_topic':'/model/mobile_manipulator/odometry',
                      'tf_topic':'/model/mobile_manipulator/tf','frame_id':'odom','child_frame_id':'base_footprint',
                      'max_linear_velocity':.05,'min_linear_velocity':-.05,
                      'max_angular_velocity':.4,'min_angular_velocity':-.4,
                      'max_linear_acceleration':.1,'min_linear_acceleration':-.1,
                      'max_angular_acceleration':.6,'min_angular_acceleration':-.6}.items():
        put(drive,key,value)
    publisher=ET.SubElement(model,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher')
    put(publisher,'topic','/model/mobile_manipulator/joint_state')
    assert args.control_plugin.is_file(),args.control_plugin
    controller=ET.SubElement(model,'plugin',filename=str(args.control_plugin),name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    put(controller,'robot_param','robot_description')
    put(controller,'robot_param_node','robot_state_publisher')
    put(controller,'parameters',args.controllers.resolve())
    poses=ET.SubElement(model,'plugin',filename='ignition-gazebo-pose-publisher-system',name='ignition::gazebo::systems::PosePublisher')
    for key,value in {'publish_link_pose':'false','publish_visual_pose':'false','publish_collision_pose':'false',
                      'publish_sensor_pose':'false','publish_model_pose':'true','publish_nested_model_pose':'true',
                      'use_pose_vector_msg':'true','update_frequency':30}.items():
        put(poses,key,value)
    ET.indent(sdf,space='  ')
    ET.ElementTree(sdf).write(args.output/'model.sdf',encoding='utf-8',xml_declaration=True)
    after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    assert before==after, 'Description sources modified unexpectedly'
    report={'unchanged_description_sha256':before,'links':len(model.findall('link')),
            'joints':len(model.findall('joint')),'controlled_joints':controlled,'wheel_groups':wheels,
            'conversion_stderr':result.stderr}
    (args.output/'model_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
