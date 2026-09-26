#!/usr/bin/env python3
"""Read-only system checks; no motion commands and bounded wall-time capture."""
import argparse
from collections import defaultdict, deque
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Image, Joy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from controller_manager_msgs.srv import ListControllers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=25.)
    parser.add_argument('--output', default='log/stage6_health.json')
    parser.add_argument('--skip-teleop', action='store_true')
    parser.add_argument('--keyboard-only', action='store_true')
    args = parser.parse_args()
    if not 5 <= args.duration <= 120:
        parser.error('duration must be 5..120 wall seconds')
    rclpy.init()
    node = rclpy.create_node('system_health')
    received = defaultdict(lambda: deque(maxlen=2))
    counts = defaultdict(int)
    first = {}
    last_wall = {}
    regressions = defaultdict(int)
    stamps = {}
    def stamp(msg):
        if hasattr(msg, 'header'):
            return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if hasattr(msg, 'clock'):
            return msg.clock.sec + msg.clock.nanosec / 1e9
        return None
    def receive(topic, msg):
        t = stamp(msg)
        if t is not None:
            if t < stamps.get(topic, t):
                regressions[topic] += 1
            stamps[topic] = t
            first.setdefault(topic, t)
        counts[topic] += 1
        last_wall[topic] = time.monotonic()
        received[topic].append(msg)
    types = {'/scan': LaserScan, '/odom': Odometry, '/clock': Clock, '/camera/image_raw': Image,
             '/sim/cmd_vel': Twist, '/map': OccupancyGrid}
    if not args.skip_teleop:
        types['/teleop/status'] = String
        if not args.keyboard_only:
            types['/joy'] = Joy
    subscriptions = []
    for topic, typ in types.items():
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic == '/map' else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(typ, topic, lambda m,t=topic: receive(t,m), qos))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    future = None
    end = time.monotonic()+args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        if future is None and client.service_is_ready():
            future = client.call_async(ListControllers.Request())
    checks = {}
    detail = {}
    def check(name, okay, data=None):
        checks[name] = bool(okay)
        if data is not None:
            detail[name] = data
        print(('PASS ' if okay else 'FAIL ') + name)
    names = [name for name,ns in node.get_node_names_and_namespaces()]
    required = ['robot_state_publisher','cmd_vel_guard','gazebo_bridge','lidar_bridge','camera_bridge','controller_manager','slam_toolbox']
    if not args.skip_teleop:
        required += ['teleop_router']
        if not args.keyboard_only:
            required += ['joy_node']
    check('required_nodes', all(name in names for name in required), {'missing': [n for n in required if n not in names]})
    for topic in types:
        continuous = topic not in ['/map']
        good = counts[topic] >= (3 if continuous else 1)
        if topic in stamps and topic != '/map':
            good = good and stamps[topic] > first[topic] and not regressions[topic]
        if continuous:
            good = good and time.monotonic()-last_wall.get(topic, 0) < 5
        check('topic '+topic, good, {'samples': counts[topic], 'stamp_advance': stamps.get(topic,0)-first.get(topic,0)})
    def latest(topic):
        return received[topic][-1] if received[topic] else None
    scan, camera, odom, grid = (latest(t) for t in ['/scan','/camera/image_raw','/odom','/map'])
    check('scan_payload', scan is not None and len(scan.ranges)==360 and any(math.isfinite(x) and scan.range_min<=x<=scan.range_max for x in scan.ranges))
    check('camera_payload', camera is not None and camera.width>0 and camera.height>0 and len(camera.data)==camera.step*camera.height)
    check('odom_payload', odom is not None and odom.header.frame_id=='odom' and all(math.isfinite(v) for v in [odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.orientation.w]))
    check('slam_map', grid is not None and grid.header.frame_id=='map' and grid.info.resolution>0 and len(grid.data)==grid.info.width*grid.info.height and any(v>=0 for v in grid.data))
    for frame in ['odom','base_footprint','base_link','lidar_link','lidar_scan_frame','camera_optical_frame']:
        try:
            buffer.lookup_transform('map', frame, Time())
            check('TF map -> '+frame, True)
        except Exception as exc:
            check('TF map -> '+frame, False, str(exc))
    controllers = {c.name:c.state for c in future.result().controller} if future and future.done() and future.result() else {}
    for name in ['arm_controller','gripper_controller']:
        check(name+' active', controllers.get(name)=='active', controllers.get(name))
    subscribers = [e.node_name for e in node.get_subscriptions_info_by_topic('/cmd_vel')]
    publishers = [e.node_name for e in node.get_publishers_info_by_topic('/sim/cmd_vel')]
    bridges = [e.node_name for e in node.get_subscriptions_info_by_topic('/sim/cmd_vel')]
    check('cmd_vel -> guard -> sim bridge', 'cmd_vel_guard' in subscribers and publishers==['cmd_vel_guard'] and 'gazebo_bridge' in bridges,
          {'input_subscribers': subscribers, 'output_publishers': publishers, 'output_subscribers': bridges})
    service_names = dict(node.get_service_names_and_types())
    check('SLAM save_map service', '/slam_toolbox/save_map' in service_names)
    if not args.skip_teleop and not args.keyboard_only:
        check('one joy publisher', len(node.get_publishers_info_by_topic('/joy'))==1)
    report = {'pass': all(checks.values()), 'checks': checks, 'detail': detail, 'read_only': True}
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2)+'\n')
    print('\n'+('PASS' if report['pass'] else 'FAIL')+f' {sum(checks.values())}/{len(checks)}; '+str(path))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if report['pass'] else 1)


if __name__ == '__main__':
    main()
