#!/usr/bin/env python3
"""Exercise the M2 navigation, obstacle and operator paths in an isolated simulation."""

import argparse
import json
import math
import os
from pathlib import Path
import pty
import subprocess
import tempfile
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


OBSTACLE = (.62, -.51)


def yaw(rotation):
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def speed(command):
    return abs(command.linear.x), abs(command.angular.z)


class Trial(Node):
    def __init__(self):
        super().__init__('m2_navigation_acceptance',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pose = None
        self.odom = None
        self.command = Twist()
        self.mode = None
        self.map = None
        self.obstacle_returns = 0
        self.trace = []
        self.pump_ui = lambda: None
        self.create_subscription(TFMessage, '/ground_truth/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/sim/cmd_vel', self.on_command, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(String, '/velocity_arbiter/mode', self.on_mode, latched)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.activate = self.create_client(Trigger, '/velocity_arbiter/enable_navigation')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')

    def on_pose(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id.split('::')[-1] == 'mobile_manipulator':
                t = transform.transform
                self.pose = (t.translation.x, t.translation.y, yaw(t.rotation))
                if self.now() - (self.trace[-1][0] if self.trace else -1.) >= .08:
                    self.trace.append([self.now(), *self.pose])

    def on_odom(self, msg):
        self.odom = msg.twist.twist

    def on_scan(self, msg):
        if self.pose is None:
            return
        x, y, theta = self.pose
        cosine, sine = math.cos(theta), math.sin(theta)
        for i, distance in enumerate(msg.ranges):
            if not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            lx = -.13 + distance * math.cos(angle)
            ly = -.08 + distance * math.sin(angle)
            if (abs(x + cosine*lx - sine*ly - OBSTACLE[0]) < .115 and
                    abs(y + sine*lx + cosine*ly - OBSTACLE[1]) < .115):
                self.obstacle_returns += 1
                break

    def on_command(self, msg):
        self.command = msg

    def on_map(self, msg):
        self.map = msg

    def on_mode(self, msg):
        self.mode = msg.data

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def wait(self, predicate, timeout, reason):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > end:
                raise RuntimeError(reason)
            self.pump_ui()
            rclpy.spin_once(self, timeout_sec=.05)

    def future(self, future, timeout, reason):
        self.wait(future.done, timeout, reason)
        return future.result()

    def wall_delay(self, duration):
        end = time.monotonic() + duration
        self.wait(lambda: time.monotonic() >= end, duration + 2, 'Unable to spin node')

    def ready(self):
        self.wait(lambda: self.action.server_is_ready() and self.lifecycle.service_is_ready()
                  and self.map is not None and self.pose is not None and self.odom is not None,
                  150, 'Nav2, SLAM map or live odometry missing')
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            state = self.future(self.lifecycle.call_async(GetState.Request()), 12, 'Lifecycle query failed')
            if state.current_state.label == 'active':
                break
            self.wall_delay(.5)
        else:
            raise RuntimeError(f'BT navigator state is {state.current_state.label}')
        for topic in ('/cmd_vel', '/cmd_vel/nav', '/sim/cmd_vel'):
            publishers = sorted({p.node_name for p in self.get_publishers_info_by_topic(topic)})
            print(topic, publishers, flush=True)
            if topic == '/cmd_vel' and publishers != ['velocity_arbiter']:
                raise RuntimeError('Unexpected direct Guard input')
            if topic == '/cmd_vel/nav' and publishers != ['behavior_server', 'controller_server']:
                raise RuntimeError('Navigation or recovery output bypasses arbitration')

    def send(self, x, y, heading):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(heading / 2)
        goal.pose.pose.orientation.w = math.cos(heading / 2)
        handle = self.future(self.action.send_goal_async(goal), 20, 'No goal response')
        if not handle.accepted:
            raise RuntimeError('Navigation target rejected')
        return handle, handle.get_result_async()

    def arrive(self, goal):
        self.trace.clear()
        start = self.pose
        handle, result = self.send(*goal)
        try:
            self.wait(result.done, 1000, 'Navigation did not finish within the trial limit')
        except (RuntimeError, KeyboardInterrupt):
            handle.cancel_goal_async()
            self.wall_delay(.5)
            raise
        stop = self.pose
        status = result.result().status
        return {
            'status': status,
            'start': start,
            'finish': stop,
            'goal': goal,
            'error_m': math.dist(stop[:2], goal[:2]),
            'trajectory': self.trace.copy(),
        }

    def block_path(self):
        model = Path(get_package_share_directory('mobile_manipulator_autonomy')) / 'models/m2_obstacle.sdf'
        result = subprocess.run(
            ['ign', 'service', '-s', '/world/flat_world/create',
             '--reqtype', 'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '8000', '--req', 'sdf_filename: ' + json.dumps(str(model))],
            check=True, capture_output=True, text=True, timeout=12,
        )
        if 'data: true' not in result.stdout:
            raise RuntimeError(f'Gazebo obstacle creation failed: {result.stdout}')
        self.obstacle_returns = 0
        self.wait(lambda: self.obstacle_returns >= 2, 45, 'LiDAR did not observe the added obstacle')

    def moving_goal(self):
        x, y, heading = self.pose
        goal = (x + .45 * math.cos(heading), y + .45 * math.sin(heading), heading)
        handle, result = self.send(*goal)
        self.wait(lambda: speed(self.command)[0] > .015 or result.done(), 60, 'Navigation did not start driving')
        if result.done():
            raise RuntimeError(f'Test goal terminated before intervention: {result.result().status}')
        return handle, result

    def stopped(self):
        self.wait(lambda: max(speed(self.command)) < .002 and max(speed(self.odom)) < .012,
                  35, 'Robot did not settle after stopping')
        start = self.pose
        self.wall_delay(2.0)
        drift = math.dist(start[:2], self.pose[:2])
        if drift > .015 or max(speed(self.command)) >= .002:
            raise RuntimeError(f'Output or robot resumed after stopping (drift {drift} m)')
        return drift

    def reenable(self):
        response = self.future(self.activate.call_async(Trigger.Request()), 10, 'Arbiter did not respond')
        if not response.success:
            raise RuntimeError(response.message)
        self.wait(lambda: self.mode == 'navigation', 8, 'Navigation mode not restored')

    def test_cancel(self):
        handle, result = self.moving_goal()
        reply = self.future(handle.cancel_goal_async(), 15, 'No response to cancel')
        status = self.future(result, 20, 'Cancelled navigation remained active').status
        return {'status': status, 'accepted': bool(reply.goals_canceling), 'stop_drift_m': self.stopped()}

    def test_keyboard(self):
        _, result = self.moving_goal()
        master, slave = pty.openpty()
        child = subprocess.Popen(['ros2', 'run', 'mobile_manipulator_gazebo', 'teleop_keyboard.py'],
                                 stdin=slave, stdout=slave, stderr=slave)
        os.close(slave)
        try:
            self.wait(lambda: 'teleop_keyboard' in self.get_node_names(), 20, 'Keyboard did not start')
            self.wall_delay(.4)
            start = self.pose
            end = time.monotonic() + 35
            minimum = 0.
            travel = 0.
            while time.monotonic() < end and travel > -.003:
                os.write(master, b's')
                self.wall_delay(.10)
                minimum = min(minimum, self.command.linear.x)
                travel = ((self.pose[0] - start[0]) * math.cos(start[2])
                          + (self.pose[1] - start[1]) * math.sin(start[2]))
            status = self.future(result, 25, 'Manual takeover did not cancel the Nav2 goal').status
            drift = self.stopped()
            return {'status': status, 'manual_output_mps': minimum, 'manual_travel_m': travel,
                    'mode_after_release': self.mode, 'stop_drift_m': drift}
        finally:
            os.write(master, b'q')
            try:
                child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=6)
            finally:
                os.close(master)

    def test_panel(self):
        import tkinter as tk
        from robot_panel import Panel, PanelBackend

        with tempfile.TemporaryDirectory(prefix='m2-panel-') as workspace:
            root = tk.Tk()
            root.withdraw()
            backend = PanelBackend(Path(workspace))
            try:
                panel = Panel(root, backend)
                self.pump_ui = root.update
                self.wall_delay(.8)
                _, result = self.moving_goal()
                panel.buttons['STOP'].invoke()
                status = self.future(result, 25, 'Panel Stop did not cancel navigation').status
                return {'status': status, 'mode_after_stop': self.mode, 'stop_drift_m': self.stopped()}
            finally:
                self.pump_ui = lambda: None
                root.destroy()
                backend.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--obstacle-present', action='store_true', help='Reuse the trial obstacle already in the world')
    parser.add_argument('--phase', choices=('all', 'basic', 'obstacle', 'operator'), default='all')
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID', '0') == '0' or not os.getenv('IGN_PARTITION'):
        parser.error('Use an isolated ROS_DOMAIN_ID and IGN_PARTITION')
    rclpy.init(args=['--ros-args', '-r', 'robot_panel:/cmd_vel:=/cmd_vel/manual'])
    trial = Trial()
    report = {'passed': False}
    with args.output.open('x') as output:
        try:
            trial.ready()
            if args.phase in ('all', 'basic'):
                print('START basic', flush=True)
                row = report['basic'] = trial.arrive((.62, 0., 0.))
                print('BASIC', row['status'], row['finish'], 'error_m', row['error_m'], flush=True)
                if row['status'] != GoalStatus.STATUS_SUCCEEDED or row['error_m'] > .14:
                    raise RuntimeError('Basic navigation did not reach the target')
            if args.phase in ('all', 'obstacle'):
                print('START obstacle', flush=True)
                if args.obstacle_present:
                    trial.wait(lambda: trial.obstacle_returns >= 2, 45, 'LiDAR did not observe the existing obstacle')
                else:
                    trial.block_path()
                row = report['obstacle'] = trial.arrive((.62, -1.02, -math.pi/2))
                corridor = [abs(x - OBSTACLE[0]) for _, x, y, _ in row['trajectory']
                            if abs(y - OBSTACLE[1]) < .10]
                row['detour_m'] = max(corridor, default=0.)
                row['seen_scan_samples'] = trial.obstacle_returns
                print('OBSTACLE', row['status'], 'detour_m', row['detour_m'], 'laser samples', row['seen_scan_samples'], flush=True)
                if row['status'] != GoalStatus.STATUS_SUCCEEDED or row['error_m'] > .16 or row['detour_m'] < .26:
                    raise RuntimeError('Obstacle run did not complete a measured detour')
            if args.phase in ('all', 'operator'):
                print('START cancel', flush=True)
                report['cancel'] = trial.test_cancel()
                print('START keyboard', flush=True)
                report['keyboard'] = trial.test_keyboard()
                trial.wall_delay(.8)
                trial.reenable()
                print('START panel', flush=True)
                report['panel'] = trial.test_panel()
                if any(report[key]['status'] != GoalStatus.STATUS_CANCELED for key in ('cancel','keyboard','panel')):
                    raise RuntimeError('A navigation goal was not cancelled')
                if not (report['cancel']['accepted'] and report['keyboard']['manual_output_mps'] < -.015
                        and report['keyboard']['manual_travel_m'] <= -.003
                        and report['keyboard']['mode_after_release'] == 'manual'
                        and report['panel']['mode_after_stop'] == 'manual'):
                    raise RuntimeError('Takeover did not latch the manual mode')
            report['passed'] = True
        except (AssertionError, RuntimeError) as e:
            report['error'] = str(e)
        finally:
            json.dump(report, output, indent=2)
            output.write('\n')
            trial.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(json.dumps({key: {k: v for k, v in value.items() if k != 'trajectory'}
                      if isinstance(value, dict) else value for key, value in report.items()}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
