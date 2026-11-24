#!/usr/bin/env python3
"""Navigate, confirm a stationary base and grasp once in the M4.2 demo."""

from collections import deque
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import yaml

import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from acceptance_test import angles, wrap
from arm_control import ARM
from grasp_contact_probe import stamp
from m4_vision_grasp import VisionGrasp, main


WORKCELLS = yaml.safe_load(
    (Path(get_package_share_directory('mobile_manipulator_autonomy'))
     /'config'/'workcells.yaml').read_text())
M43_CELL = WORKCELLS['m43']
DOCK = tuple(M43_CELL['pickup_dock'])
SETTLE_S = 1.
LINEAR_LIMIT = .002
ANGULAR_LIMIT = .01


def motion(twist):
    return math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z)


class DockGrasp(VisionGrasp):
    def __init__(self):
        super().__init__()
        # The inherited physics probe has a test publisher. Only M2 owns Guard input here.
        self.destroy_publisher(self.publisher)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.clock_service = self.create_client(SetParameters, '/gz_ros2_control/set_parameters')
        self.stop = self.create_publisher(String, '/teleop/key', 10)
        self.mode = None
        self.guard = None
        self.guard_time = -math.inf
        self.monitor_motion = False
        self.unexpected_motion = False
        self.rgb = deque(maxlen=16)
        self.selected_rgb = None
        self.create_subscription(String, '/velocity_arbiter/mode', self.on_mode,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/sim/cmd_vel', self.on_guard, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_rgb, qos_profile_sensor_data)

    def on_mode(self, msg):
        self.mode = msg.data
        if self.monitor_motion and self.mode != 'manual':
            self.unexpected_motion = True

    def on_rgb(self, image):
        self.rgb.append(image)

    def on_guard(self, msg):
        self.guard = motion(msg)
        self.guard_time = time.monotonic()
        if self.monitor_motion and not self.is_still(self.guard):
            self.unexpected_motion = True

    def on_odom(self, msg):
        super().on_odom(msg)
        if self.monitor_motion and not self.is_still(motion(msg.twist.twist)):
            self.unexpected_motion = True

    @staticmethod
    def is_still(velocity):
        linear, angular = velocity
        return linear < LINEAR_LIMIT and angular < ANGULAR_LIMIT

    def stationary(self):
        return (self.mode == 'manual' and self.guard is not None
                and time.monotonic()-self.guard_time < .5 and self.is_still(self.guard)
                and self.odom is not None and -.03 <= self.now()-stamp(self.odom.header) < .3
                and self.is_still(motion(self.odom.twist.twist)))

    def wait(self, predicate, seconds, reason):
        limit = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() >= limit:
                raise RuntimeError(reason)
            rclpy.spin_once(self, timeout_sec=.05)

    def odom_pose(self):
        transform = self.tf.lookup_transform('odom', 'base_footprint', Time()).transform
        return [transform.translation.x, transform.translation.y, angles(transform.rotation)[2]]

    def configure_controller_clock(self):
        self.wait(self.clock_service.service_is_ready, 25, 'Controller clock service unavailable')
        parameter = Parameter('use_sim_time', value=True).to_parameter_msg()
        request = SetParameters.Request(parameters=[parameter])
        response = self.wait_future(self.clock_service.call_async(request), 30)
        if not response.results[0].successful:
            raise RuntimeError(f'Controller rejected simulation clock: {response.results[0].reason}')

    def navigate(self, report):
        self.wait(lambda: self.nav.server_is_ready() and self.lifecycle.service_is_ready()
                  and self.mode is not None, 100, 'Navigation action or velocity arbiter unavailable')
        deadline = time.monotonic()+100
        while time.monotonic() < deadline:
            state = self.wait_future(self.lifecycle.call_async(GetState.Request()), 8)
            if state.current_state.label == 'active' and self.tf.can_transform('odom', 'base_footprint', Time()):
                break
            self.spin_for(.5)
        else:
            raise RuntimeError('Nav2 did not activate with odometry TF')
        self.configure_controller_clock()
        if self.mode != 'navigation':
            raise RuntimeError('M2 arbiter is in manual takeover')
        publishers = {info.node_name for info in self.get_publishers_info_by_topic('/cmd_vel')}
        if publishers != {'velocity_arbiter'}:
            raise RuntimeError(f'Unexpected Guard input publishers: {publishers}')
        row = report['navigation'] = {}
        self.navigate_to(row, DOCK)

    def navigate_to(self, row, target):
        row.update(goal_odom=target, start_world=self.pose.copy(), start_odom=self.odom_pose(),
                   start_s=self.now())
        row['guard_input_publishers'] = sorted(
            {info.node_name for info in self.get_publishers_info_by_topic('/cmd_vel')})
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'odom'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z = math.sin(target[2]/2)
        goal.pose.pose.orientation.w = math.cos(target[2]/2)
        row.update(self.run_base_action(self.nav, goal, 'NavigateToPose'))
        if row['status'] != GoalStatus.STATUS_SUCCEEDED or self.mode != 'navigation':
            raise RuntimeError(f"Navigation failed or was interrupted: status {row['status']}")

    def run_base_action(self, client, goal, name):
        handle = self.wait_future(client.send_goal_async(goal), 15)
        if not handle.accepted:
            raise RuntimeError(f'Nav2 rejected {name}')
        try:
            result = self.base_result(handle)
        except (RuntimeError, TimeoutError, KeyboardInterrupt):
            self.wait_future(handle.cancel_goal_async(), 10)
            self.stop.publish(String(data='stop'))
            raise
        return dict(status=result.status, result_s=self.now(),
                    result_world=self.pose.copy(), result_odom=self.odom_pose())

    def base_result(self, handle):
        return self.wait_future(handle.get_result_async(), 420)

    def settle(self, row):
        self.stop.publish(String(data='stop'))
        self.wait(lambda: self.mode == 'manual', 10, 'Arbiter did not latch out Nav2')
        interval = None
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if not self.stationary():
                interval = None
                continue
            if interval is None:
                interval = (self.now(), self.pose.copy())
            if self.now()-interval[0] >= SETTLE_S:
                drift = math.dist(interval[1][:2], self.pose[:2])
                yaw_drift = abs(wrap(interval[1][5]-self.pose[5]))
                if drift > .001 or yaw_drift > .005:
                    interval = None
                    continue
                dock_world = self.pose.copy()
                dock_odom = self.odom_pose()
                target = row['goal_odom']
                row.update(
                    settled_s=self.now(), stationary_since_s=interval[0],
                    settled_world=dock_world, settled_odom=dock_odom, arbiter_mode=self.mode,
                    settle_drift_m=drift, settle_yaw_drift_rad=yaw_drift,
                    settled_odom_speed=motion(self.odom.twist.twist), settled_guard_speed=self.guard,
                    odom_position_error_m=math.dist(dock_odom[:2], target[:2]),
                    odom_yaw_error_rad=abs(wrap(dock_odom[2]-target[2])),
                    world_position_error_m=math.dist(dock_world[:2], target[:2]),
                    world_yaw_error_rad=abs(wrap(dock_world[5]-target[2])),
                )
                self.monitor_motion = True
                return row['settled_s']
        raise RuntimeError('Base did not sustain a settled state')

    def capture_target(self, after):
        image_time, center = super().capture_target(after)
        self.wait(lambda: any(stamp(frame.header) == image_time for frame in self.rgb), 3,
                  'No raw RGB message matched the localized frame')
        self.selected_rgb = next(frame for frame in self.rgb if stamp(frame.header) == image_time)
        return image_time, center

    def goal(self, *args, **kwargs):
        if not self.stationary() or self.unexpected_motion:
            raise RuntimeError('Base motion or a velocity mode change blocked the next arm/gripper action')
        super().goal(*args, **kwargs)

    def trial(self, report):
        self.ready()
        if any(abs(self.joints[name]) > .02 for name in ARM):
            raise RuntimeError('Arm must be home for navigation')
        self.navigate(report)
        row = report['navigation']
        settled = self.settle(row)
        report['checks']['nav_drove_to_dock'] = math.dist(row['start_world'][:2], self.pose[:2]) > .2
        if not report['checks']['nav_drove_to_dock']:
            raise RuntimeError('Navigation did not physically drive to the dock')
        report['checks']['stationary_before_capture'] = True
        self.grasp(report, after=settled, release=False)
        rgb_stamp = stamp(self.selected_rgb.header)
        report['vision']['after_settle_s'] = rgb_stamp-settled
        report['checks']['fresh_rgb_after_settle'] = rgb_stamp > settled
        row['drift_during_grasp_m'] = math.dist(row['settled_world'][:2], self.pose[:2])
        report['checks']['stationary_through_hold'] = (not self.unexpected_motion and self.stationary()
                                                      and row['drift_during_grasp_m'] <= .002)


def save_rgb(node, report, output):
    settled = report.get('navigation', {}).get('settled_s')
    if settled is None or not node.rgb:
        return
    image = node.selected_rgb or node.rgb[-1]
    image_stamp = stamp(image.header)
    if image_stamp > settled:
        path = output.with_suffix('.png')
        if cv2.imwrite(str(path), CvBridge().imgmsg_to_cv2(image, 'bgr8')):
            report['raw_rgb'] = {'file': str(path), 'stamp_s': image_stamp,
                                 'size': [image.width, image.height]}


if __name__ == '__main__':
    raise SystemExit(main(DockGrasp, __doc__, save_rgb))
