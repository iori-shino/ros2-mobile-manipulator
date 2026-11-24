#!/usr/bin/env python3
"""Select a velocity source and latch manual takeover until explicitly re-enabled."""

import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class VelocityArbiter(Node):
    def __init__(self):
        super().__init__('velocity_arbiter')
        self.navigation_enabled = True
        self.last_manual = -math.inf
        self.cancel_request = None
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode = self.create_publisher(String, '~/mode', qos)
        self.cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_subscription(Twist, '/cmd_vel/nav', self.on_nav, 10)
        self.create_subscription(Twist, '/cmd_vel/manual', self.on_manual, 10)
        self.create_subscription(String, '/teleop/key', self.on_key, 10)
        self.create_service(Trigger, '~/enable_navigation', self.enable)
        self.publish_mode()

    def publish_mode(self):
        self.mode.publish(String(data='navigation' if self.navigation_enabled else 'manual'))

    def on_nav(self, command):
        if self.navigation_enabled:
            self.command.publish(command)

    def on_manual(self, command):
        linear, angular = command.linear.x, command.angular.z
        if not (math.isfinite(linear) and math.isfinite(angular)):
            return
        if linear or angular:
            self.last_manual = time.monotonic()
            self.disengage()
        if not self.navigation_enabled:
            self.command.publish(command)

    def on_key(self, message):
        if message.data in ('stop', ' ', 'q', 'dock_stop'):
            self.disengage()
            self.command.publish(Twist())

    def disengage(self):
        if self.navigation_enabled:
            self.navigation_enabled = False
            self.command.publish(Twist())
            self.publish_mode()
            if self.cancel.service_is_ready():
                self.cancel_request = self.cancel.call_async(CancelGoal.Request())
            self.get_logger().info('Navigation locked. Re-enable explicitly to send a new goal.')

    def enable(self, _request, response):
        if time.monotonic() - self.last_manual < .5:
            response.message = 'Release manual controls before enabling navigation.'
            return response
        if self.cancel_request is not None and not self.cancel_request.done():
            response.message = 'Wait for navigation cancellation.'
            return response
        self.navigation_enabled = True
        self.publish_mode()
        response.success = True
        response.message = 'Navigation enabled. Send a new goal.'
        return response


def main():
    rclpy.init()
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
