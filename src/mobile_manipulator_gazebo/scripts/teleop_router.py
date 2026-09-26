#!/usr/bin/env python3
"""One teleop publisher into the existing /cmd_vel -> guard -> simulation chain."""
import json
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from teleop_core import TeleopCore


class Router(Node):
    def __init__(self):
        super().__init__('teleop_router')
        path = self.declare_parameter('mapping_file', '').value
        if path:
            with open(path) as file:
                self.core = TeleopCore(yaml.safe_load(file))
        else:
            self.core = TeleopCore()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/teleop/status', 10)
        if self.core.joy_enabled:
            self.create_subscription(Joy, '/joy', self.joy, qos_profile_sensor_data)
        self.create_subscription(String, '/teleop/key', self.key, 10)
        self.active = False
        self.ticks = 0
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def joy(self, msg):
        self.core.joy(msg.axes, msg.buttons, time.monotonic())
        self.tick()

    def key(self, msg):
        self.core.key(msg.data, time.monotonic())
        if msg.data in (' ', 'q', 'stop'):
            self.publisher.publish(Twist())
        self.tick()

    def tick(self):
        now = time.monotonic()
        (x, z), source = self.core.output(now)
        # Idle joystick must not flood zeros over the existing arm/base demo tools.
        if source != 'idle' or self.active:
            command = Twist()
            command.linear.x, command.angular.z = x, z
            self.publisher.publish(command)
        self.active = source != 'idle'
        self.ticks += 1
        if self.ticks % 10 == 0:
            self.status.publish(String(data=json.dumps({'source': source, 'armed': self.core.armed,
                'joy_fresh': now-self.core.last_joy < self.core.c['joy_timeout'], 'linear': x, 'angular': z})))


def main():
    rclpy.init()
    node = Router()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
