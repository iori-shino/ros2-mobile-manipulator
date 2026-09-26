#!/usr/bin/env python3
"""Relay /cmd_vel to Fortress, stop on stale input, cap course-project speeds."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist


class CommandGuard(Node):
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self.publisher=self.create_publisher(Twist,'/sim/cmd_vel',10)
        self.subscription=self.create_subscription(Twist,'/cmd_vel',self.receive,10)
        self.command=Twist()
        self.last=-math.inf
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self,message):
        if not all(math.isfinite(v) for v in [message.linear.x,message.angular.z]):
            self.last=-math.inf
            return
        self.command=Twist()
        self.command.linear.x=max(-.05,min(.05,message.linear.x))
        self.command.angular.z=max(-.4,min(.4,message.angular.z))
        self.last=time.monotonic()

    def tick(self):
        self.publisher.publish(self.command if time.monotonic()-self.last < .5 else Twist())


def main():
    rclpy.init()
    node=CommandGuard()
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


if __name__=='__main__':
    main()
