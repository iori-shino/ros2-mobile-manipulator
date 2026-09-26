#!/usr/bin/env python3
"""A short, bounded /cmd_vel command for user acceptance."""
import argparse
import time
import rclpy
from geometry_msgs.msg import Twist


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('direction',choices=['forward','backward','left','right','stop'])
    parser.add_argument('--duration',type=float,default=3.)
    args=parser.parse_args()
    if not 0 < args.duration <= 30:
        parser.error('duration must be between 0 and 30 seconds')
    rclpy.init()
    node=rclpy.create_node('base_motion_demo')
    publisher=node.create_publisher(Twist,'/cmd_vel',10)
    twist=Twist()
    twist.linear.x={'forward':.04,'backward':-.04}.get(args.direction,0.)
    twist.angular.z={'left':.3,'right':-.3}.get(args.direction,0.)
    try:
        # Allow ROS discovery while sending zero.
        end=time.monotonic()+1
        while time.monotonic()<end:
            publisher.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        end=time.monotonic()+args.duration
        while time.monotonic()<end:
            publisher.publish(twist);rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            if rclpy.ok():
                publisher.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
