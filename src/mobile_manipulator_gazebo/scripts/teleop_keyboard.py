#!/usr/bin/env python3
"""TTY key events with bounded motion, immediate space stop and terminal cleanup."""
import select
import sys
import termios
import time
import tty
import rclpy
from std_msgs.msg import String


def main():
    if not sys.stdin.isatty():
        raise SystemExit('请在交互终端运行（SSH 请使用 ssh -t）。')
    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(String, '/teleop/key', 10)
    saved = termios.tcgetattr(sys.stdin)
    print('W/S 前进/后退，A/D 左转/右转；按住连续运动。空格停止；Q/Ctrl-C 退出。\n'
          '松键后最多 0.30 秒停止；首次长按可能因系统重复延迟短暂停顿。手柄 L 按住时优先。', flush=True)
    try:
        # DDS discovery may exceed one second in a loaded VM. Remain in
        # normal terminal mode until a subscriber is discovered, with a bound.
        end = time.monotonic()+5
        while not pub.get_subscription_count() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if not pub.get_subscription_count():
            raise RuntimeError('teleop_router 未启动，请先启动 teleop.launch.py')
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            if select.select([sys.stdin], [], [], .05)[0]:
                key = sys.stdin.read(1).lower()
                if not key or key in ('q', '\x03', '\x04'):
                    break
                if key in 'wasd ':
                    pub.publish(String(data=key))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        for _ in range(3):
            if rclpy.ok():
                pub.publish(String(data='stop'))
                rclpy.spin_once(node, timeout_sec=.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
