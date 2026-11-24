#!/usr/bin/env python3
"""Validate the M3 demo against live RGB, TF and independent Fortress pose data."""

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_geometry_msgs import do_transform_point
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray, Detection3DArray


COLORS = {'red', 'blue', 'yellow'}


def stamp(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def vector(p):
    return [p.x, p.y, p.z]


def inverse_rotate(q, point):
    x, y, z = vector(q)
    a, b, c = point
    cross = [y*c-z*b, z*a-x*c, x*b-y*a]
    dot = x*a+y*b+z*c
    return [(2*q.w**2-1)*v + 2*dot*axis - 2*q.w*k
            for v, axis, k in zip(point, vector(q), cross)]


def ign_service(endpoint, kind, payload):
    out = subprocess.run(['ign', 'service', '-s', f'/world/flat_world/{endpoint}',
                          '--reqtype', f'ignition.msgs.{kind}', '--reptype', 'ignition.msgs.Boolean',
                          '--timeout', '5000', '--req', payload],
                         capture_output=True, text=True, timeout=12, check=True).stdout
    if 'data: true' not in out:
        raise RuntimeError(f'Gazebo did not confirm {endpoint}: {out}')


def cube_world(color):
    out = subprocess.run(['ign', 'topic', '-e', '-t', f'/model/m3_{color}_cube/pose', '-n', '1'],
                         capture_output=True, text=True, timeout=12, check=True).stdout
    position = re.search(r'position\s*\{([^}]*)\}', out)
    if position is None:
        raise RuntimeError(f'No Fortress pose for {color}')
    values = dict(re.findall(r'([xyz]):\s*([^\s]+)', position[1]))
    return [float(values.get(axis, 0.)) for axis in 'xyz']


class Probe(Node):
    def __init__(self):
        super().__init__('m3_vision_probe')
        self.frames = {}
        self.seen = []
        self.base = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        for kind, topic, key, qos in (
                (Detection2DArray, '/m3/detections_2d', 'pixels', 10),
                (Detection3DArray, '/m3/detections_3d', 'meters', 10),
                (Image, '/m3/debug_image', 'debug', qos_profile_sensor_data),
                (Image, '/camera/image_raw', 'rgb', qos_profile_sensor_data)):
            self.create_subscription(kind, topic, self.collect(key), qos)
        self.create_subscription(TFMessage, '/ground_truth/pose', self.on_base, qos_profile_sensor_data)

    def on_base(self, msg):
        for pose in msg.transforms:
            if pose.child_frame_id == 'mobile_manipulator':
                self.base = pose.transform

    def collect(self, key):
        def receive(msg):
            self.frames.setdefault(stamp(msg.header), {})[key] = msg
            if key == 'meters':
                self.seen.append((stamp(msg.header), {d.id for d in msg.detections}))
        return receive

    def wait(self, predicate, seconds, reason):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.1)
            result = predicate()
            if result:
                return result
        raise RuntimeError(reason)

    def matched(self, colors, after=0, debug=False, count=1):
        keys = {'pixels', 'meters', 'rgb', 'debug'} if debug else {'pixels', 'meters'}
        usable = [(t, v) for t, v in self.frames.items() if t > after and keys <= v.keys()
                  and all({d.id for d in v[k].detections} == colors for k in ('pixels', 'meters'))]
        return max(usable) if len(usable) >= count else None

    def truth_in_arm(self, world, transform):
        delta = [p-q for p, q in zip(world, vector(self.base.translation))]
        point = PointStamped(point=Point(x=0., y=0., z=0.))
        point.point.x, point.point.y, point.point.z = inverse_rotate(self.base.rotation, delta)
        return vector(do_transform_point(point, transform).point)

    def run(self, output, evidence):
        t, frame = self.wait(lambda: self.base and self.matched(COLORS, debug=True, count=3),
                             55, 'Three complete live color detections did not arrive')
        rgb = frame['rgb']
        assert frame['pixels'].header.frame_id == rgb.header.frame_id == 'camera_optical_frame'
        assert frame['meters'].header.frame_id == 'arm_base_link'
        pixel_rows = {row.id: row for row in frame['pixels'].detections}
        metric_rows = {row.id: row for row in frame['meters'].detections}
        transform = self.tf.lookup_transform('arm_base_link', 'base_footprint',
                                             Time.from_msg(rgb.header.stamp))
        results = {}
        margins = []
        for color in sorted(COLORS):
            d2, d3 = pixel_rows[color], metric_rows[color]
            assert stamp(d2.header) == stamp(d3.header) == t
            assert d2.results[0].hypothesis.class_id == d3.results[0].hypothesis.class_id == color
            xy = d2.bbox.center.position
            margins.extend((xy.x-d2.bbox.size_x/2, rgb.width-xy.x-d2.bbox.size_x/2,
                            xy.y-d2.bbox.size_y/2, rgb.height-xy.y-d2.bbox.size_y/2))
            truth = self.truth_in_arm(cube_world(color), transform)
            measured = vector(d3.bbox.center.position)
            results[color] = {'pixel': [xy.x, xy.y], 'position_arm_m': measured,
                              'gazebo_arm_m': truth, 'error_m': math.dist(measured, truth)}
        evidence.update(stamp_ns=t, rgb_size=[rgb.width, rgb.height],
                        output_frame='arm_base_link', results=results, min_image_margin_px=min(margins))
        assert max(row['error_m'] for row in results.values()) < .010, results
        debug = CvBridge().imgmsg_to_cv2(frame['debug'], 'bgr8')
        assert cv2.imwrite(str(output.parent/'debug-three-cubes.png'), debug)
        print('Three colors and metric coordinates:', results, flush=True)

        ign_service('remove', 'Entity', 'name: "m3_yellow_cube", type: MODEL')
        after, _ = self.wait(lambda: self.matched({'red', 'blue'}, after=t),
                             35, 'Yellow detection persisted after the object was removed')
        evidence['disappearance'] = {'stamp_ns': after, 'remaining': ['blue', 'red']}
        print('Yellow disappeared from both outputs.', flush=True)

        ign_service('control', 'WorldControl', 'pause: true')
        try:
            self.wait(lambda: self.seen and self.seen[-1][0] > after and not self.seen[-1][1],
                      14, 'Detections did not expire after camera stopped')
            expiry = self.seen[-1][0]
            before_replay = len(self.seen)
            publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
            self.wait(lambda: publisher.get_subscription_count() >= 2, 8, 'Camera replay subscribers missing')
            stale = deepcopy(rgb)
            stale.header.stamp = Time(nanoseconds=max(1, t-10**9)).to_msg()
            publisher.publish(stale)
            end = time.monotonic()+2
            self.wait(lambda: time.monotonic() >= end, 4, 'Replay observation timed out')
            assert all(not colors for _, colors in self.seen[before_replay:]), 'Old RGB republished valid targets'
            evidence['expiry'] = {'expired_stamp_ns': expiry, 'old_image_rejected': True}
        finally:
            ign_service('control', 'WorldControl', 'pause: false')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID', '0') == '0' or not os.getenv('IGN_PARTITION'):
        parser.error('Use an isolated ROS_DOMAIN_ID and IGN_PARTITION')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as file:
        evidence = {'passed': False}
        rclpy.init()
        node = Probe()
        try:
            node.run(args.output, evidence)
            evidence['passed'] = True
        except (RuntimeError, AssertionError, subprocess.SubprocessError) as exc:
            evidence['error'] = str(exc)
        finally:
            json.dump(evidence, file, indent=2)
            file.write('\n')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(json.dumps(evidence))
    return 0 if evidence['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
