#!/usr/bin/env python3
"""Observe three colors, dynamic cube settling and LiDAR returns in the M5 demo."""

import argparse
import json
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from m3_vision_probe import COLORS, Probe, cube_world, inverse_rotate, stamp, vector
from m4_vision_grasp import APPROACH_HEIGHT, solve_above


def rotate(q, point):
    return inverse_rotate(type(q)(x=-q.x, y=-q.y, z=-q.z, w=q.w), point)


class SceneProbe(Probe):
    def __init__(self):
        super().__init__()
        self.scan = None
        self.last_hits = {}
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.scan = msg

    def obstacle_hits(self, obstacles):
        if self.scan is None or self.base is None:
            return None
        scan = self.scan
        if not self.tf.can_transform('base_footprint', scan.header.frame_id, Time()):
            return None
        beam = self.tf.lookup_transform('base_footprint', scan.header.frame_id, Time()).transform
        hits = {entry['model']: 0 for entry in obstacles}
        for i, distance in enumerate(scan.ranges):
            if not scan.range_min <= distance <= scan.range_max:
                continue
            angle = scan.angle_min+i*scan.angle_increment
            local = rotate(beam.rotation, (distance*math.cos(angle), distance*math.sin(angle), 0.))
            local = [a+b for a,b in zip(local, vector(beam.translation))]
            world = [a+b for a,b in zip(rotate(self.base.rotation, local), vector(self.base.translation))]
            for entry in obstacles:
                if all(abs(a-b) <= size/2+.015 for a,b,size in
                       zip(world, entry['center_world_m'], entry['size_m'])):
                    hits[entry['model']] += 1
        self.last_hits = hits
        return hits if all(hits.values()) else None

    def inspect(self, manifest, output, evidence):
        self.wait(lambda: self.base is not None and abs(self.base.translation.z+.001) < .001,
                  140, 'Robot did not settle at the observation pose')
        cutoff = max(self.frames, default=0)
        t, frame = self.wait(lambda: self.matched(COLORS, after=cutoff, debug=True, count=3),
                             120, 'No repeated same-frame detections of all three colors')
        transform = self.tf.lookup_transform('arm_base_link', 'base_footprint',
                                             Time.from_msg(frame['rgb'].header.stamp))
        pixels = {item.id: item for item in frame['pixels'].detections}
        results = {}
        margins = []
        for item in frame['meters'].detections:
            color = item.id
            row = manifest['cubes'][color]
            spawn = row['spawn_world_m']
            center = cube_world(color)
            truth = self.truth_in_arm(center, transform)
            detection = vector(item.bbox.center.position)
            pixel = pixels[color].bbox
            u, v = pixel.center.position.x, pixel.center.position.y
            margins.extend((u-pixel.size_x/2, frame['rgb'].width-u-pixel.size_x/2,
                            v-pixel.size_y/2, frame['rgb'].height-v-pixel.size_y/2))
            assert stamp(item.header) == stamp(pixels[color].header) == t
            assert item.header.frame_id == 'arm_base_link'
            assert math.dist(center[:2], spawn[:2]) < .003
            assert .0005 < spawn[2]-center[2] < .0025
            assert abs(center[2]-manifest['table_top_m']-row['size_m']/2) < .002
            error = math.dist(truth, detection)
            assert error < .025, (color, truth, detection)
            dock = row['pickup_dock_hint_world']
            nominal = [center[0]-dock[0], center[1]-dock[1], truth[2]]
            solve_above(nominal, 0.)
            solve_above(nominal, APPROACH_HEIGHT)
            results[color] = {'pixel': [u,v], 'position_arm_m': detection,
                              'error_m': error, 'spawn_world_m': spawn,
                              'gazebo_rest_world_m': center, 'nominal_pickup_ik': True}
        path = output.with_suffix('.png')
        debug = CvBridge().imgmsg_to_cv2(frame['debug'], 'bgr8')
        assert cv2.imwrite(str(path), debug)
        evidence.update(stamp_ns=t, camera_size=[frame['rgb'].width, frame['rgb'].height],
                        min_image_margin_px=min(margins), detections=results, debug=str(path))
        try:
            self.wait(lambda: self.obstacle_hits(manifest['obstacles']), 50,
                      'Expected a LiDAR return from each configured obstacle')
        finally:
            evidence['lidar_hits_per_obstacle'] = self.last_hits
            if self.scan is not None:
                evidence['scan_frame'] = self.scan.header.frame_id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    rclpy.init()
    node = SceneProbe()
    evidence = {'seed': manifest['seed'], 'passed': False}
    with args.output.open('x') as output:
        try:
            node.inspect(manifest, args.output, evidence)
            evidence['passed'] = True
        except (AssertionError, RuntimeError, TimeoutError) as exc:
            evidence['error'] = str(exc)
        finally:
            json.dump(evidence, output, indent=2, allow_nan=False)
            output.write('\n')
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(evidence), flush=True)
    return 0 if evidence['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
