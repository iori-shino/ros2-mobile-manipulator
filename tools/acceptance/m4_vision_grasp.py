#!/usr/bin/env python3
"""Execute one camera-guided grasp in the isolated M4 world and record physics."""

import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.time import Time
from tf2_ros import TransformException
from vision_msgs.msg import Detection3DArray

from arm_control import ARM, LIMITS
from grasp_contact_probe import GraspProbe, inverse_rotate, stamp, xyz


# M1 measured cube center in gripper_base, at the center between the contact pads.
TCP = (.011, 0., .03308)
FINGER_END = .045
APPROACH_HEIGHT = .042
CUBE_SIZE = .035
MAX_IMAGE_AGE_S = .5
# Project arm.xacro onto the pitch plane with J2+J3+J4 fixed at pi/2.
UPPER, LOWER = .110, .100
SHOULDER_Z = .060 + .040
LATERAL = .040 - .024 + .022 - .016 - .002 + TCP[0]
TOOL_RADIAL = .075 + .0075 + TCP[1]
TOOL_DROP = .0675 + TCP[2]


def solve_above(center, clearance):
    """Solve the elbow branch used by M1, with the tool pointing downward."""
    x, y, z = center
    radius = math.hypot(x, y)
    if not (math.isfinite(z) and radius > abs(LATERAL)):
        raise RuntimeError('Degenerate TCP position')
    radial = math.sqrt(radius**2 - LATERAL**2)
    yaw = math.atan2(y, x) - math.atan2(LATERAL, radial)
    wrist_x, wrist_z = radial-TOOL_RADIAL, z+clearance+TOOL_DROP-SHOULDER_Z
    c3 = (wrist_x**2 + wrist_z**2 - UPPER**2 - LOWER**2) / (2*UPPER*LOWER)
    if not -1. <= c3 <= 1.:
        raise RuntimeError(f'Target and vertical approach are outside arm reach at z+{clearance:.3f} m')
    elbow = math.acos(c3)
    shoulder = math.atan2(wrist_x, wrist_z) - math.atan2(LOWER*math.sin(elbow),
                                                           UPPER+LOWER*math.cos(elbow))
    wrist = math.pi/2 - shoulder - elbow
    goal = [yaw, shoulder, elbow, wrist]
    if any(abs(angle) >= limit for angle, limit in zip(goal, LIMITS)):
        raise RuntimeError(f'Grasp would exceed a joint limit: {goal}')
    return goal


def checked_approach(center):
    """Validate the complete existing M4 approach and lift before commanding it."""
    x, y, z = center
    if not (.266 <= x <= .289 and abs(y) <= .020 and .032 <= z <= .038):
        raise RuntimeError('Visual detection is outside the permitted M4 work area')
    # +Z_tool points down. Verify the pad ends still clear the support.
    pad_clearance = CUBE_SIZE/2 + TCP[2] - FINGER_END
    if pad_clearance < .004:
        raise RuntimeError('Insufficient fingertip clearance above support')
    for step in range(11):
        solve_above(center, APPROACH_HEIGHT * step/10)
    above, grab = solve_above(center, APPROACH_HEIGHT), solve_above(center, 0.)
    return above, grab, pad_clearance


class VisionGrasp(GraspProbe):
    target_color = 'red'

    def __init__(self):
        super().__init__()
        self.detection = None
        self.create_subscription(Detection3DArray, '/m3/detections_3d', self.on_detection, 5)

    def on_detection(self, msg):
        image_time = stamp(msg.header)
        accepted = [item for item in msg.detections if item.id == self.target_color
                    and item.header.frame_id == 'arm_base_link'
                    and item.header.stamp == msg.header.stamp]
        if (msg.header.frame_id != 'arm_base_link' or len(accepted) != 1
                or not image_time or not 0 <= self.now()-image_time <= MAX_IMAGE_AGE_S):
            self.detection = None
            return
        center = xyz(accepted[0].bbox.center.position)
        self.detection = (image_time, center) if all(map(math.isfinite, center)) else None

    def capture_target(self, after):
        deadline = time.monotonic() + 45
        last = after
        frames = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if self.detection is None or not 0 <= self.now()-self.detection[0] <= MAX_IMAGE_AGE_S:
                frames.clear()
            elif self.detection[0] > last:
                last, center = self.detection
                frames.append(center)
                if len(frames) >= 3:
                    if max(math.dist(center, other) for other in frames[-3:]) <= .003:
                        return last, center
                    frames = frames[-2:]
        raise RuntimeError(f'No fresh, stable {self.target_color} target fully visible after cutoff')

    def reference_center(self):
        """Read independent Gazebo truth for scoring only."""
        if (self.cube is None or self.base is None or self.now()-stamp(self.cube.header) > .3
                or self.cube.header.frame_id != self.base.header.frame_id):
            raise RuntimeError('Fresh test-world cube/base pose is unavailable for verification')
        base = self.base.transform
        delta = [a-b for a, b in zip(xyz(self.cube.transform.translation), xyz(base.translation))]
        in_base = inverse_rotate(base.rotation, delta)
        arm = self.tf.lookup_transform('base_footprint', 'arm_base_link', Time()).transform
        return inverse_rotate(arm.rotation, [a-b for a, b in zip(in_base, xyz(arm.translation))])

    def check_tcp(self, target):
        tool = self.tf.lookup_transform('arm_base_link', 'gripper_base', Time()).transform
        q = tool.rotation
        rotated = inverse_rotate(type(q)(x=-q.x, y=-q.y, z=-q.z, w=q.w), TCP)
        actual = [a+b for a, b in zip(rotated, xyz(tool.translation))]
        return math.dist(actual, target)

    def trial(self, report):
        self.ready()
        first_pose = self.pose.copy()
        self.run_for(.7, publish=False)
        if (math.dist(first_pose[:3], self.pose[:3]) > .001
                or math.dist(first_pose[3:], self.pose[3:]) > .01
                or math.dist(self.pose[:3], [0., 0., -.001]) > .004
                or any(abs(v) > .02 for v in self.pose[3:])
                or any(abs(self.joints[name]) > .02 for name in ARM)):
            raise RuntimeError('Start with the stationary base at the test pose and the arm at home')

        self.grasp(report, after=self.now())

    def grasp(self, report, after, release=True):
        checks = report['checks']
        metrics = report['metrics']
        image_time, center = self.capture_target(after)
        report['vision'] = {'frame': 'arm_base_link', 'stamp_s': image_time, 'center_m': center,
                            'grasp_tcp_offset_m': TCP}
        above, grab, pad_clearance = checked_approach(center)
        metrics['nominal_pad_clearance_m'] = pad_clearance
        truth = self.reference_center()
        report['vision']['gazebo_reference_m'] = truth
        metrics['vision_position_error_m'] = math.dist(center, truth)

        start = self.pause('before_approach', .3)[-1]['cube_xyz_m']
        seated, hold = VisionGrasp.grasp_motion(
            self, report, center, above, grab,
            approach_unchanged=lambda: math.dist(start, self.trace[-1]['cube_xyz_m']) < .002)
        held_height = min(row['cube_xyz_m'][2] for row in hold)
        held = [row['cube_in_gripper_base_m'] for row in hold]
        reference = seated[-1]['cube_in_gripper_base_m']
        if not all(held) or reference is None:
            raise RuntimeError('Missing tool-relative poses for held cube')
        metrics.update(
            cube_lift_m=held_height-start[2],
            hold_drift_m=max(math.dist(held[0], point) for point in held),
            lift_slip_m=max(math.dist(reference, point) for point in held),
            hold_bilateral_fraction=sum(row['state'] == 'both' for row in hold)/len(hold),
        )
        checks.update(
            lifted=metrics['cube_lift_m'] > .035,
            held=metrics['hold_bilateral_fraction'] > .8 and metrics['hold_drift_m'] < .004
            and metrics['lift_slip_m'] < .008,
            support_clear=all(not pairs for pairs in self.other_contacts.values()),
        )
        if release:
            self.goal('release', 'gripper', [.015, .015], 2)
            released = self.pause('after_release', 1.)
            height = released[-1]['cube_xyz_m'][2]
            metrics.update(release_drop_m=held_height-height,
                           release_height_error_m=abs(height-start[2]))
            checks['released'] = (metrics['release_drop_m'] > .035
                                  and metrics['release_height_error_m'] < .003
                                  and all(row['state'] == 'none' for row in released[-5:]))
            checks['support_clear'] = all(not pairs for pairs in self.other_contacts.values())

    def grasp_motion(self, report, center, above, grab, approach_unchanged):
        checks, metrics = report['checks'], report['metrics']
        trace_start = len(self.trace)
        self.goal('open', 'gripper', [.015, .015], 2)
        self.goal('above_cube', 'arm', above, 7)
        self.goal('approach', 'arm', grab, 4)
        metrics['tcp_tracking_error_m'] = self.check_tcp(center)
        checks['open_approach_clear'] = (
            all(row['state'] == 'none' for row in self.trace[trace_start:])
            and all(not pairs for pairs in self.other_contacts.values())
            and approach_unchanged()
            and metrics['tcp_tracking_error_m'] < .003)
        if not checks['open_approach_clear']:
            raise RuntimeError('Open approach hit an object or did not reach the commanded TCP')

        self.goal('close', 'gripper', [.0065, .0065], 2, contact_tolerance=True)
        seated = self.pause('before_lift', .5)
        checks['bilateral_before_lift'] = len(seated) >= 5 and all(row['state'] == 'both' for row in seated[-5:])
        if not checks['bilateral_before_lift']:
            self.goal('release_after_missed_contact', 'gripper', [.015, .015], 2)
            self.goal('retreat', 'arm', above, 4)
            raise RuntimeError('Left and right contacts were not sustained before lift')
        self.goal('lift', 'arm', above, 4)
        hold = self.pause('hold', 2.)
        return seated, hold


def main(probe=VisionGrasp, description=__doc__, finalize=None):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--output', required=True, type=Path, help='New experiment JSON file')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or not os.environ.get('IGN_PARTITION'):
        parser.error('Use a separate ROS_DOMAIN_ID and IGN_PARTITION for this motion trial')

    rclpy.init()
    node = probe()
    report = {'passed': False, 'checks': {}, 'metrics': {}, 'actions': node.actions, 'samples': node.trace}
    try:
        with args.output.open('x') as output:
            try:
                node.trial(report)
                report['passed'] = all(report['checks'].values())
            except (AssertionError, RuntimeError, TimeoutError, TransformException) as exc:
                report['error'] = str(exc)
            finally:
                report['tip_collision_evidence'] = {side: sorted(pairs) for side, pairs in node.other_contacts.items()}
                if finalize:
                    finalize(node, report, args.output)
                json.dump(report, output, indent=2, allow_nan=False)
                output.write('\n')
        print(json.dumps({key: value for key, value in report.items() if key not in ('actions', 'samples')}))
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
