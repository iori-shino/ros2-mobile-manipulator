#!/usr/bin/env python3
"""Observe fingertip contact, or replay a fixed dynamic-cube trial in the test world."""

import argparse
import json
import math
import os
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_control import ARM, JAWS, ArmClient


SIDES = ('left', 'right')
TARGET = ('grasp_cube', 'body', 'collision')
CONTACT_WINDOW = 0.08  # simulation seconds
PREGRASP = [0., .759026, .753995, .057775]
GRASP = [0., .85, 1.10, -.379204]
CUBE_REST = [.258, .031, .135]


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def scoped(name, suffix):
    return tuple(name.split('::')[-len(suffix):]) == suffix


def inverse_rotate(q, point):
    """Rotate a vector into the frame whose orientation is q."""
    cross = [q.y * point[2] - q.z * point[1],
             q.z * point[0] - q.x * point[2],
             q.x * point[1] - q.y * point[0]]
    twice_dot = 2 * sum(a * b for a, b in zip(xyz(q), point))
    return [(2 * q.w**2 - 1) * v + twice_dot * axis - 2 * q.w * c
            for v, axis, c in zip(point, xyz(q), cross)]


class GraspProbe(ArmClient):
    target_collision = TARGET

    def __init__(self):
        super().__init__()
        self.seen = {}
        self.matches = {side: 0 for side in SIDES}
        self.other_contacts = {side: set() for side in SIDES}
        self.cube = None
        self.base = None
        self.phase = 'idle'
        self.current_state = None
        self.trace = []
        self.actions = []
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.contact_state = self.create_publisher(String, '/grasp/contact_state', 10)
        self.create_subscription(TFMessage, '/grasp/cube_pose', self.on_cube, 10)
        for side in SIDES:
            self.create_subscription(Contacts, f'/grasp/contact/{side}',
                                     self.contact_callback(side), qos_profile_sensor_data)
        self.create_timer(.05, self.sample)

    def on_joints(self, msg):
        self.joint_samples.clear()
        super().on_joints(msg)

    def on_pose(self, msg):
        self.samples.clear()
        super().on_pose(msg)
        for transform in msg.transforms:
            if scoped(transform.child_frame_id, ('mobile_manipulator',)):
                self.base = transform

    def on_cube(self, msg):
        for transform in msg.transforms:
            if scoped(transform.child_frame_id, self.target_collision[:1]):
                self.cube = transform

    def contact_callback(self, side):
        def receive(msg):
            tip = (f'{side}_jaw', f'{side}_jaw_collision_1')
            for pair in msg.contacts:
                first, second = pair.collision1.name, pair.collision2.name
                if ((scoped(first, tip) and scoped(second, self.target_collision)) or
                        (scoped(second, tip) and scoped(first, self.target_collision))):
                    self.seen[side] = stamp(msg.header)
                    self.matches[side] += 1
                elif scoped(first, tip) or scoped(second, tip):
                    other = second if scoped(first, tip) else first
                    self.other_contacts[side].add((self.phase, other))
        return receive

    def cube_in_tool_frame(self):
        if self.base is None or self.base.header.frame_id != self.cube.header.frame_id:
            return None
        try:
            tool = self.tf.lookup_transform('base_footprint', 'gripper_base', Time()).transform
        except TransformException:
            return None
        base = self.base.transform
        delta = [a - b for a, b in zip(xyz(self.cube.transform.translation), xyz(base.translation))]
        cube_local = inverse_rotate(base.rotation, delta)
        delta = [a - b for a, b in zip(cube_local, xyz(tool.translation))]
        return inverse_rotate(tool.rotation, delta)

    def sample(self):
        now = self.now()
        active = {side: now - self.seen.get(side, -math.inf) < CONTACT_WINDOW for side in SIDES}
        state = ('both' if all(active.values()) else
                 next((side for side in SIDES if active[side]), 'none'))
        if state != self.current_state:
            self.current_state = state
            self.get_logger().info(f'{self.phase}: {state}')
        self.contact_state.publish(String(data=state))
        if self.cube is None or self.phase == 'idle':
            return
        cube = self.cube.transform
        self.trace.append({
            'phase': self.phase, 'time_s': now, 'contact': active, 'state': state,
            'matches': self.matches.copy(), 'last_contact_s': self.seen.copy(),
            'joints': {name: self.joints.get(name) for name in ARM + JAWS},
            'cube_stamp_s': stamp(self.cube.header), 'world_frame': self.cube.header.frame_id,
            'cube_xyz_m': xyz(cube.translation), 'cube_xyzw': [*xyz(cube.rotation), cube.rotation.w],
            'cube_in_gripper_base_m': self.cube_in_tool_frame(),
        })

    def finish_trajectory(self, handle, duration):
        return self.wait_future(handle.get_result_async(), duration * 15 + 30)

    def goal(self, phase, group, positions, duration, contact_tolerance=False):
        self.phase = phase
        joints = ARM if group == 'arm' else JAWS
        client = self.trajectory_clients[group]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [JointTrajectoryPoint(positions=positions, velocities=[0.] * len(joints))]
        goal.trajectory.points[0].time_from_start.sec = duration
        if contact_tolerance:
            # A held object blocks the finger just short of the commanded setpoint.
            goal.goal_tolerance = [JointTolerance(name=name, position=.0018) for name in joints]
        row = {'phase': phase, 'group': group, 'target': dict(zip(joints, positions)),
               'start_s': self.now()}
        self.actions.append(row)
        handle = self.wait_future(client.send_goal_async(goal), 12)
        row['accepted'] = handle.accepted
        if not handle.accepted:
            raise RuntimeError(f'{phase}: action rejected')
        try:
            result = self.finish_trajectory(handle, duration)
        except Exception:
            self.wait_future(handle.cancel_goal_async(), 5)
            raise
        row.update(status=result.status, error_code=result.result.error_code,
                   error_string=result.result.error_string, end_s=self.now())
        known_contact_result = (contact_tolerance and result.status == GoalStatus.STATUS_ABORTED
                                and result.result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
        if result.status != GoalStatus.STATUS_SUCCEEDED and not known_contact_result:
            raise RuntimeError(f'{phase}: trajectory ended with {row}')
        self.run_for(.3, publish=False)

    def pause(self, phase, duration):
        self.phase = phase
        self.run_for(duration, publish=False)
        return [sample for sample in self.trace if sample['phase'] == phase]

    def verify_start(self):
        self.ready()
        self.run_for(.8, publish=False)
        if (self.cube is None or self.base is None or self.cube_in_tool_frame() is None
                or self.now() - stamp(self.cube.header) > .3):
            raise RuntimeError('Missing fresh cube, robot, or tool pose in the test world')
        if math.dist(xyz(self.cube.transform.translation), CUBE_REST) > .003:
            raise RuntimeError('Start a fresh grasp test world with the cube centered on its support')
        if math.dist(self.pose[:3], [0., 0., -.001]) > .004 or any(abs(v) > .02 for v in self.pose[3:]):
            raise RuntimeError('The base has moved from the test start pose')
        if any(abs(self.joints[name]) > .02 for name in ARM):
            raise RuntimeError('The arm must start at its home pose')

    def trial(self, mode, close):
        self.verify_start()
        clear = self.pause('before_approach', .3)
        start = clear[-1]['cube_xyz_m']
        checks = {'initial_no_contact': all(row['state'] == 'none' for row in clear)}
        self.goal('open', 'gripper', [.015, .015], 2)
        self.goal('above_cube', 'arm', PREGRASP, 7)
        self.goal('approach', 'arm', GRASP, 4)
        checks['open_approach_clear'] = (
            all(row['state'] == 'none' for row in self.trace)
            and math.dist(start, self.trace[-1]['cube_xyz_m']) < .002)
        if not all(checks.values()):
            raise RuntimeError('Unexpected contact or cube movement before closing')

        if mode == 'sides':
            for index, side in enumerate(SIDES):
                target = [.015, .015]
                target[index] = .0068
                self.goal(f'{side}_only', 'gripper', target, 2, contact_tolerance=True)
                states = {row['state'] for row in self.trace if row['phase'] == f'{side}_only'}
                checks[f'observed_{side}_only'] = side in states and states <= {'none', side}
                self.goal(f'open_after_{side}', 'gripper', [.015, .015], 2)
            checks['separation'] = self.trace[-1]['state'] == 'none'
            return checks, {}

        self.goal('close', 'gripper', [close, close], 2, contact_tolerance=True)
        seated = self.pause('before_lift', .5)
        checks['bilateral_before_lift'] = any(row['state'] == 'both' for row in seated)
        if not checks['bilateral_before_lift']:
            raise RuntimeError('The two fingertips did not maintain bilateral contact before lifting')
        self.goal('lift', 'arm', PREGRASP, 4)
        hold = self.pause('hold', 2.)
        self.goal('release', 'gripper', [.015, .015], 2)
        released = self.pause('after_release', 1.)
        before = seated[-1]
        after = released[-1]
        held_height = min(row['cube_xyz_m'][2] for row in hold)
        relative = [row['cube_in_gripper_base_m'] for row in hold]
        reference = before['cube_in_gripper_base_m']
        if not all(relative) or reference is None:
            raise RuntimeError('Missing tool-relative cube pose during the hold')
        hold_drift = max(math.dist(relative[0], value) for value in relative)
        lift_slip = max(math.dist(reference, value) for value in relative)
        metrics = {
            'cube_lift_m': held_height - start[2],
            'hold_relative_drift_m': hold_drift,
            'lift_and_hold_relative_change_m': lift_slip,
            'hold_bilateral_fraction': sum(row['state'] == 'both' for row in hold) / len(hold),
            'release_drop_m': held_height - after['cube_xyz_m'][2],
            'released_cube_xyz_m': after['cube_xyz_m'],
        }
        checks.update(
            lifted=metrics['cube_lift_m'] > .025,
            held=hold_drift < .004 and lift_slip < .008 and metrics['hold_bilateral_fraction'] > .8,
            fell_after_opening=metrics['release_drop_m'] > .025,
            contact_cleared=all(row['state'] == 'none' for row in released[-5:]),
        )
        return checks, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('observe', 'sides', 'grasp'), default='observe')
    parser.add_argument('--close', type=float, default=.0065, help='grasp target per finger, in meters')
    parser.add_argument('--output', type=Path, help='new JSON file for a motion trial')
    args = parser.parse_args()
    if args.mode != 'observe' and (
            not args.output or os.getenv('ROS_DOMAIN_ID', '0') == '0' or not os.getenv('IGN_PARTITION')):
        parser.error('motion trials require --output, a separate ROS_DOMAIN_ID and IGN_PARTITION')
    if not .006 <= args.close < .0075:
        parser.error('--close must be between .006 and .0075 m (upper bound excluded)')
    rclpy.init()
    node = GraspProbe()
    try:
        if args.mode == 'observe':
            rclpy.spin(node)
            return 0
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as output:
            report = {'mode': args.mode, 'passed': False, 'close_target_m': args.close,
                      'actions': node.actions, 'samples': node.trace}
            try:
                report['checks'], report['metrics'] = node.trial(args.mode, args.close)
                report['passed'] = all(report['checks'].values())
            except (RuntimeError, AssertionError, TimeoutError) as exc:
                report['error'] = str(exc)
            finally:
                report['other_fingertip_contacts'] = {side: sorted(pairs) for side, pairs in node.other_contacts.items()}
                json.dump(report, output, indent=2, allow_nan=False)
                output.write('\n')
            print(json.dumps({k: v for k, v in report.items() if k not in ('actions', 'samples')}),
                  flush=True)
            return 0 if report['passed'] else 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
