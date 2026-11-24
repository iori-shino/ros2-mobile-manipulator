#!/usr/bin/env python3
"""Inject faults into the M5 gates, optionally during a live Gazebo dock or grasp."""

import argparse
import functools
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from rclpy.clock import Clock
from std_msgs.msg import Header, String
from vision_msgs.msg import Detection3D, Detection3DArray

import m4_vision_grasp as vision
from arm_control import ARM
from m42_dock_grasp import DockGrasp
from m43_carry_place import CarryPlace
from m52_random_pick import RandomPick


def blocked(callback, expected):
    try:
        callback()
    except RuntimeError as error:
        if expected not in str(error):
            raise
        return str(error)
    raise RuntimeError(f'Expected refusal: {expected}')


def simulated_faults():
    checks = {}
    probe = SimpleNamespace(capture_target=lambda _: (10., (.4, .008, .035)),
                            goal=Mock(), reference_center=Mock())
    report = {'checks': {}, 'metrics': {}}
    checks['unreachable'] = blocked(
        lambda: vision.VisionGrasp.grasp(probe, report, 9.), 'outside the permitted M4 work area')
    probe.goal.assert_not_called()
    probe.reference_center.assert_not_called()

    probe = SimpleNamespace(now=lambda: 10., target_color='yellow')
    header = Header(frame_id='arm_base_link', stamp=Time(sec=9, nanosec=750_000_000))
    item = Detection3D(id='yellow', header=header)
    item.bbox.center.position = Point(x=.275, y=.008, z=.035)
    message = Detection3DArray(header=header, detections=[item])
    vision.VisionGrasp.on_detection(probe, message)
    assert probe.detection is not None
    # A previously accepted message must also expire while it sits in the cache.
    probe.now = lambda: 11.
    expired = iter((9.70, 9.75, 9.80))

    def inbox(*args, **kwargs):
        probe.detection = (next(expired), (.275, .008, .035))

    with patch.object(vision.rclpy, 'spin_once', side_effect=inbox), patch.object(
            vision.time, 'monotonic', side_effect=(0., 1., 2., 3., 46.)):
        checks['stale_cached_frame'] = blocked(
            lambda: vision.VisionGrasp.capture_target(probe, 9.), 'No fresh, stable yellow target')
    vision.VisionGrasp.on_detection(probe, message)
    assert probe.detection is None
    checks['stale_on_receipt'] = 'The same RGB timestamp was refused after expiration'
    checks['stale_after_synchronization'] = blocked(
        lambda: RandomPick.require_fresh_image(probe, 9.8), 'Pickup image expired')

    probe = SimpleNamespace(
        trajectory_clients={}, pose=[0.] * 6, joint_samples=[1], now=lambda: 1.,
        joints=dict.fromkeys(ARM, 0.), wait=lambda condition, *_: condition(),
        activate_nav=Mock(), get_logger=Mock(), plan_dock=lambda _: (.19, 0., 0.),
        odom_pose=lambda: (0., 0., 0.), get_clock=Clock, nav=object(), mode='navigation',
        get_publishers_info_by_topic=lambda _: [], settle=Mock(), grasp=Mock(),
        run_base_action=lambda *_: {'status': GoalStatus.STATUS_ABORTED},
    )
    probe.navigate_to = functools.partial(DockGrasp.navigate_to, probe)
    probe.pickup_pose = probe.odom_pose
    probe.pickup_dock = functools.partial(RandomPick.pickup_dock, probe)
    checks['navigation_aborted'] = blocked(
        lambda: RandomPick.trial(probe, {}), 'Navigation failed or was interrupted: status 6')
    probe.settle.assert_not_called()
    probe.grasp.assert_not_called()

    probe = SimpleNamespace(local_motion=True, mode='manual',
                            wait_sim=lambda ready, *_: ready())
    handle = Mock()
    handle.get_result_async.return_value.done.return_value = False
    checks['local_dock_stopped'] = blocked(
        lambda: RandomPick.base_result(probe, handle), 'Manual stop interrupted local docking')

    center = (.275, .008, .035)
    sample = {'state': 'none', 'cube_xyz_m': center}
    probe = SimpleNamespace(
        capture_target=lambda _: (10., center), reference_center=lambda: center,
        pause=lambda *_: [sample] * 5, goal=Mock(), check_tcp=lambda _: 0.,
        trace=[sample], other_contacts={},
    )
    report = {'checks': {}, 'metrics': {}}
    checks['missing_bilateral_contact'] = blocked(
        lambda: vision.VisionGrasp.grasp(probe, report, 9.), 'not sustained before lift')
    checks['missed_grasp_actions'] = [call.args[0] for call in probe.goal.call_args_list]
    assert checks['missed_grasp_actions'][-2:] == ['release_after_missed_contact', 'retreat']
    assert 'lift' not in checks['missed_grasp_actions']
    assert not report['checks']['bilateral_before_lift']

    carry = object.__new__(CarryPlace)
    carry.goal = Mock()
    carry.drive_loaded = Mock()
    carry.place = Mock()
    report = {'checks': {'lifted': False}}
    with patch.object(DockGrasp, 'trial'):
        checks['failed_pickup_blocks_transport'] = blocked(
            lambda: carry.pickup_and_place(report), 'pickup did not satisfy its checks')
    carry.goal.assert_not_called()
    carry.drive_loaded.assert_not_called()
    carry.place.assert_not_called()
    return {'passed': True, 'method': 'injected inputs into the original code', 'checks': checks}


class StopProbe(RandomPick):
    """Trigger a Nav2 abort, then Stop a deliberate new dock while it is moving."""

    bad_tree = Path('/m53-deliberately-missing-behavior-tree.xml')

    def run_base_action(self, client, goal, name):
        if self.inject_failure and isinstance(goal, NavigateToPose.Goal):
            assert not self.bad_tree.exists()
            goal.behavior_tree = str(self.bad_tree)
        return super().run_base_action(client, goal, name)

    def base_result(self, handle):
        if self.inject_failure:
            return super().base_result(handle)
        self.wait(lambda: math.dist(self.odom_pose()[:2], self.start_odom[:2]) >= .04,
                  90, 'Base did not reach the operator Stop trigger')
        assert self.mode == 'navigation'
        self.stop.publish(String(data='stop'))
        return super().base_result(handle)

    def navigate_to(self, row, target):
        if self.inject_failure:
            return super().navigate_to(row, target)
        self.start_odom = self.odom_pose()
        super().navigate_to(row, target)

    def trial(self, report):
        self.inject_failure = True
        report['navigation_failure'] = blocked(lambda: super(StopProbe, self).trial(report),
                                               'Navigation failed or was interrupted: status 6')
        report['aborted_navigation'] = report.pop('navigation')
        report['checks']['navigation_abort_no_arm'] = not self.actions
        self.inject_failure = False
        # Start a deliberate new attempt on the same active Nav2 instance.
        with patch.object(self, 'activate_nav'):
            report['interruption'] = blocked(lambda: super(StopProbe, self).trial(report),
                                             'Navigation failed or was interrupted: status 5')
        self.wait(self.stationary, 15, 'Base did not stop after operator Stop')
        start, start_s = self.pose.copy(), self.now()
        self.monitor_motion = True
        self.wait_sim(lambda: self.now()-start_s >= 2., 3., 'Watch for old navigation resumption')
        report['post_stop_drift_m'] = math.dist(start[:2], self.pose[:2])
        report['checks'].update(
            active_goal_cancelled=report['navigation']['status'] == GoalStatus.STATUS_CANCELED,
            old_task_did_not_resume=self.stationary() and not self.unexpected_motion
            and report['post_stop_drift_m'] < .002,
            no_arm_or_gripper_goals=not self.actions,
        )


class MissedContactProbe(RandomPick):
    def contact_callback(self, side):
        if side == 'right':
            return lambda _message: None
        return super().contact_callback(side)

    def trial(self, report):
        report['injection'] = 'Suppress the right fingertip contact callback'
        report['blocked'] = blocked(lambda: super(MissedContactProbe, self).trial(report),
                                    'not sustained before lift')
        phases = [action['phase'] for action in self.actions]
        report['checks']['missed_contact_retreats_without_lift'] = (
            phases[-2:] == ['release_after_missed_contact', 'retreat'] and 'lift' not in phases)
        report['checks']['missing_contact_detected'] = report['checks'].pop('bilateral_before_lift') is False


if __name__ == '__main__':
    if os.getenv('M53_LIVE_STOP') == '1':
        raise SystemExit(vision.main(functools.partial(StopProbe, 'red'), __doc__))
    if os.getenv('M53_LIVE_MISSED_CONTACT') == '1':
        raise SystemExit(vision.main(functools.partial(MissedContactProbe, 'red'), __doc__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = simulated_faults()
    with args.output.open('x') as output:
        json.dump(result, output, indent=2, ensure_ascii=False)
        output.write('\n')
    print(json.dumps(result, ensure_ascii=False))
