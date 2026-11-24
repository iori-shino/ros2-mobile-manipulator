#!/usr/bin/env python3
"""Exercise selected sorter failure branches without sending ROS commands."""

import functools
import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.clock import Clock

from grasp_contact_probe import GraspProbe
from sort_cubes import SortCubes
from room_alignment import fit_room


def refuses(action, message):
    try:
        action()
    except RuntimeError as error:
        assert message in str(error), error
        return
    raise AssertionError(f'Expected failure: {message}')


def main():
    # Wall fitting is bounded by the transform from live SLAM.
    x, y, theta = -.65, -.68, math.pi-.08
    c, s = math.cos(theta), math.sin(theta)
    walls = [(a, q/10) for a in (-2.45, 3.45) for q in range(-24, 25)]
    walls += [(q/10, b) for b in (-2.45, 2.45) for q in range(-24, 35)]
    local = [(c*(a-x)+s*(b-y), -s*(a-x)+c*(b-y)) for a, b in walls]
    pose = fit_room(local, (-.60, -.85, math.pi+.07), (-2.45, 3.45, -2.45, 2.45))
    assert math.dist(pose, (x, y, theta)) < .001

    base = SimpleNamespace(stage=Mock(), resume_navigation=Mock(), backup=Mock(),
                           wait=Mock(), run_base_action=Mock(return_value={'status': GoalStatus.STATUS_ABORTED}),
                           navigate_to=Mock())
    refuses(lambda: SortCubes.deliver(base, {}, {}), 'Loaded retreat failed')
    base.navigate_to.assert_not_called()

    probe = SimpleNamespace(stage=Mock(), tf=Mock(), now=lambda: 4., goal=Mock(),
                            room_pose=lambda: (0., 0., 0.),
                            station_contact=-math.inf, stationary=lambda: True,
                            trace=[{'state': 'none'}], check_motion=Mock())
    transform = TransformStamped()
    transform.transform.translation.z = .101
    transform.transform.rotation.w = 1.
    probe.tf.lookup_transform.return_value = transform

    def expired(condition, *_):
        assert not condition()
        raise RuntimeError('No verified contact on the support')
    probe.wait_sim = expired
    refuses(lambda: SortCubes.place(probe, {}, {'table_world_xy': (.275, .008)}),
            'No verified contact')
    assert [call.args[0] for call in probe.goal.call_args_list] == [
        'place_above', 'place_lower', 'place_release']

    probe = SimpleNamespace(operator_stop=True, contact_problem=None, mode='navigation',
                            monitor_motion=False, stop=Mock(), wait_sim=lambda check, *_: check())
    probe.check_motion = functools.partial(SortCubes.check_motion, probe)
    handle = Mock(accepted=True)
    future = Mock(done=Mock(return_value=True), result=Mock(return_value=handle))
    client = Mock(send_goal_async=Mock(return_value=future))
    probe.wait_future = lambda response, *_: response.result()

    # Arm action uses the original FollowJointTrajectory client and cancel path.
    probe.get_clock = Clock
    probe.finish_trajectory = functools.partial(SortCubes.finish_trajectory, probe)
    probe.trajectory_clients = {'arm': client}
    probe.now = lambda: 4.
    probe.actions = []
    refuses(lambda: GraspProbe.goal(probe, 'test', 'arm', [0.]*4, 4),
            'Operator stopped')
    handle.cancel_goal_async.assert_called_once()
    print(json.dumps({'nav_failure_blocks_delivery': True,
                      'missing_support_blocks_completion': True,
                      'stop_cancels_arm_action': True, 'room_fit_synthetic': True}))


if __name__ == '__main__':
    main()
