#!/usr/bin/env python3
"""Navigate from an overview detection and grasp one selected M5 cube."""

import functools
import math
import os
import time

from geometry_msgs.msg import Point, PointStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point

from acceptance_test import angles
from arm_control import ARM
from grasp_contact_probe import stamp, xyz
from m4_vision_grasp import MAX_IMAGE_AGE_S, checked_approach, main
from m42_dock_grasp import DockGrasp, WORKCELLS, save_rgb


def preferred_reach(y, z):
    # Map the admissible X range with the same IK and limits used for execution.
    feasible = []
    # The front camera can crop the cube top at the innermost IK-valid
    # positions. Keep the nominal grasp center above the 270 mm view edge.
    for mm in range(272, 290):
        x = mm / 1000
        try:
            checked_approach((x, y, z))
        except RuntimeError:
            continue
        feasible.append(x)
    if not feasible:
        raise RuntimeError('No reachable pickup depth at this target height and lateral offset')
    x = (min(feasible)+max(feasible))/2
    checked_approach((x, y, z))
    return x


def table_clear(dock):
    scene = WORKCELLS['m5']
    front = scene['pickup_table_world_xy'][0]-scene['pickup_table_size_xy'][0]/2
    # Nav2's configured footprint has front x=0.200, side y=0.190 and 15 mm padding.
    theta = dock[2]
    nose = .200*abs(math.cos(theta))+.190*abs(math.sin(theta))+.015
    return dock[0]+nose+.003 < front


def side_approach(center, origin):
    """Choose the nearest reachable stop that preserves the table clearance."""
    x, y, yaw = origin
    nominal = center[0]-preferred_reach(center[1], center[2])
    c, s = math.cos(yaw), math.sin(yaw)
    for mm in range(7):
        distance = nominal-mm/1000
        target = (x+distance*c, y+distance*s, yaw)
        if not (.035 <= distance <= .16 and table_clear(target)):
            continue
        try:
            checked_approach((center[0]-distance, center[1], center[2]))
        except RuntimeError:
            continue
        return distance, target
    raise RuntimeError('No side approach satisfies both table clearance and full grasp reach')


class RandomPick(DockGrasp):
    def __init__(self, color):
        self.target_color = color
        self.local_motion = False
        self.target_collision = (f'm3_{color}_cube', 'body', 'collision')
        super().__init__()

    def activate_nav(self):
        manager = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        clients = {name: self.create_client(GetState, f'/{name}/get_state')
                   for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')}
        self.wait(lambda: manager.service_is_ready() and all(x.service_is_ready() for x in clients.values()),
                  150, 'Nav2 lifecycle services did not become ready')
        for name, client in clients.items():
            for _ in range(3):
                response = client.call_async(GetState.Request())
                try:
                    state = self.wait_future(response, 12).current_state.label
                    break
                except TimeoutError:
                    client.remove_pending_request(response)
            else:
                raise RuntimeError(f'{name} lifecycle service did not reply')
            if state != 'unconfigured':
                raise RuntimeError(f'Expected fresh Nav2 component: {name} is {state}')
        request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
        if not self.wait_future(manager.call_async(request), 180).success:
            raise RuntimeError('Nav2 lifecycle manager failed to activate nodes')
        self.wait(lambda: self.nav.server_is_ready() and self.mode == 'navigation'
                  and self.tf.can_transform('map', 'odom', Time()), 35, 'Nav2, arbiter or SLAM TF unavailable')
        if {info.node_name for info in self.get_publishers_info_by_topic('/cmd_vel')} != {'velocity_arbiter'}:
            raise RuntimeError('Guard must receive commands only from the velocity arbiter')

    def wait_sim(self, finished, seconds, phase):
        initial = previous = self.now()
        updated = time.monotonic()
        while not finished():
            rclpy.spin_once(self, timeout_sec=.05)
            now = self.now()
            if now < previous:
                raise RuntimeError(f'Simulation time reset during {phase}')
            if now > previous:
                previous, updated = now, time.monotonic()
            if now-initial > seconds or time.monotonic()-updated > 20.:
                raise TimeoutError(f'{phase} exceeded its simulation deadline or clock stopped')

    def run_for(self, seconds, *, publish):
        if publish:
            raise RuntimeError('The base may only receive motion from Nav2')
        end = self.now()+seconds
        self.wait_sim(lambda: self.now() >= end, seconds+1., self.phase)
        if self.pose is None or time.monotonic()-self.pose_stamp >= 3.:
            raise RuntimeError('Gazebo pose feedback unavailable')
        return self.pose.copy()

    def finish_trajectory(self, handle, duration):
        future = handle.get_result_async()
        self.wait_sim(future.done, duration+5., 'Grasp trajectory')
        return future.result()

    def run_base_action(self, client, goal, name):
        if isinstance(goal, NavigateToPose.Goal):
            # Freeze the waypoint in the planner's map frame. Replanning later
            # must not require a historic map-to-odom transform for this goal.
            transform = self.pickup_map_from_odom
            pose = goal.pose.pose
            goal.pose.pose.position = do_transform_point(
                PointStamped(point=pose.position), transform).point
            heading = angles(pose.orientation)[2]+angles(transform.transform.rotation)[2]
            pose.orientation.z, pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = transform.header.stamp
        result = super().run_base_action(client, goal, name)
        if isinstance(goal, NavigateToPose.Goal):
            result['goal_map'] = [goal.pose.pose.position.x, goal.pose.pose.position.y, heading]
            result['goal_map_stamp_s'] = stamp(goal.pose.header)
        return result

    def base_result(self, handle):
        future = handle.get_result_async()
        self.wait_sim(lambda: future.done() or (self.local_motion and self.mode != 'navigation'),
                      35., 'Pickup navigation')
        if self.local_motion and self.mode != 'navigation':
            raise RuntimeError('Manual stop interrupted local docking')
        return future.result()

    def require_fresh_image(self, image_s):
        if not 0 <= self.now()-image_s <= MAX_IMAGE_AGE_S:
            raise RuntimeError('Pickup image expired while waiting for synchronized RGB or TF')

    def capture_target(self, after):
        image_s, center = super().capture_target(after)
        self.require_fresh_image(image_s)
        return image_s, center

    def plan_dock(self, report):
        image_s, center = self.capture_target(self.now())
        scene = WORKCELLS['m5']
        # This fixture is aligned with the original odometry +X axis.
        stamp_tf = Time.from_msg(self.selected_rgb.header.stamp)
        self.wait(lambda: self.tf.can_transform('odom', 'arm_base_link', stamp_tf),
                  5, 'Odometry TF did not reach the overview image time')
        self.wait(lambda: self.tf.can_transform('map', 'odom', stamp_tf), 5,
                  'SLAM TF did not reach the overview image time')
        self.pickup_map_from_odom = self.tf.lookup_transform('map', 'odom', stamp_tf)
        transform = self.tf.lookup_transform('odom', 'arm_base_link', stamp_tf)
        self.require_fresh_image(image_s)
        observed = xyz(do_transform_point(PointStamped(point=Point(x=center[0], y=center[1],
                                                                    z=center[2])), transform).point)
        dy = scene['cube_from_pickup_dock_xy'][1]
        dx = preferred_reach(dy, center[2])
        target = (observed[0]-dx, observed[1]-dy, 0.)
        report['overview'] = {'color': self.target_color, 'image_s': image_s, 'center_arm_m': center,
                              'center_odom_m': observed, 'requested_dock_odom': target}
        low, high = scene['cube_x_range_m']
        lateral = max(abs(v) for v in scene['cube_lanes_y_m'])+scene['cube_y_jitter_m']
        if (not low-.015 <= observed[0] <= high+.015 or abs(observed[1]) > lateral+.015
                or not table_clear(target) or not .032 <= center[2] <= .038):
            raise RuntimeError('Visible target has no safe nominal pickup dock in this fixture')
        report['checks']['nominal_pickup_reachable'] = True
        return target

    def check_docked_reach(self, report, settled):
        image_s, center = self.capture_target(settled)
        report['first_docked_vision'] = {'stamp_s': image_s, 'center_arm_m': center}
        try:
            checked_approach(center)
            return settled
        except RuntimeError as error:
            report['first_docked_vision']['reach_error'] = str(error)

        x, y, yaw = self.pickup_pose()
        desired = preferred_reach(center[1], center[2])
        distance = center[0]-desired
        target = (x+distance*math.cos(yaw), y+distance*math.sin(yaw), yaw)
        if (not self.stationary() or self.unexpected_motion or abs(yaw) > .03
                or not .003 < abs(distance) <= .025 or not table_clear(target)
                or any(abs(self.joints[name]) > .02 for name in ARM)):
            raise RuntimeError('No safe one-step docking correction for the new observation')

        row = report['correction'] = {'goal_odom': target, 'relative_m': distance}
        return self.drive_local(row, distance)

    def drive_local(self, row, distance):
        action, name = ((DriveOnHeading, '/drive_on_heading') if distance > 0
                        else (BackUp, '/backup'))
        goal = action.Goal()
        goal.target.x = distance
        goal.speed = .02 if abs(distance) > .025 else .015
        goal.time_allowance.sec = 14 if abs(distance) > .025 else 8
        return self.local_action(row, action, name, goal)

    def local_motion_start(self):
        return {'start_world': self.pose.copy()}

    def local_action(self, row, action, name, goal):
        if (not self.stationary() or self.unexpected_motion
                or any(abs(self.joints[joint]) > .02 for joint in ARM)):
            raise RuntimeError('Local docking requires the base stopped and the arm at home')
        client = ActionClient(self, action, name)
        enable = self.create_client(Trigger, '/velocity_arbiter/enable_navigation')
        self.wait(lambda: client.server_is_ready() and enable.service_is_ready(),
                  20, 'Nav2 local docking action or velocity arbiter unavailable')
        row.update(start_odom=self.odom_pose(), start_s=self.now(),
                   **self.local_motion_start())
        self.monitor_motion = False
        try:
            response = self.wait_future(enable.call_async(Trigger.Request()), 12)
            if not response.success:
                raise RuntimeError(response.message)
            self.wait(lambda: self.mode == 'navigation', 10, 'Arbiter did not enable local docking')
            self.local_motion = True
            row.update(self.run_base_action(client, goal, name))
            if row['status'] != GoalStatus.STATUS_SUCCEEDED or self.mode != 'navigation':
                raise RuntimeError('Nav2 refused or aborted local docking')
            return self.settle(row)
        except (RuntimeError, TimeoutError, KeyboardInterrupt):
            self.stop.publish(String(data='stop'))
            self.spin_for(.3)
            raise
        finally:
            self.local_motion = False
            client.destroy()
            self.destroy_client(enable)

    def finish_side_dock(self, report, settled):
        # At the lane entrance, use the camera again before the short approach.
        # This avoids steering into the table from a stale global heading.
        image_s, center = self.capture_target(settled)
        if not .32 <= center[0] <= .44:
            raise RuntimeError('The side pickup is outside the visual alignment distance')
        dy = WORKCELLS['m5']['cube_from_pickup_dock_xy'][1]
        yaw_turn = math.atan2(center[1], center[0])-math.asin(dy/math.hypot(*center[:2]))
        report['lineup_vision'] = {'stamp_s': image_s, 'center_arm_m': center, 'turn_rad': yaw_turn}
        x, y, yaw = self.pickup_pose()
        if abs(yaw_turn) > .24 or not table_clear((x, y, yaw+yaw_turn)):
            raise RuntimeError('The side pickup has no safe visual alignment at the lane entrance')
        if abs(yaw_turn) > .015:
            goal = Spin.Goal(target_yaw=yaw_turn)
            goal.time_allowance.sec = 8
            row = report['lineup_turn'] = {'goal_odom': (x, y, yaw+yaw_turn)}
            settled = self.local_action(row, Spin, '/spin', goal)
            image_s, center = self.capture_target(settled)
        origin = self.pickup_pose()
        row = report['navigation'] = {'guidance_stamp_s': image_s,
                                      'guidance_center_arm_m': center, 'guidance_base_pose': origin}
        distance, target = side_approach(center, origin)
        row.update(goal_odom=target, visual_approach_m=distance)
        return self.drive_local(row, distance)

    def trial(self, report):
        self.wait(lambda: all(client.server_is_ready() for client in self.trajectory_clients.values())
                  and self.pose is not None and self.joint_samples and self.now() > 0,
                  150, 'Arm actions, joint states or simulation clock missing')
        if any(abs(self.joints[name]) > .02 for name in ARM):
            raise RuntimeError('Arm must be home for navigation')
        self.activate_nav()
        settled = self.pickup_dock(report)
        row = report.get('correction', report['navigation'])
        self.grasp(report, after=settled, release=False)
        report['checks']['fresh_image_after_dock'] = stamp(self.selected_rgb.header) > settled
        report['checks']['stationary_through_hold'] = (
            self.stationary() and not self.unexpected_motion
            and math.dist(row['settled_world'][:2], self.pose[:2]) <= .002)

    def pickup_pose(self):
        return self.odom_pose()

    def reach_pickup_lineup(self, row, lineup):
        self.navigate_to(row, lineup)
        return self.settle(row)

    def pickup_dock(self, report):
        self.get_logger().info('M5.2 Nav2 active; waiting for overview detection')
        dock = self.plan_dock(report)
        self.get_logger().info(f'M5.2 visually planned dock {dock}; starting Nav2')
        if abs(dock[1]-self.pickup_pose()[1]) > .035:
            lineup = (.065, dock[1], 0.)
            self.get_logger().info(f'Aligning with the pickup lane at {lineup}')
            row = report['lineup_navigation'] = {}
            settled = self.finish_side_dock(report, self.reach_pickup_lineup(row, lineup))
        else:
            row = report['navigation'] = {}
            self.navigate_to(row, dock)
            settled = self.settle(row)
        row = report['navigation']
        self.get_logger().info('M5.2 dock stationary; reacquiring target')
        start = report.get('lineup_navigation', row)['start_odom']
        report['checks']['base_navigated'] = math.dist(start[:2], row['settled_odom'][:2]) > .05
        if not report['checks']['base_navigated']:
            raise RuntimeError('Base did not travel to the pickup dock')
        return self.check_docked_reach(report, settled)


if __name__ == '__main__':
    color = os.environ.get('M52_TARGET_COLOR', 'red')
    if color not in ('red', 'blue', 'yellow'):
        raise SystemExit('M52_TARGET_COLOR must be red, blue or yellow')
    raise SystemExit(main(functools.partial(RandomPick, color), __doc__, save_rgb))
