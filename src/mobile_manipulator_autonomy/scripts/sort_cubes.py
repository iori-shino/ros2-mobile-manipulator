#!/usr/bin/env python3
"""Run the M6 sort using ROS localization, RGB and physical contact sensors."""

import functools
import math
import os
from pathlib import Path
import sys
from threading import Thread
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point, PointStamped, Twist
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav2_msgs.msg import SpeedLimit
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException

# The established controller clients are installed by the Gazebo package.
gazebo_prefix = Path(get_package_prefix('mobile_manipulator_gazebo'))
sys.path.append(str(gazebo_prefix/'lib'/'mobile_manipulator_gazebo'))

from acceptance_test import angles, wrap
from arm_control import ARM, JAWS
from grasp_contact_probe import CONTACT_WINDOW, SIDES, scoped, stamp, xyz
from m42_dock_grasp import SETTLE_S, WORKCELLS, motion
from m4_vision_grasp import TCP, checked_approach, main, solve_above
from m52_random_pick import RandomPick, preferred_reach, table_clear
from room_alignment import fit_room

# An alternate Gazebo layout must use the same workcells in the sorter.
if workcell_config := os.getenv('M6_WORKCELLS'):
    import yaml
    WORKCELLS.update(yaml.safe_load(Path(workcell_config).read_text()))

SCENE = WORKCELLS['m5']
FRAME = WORKCELLS['m6']['station_frame']
PRIORITY = WORKCELLS['m6']['color_priority']
STOW_TCP = (.165, .031, .150)
# The collision pads start with an inner gap of 20 mm. A measured 13 mm
# retraction on each side leaves 46 mm for the 35 mm cube.
OPEN_POSITION = .014
MIN_OPEN = .013
PADS = [(f'{side}_jaw', f'{side}_jaw_collision_1') for side in SIDES]


class DockStop:
    """Distinguish a planned station stop from an operator cancellation."""

    def __init__(self, publisher):
        self.publisher = publisher

    def publish(self, _message):
        self.publisher.publish(String(data='dock_stop'))


class SortCubes(RandomPick):
    def __init__(self, selection):
        super().__init__(selection[0])
        self.selection = selection
        for subscription in list(self.subscriptions):
            if subscription.topic_name in ('/ground_truth/pose', '/grasp/cube_pose', '/sim/cmd_vel'):
                self.destroy_subscription(subscription)
        self.motion_trip = None
        self.joint_time = -math.inf
        self.inventory = {}
        self.inventory_time = -math.inf
        self.pick_obstruction = None
        self.stop = DockStop(self.stop)
        self.operator_stop = False
        self.carrying = False
        self.loss_since = None
        self.contact_problem = None
        self.station_contact = -math.inf
        self.scan = None
        self.room_anchor = None
        self.workcell_registration = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.drive_on_heading = ActionClient(self, DriveOnHeading, '/drive_on_heading')
        self.stage_log = []
        self.enable = self.create_client(Trigger, '/velocity_arbiter/enable_navigation')
        self.backup = ActionClient(self, BackUp, '/backup')
        self.spin = ActionClient(self, Spin, '/spin')
        self.speed = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.create_subscription(String, '/teleop/key', self.on_stop, 10)
        self.create_subscription(String, '/sorting/stop', self.on_stop, 10)
        for color in PRIORITY:
            self.create_subscription(Contacts, f'/sorting/contact/{color}',
                                     functools.partial(self.on_support, color), qos_profile_sensor_data)

        # Guard uses its wall-clock watchdog. Keep its latest feedback out of
        # the queue servicing camera, simulation TF and task Actions.
        self.guard_node = Node('sorting_guard_feedback', context=self.context)
        self.guard_node.create_subscription(Twist, '/sim/cmd_vel', self.on_guard, 1)
        self.guard_executor = SingleThreadedExecutor(context=self.context)
        self.guard_executor.add_node(self.guard_node)
        self.guard_thread = Thread(target=self.guard_executor.spin, name='sorting_guard_feedback',
                                   daemon=True)
        self.guard_thread.start()

    def destroy_node(self):
        self.guard_executor.shutdown(timeout_sec=2.)
        self.guard_thread.join(timeout=2.)
        self.guard_node.destroy_node()
        return super().destroy_node()

    def on_detection(self, msg):
        super().on_detection(msg)
        image_s = stamp(msg.header)
        if msg.header.frame_id == 'arm_base_link' and 0. <= self.now()-image_s <= .5:
            self.inventory_time = image_s
            self.inventory = {item.id: xyz(item.bbox.center.position) for item in msg.detections
                              if item.id in PRIORITY and item.header == msg.header}

    def on_mode(self, msg):
        super().on_mode(msg)
        if self.monitor_motion and self.mode != 'manual' and self.motion_trip is None:
            self.motion_trip = ('arbiter mode changed', self.mode)

    def on_guard(self, msg):
        super().on_guard(msg)
        if self.monitor_motion and not self.is_still(self.guard) and self.motion_trip is None:
            self.motion_trip = ('Guard output exceeded stationary limit', self.guard)

    def on_odom(self, msg):
        super().on_odom(msg)
        velocity = motion(msg.twist.twist)
        if self.monitor_motion and not self.is_still(velocity) and self.motion_trip is None:
            self.motion_trip = ('odometry exceeded stationary limit', velocity)

    def base_feedback(self):
        return {
            'first_motion_trip': self.motion_trip,
            'mode': self.mode,
            'guard_speed': self.guard,
            'guard_age_wall_s': round(time.monotonic()-self.guard_time, 3) if self.guard is not None else None,
            'odom_speed': motion(self.odom.twist.twist) if self.odom is not None else None,
            'odom_age_sim_s': round(self.now()-stamp(self.odom.header), 3) if self.odom is not None else None,
        }

    def on_joints(self, msg):
        super().on_joints(msg)
        if all(joint in msg.name for joint in JAWS):
            self.joint_time = stamp(msg.header)

    def goal(self, phase, group, positions, duration, **kwargs):
        if phase in ('open', 'place_release', 'release_after_missed_contact'):
            self.check_motion()
            if not self.stationary():
                raise RuntimeError('Base motion blocked the gripper')
            aperture = [self.joints.get(joint, -math.inf) for joint in JAWS]
            if (phase == 'open' and 0 <= self.now()-self.joint_time <= .2
                    and min(aperture) >= MIN_OPEN):
                self.get_logger().info(f'Using measured open jaws: {aperture}')
                self.actions.append({'phase': phase, 'start_s': self.now(), 'group': group,
                                     'observed_jaw_positions_m': aperture,
                                     'skipped': 'Already open with fresh joint feedback'})
                return
            self.get_logger().info(
                f'{phase} jaw feedback {aperture}, age {self.now()-self.joint_time:.3f} s')
            positions = [OPEN_POSITION, OPEN_POSITION]
        return super().goal(phase, group, positions, duration, **kwargs)

    def on_scan(self, scan):
        self.scan = scan

    def room_pose(self):
        scan = self.scan
        if scan is None or not 0 <= self.now()-stamp(scan.header) <= .5:
            raise RuntimeError('A fresh LiDAR scan is required for station alignment')
        offset = self.tf.lookup_transform('base_footprint', scan.header.frame_id,
                                          Time.from_msg(scan.header.stamp)).transform
        yaw = angles(offset.rotation)[2]
        points = []
        for index, distance in enumerate(scan.ranges):
            if scan.range_min <= distance < scan.range_max:
                ray = yaw + scan.angle_min + index*scan.angle_increment
                points.append((offset.translation.x+distance*math.cos(ray),
                               offset.translation.y+distance*math.sin(ray)))
        prior = self.map_pose()
        odom = self.odom_pose()
        bounds = WORKCELLS['m6']['room_walls_xy']
        try:
            estimate = fit_room(points, prior, bounds)
        except RuntimeError:
            alternatives = []
            if self.room_anchor is not None:
                before_time, before_odom, before_room = self.room_anchor
                if (self.now()-before_time < 25.
                        and math.dist(odom[:2], before_odom[:2]) < .25):
                    alternatives.append(self.register_pose(odom, before_odom, before_room))
            for candidate in alternatives:
                try:
                    estimate = fit_room(points, candidate, bounds)
                    break
                except RuntimeError:
                    continue
            else:
                self.localization_failure = {'scan_s': stamp(scan.header), 'map_prior': prior,
                                             'odom_prior': alternatives, 'points_base_m': points}
                raise
        self.room_anchor = (self.now(), odom, estimate)
        return estimate

    @staticmethod
    def register_pose(target, origin, destination):
        theta = wrap(destination[2]-origin[2])
        dx, dy = target[0]-origin[0], target[1]-origin[1]
        c, s = math.cos(theta), math.sin(theta)
        return (destination[0]+c*dx-s*dy, destination[1]+s*dx+c*dy,
                wrap(target[2]+theta))

    def room_goal_in_map(self, target, room):
        return self.register_pose(target, room, self.map_pose())

    def on_stop(self, msg):
        if msg.data in ('stop', ' ', 'q'):
            self.operator_stop = True
            self.stop.publish(None)

    def stage(self, name):
        self.check_motion()
        self.stage_log.append({'state': name, 'simulation_s': self.now()})
        self.get_logger().info(f'M6 {name}')

    def check_motion(self):
        if self.operator_stop:
            raise RuntimeError('Operator stopped the sorting task')
        if self.contact_problem:
            raise RuntimeError(self.contact_problem)
        if self.monitor_motion and self.unexpected_motion:
            raise RuntimeError(f'Unexpected base motion during a stationary stage: {self.base_feedback()}')

    def map_pose(self):
        transform = self.tf.lookup_transform(FRAME, 'base_footprint', Time())
        age = self.now()-stamp(transform.header)
        if not -.25 <= age <= .5:
            raise RuntimeError(f'Stale {FRAME} localization ({age:.3f} s)')
        return (*xyz(transform.transform.translation)[:2], angles(transform.transform.rotation)[2])

    def pickup_pose(self):
        return self.room_pose()

    def plan_dock(self, report):
        image_s, center = self.capture_target(self.now())
        self.stage('PLAN_PICK')
        image_stamp = Time.from_msg(self.selected_rgb.header.stamp)
        self.wait(lambda: self.tf.can_transform(FRAME, 'arm_base_link', image_stamp),
                  5, 'Missing image-time localization')
        transform = self.tf.lookup_transform(FRAME, 'arm_base_link', image_stamp)
        self.require_fresh_image(image_s)
        point = PointStamped(point=Point(x=center[0], y=center[1], z=center[2]))
        projected = xyz(do_transform_point(point, transform).point)
        room, mapping = self.room_pose(), self.map_pose()
        self.pickup_registration = (room, mapping)
        if self.workcell_registration is None:
            self.workcell_registration = self.pickup_registration
        x, y, _ = self.register_pose((*projected[:2], 0.), mapping, room)
        offset_y = SCENE['cube_from_pickup_dock_xy'][1]
        dock = (x-preferred_reach(offset_y, center[2]), y-offset_y, 0.)
        low, high = SCENE['cube_x_range_m']
        limits = max(map(abs, SCENE['cube_lanes_y_m'])) + SCENE['cube_y_jitter_m']
        if not (low-.015 <= x <= high+.015 and abs(y) <= limits+.015 and table_clear(dock)):
            raise RuntimeError('No safe pickup dock for the visual position in map')
        report['observed_cubes_arm_m'] = self.inventory.copy()
        report['overview'] = {'color': self.target_color, 'image_s': image_s,
                              'target_map_xy': projected[:2], 'target_room_xy': [x, y],
                              'dock_room': dock}
        self.phase = 'pickup_navigation'
        self.stage('NAVIGATE_TO_PICK')
        return dock

    def navigate_to(self, row, target):
        if self.phase == 'pickup_navigation':
            row['goal_room'] = target
            target = self.register_pose(target, *self.pickup_registration)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z = math.sin(target[2]/2)
        goal.pose.pose.orientation.w = math.cos(target[2]/2)
        row.update(goal_map=target, start_odom=self.odom_pose(), start_s=self.now())
        row.update(self.run_base_action(self.nav, goal, 'NavigateToPose'))
        if row['status'] != GoalStatus.STATUS_SUCCEEDED or self.mode != 'navigation':
            raise RuntimeError(f'Navigation ended with status {row["status"]}')

    def run_base_action(self, client, goal, name):
        handle = self.wait_future(client.send_goal_async(goal), 15)
        if not handle.accepted:
            raise RuntimeError(f'Nav2 rejected {name}')
        try:
            future = handle.get_result_async()
            def done():
                self.check_motion()
                if self.mode != 'navigation':
                    raise RuntimeError('Manual takeover interrupted navigation')
                return future.done()
            limit = 100. if self.carrying or self.phase == 'return' else 35.
            self.wait_sim(done, limit, name)
        except (RuntimeError, TimeoutError, KeyboardInterrupt):
            self.stop.publish(None)
            super().wait_future(handle.cancel_goal_async(), 5)
            raise
        return {'status': future.result().status, 'result_s': self.now(),
                'result_odom': self.odom_pose(), 'result_map': self.map_pose()}

    def settle(self, row):
        self.stop.publish(None)
        self.wait(lambda: self.mode == 'manual', 10, 'Arbiter did not inhibit navigation')
        interval = None

        def settled():
            nonlocal interval
            if not self.stationary():
                interval = None
                return False
            odom = self.odom_pose()
            if interval is None:
                interval = (self.now(), odom)
            start, initial = interval
            if math.dist(initial[:2], odom[:2]) > .001 or abs(wrap(initial[2]-odom[2])) > .005:
                interval = None
                return False
            return self.now()-start >= SETTLE_S

        def complete():
            self.check_motion()
            return settled()

        self.wait_sim(complete, 12., 'Stationary verification')
        row.update(settled_s=self.now(), settled_odom=self.odom_pose(), settled_map=self.map_pose())
        self.monitor_motion = True
        return row['settled_s']

    def align_pickup_heading(self, row, heading, settled):
        for _ in range(2):
            room = self.room_pose()
            residual = wrap(heading-room[2])
            if abs(residual) <= .028:
                return settled
            if abs(residual) > 1.70 or room[0] > .112 or abs(room[1]) > .19:
                raise RuntimeError(f'Pickup lineup cannot safely rotate at {room}')
            goal = Spin.Goal(target_yaw=residual)
            goal.time_allowance.sec = 12
            turn = {'goal_room': (*room[:2], heading), 'room_before': room}
            row['turns'].append(turn)
            settled = self.local_action(turn, Spin, '/spin', goal)
        if abs(wrap(heading-self.room_pose()[2])) > .028:
            raise RuntimeError('Pickup lineup heading did not converge')
        return settled

    def reach_pickup_lineup(self, row, lineup):
        row.update(start_odom=self.odom_pose(), turns=[], drives=[], goal_room=lineup)
        settled = self.settle(row)
        for _ in range(2):
            room = self.room_pose()
            dx, dy = lineup[0]-room[0], lineup[1]-room[1]
            distance = math.hypot(dx, dy)
            if distance <= .008:
                break
            if not (.008 < distance <= .15 and room[0] <= .112 and abs(room[1]) <= .19):
                raise RuntimeError(f'Pickup lineup is outside its safe approach area: {room}')
            heading = math.atan2(dy, dx)
            settled = self.align_pickup_heading(row, heading, settled)
            drive = {'goal_room': lineup, 'room_before': self.room_pose(), 'relative_m': distance}
            row['drives'].append(drive)
            settled = self.drive_local(drive, distance)
        settled = self.align_pickup_heading(row, 0., settled)
        room = self.room_pose()
        row.update(settled_s=settled, settled_room=room, settled_odom=self.odom_pose())
        if math.dist(room[:2], lineup[:2]) > .017:
            raise RuntimeError(f'Pickup lineup remained outside its position tolerance: {room}')
        return settled

    def check_docked_reach(self, report, settled):
        self.stage('VERIFY_PICK_DOCK')
        return super().check_docked_reach(report, settled)

    def local_motion_start(self):
        return {'start_room': self.room_pose()}

    def run_for(self, seconds, *, publish):
        if publish:
            raise RuntimeError('The sorter sends base commands only through Nav2')
        end = self.now()+seconds
        def done():
            self.check_motion()
            return self.now() >= end
        self.wait_sim(done, seconds+1., self.phase)

    def finish_trajectory(self, handle, duration):
        future = handle.get_result_async()

        def done():
            self.check_motion()
            if not self.stationary() or self.unexpected_motion:
                raise RuntimeError(f'Stationary feedback interrupted arm operation: {self.base_feedback()}')
            return future.done()

        self.wait_sim(done, duration+5., 'Arm/gripper trajectory')
        return future.result()

    def sample(self):
        now = self.now()
        active = [side for side in SIDES if now-self.seen.get(side, -math.inf) < CONTACT_WINDOW]
        state = 'both' if len(active) == 2 else next(iter(active), 'none')
        self.trace.append({'phase': self.phase, 'time_s': now, 'state': state})
        if self.carrying and state != 'both':
            self.loss_since = now if self.loss_since is None else self.loss_since
            if now-self.loss_since > .25:
                self.contact_problem = 'Sustained bilateral grip loss'
        else:
            self.loss_since = None

    def on_support(self, color, msg):
        if color != self.target_color:
            return
        support = (f'm5_drop_{color}', 'body', 'collision')
        for pair in msg.contacts:
            a, b = pair.collision1.name, pair.collision2.name
            other = b if scoped(a, self.target_collision) else a if scoped(b, self.target_collision) else ''
            if scoped(other, support):
                self.station_contact = stamp(msg.header)
            if (self.phase in ('above_cube', 'approach') and other
                    and not scoped(other, ('m5_pickup_table', 'body', 'collision'))):
                self.pick_obstruction = other
            if self.carrying and other and not any(scoped(other, pad) for pad in PADS):
                self.contact_problem = f'Unexpected payload contact during {self.phase}: {other}'

    def resume_navigation(self, carrying=False):
        self.check_motion()
        self.wait(lambda: self.enable.service_is_ready() and self.speed.get_subscription_count(),
                  20, 'Velocity arbiter or Nav2 speed limit missing')
        self.speed.publish(SpeedLimit(percentage=False, speed_limit=.035 if carrying else 0.))
        self.monitor_motion = False
        response = self.wait_future(self.enable.call_async(Trigger.Request()), 12)
        if not response.success:
            raise RuntimeError(response.message)
        self.wait(lambda: self.mode == 'navigation', 10, 'Navigation did not enable')

    def grasp_carrier(self, report, settled):
        self.stage('REFRESH_VISION')
        image_s, center = self.capture_target(settled)
        self.stage('CHECK_REACHABILITY')
        above, grab, clearance = checked_approach(center)
        report['vision'] = {'stamp_s': image_s, 'center_arm_m': center, 'tcp_offset_m': TCP}
        report['metrics']['pad_clearance_m'] = clearance
        self.stage('PICK')
        self.pick_obstruction = None
        _, hold = self.grasp_motion(
            report, center, above, grab, approach_unchanged=lambda: self.pick_obstruction is None)
        self.stage('VERIFY_GRASP')
        if not hold or hold[-1]['state'] != 'both' or sum(row['state'] == 'both' for row in hold)/len(hold) <= .8:
            raise RuntimeError('Bilateral contact did not persist through lift and hold')
        self.carrying = True
        self.stage('STOW_ARM')
        self.goal('stow', 'arm', solve_above(STOW_TCP, 0.), 4)

    def deliver(self, report, zone):
        self.stage('NAVIGATE_TO_PLACE')
        self.phase = 'loaded_retreat'
        self.resume_navigation(carrying=True)
        self.wait(self.backup.server_is_ready, 20, 'Nav2 backup unavailable')
        retreat = BackUp.Goal(speed=.030)
        retreat.target.x = -.120
        retreat.time_allowance.sec = 18
        report['retreat'] = self.run_base_action(self.backup, retreat, 'BackUp')
        if report['retreat']['status'] != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Loaded retreat failed')
        self.phase = 'loaded_turn'
        x, y, heading = self.room_pose()
        gx, gy, _ = zone['dock_world']
        turn = Spin.Goal(target_yaw=wrap(math.atan2(gy-y, gx-x)-heading))
        turn.time_allowance.sec = 23
        self.wait(self.spin.server_is_ready, 20, 'Nav2 Spin unavailable')
        report['turn'] = self.run_base_action(self.spin, turn, 'Spin')
        if report['turn']['status'] != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Loaded turn failed')
        # Register the station after the loaded turn has come to rest.
        turned_at = self.settle(report['turn'])
        self.wait_sim(lambda: self.scan is not None and stamp(self.scan.header) > turned_at,
                      2., 'Post-turn LiDAR scan')
        self.phase = 'loaded_navigation'
        dock = zone['dock_world']
        staging = (dock[0]+.28, dock[1], dock[2])
        reports = report['placement_navigation'] = []
        for attempt in range(3):
            room = self.room_pose()
            if attempt and abs(room[0]-staging[0]) <= .042 and abs(room[1]-staging[1]) <= .018:
                break
            # A long approach uses the initial workcell alignment. Local
            # LiDAR recalibration is applied only near the station.
            if attempt:
                target = self.room_goal_in_map(staging, room)
            else:
                target = self.register_pose(staging, *self.workcell_registration)
            self.resume_navigation(carrying=True)
            row = {'room_before': room, 'staging_target': staging,
                   'registration': 'local_lidar' if attempt else 'initial_workcell'}
            reports.append(row)
            self.navigate_to(row, target)
            self.settle(row)
            row['room_after'] = self.room_pose()
        self.stage('VERIFY_PLACE_DOCK')
        room = self.room_pose()
        if not (.22 <= room[0]-dock[0] <= .34 and abs(room[1]-dock[1]) <= .018):
            raise RuntimeError(f'LiDAR localization reports an unaligned placement lane: {room}')
        turns = report['placement_alignment_turns'] = []
        for _ in range(2):
            bearing = math.atan2(dock[1]-room[1], dock[0]-room[0])
            aim = max(-.023, min(.023, wrap(bearing-dock[2])))
            residual = wrap(dock[2]+aim-room[2])
            if abs(residual) <= .012:
                break
            if abs(residual) > .46:
                raise RuntimeError('Placement yaw is outside the permitted local correction')
            self.resume_navigation(carrying=True)
            correction = Spin.Goal(target_yaw=residual)
            correction.time_allowance.sec = 8
            row = self.run_base_action(self.spin, correction, 'Placement alignment Spin')
            turns.append(row)
            if row['status'] != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError('Placement heading correction failed')
            self.settle(row)
            room = self.room_pose()
            row['room_after'] = room
        distance = room[0]-dock[0]
        if not (.22 <= distance <= .34 and abs(room[1]-dock[1]) <= .018
                and abs(wrap(room[2]-dock[2])) <= .036):
            raise RuntimeError(f'LiDAR placement alignment did not converge: {room}')
        self.phase = 'loaded_final_approach'
        self.resume_navigation(carrying=True)
        self.wait(self.drive_on_heading.server_is_ready, 15, 'Nav2 straight approach unavailable')
        approach = DriveOnHeading.Goal(speed=.025)
        approach.target.x = distance
        approach.time_allowance.sec = 18
        row = report['placement_approach'] = self.run_base_action(
            self.drive_on_heading, approach, 'DriveOnHeading')
        if row['status'] != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Loaded final approach failed')
        self.settle(row)
        room = self.room_pose()
        row['room_after'] = room
        if math.dist(room[:2], dock[:2]) > .012 or abs(wrap(room[2]-dock[2])) > .040:
            raise RuntimeError(f'LiDAR localization reports placement dock out of tolerance: {room}')

    def place(self, report, zone):
        self.stage('PLACE')
        x, y, heading = self.room_pose()
        dx, dy = zone['table_world_xy'][0]-x, zone['table_world_xy'][1]-y
        base = self.tf.lookup_transform('base_footprint', 'arm_base_link', Time()).transform.translation
        frame_z = WORKCELLS['base_footprint_rest_z_m']
        surface = SCENE['table_top_m']
        target = [math.cos(heading)*dx+math.sin(heading)*dy-base.x,
                  -math.sin(heading)*dx+math.cos(heading)*dy-base.y,
                  surface+.035/2-frame_z-base.z]
        report['placement_target_arm_m'] = target
        # Both endpoints and intermediate points must be reachable before moving.
        if not (.262 <= target[0] <= .292 and abs(target[1]) <= .030):
            raise RuntimeError(f'Placement is outside its validated arm bounds: {target}')
        for index in range(11):
            solve_above(target, .010+.002*index)
        self.goal('place_above', 'arm', solve_above(target, .030), 4)
        self.goal('place_lower', 'arm', solve_above(target, .010), 3)
        self.carrying = False
        released_after = self.now()
        self.goal('place_release', 'gripper', [.015, .015], 2)
        self.stage('VERIFY_RELEASE')
        interval = None

        def contact_stable():
            nonlocal interval
            self.check_motion()
            stable = (self.station_contact > released_after
                      and self.now()-self.station_contact < .25
                      and self.stationary() and self.trace and self.trace[-1]['state'] == 'none')
            if not stable:
                interval = None
                return False
            interval = self.now() if interval is None else interval
            return self.now()-interval >= 1.0

        self.wait_sim(contact_stable, 5., 'Verified placement on matching support')
        report['support_contact_s'] = self.station_contact
        return solve_above(target, .030)

    def return_to_observation(self, report):
        self.resume_navigation()
        retreat = BackUp.Goal(speed=.030)
        retreat.target.x = -.120
        retreat.time_allowance.sec = 18
        row = report['return_retreat'] = self.run_base_action(self.backup, retreat, 'BackUp')
        if row['status'] != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Unable to clear the placement table before returning')
        target = SCENE['observation_world']
        rows = report['return'] = []
        for _ in range(3):
            room = self.room_pose()
            if math.dist(room[:2], target[:2]) <= .015:
                if abs(wrap(room[2]-target[2])) > .04:
                    endpoint = report['return_heading'] = {'turns': []}
                    settled = self.settle(endpoint)
                    self.align_pickup_heading(endpoint, target[2], settled)
                self.resume_navigation()
                return
            self.resume_navigation()
            row = {'room_before': room}
            rows.append(row)
            self.navigate_to(row, self.room_goal_in_map(target, room))
            self.settle(row)
        room = self.room_pose()
        if math.dist(room[:2], target[:2]) > .015 or abs(wrap(room[2]-target[2])) > .04:
            raise RuntimeError('Unable to align with the observation station')
        self.resume_navigation()

    def trial(self, report):
        report.update(states=self.stage_log, completed=[], objects={})
        try:
            self.stage('IDLE')
            self.wait(lambda: self.now() > 0 and self.odom is not None and self.joint_samples
                      and all(c.server_is_ready() for c in self.trajectory_clients.values()),
                      150, 'Robot startup incomplete')
            if any(abs(self.joints[name]) > .02 for name in ARM):
                raise RuntimeError('Start a new sorting task with the arm at home')
            self.activate_nav()
            for color in self.selection:
                self.stage('OBSERVE')
                after = self.now()
                def inventory_ready():
                    self.check_motion()
                    return (self.inventory_time > after and 0 <= self.now()-self.inventory_time <= .5
                            and color in self.inventory)
                self.wait(inventory_ready, 45, f'No fresh observation of the remaining {color} cube')
                self.stage('SELECT_TARGET')
                self.target_color = color
                self.target_collision = (f'm3_{color}_cube', 'body', 'collision')
                self.detection = None
                self.seen.clear()
                self.station_contact = -math.inf
                self.other_contacts = {side: set() for side in SIDES}
                item = report['objects'][color] = {'checks': {}, 'metrics': {}}
                settled = self.pickup_dock(item)
                self.grasp_carrier(item, settled)
                zone = SCENE['drop_zones'][color]
                self.deliver(item, zone)
                retreat = self.place(item, zone)
                self.stage('MARK_COMPLETE')
                report['completed'].append(color)
                self.stage('CLEAR_PLACEMENT_ARM')
                self.goal('place_retreat', 'arm', retreat, 4)
                self.goal('home', 'arm', [0.]*4, 7)
                if color != self.selection[-1]:
                    self.stage('RETURN_TO_OBSERVE')
                    self.phase = 'return'
                    self.return_to_observation(item)
            self.stage('TASK_COMPLETE')
            report['checks']['selected_deliveries_complete'] = report['completed'] == list(self.selection)
        except (RuntimeError, TimeoutError, TransformException, KeyboardInterrupt):
            if getattr(self, 'localization_failure', None):
                report['localization_failure'] = self.localization_failure
            self.stage_log.append({'state': 'STOPPED' if self.operator_stop else 'FAILED',
                                   'simulation_s': self.now()})
            self.stop.publish(None)
            self.spin_for(.3)
            raise


if __name__ == '__main__':
    colors = tuple((os.getenv('M6_COLORS') or ','.join(PRIORITY)).split(','))
    if not colors or len(set(colors)) != len(colors) or any(color not in PRIORITY for color in colors):
        raise SystemExit('M6_COLORS must name distinct configured colors')
    raise SystemExit(main(functools.partial(SortCubes, colors), __doc__))
