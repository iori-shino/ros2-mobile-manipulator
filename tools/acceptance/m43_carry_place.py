#!/usr/bin/env python3
"""Run one visual pickup, arm stow, loaded Nav2 trip and physical placement."""

import math
import time

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, Spin
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from std_srvs.srv import Trigger

from acceptance_test import wrap
from arm_control import ARM
from grasp_contact_probe import SIDES, TARGET, scoped, stamp
from m4_vision_grasp import main, solve_above
from m42_dock_grasp import DockGrasp, M43_CELL, WORKCELLS, motion, save_rgb


STOW_TCP = (.165, .031, .150)
DROP_DOCK = tuple(M43_CELL['drop_dock_world'])
# Test-station center in the Gazebo world, matching the placement support in launch.
DROP_CENTER = tuple(M43_CELL['drop_table_world_xy'])
REST_HEIGHT = M43_CELL['table_top_m'] + .035/2
RETREAT = .120
CARRY_SPEED = .035
MAX_SHIFT = .008
PADS = tuple((f'{side}_jaw', f'{side}_jaw_collision_1') for side in SIDES)


class CarryPlace(DockGrasp):
    def __init__(self):
        super().__init__()
        self.enable_nav = self.create_client(Trigger, '/velocity_arbiter/enable_navigation')
        self.backup = ActionClient(self, BackUp, '/backup')
        self.spin = ActionClient(self, Spin, '/spin')
        self.nav_manager = self.create_client(ManageLifecycleNodes,
                                              '/lifecycle_manager_navigation/manage_nodes')
        self.speed_limit = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.create_subscription(Contacts, '/carry/cube_contacts', self.on_cube_contacts,
                                 qos_profile_sensor_data)
        self.cube_contacts = {}
        self.payload_reference = None
        self.payload_problem = None
        self.loss_since = None
        self.recent_drop_contact = -math.inf

    def ready(self):
        self.wait(lambda: all(client.server_is_ready() for client in self.trajectory_clients.values())
                  and self.pose is not None and self.joint_samples and self.now() > 0,
                  150, 'Arm Action discovery or physical feedback did not become ready')

    def configure_controller_clock(self):
        # The carry launch has already checked the plugin's clock-service response.
        pass

    def navigate(self, report):
        self.wait(lambda: self.now() > 0 and self.odom is not None
                  and abs(self.now()-stamp(self.odom.header)) < .3
                  and self.tf.can_transform('odom', 'base_footprint', Time())
                  and self.rgb, 90, 'Waiting for live clock, odometry TF and RGB before Nav2')
        states = {}
        report['nav_startup'] = {'ready_sim_s': self.now(), 'states_before_activation': states}
        # A service advertising a name can still be busy creating its costmaps.
        clients = {name: self.create_client(GetState, f'/{name}/get_state')
                   for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')}
        try:
            self.wait(lambda: self.nav_manager.service_is_ready()
                      and all(client.service_is_ready() for client in clients.values()),
                      160, 'Nav2 component loading did not expose all lifecycle services')
            for name, client in clients.items():
                for attempt in range(3):
                    response = client.call_async(GetState.Request())
                    try:
                        state = self.wait_future(response, 12).current_state
                        break
                    except TimeoutError:
                        client.remove_pending_request(response)
                        self.get_logger().warning(f'{name}/get_state missing reply ({attempt+1}/3)')
                else:
                    raise RuntimeError(f'{name}/get_state gave no reply to three read-only requests')
                states[name] = state.label
        finally:
            for client in clients.values():
                self.destroy_client(client)
        if not all(label == 'unconfigured' for label in states.values()):
            raise RuntimeError(f'Expected a fresh, inactive Nav2 graph: {states}')
        self.get_logger().info('Controller clock, sensors and Nav2 state services ready; activating Nav2')
        req = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
        result = self.wait_future(self.nav_manager.call_async(req), 180)
        if not result.success:
            raise RuntimeError('Nav2 lifecycle manager rejected activation; see launch log')
        report['nav_startup']['activated_sim_s'] = self.now()
        super().navigate(report)

    def on_cube_contacts(self, msg):
        for pair in msg.contacts:
            a, b = pair.collision1.name, pair.collision2.name
            if scoped(b, TARGET):
                a, b = b, a
            if not scoped(a, TARGET):
                continue
            self.cube_contacts.setdefault(self.phase, set()).add(b)
            if scoped(b, ('place_support', 'body', 'collision')):
                self.recent_drop_contact = stamp(msg.header)
            if self.payload_reference is not None and not any(scoped(b, pad) for pad in PADS):
                self.payload_problem = f'Unexpected cube contact during {self.phase}: {b}'

    def sample(self):
        old_count = len(self.trace)
        super().sample()
        if len(self.trace) == old_count:
            return
        row = self.trace[-1]
        row['base_world_xy_yaw'] = [*self.pose[:2], self.pose[5]] if self.pose else None
        row['odom_speed'] = motion(self.odom.twist.twist) if self.odom else None
        if self.payload_reference is None:
            return
        relative = row['cube_in_gripper_base_m']
        if relative is not None:
            row['payload_shift_m'] = math.dist(relative, self.payload_reference)
        valid = (row['state'] == 'both' and relative is not None
                 and row['time_s']-row['cube_stamp_s'] < .25 and row['payload_shift_m'] <= MAX_SHIFT)
        if valid:
            self.loss_since = None
        elif self.loss_since is None:
            self.loss_since = row['time_s']
        elif row['time_s']-self.loss_since > .25 and self.payload_problem is None:
            self.payload_problem = f'Sustained grip loss, stale pose or object motion during {self.phase}'

    def wait_simulation(self, complete, seconds, operation):
        start = previous = self.now()
        last_tick = time.monotonic()
        while not complete():
            rclpy.spin_once(self, timeout_sec=.05)
            now = self.now()
            if now < previous:
                raise RuntimeError(f'Simulation clock reset during {operation}')
            if now > previous:
                previous, last_tick = now, time.monotonic()
            if now-start > seconds:
                raise TimeoutError(f'{operation} exceeded {seconds:g} simulation seconds')
            if time.monotonic()-last_tick > 20:
                raise RuntimeError(f'Simulation clock stalled during {operation}')

    def finish_trajectory(self, handle, duration):
        future = handle.get_result_async()
        self.wait_simulation(future.done, duration+5., 'Arm/gripper trajectory')
        return future.result()

    def run_for(self, seconds, *, publish):
        if publish:
            raise RuntimeError('The carry task may only drive through Nav2')
        end = self.now()+seconds
        self.wait_simulation(lambda: self.now() >= end, seconds+1., self.phase)
        assert self.pose is not None and time.monotonic()-self.pose_stamp < 3., 'Ground-truth pose missing/stale'
        return self.pose.copy()

    def base_result(self, handle):
        future = handle.get_result_async()

        def result_ready():
            if self.payload_problem:
                raise RuntimeError(self.payload_problem)
            if self.mode != 'navigation':
                raise RuntimeError('Navigation was interrupted')
            return future.done()

        self.wait_simulation(result_ready, 45., 'Nav2 action')
        return future.result()

    def check_load(self, report, phase):
        samples = [s for s in self.trace if s['phase'] == phase and 'payload_shift_m' in s]
        if not samples:
            raise RuntimeError(f'No tool-relative cube measurements during {phase}')
        stats = {
            'samples': len(samples),
            'max_shift_m': max(s['payload_shift_m'] for s in samples),
            'dual_contact_fraction': sum(s['state'] == 'both' for s in samples)/len(samples),
            'cube_contacts': sorted(self.cube_contacts.get(phase, ())),
        }
        report['payload'][phase] = stats
        report['checks'][phase+'_retained'] = (self.payload_problem is None
                                              and stats['max_shift_m'] <= MAX_SHIFT
                                              and stats['dual_contact_fraction'] >= .9
                                              and samples[-1]['state'] == 'both')
        if not report['checks'][phase+'_retained']:
            raise RuntimeError(self.payload_problem or f'Cube did not stay seated during {phase}')

    def carry_action(self, report, phase, client, goal):
        self.phase = phase
        row = report['route'][phase] = {'start_s': self.now(), 'start_world': self.pose.copy()}
        row.update(self.run_base_action(client, goal, phase))
        if row['status'] != GoalStatus.STATUS_SUCCEEDED or self.mode != 'navigation':
            self.stop_base()
            raise RuntimeError(f'Nav2 failed {phase} with status {row["status"]}')
        self.run_for(.35, publish=False)
        self.check_load(report, phase)
        return row

    def stop_base(self):
        self.stop.publish(String(data='stop'))
        self.spin_for(.35)

    def start_carry(self):
        self.wait(lambda: self.backup.server_is_ready() and self.spin.server_is_ready()
                  and self.enable_nav.service_is_ready() and self.speed_limit.get_subscription_count(),
                  30, 'Loaded Nav2 actions or speed-limit interface unavailable')
        speed = SpeedLimit(percentage=False, speed_limit=CARRY_SPEED)
        speed.header.stamp = self.get_clock().now().to_msg()
        self.speed_limit.publish(speed)
        self.monitor_motion = False
        response = self.wait_future(self.enable_nav.call_async(Trigger.Request()), 12)
        if not response.success:
            raise RuntimeError(response.message)
        self.wait(lambda: self.mode == 'navigation', 10, 'Arbiter did not enable loaded navigation')

    def drive_loaded(self, report):
        self.start_carry()
        try:
            back = BackUp.Goal()
            back.target.x = -RETREAT
            back.speed = .030
            back.time_allowance.sec = 18
            straight = self.carry_action(report, 'carry_straight', self.backup, back)
            straight['distance_m'] = math.dist(straight['start_world'][:2], straight['result_world'][:2])
            straight['yaw_change_rad'] = abs(wrap(straight['result_world'][5]-straight['start_world'][5]))
            report['checks']['loaded_straight_motion'] = (straight['distance_m'] >= .10
                                                          and straight['yaw_change_rad'] < .035)
            if not report['checks']['loaded_straight_motion']:
                raise RuntimeError('Straight loaded retreat did not meet the route check')

            turn = Spin.Goal()
            turn.target_yaw = wrap(DROP_DOCK[2]-self.odom_pose()[2])
            turn.time_allowance.sec = 23
            turning = self.carry_action(report, 'carry_turn', self.spin, turn)
            turning['yaw_change_rad'] = wrap(turning['result_world'][5]-turning['start_world'][5])
            turning['odom_yaw_change_rad'] = wrap(turning['result_odom'][2]-straight['result_odom'][2])
            self.finish_delivery(report)
        except (RuntimeError, TimeoutError, AssertionError, KeyboardInterrupt):
            self.stop_base()
            raise

    def station_goal(self):
        """Register the local odom frame to the known lab fixture using Gazebo base pose."""
        if time.monotonic()-self.pose_stamp > 2.:
            raise RuntimeError('Simulation base localization is unavailable')
        x, y, heading = self.odom_pose()
        wx, wy, _, _, _, world_heading = self.pose
        bias = wrap(heading-world_heading)
        dx, dy = DROP_DOCK[0]-wx, DROP_DOCK[1]-wy
        c, s = math.cos(bias), math.sin(bias)
        return (x+c*dx-s*dy, y+s*dx+c*dy, wrap(DROP_DOCK[2]+bias))

    def finish_delivery(self, report):
        # A lifted cube remains watched through both the correction and the final drive.
        residual = wrap(DROP_DOCK[2]-self.pose[5])
        if abs(residual) > .025:
            correction = Spin.Goal()
            correction.target_yaw = residual
            correction.time_allowance.sec = 10
            self.carry_action(report, 'carry_turn_trim', self.spin, correction)
        report['checks']['loaded_turn_motion'] = abs(wrap(DROP_DOCK[2]-self.pose[5])) < .040
        if not report['checks']['loaded_turn_motion']:
            raise RuntimeError('Loaded turn did not reach the physical station heading')

        self.phase = 'carry_to_place'
        final = report['route']['carry_to_place'] = {}
        self.navigate_to(final, self.station_goal())
        self.settle(final)
        final.update(world_goal=DROP_DOCK,
                     world_position_error_m=math.dist(final['settled_world'][:2], DROP_DOCK[:2]),
                     world_yaw_error_rad=abs(wrap(final['settled_world'][5]-DROP_DOCK[2])))
        self.check_load(report, self.phase)
        report['checks']['loaded_arrival'] = (
            math.dist(final['start_world'][:2], final['settled_world'][:2]) > .20
            and final['world_position_error_m'] < .012 and final['world_yaw_error_rad'] < .040)
        if not report['checks']['loaded_arrival']:
            raise RuntimeError('Loaded arrival was outside the placement station tolerances')

    def place(self, report):
        x, y, _, _, _, theta = self.pose
        arm = self.tf.lookup_transform('base_footprint', 'arm_base_link', Time()).transform.translation
        dx, dy = DROP_CENTER[0]-x, DROP_CENTER[1]-y
        height = REST_HEIGHT-WORKCELLS['base_footprint_rest_z_m']-arm.z
        center = [math.cos(theta)*dx+math.sin(theta)*dy-arm.x,
                  -math.sin(theta)*dx+math.cos(theta)*dy-arm.y, height]
        if not (.262 <= center[0] <= .292 and abs(center[1]) <= .030):
            raise RuntimeError(f'Placement center is outside expected robot-relative bounds: {center}')
        above = solve_above(center, .030)
        drop = solve_above(center, .010)
        report['placement'] = {'target_arm_m': center, 'above_rad': above, 'release_rad': drop}

        self.goal('place_above', 'arm', above, 4)
        self.check_load(report, 'place_above')
        self.goal('place_lower', 'arm', drop, 3)
        self.check_load(report, 'place_lower')
        held_height = self.trace[-1]['cube_xyz_m'][2]
        self.payload_reference = None
        self.goal('place_release', 'gripper', [.015, .015], 2)
        landed = self.pause('placed_stable', 1.4)
        end = landed[-1]['cube_xyz_m']
        tails = [r for r in landed if r['time_s'] >= landed[-1]['time_s']-.5]
        drift = max(math.dist(end, r['cube_xyz_m']) for r in tails)
        contact = self.now()-self.recent_drop_contact < .25
        report['placement'].update(cube_final_world_m=end, fall_m=held_height-end[2],
                                   settled_drift_m=drift, support_contact=contact,
                                   finger_contact_states=sorted({r['state'] for r in tails}))
        report['checks']['placed_and_stable'] = (
            abs(end[0]-DROP_CENTER[0]) < .040 and abs(end[1]-DROP_CENTER[1]) < .030
            and abs(end[2]-REST_HEIGHT) < .002 and held_height-end[2] > .004
            and drift < .001 and contact and all(r['state'] == 'none' for r in tails)
            and self.stationary() and not self.unexpected_motion)

    def trial(self, report):
        report.update(payload={}, route={})
        try:
            self.pickup_and_place(report)
        finally:
            report['cube_contact_pairs'] = {phase: sorted(pairs) for phase, pairs in self.cube_contacts.items()}

    def pickup_and_place(self, report):
        super().trial(report)
        if not all(report['checks'].values()):
            raise RuntimeError('The M4.2 pickup did not satisfy its checks')
        stow = solve_above(STOW_TCP, 0.)
        report['stow'] = {'target_tcp_arm_m': STOW_TCP, 'joint_target_rad': stow}
        self.payload_reference = self.trace[-1]['cube_in_gripper_base_m']
        if self.payload_reference is None:
            raise RuntimeError('Missing independent held-cube reference at pickup')
        action_start = len(self.actions)
        self.goal('stow', 'arm', stow, 4)
        self.check_load(report, 'stow')
        report['stow']['joint_actual_rad'] = [self.joints[name] for name in ARM]
        pads = report['payload']['stow']['cube_contacts']
        report['checks']['stow_arm_only_with_collision_observation'] = (
            [a['group'] for a in self.actions[action_start:]] == ['arm']
            and all(any(scoped(name, pad) for name in pads) for pad in PADS))
        if not report['checks']['stow_arm_only_with_collision_observation']:
            raise RuntimeError('Stow did not establish independent cube contact evidence for each finger')
        self.drive_loaded(report)
        self.place(report)


if __name__ == '__main__':
    raise SystemExit(main(CarryPlace, __doc__, save_rgb))
