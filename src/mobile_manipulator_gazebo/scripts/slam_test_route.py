#!/usr/bin/env python3
"""Explicit fixed test route in slam_room only; no planning or exploration."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from acceptance_test import Probe, angles, wrap


class MappingProbe(Probe):
    def __init__(self):
        super().__init__()
        self.maps = []
        self.grid = None
        self.scan = None
        self.scan_wall = 0.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.extra_subs = [
            self.create_subscription(OccupancyGrid, '/map', self.on_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)]
        self.errors = []
        self.last_report = 0.

    def on_scan(self, msg):
        self.scan = msg
        self.scan_wall = time.monotonic()

    def on_map(self, msg):
        self.grid = msg
        self.maps.append({'sim_time': self.now(), 'width': msg.info.width, 'height': msg.info.height,
            'known_cells': sum(v >= 0 for v in msg.data), 'occupied_cells': sum(v > 50 for v in msg.data),
            'sha256': hashlib.sha256(msg.data.tobytes()).hexdigest()})

    def sample_error(self):
        try:
            t = self.buffer.lookup_transform('map', 'base_link', Time()).transform
        except Exception:
            return
        self.errors.append({'sim_time': self.now(),
            'position_m': math.hypot(t.translation.x-self.pose[0], t.translation.y-self.pose[1]),
            'heading_rad': abs(wrap(angles(t.rotation)[2]-self.pose[5]))})

    def step(self, linear=0., angular=0.):
        assert self.pose is not None and time.monotonic()-self.pose_stamp < 4., 'Physical pose stale'
        assert self.scan is not None and time.monotonic()-self.scan_wall < 5., 'Scan stale'
        assert -.4 < self.pose[0] < .9 and -.9 < self.pose[1] < .4, 'Left the designated clear test area'
        assert abs(linear) <= .05 and abs(angular) <= .4
        cmd=Twist();cmd.linear.x=linear;cmd.angular.z=angular
        self.publisher.publish(cmd)
        rclpy.spin_once(self, timeout_sec=.03)
        if time.monotonic()-self.last_report > 2.:
            self.sample_error()
            self.last_report=time.monotonic()

    def straight(self, distance, heading):
        start=self.pose.copy();deadline=time.monotonic()+300
        while True:
            travel=(self.pose[0]-start[0])*math.cos(heading)+(self.pose[1]-start[1])*math.sin(heading)
            if travel >= distance-.005: break
            assert time.monotonic()<deadline, 'Straight segment timeout'
            speed=min(.05,max(.015,1.2*(distance-travel)))
            self.step(speed,max(-.10,min(.10,1.2*wrap(heading-self.pose[5]))))
        end=self.run_for(.6)
        return {'start':start,'end':end,'distance_m':math.dist(start[:2],end[:2])}

    def turn_to(self, target):
        start=self.pose.copy();deadline=time.monotonic()+240
        while abs(wrap(target-self.pose[5])) > .02:
            assert time.monotonic()<deadline, 'Turn timeout'
            error=wrap(target-self.pose[5])
            speed=math.copysign(min(.25,max(.055,1.1*abs(error))),error)
            self.step(0.,speed)
        end=self.run_for(.6)
        return {'start':start,'end':end,'yaw_rad':wrap(end[5]-start[5])}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path.home()/'robot_ws/log/stage5_route.json')
    args=parser.parse_args()
    rclpy.init();n=MappingProbe();report={'passed':False,'segments':[]}
    try:
        deadline=time.monotonic()+90
        while n.pose is None or n.odom is None or n.scan is None or n.grid is None:
            assert time.monotonic()<deadline,'Missing map, scan, odom or physical pose'
            rclpy.spin_once(n,timeout_sec=.1)
        assert math.hypot(*n.pose[:2])<.15 and abs(n.pose[5])<.12,'Start a fresh slam_room simulation at the origin'
        assert n.grid.header.frame_id=='map'
        n.run_for(1.)
        initial=n.pose.copy()
        report['initial_map']=n.maps[-1]
        report['initial_pose']=initial
        print(json.dumps({'event':'route_start','initial_map':report['initial_map']}),flush=True)
        for side in range(4):
            heading=-side*math.pi/2
            item=n.straight(.5,heading);item['command']='forward';report['segments'].append(item)
            print(json.dumps({'event':'straight_done','side':side+1,**item}),flush=True)
            item=n.turn_to(-(side+1)*math.pi/2);item['command']='right';report['segments'].append(item)
            print(json.dumps({'event':'turn_done','side':side+1,**item}),flush=True)
        n.run_for(1.)
        report['loop_endpoint_error_m']=math.dist(initial[:2],n.pose[:2])
        report['loop_heading_error_rad']=abs(wrap(n.pose[5]-initial[5]))
        assert report['loop_endpoint_error_m']<.12 and report['loop_heading_error_rad']<.10
        # Short /cmd_vel smoke, measured using Gazebo's physical model pose.
        report['base_smoke']=[]
        for label,v in [('forward',.04),('backward',-.04)]:
            a=n.pose.copy();n.run_for(2.,v);b=n.run_for(.6)
            forward=(b[0]-a[0])*math.cos(a[5])+(b[1]-a[1])*math.sin(a[5])
            assert forward*v>0 and .04<abs(forward)<.12,(label,forward)
            report['base_smoke'].append({'command':label,'physical_distance_m':forward})
        # No publisher input: the existing real-time guard must still stop.
        n.run_for(.5,.04)
        n.run_for(1.,publish=False)
        a=n.pose.copy();b=n.run_for(1.,publish=False)
        report['watchdog_rest_motion_m']=math.dist(a[:2],b[:2])
        assert report['watchdog_rest_motion_m']<.005
        n.run_for(1.)
        n.sample_error()
        report['map_updates']=n.maps
        report['final_map']=n.maps[-1]
        report['unique_maps']=len({m['sha256'] for m in n.maps})
        report['known_cells_growth']=max(m['known_cells'] for m in n.maps)-report['initial_map']['known_cells']
        report['slam_pose_errors']=n.errors
        report['max_slam_position_error_m']=max(e['position_m'] for e in n.errors)
        report['max_slam_heading_error_rad']=max(e['heading_rad'] for e in n.errors)
        report['max_tilt_rad']=max(max(abs(p[3]),abs(p[4])) for p in n.samples)
        report['joints']=n.joints
        assert report['unique_maps']>=5 and report['known_cells_growth']>20,'Map did not extend'
        assert report['max_slam_position_error_m']<.30 and report['max_slam_heading_error_rad']<.35,'SLAM pose diverged'
        assert report['max_tilt_rad']<.05
        assert all(name in n.joints for name in ['J1','J2','J3','J4','left_jaw_joint','right_jaw_joint'])
        report['passed']=True
        print(json.dumps({k:v for k,v in report.items() if k not in ['map_updates','slam_pose_errors','segments']}),flush=True)
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        for _ in range(10):
            n.publisher.publish(Twist());rclpy.spin_once(n,timeout_sec=.03)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
