#!/usr/bin/env python3
"""Exercise /cmd_vel and check actual physics poses, not wheel odometry alone."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def angles(q):
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return roll,pitch,yaw


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


class Probe(Node):
    def __init__(self):
        super().__init__('fortress_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.pose=None
        self.pose_stamp=-1.
        self.odom=None
        self.joints={}
        self.samples=[]
        self.subscriptions_keep=[
            self.create_subscription(TFMessage,'/ground_truth/pose',self.on_pose,10),
            self.create_subscription(Odometry,'/odom',self.on_odom,10),
            self.create_subscription(JointState,'/joint_states',self.on_joints,10)]

    def on_pose(self,msg):
        for t in msg.transforms:
            if t.child_frame_id.split('::')[-1] != 'mobile_manipulator':
                continue
            p=t.transform.translation
            roll,pitch,yaw=angles(t.transform.rotation)
            self.pose=[p.x,p.y,p.z,roll,pitch,yaw]
            self.pose_stamp=time.monotonic()
            self.samples.append(self.pose)

    def on_odom(self,msg):
        self.odom=msg

    def on_joints(self,msg):
        self.joints=dict(zip(msg.name,msg.position))

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def run_for(self,seconds,linear=0.,angular=0.,publish=True):
        start=self.now()
        deadline=time.monotonic()+max(30.,seconds*20.)
        cmd=Twist();cmd.linear.x=linear;cmd.angular.z=angular
        while self.now()-start < seconds:
            if time.monotonic()>deadline:
                raise RuntimeError('Simulation clock is paused or too slow')
            if publish:
                self.publisher.publish(cmd)
            rclpy.spin_once(self,timeout_sec=.03)
        assert self.pose is not None and time.monotonic()-self.pose_stamp < 3., 'Ground-truth pose missing/stale'
        return self.pose.copy()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('log/stage2_acceptance.json'))
    args=parser.parse_args()
    rclpy.init()
    probe=Probe()
    report={'tests':[],'passed':False}
    try:
        deadline=time.monotonic()+30
        while probe.pose is None or probe.odom is None or not probe.joints or probe.now()==0:
            assert time.monotonic()<deadline, 'Missing /clock, /ground_truth/pose, /odom or /joint_states'
            rclpy.spin_once(probe,timeout_sec=.2)
        initial=probe.run_for(5.)
        probe.samples=[]
        resting=probe.run_for(3.)
        assert math.dist(initial[:3],resting[:3])<.005, 'Robot drifts while stationary'
        assert abs(resting[2])<.02, f'Unexpected model ground height {resting[2]}'
        report['settled_pose_xyz_rpy']=resting
        print('PASS: settled, stable at rest',resting,flush=True)
        for label,v,w in [('forward',.04,0.),('backward',-.04,0.),('left',0.,.3),('right',0.,-.3)]:
            start=probe.pose.copy()
            probe.run_for(5.,v,w)
            end=probe.run_for(2.)
            dx,dy=end[0]-start[0],end[1]-start[1]
            forward=dx*math.cos(start[5])+dy*math.sin(start[5])
            lateral=-dx*math.sin(start[5])+dy*math.cos(start[5])
            yaw=wrap(end[5]-start[5])
            item={'command':label,'actual_forward_m':forward,'actual_lateral_m':lateral,
                  'actual_yaw_rad':yaw,'height_m':end[2]}
            report['tests'].append(item)
            print(json.dumps(item),flush=True)
            if v:
                assert forward*v>0 and .10<abs(forward)<.28, f'{label}: no correct physical translation'
                assert abs(lateral)<.035 and abs(yaw)<.15, f'{label}: excess sideways drift/yaw'
            else:
                assert yaw*w>0 and .20<abs(yaw)<2.2, f'{label}: no correct physical rotation'
                assert math.hypot(dx,dy)<.07, f'{label}: excessive translation during in-place turn'
        # Stop publishing entirely: the guard must stop a previously moving base.
        probe.run_for(1.,.04,0.)
        probe.run_for(2.,publish=False)
        a=probe.pose.copy()
        b=probe.run_for(2.,publish=False)
        assert math.dist(a[:3],b[:3])<.005, 'Command watchdog did not stop robot'
        assert probe.odom is not None
        report['watchdog_stopped']=True
        report['max_abs_roll_rad']=max(abs(p[3]) for p in probe.samples)
        report['max_abs_pitch_rad']=max(abs(p[4]) for p in probe.samples)
        report['height_range_m']=[min(p[2] for p in probe.samples),max(p[2] for p in probe.samples)]
        assert max(report['max_abs_roll_rad'],report['max_abs_pitch_rad'])<.10,'Robot tilted excessively'
        assert report['height_range_m'][1]-report['height_range_m'][0]<.01,'Robot bounced or sank'
        for name in ['J1','J2','J3','J4','left_jaw_joint','right_jaw_joint']:
            assert abs(probe.joints[name])<.02, f'{name} moved despite stage-2 lock'
        report['passed']=True
        print('PASS: forward, backward, both in-place turns, stability, passive locks and watchdog',flush=True)
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        for _ in range(10):
            if rclpy.ok():
                probe.publisher.publish(Twist());rclpy.spin_once(probe,timeout_sec=.03)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
