#!/usr/bin/env python3
"""Bounded arm/gripper action client and physical-state acceptance checks."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from acceptance_test import Probe

ARM=['J1','J2','J3','J4']
JAWS=['left_jaw_joint','right_jaw_joint']
LIMITS=[math.pi,math.pi/2,5*math.pi/6,5*math.pi/6]


class ArmClient(Probe):
    def __init__(self):
        super().__init__()
        self.trajectory_clients={name:ActionClient(self,FollowJointTrajectory,f'/{name}_controller/follow_joint_trajectory')
                      for name in ['arm','gripper']}
        self.joint_samples=[]

    def on_joints(self,msg):
        super().on_joints(msg)
        if all(n in self.joints for n in ARM+JAWS):
            self.joint_samples.append(self.joints.copy())

    def spin_for(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)

    def wait_future(self,future,timeout):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
        if not future.done():
            raise TimeoutError('Controller response timed out')
        return future.result()

    def ready(self):
        for client in self.trajectory_clients.values():
            if not client.wait_for_server(timeout_sec=20):
                raise RuntimeError('Arm/gripper controller is not active')
        end=time.monotonic()+15
        while not self.joint_samples or self.pose is None:
            assert time.monotonic()<end,'Physical joint/pose messages are missing'
            rclpy.spin_once(self,timeout_sec=.1)

    def move(self,group,positions):
        if group=='gripper':
            positions=[positions[0],positions[0]]
        names=ARM if group=='arm' else JAWS
        limits=LIMITS if group=='arm' else [.015,.015]
        assert len(positions)==len(names)
        assert all(math.isfinite(q) and (-b if group=='arm' else 0)<=q<=b for q,b in zip(positions,limits)), 'Target exceeds original joint limits'
        # Conservative speed, with margin for cubic interpolation.
        speed=.25 if group=='arm' else .008
        duration=max(2.,max(abs(q-self.joints[n]) for n,q in zip(names,positions))*1.6/speed)
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=names
        point=JointTrajectoryPoint(positions=[float(q) for q in positions],velocities=[0.]*len(names))
        point.time_from_start.sec=int(duration)
        point.time_from_start.nanosec=int((duration-int(duration))*1e9)
        goal.trajectory.points=[point]
        handle=self.wait_future(self.trajectory_clients[group].send_goal_async(goal),10)
        assert handle.accepted,'Trajectory rejected'
        try:
            result=self.wait_future(handle.get_result_async(),duration*15+30)
        except Exception:
            self.wait_future(handle.cancel_goal_async(),5)
            raise
        assert result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0, result.result
        self.spin_for(.3)
        tolerance=.025 if group=='arm' else .0005
        actual={n:self.joints[n] for n in names}
        assert all(abs(actual[n]-q)<tolerance for n,q in zip(names,positions)),f'Physical joint error: {actual}'
        if group=='gripper':
            actual['right_jaw_joint']=self.joints['right_jaw_joint']
            assert abs(actual['right_jaw_joint']-positions[0])<.0005,'Right jaw failed to follow'
        print(json.dumps({'target':dict(zip(names,positions)),'actual':actual}),flush=True)
        return actual


def demo(node):
    report={'passed':False,'tests':[]}
    node.move('arm',[0.]*4)
    node.move('gripper',[0.])
    node.spin_for(1)
    node.samples=[];node.joint_samples=[]
    start=node.pose.copy()
    for i,name in enumerate(ARM):
        for q in [.45,-.30,0.]:
            target=[0.]*4;target[i]=q
            actual=node.move('arm',target)
            report['tests'].append({'joint':name,'target_rad':q,'actual':actual})
    for q in [.015,0.,.0075,0.]:
        actual=node.move('gripper',[q])
        report['tests'].append({'jaw_target_m':q,'actual':actual,'gap_mm':20+1000*sum(actual.values())})
    node.spin_for(2)
    report['max_base_translation_m']=max(math.dist(start[:3],p[:3]) for p in node.samples)
    report['max_base_tilt_rad']=max(max(abs(p[3]),abs(p[4])) for p in node.samples)
    report['height_range_m']=[min(p[2] for p in node.samples),max(p[2] for p in node.samples)]
    report['max_jaw_asymmetry_m']=max(abs(s[JAWS[0]]-s[JAWS[1]]) for s in node.joint_samples)
    assert report['max_base_translation_m']<.015,report
    assert report['max_base_tilt_rad']<.05,report
    assert report['height_range_m'][1]-report['height_range_m'][0]<.01,report
    assert report['max_jaw_asymmetry_m']<.001,report
    for sample in node.joint_samples:
        assert all(-bound-.002<=sample[name]<=bound+.002 for name,bound in zip(ARM,LIMITS)),sample
        assert all(-.0002<=sample[name]<=.0152 for name in JAWS),sample
    report['passed']=True
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('command',choices=ARM+['open','close','home','demo'])
    parser.add_argument('position',nargs='?',type=float,help='Joint angle in radians')
    parser.add_argument('--output',type=Path,default=Path('log/stage3_arm_acceptance.json'))
    args=parser.parse_args()
    if args.command in ARM and args.position is None:
        parser.error('Specify a joint angle in radians')
    rclpy.init();node=ArmClient()
    try:
        node.ready()
        if args.command=='demo':
            report=demo(node)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(report,indent=2)+'\n')
            print('PASS: J1-J4, symmetric jaws, limits and chassis stability',flush=True)
        elif args.command in ARM:
            target=[node.joints[n] for n in ARM];target[ARM.index(args.command)]=args.position
            node.move('arm',target)
        elif args.command=='home':
            node.move('arm',[0.]*4);node.move('gripper',[0.])
        else:
            node.move('gripper',[.015 if args.command=='open' else 0.])
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
