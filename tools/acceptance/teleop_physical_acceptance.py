#!/usr/bin/env python3
"""Guided physical USB input acceptance; motion comes only from the human operator."""
from collections import deque
import json
import math
from pathlib import Path
import sys
import time
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/mobile_manipulator_gazebo/scripts'))
from acceptance_test import angles, wrap


def main():
    config=yaml.safe_load((ROOT/'src/mobile_manipulator_gazebo/config/teleop_mapping.yaml').read_text())
    rclpy.init()
    node=rclpy.create_node('teleop_physical_acceptance')
    report={'pass':False,'source':'physical USB /joy, /sim/cmd_vel, /ground_truth/pose','checks':{}}
    data={}; samples=[]; release_times=[]; stop_latencies=[]; events=deque(maxlen=12000)
    out=ROOT/'validation/stage6/physical_acceptance.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    stop_pub=node.create_publisher(String,'/teleop/key',10)
    def prompt(text):
        print('\n'+text,flush=True)
        (ROOT/'validation/stage6/physical_status.json').write_text(json.dumps({'prompt':text,'checks':report['checks']},ensure_ascii=False))
    def joy(msg):
        old=data.get('joy')
        if old and old.buttons[config['enable_button']] and not msg.buttons[config['enable_button']]:
            release_times.append(time.monotonic())
        data['joy']=msg; data['joy_time']=time.monotonic()
        events.append({'t':time.monotonic(),'joy_axes':list(msg.axes),'joy_buttons':list(msg.buttons)})
    def cmd(msg):
        now=time.monotonic()
        data['cmd']=(msg.linear.x,msg.angular.z); data['cmd_time']=now
        events.append({'t':now,'sim_cmd':data['cmd']})
        if release_times and not any(data['cmd']):
            stop_latencies.append(now-release_times.pop(0))
        assert abs(msg.linear.x)<=.05001 and abs(msg.angular.z)<=.40001,'速度越限'
    def pose(msg):
        for t in msg.transforms:
            if t.child_frame_id.split('::')[-1]=='mobile_manipulator':
                p=t.transform.translation
                value=[p.x,p.y,p.z,*angles(t.transform.rotation)]
                data['pose']=value; data['pose_time']=time.monotonic(); samples.append(value)
    subscriptions=[node.create_subscription(Joy,'/joy',joy,qos_profile_sensor_data),
        node.create_subscription(Twist,'/sim/cmd_vel',cmd,10),
        node.create_subscription(TFMessage,'/ground_truth/pose',pose,10),
        node.create_subscription(Odometry,'/odom',lambda m:data.update(odom=m),qos_profile_sensor_data),
        node.create_subscription(String,'/teleop/key',lambda m:data.update(key=m.data,key_time=time.monotonic()),10)]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
    def wait(predicate,timeout=1800.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate(): return
        raise RuntimeError('等待手柄操作/数据超时，尚未通过；请重试本验收')
    def stopped():
        return data.get('cmd')==(0.,0.) and 'odom' in data and abs(data['odom'].twist.twist.linear.x)<.003 and abs(data['odom'].twist.twist.angular.z)<.03
    def enabled():
        return bool(data['joy'].buttons[config['enable_button']])
    try:
        prompt('准备：松开手柄全部按钮和摇杆。请保持 Gazebo 可见。此程序不自动驱动车辆。')
        wait(lambda: all(k in data for k in ['joy','cmd','pose','odom']),30.)
        wait(lambda: not enabled() and stopped())
        begin=time.monotonic(); spin(2.)
        neutral=[e for e in events if e['t']>=begin]
        assert len([e for e in neutral if 'joy_axes' in e])>=20,'/joy 发布不足'
        assert all(not any(e['sim_cmd']) for e in neutral if 'sim_cmd' in e),'静止有非零命令'
        report['checks']['joy_continuous_and_idle_zero']=True
        for name,label,axis,sign in [('forward','向上（前进）',0,1),('backward','向下（后退）',0,-1),('left','向左（左转）',1,1),('right','向右（右转）',1,-1)]:
            initial=list(data['pose'])
            prompt('左摇杆先回中，按住 L，再将左摇杆'+label+'；保持到出现“松开 L”提示。')
            wait(lambda: enabled() and data['cmd'][axis]*sign>(.01 if axis==0 else .08))
            def moved():
                p=data['pose']
                value=((p[0]-initial[0])*math.cos(initial[5])+(p[1]-initial[1])*math.sin(initial[5])) if axis==0 else wrap(p[5]-initial[5])
                return value*sign>(.005 if axis==0 else .015)
            wait(moved,60.)
            report['checks'][name]={'pass':True,'before':initial,'after':list(data['pose'])}
            stop_latencies.clear(); release_times.clear()
            prompt('方向已确认。现在松开 L（可以暂时保持摇杆偏转），检查车辆停止，然后摇杆回中。')
            wait(lambda: not enabled() and bool(stop_latencies) and stopped())
            latency=stop_latencies[-1]
            assert latency<.2, f'松使能至安全输出零延迟过大：{latency}'
            report['checks'][name+'_release_stop_s']=latency
            spin(.3)
        prompt('停止键检查：摇杆回中，按住 L 后向上推；车辆开始动后，在保持 L 时按 B。')
        wait(lambda: enabled() and data['cmd'][0]>.01)
        wait(lambda: data['joy'].buttons[config['stop_button']] and data['cmd']==(0.,0.))
        report['checks']['B_stop']=True
        prompt('B 停止通过。松开手柄所有按钮和摇杆，等待车辆稳定。')
        wait(lambda: not enabled() and stopped())
        prompt('键盘检查：在另一个终端运行 ros2 run mobile_manipulator_gazebo teleop_keyboard.py，按住 W，待提示后按空格。')
        since=time.monotonic()
        initial=list(data['pose'])
        wait(lambda: data.get('key')=='w' and data.get('key_time',0)>since and data['cmd'][0]>.01)
        wait(lambda: math.dist(initial[:2],data['pose'][:2])>.003,60.)
        prompt('键盘运动已确认；请在键盘控制终端按空格停止，然后按 Q 退出。')
        since=time.monotonic()
        wait(lambda: data.get('key') in [' ','stop'] and data.get('key_time',0)>since and stopped())
        report['checks']['keyboard_after_joy']=True
        report['max_tilt_rad']=max(max(abs(p[3]),abs(p[4])) for p in samples)
        report['height_range_m']=[min(p[2] for p in samples),max(p[2] for p in samples)]
        assert report['max_tilt_rad']<.05 and report['height_range_m'][1]-report['height_range_m'][0]<.01,'物理模型不稳定'
        assert time.monotonic()-data['pose_time']<3 and time.monotonic()-data['joy_time']<.5,'数据停止'
        report['checks']['gazebo_stable']=True
        report['pass']=True
        prompt('PASS：真实手柄四方向、使能释放、B 停止、键盘切换和物理稳定性通过。')
    except Exception as exc:
        report['error']=str(exc)
        prompt('FAIL / 未完成：'+str(exc))
    finally:
        for _ in range(3):
            stop_pub.publish(String(data='stop')); spin(.05)
        report['recent_events']=list(events)
        out.write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    return 0 if report['pass'] else 1


if __name__=='__main__':
    code=main()
    input('按 Enter 关闭窗口...')
    raise SystemExit(code)
