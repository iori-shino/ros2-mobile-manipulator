#!/usr/bin/env python3
"""Exercise real ROS router/guard/keyboard on isolated topics; never moves the robot."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import termios
import time
import yaml
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/mobile_manipulator_gazebo/scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(ROOT / 'tools/acceptance'))
from test_teleop_core import CONFIG


def main():
    rclpy.init()
    node = rclpy.create_node('stage6_isolated_test')
    latest = {}
    records = []
    report = {'isolated_topics': '/stage6_test/*', 'checks': {}}
    processes = []
    subscriptions = []
    def recv(topic, msg):
        latest[topic] = (time.monotonic(), msg)
    for topic, typ in [('output', Twist), ('key', String)]:
        subscriptions.append(node.create_subscription(typ, '/stage6_test/'+topic, lambda m,t=topic: recv(t,m), 10))
    jp = node.create_publisher(Joy, '/stage6_test/joy', 10)
    kp = node.create_publisher(String, '/stage6_test/key', 10)
    cp = node.create_publisher(Twist, '/stage6_test/cmd', 10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
    def check(name, predicate, timeout=1.):
        end=time.monotonic()+timeout
        okay=False
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():
                okay=True; break
        report['checks'][name]=okay
        print(('PASS ' if okay else 'FAIL ')+name,flush=True)
        assert okay, name
    def output(x,z):
        if 'output' not in latest: return False
        msg=latest['output'][1]
        return abs(msg.linear.x-x)<1e-5 and abs(msg.angular.z-z)<1e-5
    def joy(x=0.,z=0.,enable=False,stop=False):
        msg=Joy(axes=[-z,x,0.,0.,0.,0.],buttons=[int(stop),0,0,0,int(enable)]+[0]*9)
        # joy_node repeats held input. A single best-effort packet is not a
        # reliable representation of a held button during DDS startup/load.
        for _ in range(4):
            jp.publish(msg)
            spin(.03)
    def start(script, remaps, params=()):
        cmd=[sys.executable,str(SCRIPTS/script),'--ros-args']
        for a,b in remaps.items(): cmd += ['-r',a+':='+b]
        for param in params: cmd += ['-p',param]
        proc=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True)
        processes.append(proc)
        return proc
    logpath=ROOT/'log/stage6_integration.log'
    logpath.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='stage6_') as temp, logpath.open('w') as log:
        config=Path(temp)/'fixture.yaml'
        config.write_text(yaml.safe_dump(CONFIG))
        try:
            router=start('teleop_router.py',{'__node':'stage6_test_router','/joy':'/stage6_test/joy',
                '/cmd_vel':'/stage6_test/cmd','/teleop/key':'/stage6_test/key','/teleop/status':'/stage6_test/status'},['mapping_file:='+str(config)])
            guard=start('cmd_vel_guard.py',{'__node':'stage6_test_guard','/cmd_vel':'/stage6_test/cmd','/sim/cmd_vel':'/stage6_test/output'})
            check('discovery',lambda: jp.get_subscription_count()==1 and kp.get_subscription_count()==2 and cp.get_subscription_count()==1,10.)
            joy(); joy(x=1)
            check('no enable zero',lambda: output(0,0))
            joy(x=1,enable=True)
            check('off-center enable rejected',lambda: output(0,0))
            joy(enable=True); joy(x=1,enable=True)
            check('enabled forward',lambda: output(.05,0))
            t=time.monotonic(); joy(x=1)
            check('release zero',lambda: output(0,0))
            report['release_observed_wall_s']=time.monotonic()-t
            joy(); joy(enable=True); joy(x=-1,z=1,enable=True)
            check('backward and left limits',lambda: output(-.05,.4))
            joy(x=1,enable=True,stop=True)
            check('stop button zero',lambda: output(0,0))
            joy(); joy(enable=True); joy(x=1,enable=True)
            check('enabled before loss',lambda: output(.05,0))
            spin(.4)
            check('joy silence stops',lambda: output(0,0))
            joy(x=1,enable=True)
            check('no automatic reconnect motion',lambda: output(0,0))
            joy()
            kp.publish(String(data='a')); spin(.08)
            check('keyboard after joy',lambda: output(0,.3))
            spin(.4)
            check('keyboard timeout',lambda: output(0,0))
            # Idle router is quiet, so the guard can be checked directly without a robot.
            command=Twist(); command.linear.x=1.; command.angular.z=-2.
            cp.publish(command); spin(.1)
            check('existing guard clamps',lambda: output(.05,-.4))
            spin(.6)
            check('existing guard stale stop',lambda: output(0,0))
            command.linear.x=float('nan'); cp.publish(command); spin(.1)
            check('existing guard rejects NaN',lambda: output(0,0))
            joy(); joy(enable=True); joy(x=1,enable=True)
            check('motion before router crash',lambda: output(.05,0))
            router.kill(); router.wait(timeout=3)
            spin(.65)
            check('router crash guard stop',lambda: output(0,0))
            master,slave=pty.openpty()
            before=termios.tcgetattr(slave)
            keyboard=subprocess.Popen([sys.executable,str(SCRIPTS/'teleop_keyboard.py'),'--ros-args',
                '-r','/teleop/key:=/stage6_test/key'],stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
            processes.append(keyboard)
            # Keyboard requires a subscriber; this test node already subscribes to key.
            def tty_ready():
                if keyboard.poll() is not None:
                    transcript=b''
                    while select.select([master],[],[],0)[0]:
                        transcript+=os.read(master,65536)
                    raise RuntimeError('Keyboard exited during startup: '+transcript.decode(errors='replace'))
                return not (termios.tcgetattr(slave)[3] & termios.ICANON)
            check('TTY ready',tty_ready,8.)
            os.write(master,b'w')
            check('TTY W event',lambda: 'key' in latest and latest['key'][1].data=='w')
            os.write(master,b' ')
            check('TTY space event',lambda: latest['key'][1].data==' ')
            os.write(master,b'q')
            check('TTY quit stop',lambda: latest['key'][1].data=='stop')
            keyboard.wait(timeout=3)
            check('TTY restored',lambda: termios.tcgetattr(slave)==before)
            os.close(master); os.close(slave)
        finally:
            for proc in processes:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try: proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill(); proc.wait(timeout=3)
            report['pass']=bool(report['checks']) and all(report['checks'].values()) and len(report['checks'])==22
            path=ROOT/'validation/stage6/teleop_integration.json'
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(report,indent=2)+'\n')
            node.destroy_node(); rclpy.shutdown()
    assert report['pass'], report


if __name__=='__main__': main()
