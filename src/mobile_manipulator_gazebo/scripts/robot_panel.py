#!/usr/bin/env python3
"""Tkinter launcher for existing ROS interfaces; CLI operation is independent."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import time
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from action_msgs.srv import CancelGoal
from panel_runtime import Jobs, command_for


class PanelBackend(Node):
    def __init__(self,workspace):
        super().__init__('robot_panel')
        self.jobs=Jobs(workspace)
        self.root=self.jobs.root
        self.message='Ready. Existing sessions are detected; closing this panel leaves them running.'
        self.stop_until=0.
        self.key=self.create_publisher(String,'/teleop/key',10)
        self.vel=self.create_publisher(Twist,'/cmd_vel',10)
        self.cancels=[self.create_client(CancelGoal,f'/{name}_controller/follow_joint_trajectory/_action/cancel_goal') for name in ['arm','gripper']]
        self.pending=[]
        self.nodes=set()

    def refresh(self):
        self.jobs.poll()
        self.nodes={name for name,_ in self.get_node_names_and_namespaces()}
        if time.monotonic()<self.stop_until:
            self.key.publish(String(data='stop'));self.vel.publish(Twist())
        for name,future in list(self.pending):
            if future.done():
                self.pending.remove((name,future))
                try:
                    result=future.result()
                    if result is None: self.message='Stop sent; action cancellation returned no result.'
                except Exception as exc:
                    self.message='Stop sent; action cancellation error: '+str(exc)

    def status(self):
        required={'simulation':'cmd_vel_guard','slam':'slam_toolbox','joy':'joy_node','teleop':'teleop_router','keyboard':'teleop_keyboard'}
        state={}
        for name,node in required.items():
            state[name]='Running' if node in self.nodes else 'Stopped'
            if node in self.nodes and not self.jobs.running(name): state[name]+=' (external)'
            if name in self.jobs.jobs: state[name]=self.jobs.jobs[name].result
        for name in ['arm','home','health','save_map']:
            state[name]=self.jobs.jobs[name].result if name in self.jobs.jobs else self.jobs.results.get(name,'Idle')
        return state

    def start(self,action):
        required={'simulation':'cmd_vel_guard','slam':'slam_toolbox','teleop':'teleop_router','keyboard':'teleop_keyboard'}
        if action in required and required[action] in self.nodes:
            self.message=f'{action} is already running; no duplicate was launched.';return False
        if action in ['arm','home','slam'] and 'cmd_vel_guard' not in self.nodes:
            raise RuntimeError('Start simulation first.')
        if action in ['arm','home'] and (self.jobs.running('arm') or self.jobs.running('home')):
            raise RuntimeError('An arm operation is already running.')
        if action=='keyboard':
            if not shutil.which('gnome-terminal'):
                raise RuntimeError('Open a terminal and run: ros2 run mobile_manipulator_gazebo teleop_keyboard.py')
            if 'teleop_router' not in self.nodes and not self.jobs.running('teleop'):
                self.jobs.start('teleop',command_for('teleop',self.root,'joy_node' in self.nodes))
        if action in ['arm','home']:
            self.stop_commands(cancel_arm=False)
        launched=self.jobs.start(action,command_for(action,self.root,'joy_node' in self.nodes))
        self.message=f'{action}: '+('started; output in log/panel/' if launched else 'already starting/running')
        return launched

    def save_map(self,name):
        if not re.fullmatch(r'[A-Za-z0-9_-]+',name):
            raise ValueError('Use letters, digits, underscore or hyphen for the map name.')
        if 'slam_toolbox' not in self.nodes: raise RuntimeError('Start SLAM first.')
        prefix=self.root/'maps/generated'/name
        launched=self.jobs.start('save_map',['ros2','run','mobile_manipulator_gazebo','save_slam_map.py',str(prefix)])
        self.message='Saving map through the existing SLAM service.' if launched else 'A map save is already running.'
        return launched

    def stop_commands(self,cancel_arm=True):
        self.stop_until=time.monotonic()+.3
        self.key.publish(String(data='stop'));self.vel.publish(Twist())
        if cancel_arm:
            self.jobs.stop('arm');self.jobs.stop('home')
            for client in self.cancels:
                if client.service_is_ready():
                    self.pending.append((client.srv_name,client.call_async(CancelGoal.Request())))
        self.message='STOP sent through /cmd_vel; teleop disarmed; arm goals cancellation requested. New commands can resume motion.'

    def stop_simulation(self):
        if 'cmd_vel_guard' in self.nodes and not self.jobs.running('simulation'):
            raise RuntimeError('Simulation was started outside a verified session. Stop it in its own terminal.')
        self.stop_commands()
        for name in ['slam','teleop','simulation']:
            self.jobs.stop(name)
        self.message='Stopping verified simulation/SLAM/teleop sessions. CLI tools remain available.'


class Panel:
    def __init__(self,window,backend):
        self.window=window;self.backend=backend
        window.title('Mobile manipulator — ROS 2 panel')
        window.geometry('700x485')
        frame=ttk.Frame(window,padding=14);frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='ROS 2 mobile manipulator',font=('Sans',15,'bold')).pack(anchor='w')
        ttk.Label(frame,text='Gazebo simulation · existing CLI interfaces · software stop only').pack(anchor='w',pady=(3,12))
        controls=ttk.Frame(frame);controls.pack(fill='x')
        self.buttons={}
        entries=[('Start simulation',lambda:backend.start('simulation')),('Stop simulation',backend.stop_simulation),
                 ('Keyboard teleop',lambda:backend.start('keyboard')),('Gamepad teleop',lambda:backend.start('teleop')),
                 ('Arm demo',lambda:backend.start('arm')),('Arm / gripper home',lambda:backend.start('home')),
                 ('Health check',lambda:backend.start('health')),('Start SLAM',lambda:backend.start('slam'))]
        for i,(label,callback) in enumerate(entries):
            button=ttk.Button(controls,text=label,command=lambda cb=callback:self.call(cb))
            button.grid(row=i//2,column=i%2,sticky='ew',padx=3,pady=3);self.buttons[label]=button
        controls.columnconfigure(0,weight=1);controls.columnconfigure(1,weight=1)
        maps=ttk.Frame(frame);maps.pack(fill='x',pady=8)
        self.map_name=tk.StringVar(value=datetime.now().strftime('map_%Y%m%d_%H%M%S'))
        ttk.Entry(maps,textvariable=self.map_name).pack(side='left',fill='x',expand=True)
        self.buttons['Save map']=ttk.Button(maps,text='Save map',command=lambda:self.call(lambda:backend.save_map(self.map_name.get())))
        self.buttons['Save map'].pack(side='left',padx=(6,0))
        self.buttons['STOP']=tk.Button(frame,text='STOP — base + cancel arm',bg='#b3261e',fg='white',font=('Sans',12,'bold'),command=lambda:self.call(backend.stop_commands))
        self.buttons['STOP'].pack(fill='x',pady=(0,10))
        self.states=tk.StringVar();ttk.Label(frame,textvariable=self.states,justify='left').pack(anchor='w')
        self.message=tk.StringVar();ttk.Label(frame,textvariable=self.message,wraplength=665,justify='left').pack(anchor='w',pady=10)
        self.closed=False
        window.protocol('WM_DELETE_WINDOW',self.close)
        self.tick()

    def call(self,callback):
        try: callback()
        except Exception as exc: self.backend.message='FAIL: '+str(exc)

    def tick(self):
        if self.closed: return
        if not rclpy.ok(): self.close();return
        rclpy.spin_once(self.backend,timeout_sec=0)
        self.backend.refresh()
        state=self.backend.status()
        self.states.set(' | '.join(f'{k}: {state[k]}' for k in ['simulation','slam','teleop'])+'\n'+
                        ' | '.join(f'{k}: {state[k]}' for k in ['joy','keyboard'])+'\n'+
                        ' | '.join(f'{k}: {state[k]}' for k in ['arm','home','health','save_map']))
        self.message.set(self.backend.message)
        self.window.after(50,self.tick)

    def close(self):
        self.closed=True
        # Child sessions use separate process groups and survive GUI close/crash.
        self.window.destroy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path.cwd(),help='Built workspace; defaults to current directory')
    args=parser.parse_args()
    if not (args.workspace/'install/setup.bash').exists():
        parser.error('Choose a built workspace with --workspace, or run from its root.')
    rclpy.init()
    backend=None
    try:
        window=tk.Tk()
        backend=PanelBackend(args.workspace)
        Panel(window,backend)
        window.mainloop()
    except tk.TclError as exc:
        raise SystemExit('GUI display unavailable; use the documented CLI commands. '+str(exc))
    finally:
        if backend: backend.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
