"""Small process supervisor; only verified, same-user workspace sessions are stopped."""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def identity(pid):
    try:
        folder=Path('/proc')/str(pid)
        if folder.stat().st_uid != os.getuid():
            return None
        stat=(folder/'stat').read_text().rsplit(') ',1)[1].split()
        if stat[0]=='Z': return None
        return {'pid':pid,'pgid':os.getpgid(pid),'start':stat[19],
                'command':(folder/'cmdline').read_bytes().decode().split('\0')[:-1],
                'cwd':str((folder/'cwd').resolve())}
    except (OSError,ValueError):
        return None


@dataclass
class Job:
    fingerprint: dict
    process: object = None
    stopping: float = 0.
    terminated: bool = False
    result: str = 'Running'


class Jobs:
    EXPECTED={'simulation':'sim.launch.py','slam':'slam.launch.py','joy':'joy_node','teleop':'teleop.launch.py'}
    LEGACY={'simulation':'stage2_session.json','slam':'stage5_slam_session.json',
            'joy':'stage6_joy_session.json','teleop':'stage6_teleop_session.json'}

    def __init__(self,workspace):
        self.root=Path(workspace).resolve()
        self.log=self.root/'log/panel'
        self.log.mkdir(parents=True,exist_ok=True)
        self.record=self.log/'sessions.json'
        self.jobs={}
        self.results={}
        if self.record.exists():
            try:
                for name,fp in json.loads(self.record.read_text()).items():
                    if identity(fp['pid'])==fp and fp['pgid']==fp['pid'] and fp['cwd']==str(self.root):
                        self.jobs[name]=Job(fp)
            except (ValueError,KeyError,OSError):
                pass
        for name,filename in self.LEGACY.items():
            if name in self.jobs: continue
            try:
                pid=json.loads((self.root/'log'/filename).read_text())['pid']
                fp=identity(pid)
                if (fp and fp['pgid']==pid and fp['cwd']==str(self.root)
                        and self.EXPECTED[name] in fp['command'] and any('ros2' in a for a in fp['command'])):
                    self.jobs[name]=Job(fp)
            except (ValueError,KeyError,OSError):
                pass
        self.save()

    def save(self):
        tmp=self.record.with_suffix('.tmp')
        tmp.write_text(json.dumps({k:v.fingerprint for k,v in self.jobs.items()},indent=2))
        tmp.replace(self.record)

    def running(self,name):
        job=self.jobs.get(name)
        return bool(job and identity(job.fingerprint['pid'])==job.fingerprint)

    def start(self,name,command,env=None):
        if self.running(name): return False
        with (self.log/(name+'.log')).open('w') as logfile:
            proc=subprocess.Popen(command,cwd=self.root,env=env,stdin=subprocess.DEVNULL,
                                  stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
        fp=identity(proc.pid)
        if fp is None:
            proc.wait(timeout=1)
            raise RuntimeError(f'{name} exited immediately; see log/panel/{name}.log')
        self.jobs[name]=Job(fp,process=proc)
        self.results.pop(name,None)
        self.save()
        return True

    def stop(self,name):
        job=self.jobs.get(name)
        if not job or not self.running(name): return False
        if job.fingerprint['pid']!=job.fingerprint['pgid']:
            raise RuntimeError('Refusing to signal an unowned process group')
        if not job.stopping:
            os.killpg(job.fingerprint['pgid'],signal.SIGINT)
            job.stopping=time.monotonic()
            job.result='Stopping'
        return True

    def poll(self):
        changed=False
        for name,job in list(self.jobs.items()):
            code=job.process.poll() if job.process else None
            if not self.running(name):
                self.results[name]='Stopped' if job.stopping else ('PASS' if code==0 else ('Exited' if code is None else f'FAIL ({code})'))
                del self.jobs[name];changed=True
            elif job.stopping and time.monotonic()-job.stopping>8 and not job.terminated:
                # Recheck the full fingerprint immediately before escalation.
                if self.running(name): os.killpg(job.fingerprint['pgid'],signal.SIGTERM)
                job.terminated=True
            elif job.stopping and time.monotonic()-job.stopping>15:
                job.result='Stop pending; inspect log'
        if changed: self.save()


def command_for(action,workspace,joy_running=False):
    root=Path(workspace)
    run=['ros2','run','mobile_manipulator_gazebo']
    launch=['ros2','launch','mobile_manipulator_gazebo']
    if action=='simulation':
        from ament_index_python.packages import get_package_share_directory
        world=Path(get_package_share_directory('mobile_manipulator_gazebo'))/'worlds/slam_room.sdf'
        return launch+['sim.launch.py','world:='+str(world)]
    if action=='slam': return launch+['slam.launch.py']
    if action=='teleop': return launch+['teleop.launch.py','start_joy:='+str(not joy_running).lower()]
    if action=='keyboard': return ['gnome-terminal','--wait','--title=Robot keyboard','--working-directory='+str(root),'--']+run+['teleop_keyboard.py']
    if action=='arm': return run+['arm_control.py','demo','--output',str(root/'log/panel/arm_demo.json')]
    if action=='home': return run+['arm_control.py','home']
    if action=='health': return run+['system_health.py','--output',str(root/'log/panel/health.json')]
    raise ValueError(action)
