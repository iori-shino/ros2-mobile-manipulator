"""Run one isolated Gazebo presentation using the project's M5/M6 entries."""
from contextlib import ExitStack
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import yaml

ROOT = Path(__file__).resolve().parents[3]
AUTONOMY = Path(__file__).resolve().parents[1]


def layout_for(mode, case):
    layout = yaml.safe_load((AUTONOMY / 'config/workcells.yaml').read_text())
    if mode == 'detour':
        scene = layout['m5']
        scene['clear_transfer_xy'] = [[.19, .90], [-.19, .19]]
        scene['obstacle_slots_world_xy'] = [
            [1.352044, .219344], [1.130779, 1.956769], [.470515, 1.555669],
            [.20164, -1.584447], [-.17, -.61],
        ]
        scene['obstacle_jitter_m'] = 0.
        scene['drop_zones']['red']['table_world_xy'][1] = -1.2
        scene['drop_zones']['red']['dock_world'][1] = -1.2
    path = case / 'workcells.yaml'
    path.write_text(yaml.safe_dump(layout))
    return path


def gui_config(case):
    package = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    text = (package / 'config/fortress_gui.config').read_text()
    changes = {'width': '1760', 'height': '820', 'camera_pose': '0.80 -2.05 2.20 0 0.79 2.10'}
    for key, value in changes.items():
        text = re.sub(fr'<{key}>[^<]*</{key}>', f'<{key}>{value}</{key}>', text, count=1)
    path = case / 'demo_gui.config'
    path.write_text(text)
    return path


def publish_summary(case, **values):
    staging = case / 'summary.tmp'
    staging.write_text(json.dumps(values, ensure_ascii=False, indent=2) + '\n')
    staging.replace(case / 'summary.json')


def run(case, mode, seed, show_gui):
    config = layout_for(mode, case)
    os.environ.update(ROS_LOCALHOST_ONLY='1', IGN_PARTITION=f'm6_demo_{case.name}',
                      FASTDDS_BUILTIN_TRANSPORTS='UDPv4', M6_WORKCELLS=str(config),
                      M6_COLORS='red' if mode == 'detour' else 'red,blue,yellow',
                      LP_NUM_THREADS='2', PYTHONUNBUFFERED='1')
    os.environ['PYTHONPATH'] = os.pathsep.join([
        str(AUTONOMY/'scripts'), str(ROOT/'tools/acceptance'), os.environ.get('PYTHONPATH','')])

    with ExitStack() as sessions:
        def stop(process):
            if process.poll() is not None:
                return
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=18)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)

        def start(args, log):
            stream = sessions.enter_context((case/log).open('w'))
            process = subprocess.Popen(args, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            sessions.callback(stop, process)
            return process

        scene = start(['ros2', 'launch', str(AUTONOMY/'launch/m5_scene.launch.py'),
                       f'seed:={seed}', 'pick_demo:=true', 'sorting:=true',
                       'use_components:=false', 'gui:=false', f'config:={config}',
                       f'scene_dir:={case}'], 'launch.log')
        colors = os.environ['M6_COLORS'].split(',')
        for color in colors:
            start(['ign','topic','-e','-t',f'/model/m3_{color}_cube/pose'], f'{color}_truth.log')
        start(['ign','topic','-e','-t','/world/flat_world/stats'], 'stats.log')

        for _ in range(48):
            if scene.poll() is not None:
                raise RuntimeError('Gazebo 启动中止，请查看 launch.log')
            log = (case/'launch.log').read_text(errors='replace')
            if '[lifecycle_manager-' in log and 'Creating and initializing lifecycle service clients' in log:
                break
            time.sleep(5)
        else:
            raise RuntimeError('Nav2 启动未就绪，请查看 launch.log')

        gui = None
        if show_gui:
            gui = start(['ign','gazebo','-g','--render-engine','ogre',
                         '--gui-config',str(gui_config(case))], 'gui.log')
        time.sleep(8)
        if gui and gui.poll() is not None:
            raise RuntimeError('Gazebo 画面未打开，请查看 gui.log')
        task = start([sys.executable,str(AUTONOMY/'scripts/sort_cubes.py'),
                      '--output',str(case/'result.json')], 'task.log')
        deadline = time.monotonic()+10200
        while task.poll() is None:
            if scene.poll() is not None or (gui and gui.poll() is not None):
                raise RuntimeError('仿真窗口已关闭，任务中止')
            if time.monotonic() >= deadline:
                raise RuntimeError('演示超过等待时限，请查看任务日志')
            time.sleep(4)

        time.sleep(5)
        with (case/'verification.json').open('w') as log:
            verified = subprocess.run([sys.executable,str(ROOT/'tools/acceptance/m6_verify.py'),str(case)],
                                      stdout=log,stderr=subprocess.STDOUT).returncode == 0
        result = json.loads((case/'result.json').read_text())
        passed = verified and task.returncode == 0 and result.get('completed') == colors
        publish_summary(case, passed=passed, completed=result.get('completed',[]), error=result.get('error'))
        if show_gui:
            time.sleep(15)
        return 0 if passed else 1


if __name__ == '__main__':
    case = Path(sys.argv[1])
    try:
        raise SystemExit(run(case,sys.argv[2],int(sys.argv[3]),sys.argv[4]=='gui'))
    except KeyboardInterrupt:
        publish_summary(case,passed=False,stopped=True,error='用户结束了当前演示')
        raise SystemExit(130)
    except (OSError,RuntimeError,ValueError) as error:
        publish_summary(case,passed=False,error=str(error))
        raise SystemExit(1)
