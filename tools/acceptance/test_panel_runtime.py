"""Bounded ownership tests; no ROS graph or robot motion required."""
import json
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace
import pytest
from panel_runtime import Jobs, command_for, identity


def wait_stopped(jobs,name):
    # Keep the supervisor tick running, as Tk does. A newly started Python/ROS
    # process can still be initializing when SIGINT arrives; poll escalates.
    deadline=time.monotonic()+13
    while jobs.running(name) and time.monotonic()<deadline:
        jobs.poll();time.sleep(.02)
    jobs.poll()
    assert not jobs.running(name)


def test_own_job_start_stop(tmp_path):
    jobs=Jobs(tmp_path)
    assert jobs.start('test',[sys.executable,'-c','import time; time.sleep(60)'])
    assert not jobs.start('test',[sys.executable,'-c','raise SystemExit(99)'])
    pid=jobs.jobs['test'].fingerprint['pid']
    assert jobs.running('test')
    assert jobs.stop('test')
    wait_stopped(jobs,'test')
    assert not identity(pid)
    assert jobs.results['test']=='Stopped'


def test_stale_record_never_signals(tmp_path):
    jobs=Jobs(tmp_path)
    jobs.start('test',[sys.executable,'-c','import time; time.sleep(60)'])
    actual=jobs.jobs['test'].fingerprint.copy()
    try:
        jobs.jobs['test'].fingerprint['start']='0'
        assert not jobs.stop('test')
        assert identity(actual['pid']) is not None
    finally:
        jobs.jobs['test'].fingerprint=actual
        jobs.stop('test')
        wait_stopped(jobs,'test')


def test_unrelated_session_not_adopted(tmp_path):
    (tmp_path/'log').mkdir()
    (tmp_path/'log/stage2_session.json').write_text(json.dumps({'pid':os.getpid()}))
    assert not Jobs(tmp_path).jobs


def test_owner_persists_between_panel_instances(tmp_path):
    first=Jobs(tmp_path)
    first.start('test',[sys.executable,'-c','import time; time.sleep(60)'])
    second=Jobs(tmp_path)
    try:
        assert second.running('test')
        assert second.stop('test')
        wait_stopped(second,'test')
        first.jobs['test'].process.wait(timeout=3)
    finally:
        if first.running('test'):
            first.stop('test');wait_stopped(first,'test')


def test_command_entries_use_existing_interfaces(tmp_path):
    assert command_for('teleop',tmp_path,True)[-1]=='start_joy:=false'
    assert command_for('teleop',tmp_path,False)[-1]=='start_joy:=true'
    assert 'arm_control.py' in command_for('arm',tmp_path)
    assert 'system_health.py' in command_for('health',tmp_path)
    assert command_for('keyboard',tmp_path)[-1]=='teleop_keyboard.py'


def test_panel_stop_cascade_preserves_unrelated_process(tmp_path):
    from robot_panel import PanelBackend
    jobs=Jobs(tmp_path)
    for name in ['simulation','slam','teleop','unrelated']:
        jobs.start(name,[sys.executable,'-c','import time; time.sleep(60)'])
    stopped=[]
    fake=SimpleNamespace(nodes={'cmd_vel_guard'},jobs=jobs,stop_commands=lambda:stopped.append(True))
    try:
        PanelBackend.stop_simulation(fake)
        assert stopped==[True]
        for name in ['simulation','slam','teleop']:
            wait_stopped(jobs,name)
        assert jobs.running('unrelated')
    finally:
        for name,job in list(jobs.jobs.items()):
            jobs.stop(name)
            wait_stopped(jobs,name)


def test_panel_refuses_external_simulation_teardown(tmp_path):
    from robot_panel import PanelBackend
    fake=SimpleNamespace(nodes={'cmd_vel_guard'},jobs=Jobs(tmp_path),stop_commands=lambda:None)
    with pytest.raises(RuntimeError,match='outside a verified session'):
        PanelBackend.stop_simulation(fake)
