from types import SimpleNamespace
import subprocess
import pytest
import spawn_when_ready as target


def test_waits_for_world_before_creation(monkeypatch):
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(stdout='iterations: 5' if len(calls) == 1 else 'data: true', stderr='')
    monkeypatch.setattr(target.subprocess, 'run', run)
    target.spawn('flat_world', '/tmp/model.sdf')
    assert calls[0][:2] == ['ign', 'topic']
    assert calls[1][:2] == ['ign', 'service']
    assert 'allow_renaming: false' in calls[1][-1]


@pytest.mark.parametrize('response', ['', 'data: false', 'Request timed out'])
def test_zero_exit_without_creation_is_failure(monkeypatch, response):
    results = iter(['iterations: 1', response])
    monkeypatch.setattr(target.subprocess, 'run', lambda *a, **k: SimpleNamespace(stdout=next(results), stderr=''))
    with pytest.raises(RuntimeError, match='did not confirm'):
        target.spawn('flat_world', '/tmp/model.sdf')


def test_world_timeout_never_sends_create(monkeypatch):
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        raise subprocess.TimeoutExpired(command, kwargs['timeout'])
    monkeypatch.setattr(target.subprocess, 'run', run)
    with pytest.raises(subprocess.TimeoutExpired):
        target.spawn('flat_world', '/tmp/model.sdf')
    assert len(calls) == 1
