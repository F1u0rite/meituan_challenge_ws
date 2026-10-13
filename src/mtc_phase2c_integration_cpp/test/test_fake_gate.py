"""Offline supervisor contract tests: no controller, robot or motion is started."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
from unittest.mock import patch
import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, EmitEvent
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events.process import SignalProcess

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gate = load('run_fake_gate')
recorder = load('launch_recorder')
ENVIRONMENT = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
               'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'ROS_DOMAIN_ID': '220'}


def test_reviewed_environment_allowed(tmp_path):
    gate.validate_environment(ENVIRONMENT, tmp_path)


@pytest.mark.parametrize('key,value', [
    ('ROS_DISCOVERY_SERVER', '127.0.0.1:11811'), ('ROS_DISCOVERY_SERVER', ''),
    ('ROS_SUPER_CLIENT', 'TRUE'), ('ROS_SUPER_CLIENT', 'false'),
    ('FASTDDS_ENVIRONMENT_FILE', '/tmp/unverified.json'),
    ('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
    ('FASTDDS_DEFAULT_PROFILES_FILE', '/tmp/unverified.xml'),
    ('FASTRTPS_DEFAULT_PROFILES_FILE', '/tmp/unverified.xml'),
    ('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'), ('RMW_FASTRTPS_PUBLICATION_MODE', 'ASYNCHRONOUS'),
    ('FASTDDS_UNREVIEWED_DISCOVERY', '1'), ('ROS2_EASY_MODE', '127.0.0.1'),
    ('SKIP_DEFAULT_XML_FILE', '1'), ('CYCLONEDDS_URI', 'unverified'),
    ('ROS_STATIC_PEERS', ''), ('ROS_LOCALHOST_ONLY', '1'),
    ('ROS_SECURITY_ROOT_DIRECTORY', '/tmp/unverified'),
])
def test_discovery_overrides_fail_closed(tmp_path, key, value):
    with pytest.raises(ValueError):
        gate.validate_environment(dict(ENVIRONMENT, **{key: value}), tmp_path)


@pytest.mark.parametrize('middleware', [None, '', 'rmw_cyclonedds_cpp', 'rmw_fastrtps_dynamic_cpp'])
def test_unreviewed_rmw_rejected(tmp_path, middleware):
    environment = dict(ENVIRONMENT)
    if middleware is None:
        del environment['RMW_IMPLEMENTATION']
    else:
        environment['RMW_IMPLEMENTATION'] = middleware
    with pytest.raises(ValueError):
        gate.validate_environment(environment, tmp_path)


@pytest.mark.parametrize('domain', ['', '0', '233', '-1', '1junk', '1111', '２２０'])
def test_invalid_domain_rejected(tmp_path, domain):
    with pytest.raises(ValueError):
        gate.validate_environment(dict(ENVIRONMENT, ROS_DOMAIN_ID=domain), tmp_path)


@pytest.mark.parametrize('filename', ['DEFAULT_FASTDDS_PROFILES.xml', 'DEFAULT_FASTRTPS_PROFILES.xml'])
def test_implicit_profile_rejected(tmp_path, filename):
    (tmp_path / filename).write_text('not loaded by this test')
    with pytest.raises(ValueError):
        gate.validate_environment(ENVIRONMENT, tmp_path)


def test_existing_evidence_never_overwritten(tmp_path):
    evidence = tmp_path / 'existing'
    evidence.mkdir()
    sentinel = evidence / 'owner.log'
    sentinel.write_text('original evidence')
    with patch.dict(os.environ, ENVIRONMENT, clear=True), \
         patch.object(sys, 'argv', ['gate', '--mode', 'plan', '--evidence', str(evidence)]), \
         patch.object(gate.subprocess, 'Popen', side_effect=AssertionError('must not spawn')):
        assert gate.main() == 2
    assert sentinel.read_text() == 'original evidence'
    assert not (evidence / 'results.json').exists()


def test_empty_evidence_directory_also_rejected(tmp_path):
    evidence = tmp_path / 'existing'
    evidence.mkdir()
    with pytest.raises(FileExistsError):
        gate.reserve_evidence(evidence)


def test_evidence_symlink_rejected(tmp_path):
    evidence = tmp_path / 'link'
    evidence.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(FileExistsError):
        gate.reserve_evidence(evidence)


def test_missing_child_exit_is_not_clean(tmp_path):
    records = tmp_path / 'events.jsonl'
    records.write_text(json.dumps({'event': 'start', 'name': 'fake', 'pid': 1}) + '\n')
    with pytest.raises(ValueError):
        gate.read_child_events(records)


def test_sigterm_child_code_is_preserved(tmp_path):
    records = tmp_path / 'events.jsonl'
    records.write_text('\n'.join(json.dumps(event) for event in [
        {'event': 'start', 'name': 'fake', 'pid': 1},
        {'event': 'exit', 'name': 'fake', 'pid': 1, 'returncode': -15}]) + '\n')
    events, clean = gate.read_child_events(records)
    assert not clean and events[-1]['returncode'] == -15
    assert gate.overall_result('PASS', 'FAIL') == ('FAIL', 1)


@pytest.mark.parametrize('functional,teardown,expected', [
    ('PASS', 'PASS', ('PASS', 0)), ('PASS', 'FAIL', ('FAIL', 1)),
    ('FAIL', 'PASS', ('FAIL', 1)), ('FAIL', 'FAIL', ('FAIL', 1)),
    ('BLOCKED', 'NOT RUN', ('BLOCKED', 2)), ('PASS', 'NOT RUN', ('FAIL', 1)),
])
def test_overall_result_keeps_teardown_failure(functional, teardown, expected):
    assert gate.overall_result(functional, teardown) == expected


@pytest.mark.parametrize('signal_exit', [False, True])
def test_real_launch_child_exit_not_hidden_by_parent(tmp_path, signal_exit):
    records = tmp_path / 'real_launch.jsonl'
    process = ExecuteProcess(cmd=[sys.executable, '-c',
                                 'import time; time.sleep(10)' if signal_exit else 'raise SystemExit(23)'])
    with records.open('x') as stream:
        def write(kind, event, context):
            del context
            stream.write(json.dumps(recorder.event_record(kind, event)) + '\n')
            stream.flush()
            return []
        actions = [RegisterEventHandler(OnProcessStart(on_start=lambda e, c: write('start', e, c))),
                   RegisterEventHandler(OnProcessExit(on_exit=lambda e, c: write('exit', e, c))), process]
        if signal_exit:
            actions.append(TimerAction(period=0.3, actions=[EmitEvent(event=SignalProcess(
                signal_number=signal.SIGTERM, process_matcher=lambda action: action is process))]))
        service = LaunchService()
        service.include_launch_description(LaunchDescription(actions))
        # ROS Launch returns 0 even though the actual child fails: this is the finding.
        assert service.run() == 0
    events, clean = gate.read_child_events(records)
    assert not clean
    assert events[-1]['returncode'] == (-15 if signal_exit else 23)
    assert gate.overall_result('PASS', 'FAIL') == ('FAIL', 1)
