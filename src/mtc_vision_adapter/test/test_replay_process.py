"""Acceptance tests launch the installed replay_publisher, without mocks."""
import json
import os
from pathlib import Path
import pytest
from ros_replay_validation import run_validation


@pytest.mark.parametrize('mode', ['synthetic', 'replay'])
@pytest.mark.parametrize('exit_mode', ['natural', 'sigint'])
def test_installed_replay_process(mode, exit_mode, tmp_path, monkeypatch):
    # Different local domains avoid stale discovery between short-lived cases.
    domain = 191 + (mode == 'replay') + 2*(exit_mode == 'sigint')
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    monkeypatch.delenv('ROS_STATIC_PEERS', raising=False)
    root = Path(__file__).resolve().parents[1]
    records = [json.loads(line) for line in (root/'examples/synthetic.jsonl').read_text().splitlines()]
    for record in records:
        record['mode'] = mode
        if mode == 'replay':
            record['timestamp_source'] = 'recorded_fixture'
    fixture = tmp_path/'replay.jsonl'
    fixture.write_text('\n'.join(json.dumps(record) for record in records)+'\n')
    out = Path(os.environ.get('MTC_VALIDATION_OUT', str(tmp_path/'results')))/(mode+'-'+exit_mode)
    result = run_validation(fixture, out, exit_mode)
    assert result['passed'] and result['publisher_exit'] == 0
    assert result['received_frames'] == (4 if exit_mode == 'natural' else 2)
