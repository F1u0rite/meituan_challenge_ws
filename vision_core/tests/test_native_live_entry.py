"""Offline LIVE wiring tests. Never construct or open a real SDK pipeline."""
from types import SimpleNamespace
from pathlib import Path
import subprocess
import pytest
from perception.dual_rate_rgbd import CaptureWorker
from scripts import live_rgbd_preview as entry
from scripts import dual_rate_runner as runner


def test_live_dispatch_and_stream_preferences(monkeypatch, tmp_path):
    from scripts import live_display
    monkeypatch.setenv('DISPLAY', ':test')
    received=[]
    monkeypatch.setattr(runner, 'run_dual_rate', lambda args: received.append(args))
    monkeypatch.setattr(live_display, 'run_live_display', lambda args: received.append(args))
    entry.main(['--live', '--preview-authorized', '--run-output', str(tmp_path/'out')])
    args=received[0]
    assert args.full_vision and args.dual_rate and not args.replay
    assert tuple(args.color_size)==(1280,720) and tuple(args.depth_size)==(640,480)
    assert args.fps==15 and not args.tcp_send
    assert args.replay_output==tmp_path/'out'


@pytest.mark.parametrize('flags', [
    ['--live'], ['--live','--preview-authorized','--replay','unused'],
    ['--live','--preview-authorized','--pair-manifest','unused'],
])
def test_live_bad_inputs_fail_before_dispatch(monkeypatch, flags):
    monkeypatch.setenv('DISPLAY', ':test')
    monkeypatch.setattr(runner, 'run_dual_rate', lambda args: pytest.fail('dispatched'))
    with pytest.raises(SystemExit) as exc:entry.main(flags)
    assert exc.value.code==2


def test_live_without_display_never_dispatches(monkeypatch):
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.setattr(runner, 'run_dual_rate', lambda args: pytest.fail('dispatched'))
    with pytest.raises(SystemExit):entry.main(['--live','--preview-authorized'])


def test_offline_alias_unchanged(monkeypatch, tmp_path):
    received=[]
    monkeypatch.setattr(runner, 'run_dual_rate', lambda args: received.append(args))
    entry.main(['--headless','--replay','unused','--full-vision','--dual-rate',
                '--replay-output',str(tmp_path/'out')])
    assert not received[0].live and received[0].replay==Path('unused')


def test_capture_callback_failure_is_reported_and_slot_closed():
    frame=SimpleNamespace(timestamps=dict(color_frame_number=1,depth_frame_number=1,
        color_sdk_ms=100.,depth_sdk_ms=100.,color_timestamp_domain='global_time',
        depth_timestamp_domain='global_time'))
    class Source:
        stopped=False
        def read(self,latest):return frame
        def stop(self):self.stopped=True
    def failed_writer(*args):raise OSError('offline injected writer error')
    source=Source();worker=CaptureWorker(source,on_frame=failed_writer)
    worker.start();worker.thread.join(2)
    assert not worker.thread.is_alive() and worker.slot.closed
    assert 'OSError' in worker.error and worker.slot.get(.01) is None
    worker.close();assert source.stopped


def test_source_start_failure_still_releases_source_and_window(monkeypatch,tmp_path):
    import perception.realtime_rgbd as core
    events=[]
    class Pipeline:
        config={}
    class CAD:
        pipeline=Pipeline();thread=None
        def __init__(self,*args):pass
    class Source:
        def __init__(self,**kw):
            assert kw==dict(preferred_color=(1280,720),preferred_depth=(640,480),fps=15)
        def start(self):raise RuntimeError('offline injected start failure')
        def stop(self):events.append('source stopped')
    monkeypatch.setattr(core,'RGBDVisionPipeline',lambda *args:Pipeline())
    monkeypatch.setattr(runner,'FastPerception',lambda:None)
    monkeypatch.setattr(runner,'CADWorker',CAD)
    monkeypatch.setattr(runner,'RealSenseRGBDSource',Source)
    monkeypatch.setattr(runner.cv2,'destroyAllWindows',lambda:events.append('windows closed'))
    args=SimpleNamespace(replay=None,replay_output=tmp_path/'out',tcp_send=False,
        vision_config=None,color_size=(1280,720),depth_size=(640,480),fps=15,headless=False)
    with pytest.raises(RuntimeError,match='start failure'):runner.run_dual_rate(args)
    assert events==['source stopped','windows closed']


def test_cleanup_failure_does_not_skip_other_resources(monkeypatch,tmp_path):
    import perception.realtime_rgbd as core
    events=[]
    class Pipeline:config={}
    class Thread:
        def is_alive(self):return True
    class CAD:
        pipeline=Pipeline();thread=Thread()
        def __init__(self,*args):pass
        def start(self):pass
        def close(self):events.append('cad closed')
    class Source:
        def __init__(self,**kw):pass
        def start(self):raise RuntimeError('start failure')
        def stop(self):
            events.append('source stopped')
            raise OSError('injected stop failure')
    monkeypatch.setattr(core,'RGBDVisionPipeline',lambda *args:Pipeline())
    monkeypatch.setattr(runner,'FastPerception',lambda:None)
    monkeypatch.setattr(runner,'CADWorker',CAD)
    monkeypatch.setattr(runner,'RealSenseRGBDSource',Source)
    monkeypatch.setattr(runner.cv2,'destroyAllWindows',lambda:events.append('windows closed'))
    args=SimpleNamespace(replay=None,replay_output=tmp_path/'out',tcp_send=False,
        vision_config=None,color_size=(1280,720),depth_size=(640,480),fps=15,headless=False)
    with pytest.raises(OSError,match='stop failure'):runner.run_dual_rate(args)
    assert events==['source stopped','cad closed','windows closed']


def test_desktop_launcher_no_display_and_help_are_camera_free():
    import os
    script=Path(__file__).resolve().parents[1]/'scripts/start_native_live_ui.sh'
    env=dict(os.environ);env.pop('DISPLAY',None)
    assert subprocess.run(['bash','-n',str(script)],check=False).returncode==0
    result=subprocess.run(['bash',str(script)],env=env,capture_output=True,text=True)
    assert result.returncode==2 and 'No DISPLAY' in result.stderr
    assert subprocess.run(['bash',str(script),'--help'],env=env,capture_output=True).returncode==0


def test_launcher_activates_existing_environment_and_passes_live_args(tmp_path):
    import os,json,sys
    venv=tmp_path/'venv';(venv/'bin').mkdir(parents=True)
    (venv/'bin/activate').write_text('export PATH="'+str(venv/'bin')+':$PATH"\n')
    # Executable argument recorder replaces Python; no UI or SDK executes.
    executable=venv/'bin/python'
    executable.write_text('#!'+sys.executable+'\nimport os,json,sys\n'
        'open(os.environ["OFFLINE_LAUNCH_CONTRACT"],"w").write(json.dumps('
        '{"argv":sys.argv[1:],"cwd":os.getcwd(),"path":os.environ["PYTHONPATH"]}))\n')
    executable.chmod(0o700)
    evidence=tmp_path/'args.json'
    env=dict(os.environ,DISPLAY=':offline-mocked',MTC_VISION_VENV=str(venv),
        MTC_VISION_OUTPUT_ROOT=str(tmp_path/'private'),OFFLINE_LAUNCH_CONTRACT=str(evidence))
    script=Path(__file__).resolve().parents[1]/'scripts/start_native_live_ui.sh'
    subprocess.run(['bash',str(script)],env=env,check=True,capture_output=True)
    record=json.loads(evidence.read_text());args=record['argv']
    assert '--live' in args and '--preview-authorized' in args
    assert args[args.index('--fps')+1]=='15'
    assert args[args.index('--color-size')+1:args.index('--color-size')+3]==['1280','720']
    assert args[args.index('--depth-size')+1:args.index('--depth-size')+3]==['640','480']
    workspace=script.parents[2]
    assert record['cwd']==str(workspace)
    assert str(workspace/'src/mtc_vision_adapter') in record['path'].split(':')
    assert args[args.index('--run-output')+1].startswith(str(tmp_path/'private'))
