"""H2 result mapping and real DDS roundtrip of explicit semantic fixtures; no SDK."""
import json,math,time
from types import SimpleNamespace
import pytest
from mtc_vision_adapter.native_rgbd import native_filter_reasons,native_array
from mtc_vision_adapter.native_result_publisher import NativeResultPublisher


def test_valid_pose_flags_rejected_before_dispatch():
    sink=NativeResultPublisher.__new__(NativeResultPublisher);sink.closed=False
    with pytest.raises(ValueError,match='INVALID'):
        sink.publish_result(dict(objects=[item('unsafe',ring_pose_valid=True)]),'offline',{})


@pytest.mark.parametrize('flags',[
    ['--preview-authorized','--headless','--ros-publish'],
    ['--live','--preview-authorized','--headless','--ros-wait-subscriber-s','-1'],
])
def test_ros_cli_validation_precedes_camera_dispatch(monkeypatch,flags):
    from scripts import live_rgbd_preview as entry,dual_rate_runner as runner
    monkeypatch.setattr(runner,'run_dual_rate',lambda args:pytest.fail('dispatched'))
    with pytest.raises(SystemExit) as exc:entry.main(flags)
    assert exc.value.code==2


def test_h2_manual_launcher_contract_without_camera(tmp_path):
    import os,sys,subprocess
    from pathlib import Path
    repo=Path(__file__).resolve().parents[3]
    script=repo/'vision_core/scripts/start_native_live_ros_ui.sh'
    env=dict(os.environ);env.pop('DISPLAY',None)
    assert subprocess.run(['bash','-n',str(script)],check=False).returncode==0
    assert subprocess.run(['bash',str(script),'--help'],env=env,capture_output=True).returncode==0
    assert subprocess.run(['bash',str(script)],env=env,capture_output=True).returncode==2
    venv=tmp_path/'venv';(venv/'bin').mkdir(parents=True)
    (venv/'bin/activate').write_text('export PATH="'+str(venv/'bin')+':$PATH"\n')
    python=venv/'bin/python';record=tmp_path/'contract.json'
    python.write_text('#!'+sys.executable+'\nimport os,json,sys\n'
        'open(os.environ["H2_CONTRACT"],"w").write(json.dumps({"argv":sys.argv[1:],"cwd":os.getcwd()}))\n')
    python.chmod(0o700)
    env.update(DISPLAY=':offline-mocked',MTC_VISION_VENV=str(venv),H2_CONTRACT=str(record))
    subprocess.run(['bash',str(script)],env=env,check=True,capture_output=True)
    data=json.loads(record.read_text())
    assert data['cwd']==str(repo)
    assert all(flag in data['argv'] for flag in ('--live','--preview-authorized','--ros-publish'))


def item(identity,color='red',**changes):
    obj=dict(object_id=identity,color=color,confidence=.8,detection_status='CONFIRMED_OBJECT',
        detection_stale=False,detection_ambiguity=False,body_pose_valid=False,ring_pose_valid=False,
        cad_stale=True,pose_ambiguity=True,body_pose_candidate=None)
    obj.update(changes);return obj


@pytest.mark.parametrize('changes,reason',[
    (dict(detection_status='AMBIGUOUS'),'DETECTION_NOT_CONFIRMED'),
    (dict(color='unknown'),'COLOR_UNKNOWN'),
    (dict(detection_stale=True),'DETECTION_STALE_OR_UNKNOWN'),
    (dict(detection_stale=None),'DETECTION_STALE_OR_UNKNOWN'),
    (dict(detection_ambiguity=True),'DETECTION_AMBIGUOUS'),
])
def test_reason_matches_unchanged_array_policy(changes,reason):
    obj=item('test',**changes)
    assert reason in native_filter_reasons(obj)
    assert native_array(dict(objects=[obj]))['detections']==[]


def test_pipeline_counters_are_separate():
    from scripts.dual_rate_runner import pipeline_diagnostics
    sync=dict(observed_id_gaps={'color':4},observed_repeats={'depth':8},
        rejected=9,reasons={'REUSED_DEPTH_FRAME':9},accepted=11)
    capture=SimpleNamespace(pair_guard=SimpleNamespace(snapshot=lambda:sync),
        observed_frames=20,sdk_dropped=7,slot=SimpleNamespace(dropped=6))
    cad=SimpleNamespace(slot=SimpleNamespace(dropped=3),completed_jobs=4,error=None)
    report=pipeline_diagnostics(capture,cad,dict(objects=[dict(historical_cad={'status':'STALE'})],cad_result_age_ms=1500))
    assert report['sdk_queue_drained']==7 and report['capture_latest_queue_replacements']==6
    assert report['sync_rejected']==9 and report['cad_latest_queue_replacements']==3
    assert report['sdk_observed_id_gaps']=={'color':4}
    assert report['historical_stale_cad_targets']==1 and report['usb_packet_loss'] is None


def test_actual_ros_roundtrip_preserves_filters_and_invalid_poses(monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    from mtc_interfaces.msg import BatteryDetectionArray
    from std_msgs.msg import String
    monkeypatch.setenv('ROS_DOMAIN_ID','213')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER',raising=False)
    monkeypatch.delenv('ROS_STATIC_PEERS',raising=False)
    context=Context();rclpy.init(context=context)
    node=Node('h2_offline_semantic_subscriber',context=context);sink=None
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    arrays=[];diagnostics=[]
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(BatteryDetectionArray,'/mtc/perception/batteries',arrays.append,qos)
    node.create_subscription(String,'/mtc/perception/native_diagnostics',lambda msg:diagnostics.append(json.loads(msg.data)),qos)
    try:
        sink=NativeResultPublisher(wait_for_subscriber_s=10)
        objects=[item(c,c) for c in ('red','blue','yellow','green')]+[
            item('ambiguous',detection_status='AMBIGUOUS',confirmation={'reasons':['NO_TABLE']}),
            item('stale',detection_stale=True),item('unknown','unknown')]
        result=dict(objects=objects,source_frame_id='SEMANTIC_FIXTURE_ONLY',
            timestamps=dict(color_frame_number=10,depth_frame_number=11,
                color_sdk_ms=100.,depth_sdk_ms=100.5,color_timestamp_domain='global_time',
                depth_timestamp_domain='global_time'),body_pose_valid=False,ring_pose_valid=False)
        report=sink.publish_result(result,'offline',dict(usb_packet_loss=None))
        deadline=time.monotonic()+10
        while (not arrays or not diagnostics) and time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
        assert len(arrays)==len(diagnostics)==1
        assert len(arrays[0].detections)==4 and report['filtered']==3
        assert [d.object_id for d in arrays[0].detections]==['red','blue','yellow','green']
        assert [d.color for d in arrays[0].detections]==[1,2,3,4]
        assert arrays[0].header.stamp.sec==arrays[0].header.stamp.nanosec==0
        for d in arrays[0].detections:
            assert d.confidence==pytest.approx(.8) and d.ring_pose_valid is False
            for pose in (d.body_pose,d.ring_pose):
                assert all(math.isnan(v) for v in (pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,
                    pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))
        assert diagnostics[0]['result']==result and not diagnostics[0]['actual_camera_started']
        assert diagnostics[0]['filter_report'][4]['confirmation']=={'reasons':['NO_TABLE']}
        assert 'DETECTION_NOT_CONFIRMED' in diagnostics[0]['filter_report'][4]['filter_reasons']
        assert sink.snapshot()['published_targets']==4
        sink.close();sink.close()
    finally:
        try:
            if sink:sink.close()
        finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown(context=context)
