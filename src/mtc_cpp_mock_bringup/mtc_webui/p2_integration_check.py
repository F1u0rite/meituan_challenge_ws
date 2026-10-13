"""P2 native ROS fixture only: no hardware, actual FSM, MoveIt or measured calibration.

Requires a network-isolated container. UUIDs come from real fixture Action goals.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import signal
import tempfile
import time
import unittest
from mtc_webui.p1_integration_check import P1Fixture, configuration as p1_configuration
from mtc_webui.p0_integration_check import P0Fixture, ValidationBackend
from mtc_webui.ros_integration_check import wait_for
from mtc_webui.ros_transport import RosTransport


class P2Fixture(P1Fixture):
    start_motion = P0Fixture.start_motion
    execute_fjt = P0Fixture.execute_fjt
    def __init__(self):
        self.emit_tf=False;self.trace_payload=None;self.cal_valid=False
        self.motion_id=None;self.fjt_handle=None;self.motion_state='IDLE';self.elapsed=self.duration=None
        super().__init__()
        from rclpy.action import ActionServer, ActionClient, CancelResponse
        from mtc_interfaces.action import PickPlace
        from control_msgs.action import FollowJointTrajectory
        from tf2_ros import TransformBroadcaster
        self.tf_broadcaster=TransformBroadcaster(self.node)
        self.cal=self.node.create_publisher(self.string_type,'/p2/calibration',10)
        self.trace=self.node.create_publisher(self.string_type,'/p2/trace',10)
        self.pick_server=ActionServer(self.node,PickPlace,'/mtc/manipulation/pick_place',self.execute_pick,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.pick_client=ActionClient(self.node,PickPlace,'/mtc/manipulation/pick_place',callback_group=self.group)
        self.fjt_server=ActionServer(self.node,FollowJointTrajectory,'/p2/follow_joint_trajectory',self.execute_fjt,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.fjt_client=ActionClient(self.node,FollowJointTrajectory,'/p2/follow_joint_trajectory',callback_group=self.group)
    def publish(self):
        super().publish()
        if not self.emit or not hasattr(self,'cal'):return
        stamp=self.node.get_clock().now().nanoseconds
        cal=dict(schema_version=1,stamp_ns=stamp,calibration_id='ISOLATED_FIXTURE_NOT_MEASURED',valid=self.cal_valid,
                 evidence='measured' if self.cal_valid else 'nominal',reference='synthetic test stimulus; no real calibration',
                 reason='ISOLATED FIXTURE: valid=true is an injected consumer test, NOT hardware evidence')
        m=self.string_type();m.data=json.dumps(cal);self.cal.publish(m)
        if self.trace_payload:
            m=self.string_type();m.data=json.dumps(dict(schema_version=1,stamp_ns=stamp,**self.trace_payload));self.trace.publish(m)
        if self.emit_tf:
            from geometry_msgs.msg import TransformStamped
            t=TransformStamped();t.header.stamp=self.node.get_clock().now().to_msg();t.header.frame_id='p2_base';t.child_frame_id='p2_camera';t.transform.rotation.w=1.;t.transform.translation.z=.5
            self.tf_broadcaster.sendTransform(t)
    def execute(self,handle):
        from mtc_interfaces.action import ExecuteTask,PickPlace
        self.task_id=handle.request.task_id;self.state='EXECUTE_ITEM';self.total_count=len(handle.request.ordered_colors);self.stop_confirmed=False
        parent=bytes(handle.goal_id.uuid).hex()
        self.node.get_logger().info(f'MTC_TRACE event=task_start parent={parent} task={json.dumps(self.task_id)}')
        assert self.pick_client.wait_for_server(timeout_sec=3)
        goal=PickPlace.Goal();goal.request_id=self.task_id+'/attempt-1';goal.object_id='fixture-battery-1';goal.expected_color=2;goal.target_slot='T0'
        future=self.pick_client.send_goal_async(goal);wait_for(future.done);child=future.result()
        child_id=bytes(child.goal_id.uuid).hex()
        self.node.get_logger().info(f'MTC_TRACE event=child_response parent={parent} child={child_id} accepted=1 request={json.dumps(goal.request_id)}')
        self.trace_payload=dict(link_id=self.task_id,task_id=self.task_id,execute_goal_id=parent,pick_goal_id=child_id,command_id='p2-fixture-command')
        result=child.get_result_async();cancelled=False
        while self.running and not result.done():
            f=ExecuteTask.Feedback();f.state='EXECUTE_ITEM';f.substate='TEST_LIFT';f.current_object_id='fixture-battery-1';f.current_slot='T0';f.total_count=self.total_count;handle.publish_feedback(f)
            if self.motion_id and getattr(self,'motion_pick_goal',None)==child_id:self.trace_payload['fjt_goal_id']=self.motion_id
            if handle.is_cancel_requested and not cancelled:child.cancel_goal_async();cancelled=True
            time.sleep(.1)
        r=ExecuteTask.Result();r.success=not cancelled;r.error_code=520 if cancelled else 0;r.message='Injected ABORTED/520' if cancelled else 'fixture success'
        r.completed_count=0 if cancelled else self.total_count;r.completed_slots=[] if cancelled else ['T0']
        handle.abort() if cancelled else handle.succeed();self.state='FAULT' if cancelled else 'WAIT_TASK';self.stop_confirmed=not cancelled
        return r
    def execute_pick(self,handle):
        from mtc_interfaces.action import PickPlace
        self.motion_id=None;self.motion_pick_goal=bytes(handle.goal_id.uuid).hex()
        motion=self.start_motion(5);result=motion.get_result_async();cancelled=False
        while self.running and not result.done():
            f=PickPlace.Feedback();f.substate='TEST_LIFT';f.detail='ISOLATED NATIVE PICK FIXTURE';handle.publish_feedback(f)
            if handle.is_cancel_requested and not cancelled:motion.cancel_goal_async();cancelled=True
            time.sleep(.1)
        r=PickPlace.Result();r.success=not cancelled;r.placement_verified=False;r.error_code=520 if cancelled else 0;r.message='fixture stop unconfirmed' if cancelled else 'fixture success'
        handle.abort() if cancelled else handle.succeed();return r
    def close(self):
        self.running=False
        super().close()


def configuration():
    return replace(p1_configuration(),calibration_topic='/p2/calibration',calibration_publisher='/webui_isolated_fixture',
        trace_topic='/p2/trace',trace_publisher='/webui_isolated_fixture',pick_publisher='/webui_isolated_fixture',
        fjt_action='/p2/follow_joint_trajectory',fjt_publisher='/webui_isolated_fixture',tf_pairs=[['p2_base','p2_camera']])


class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.fixture=P2Fixture();cls.backend=ValidationBackend(configuration(),Path(cls.tmp.name)/'session');RosTransport(cls.backend).start()
        try:wait_for(lambda:cls.backend.snapshot()['calibration'].get('fresh'),timeout=12)
        except Exception:cls.backend.close();cls.fixture.close();cls.tmp.cleanup();raise
    @classmethod
    def tearDownClass(cls):cls.backend.close();cls.fixture.close();cls.tmp.cleanup()
    def test_01_missing_tf_and_nominal_calibration_remain_separate(self):
        wait_for(lambda:self.backend.snapshot()['tf'].get('fresh'))
        s=self.backend.snapshot();self.assertFalse(s['tf']['data']['available']);self.assertFalse(s['calibration']['data']['valid'])
    def test_02_native_tf_lookup_and_explicit_calibration_consumer(self):
        self.fixture.emit_tf=True;self.fixture.cal_valid=True
        wait_for(lambda:self.backend.snapshot()['tf']['data']['available'])
        wait_for(lambda:self.backend.snapshot()['calibration']['data']['valid'])
        self.assertFalse(self.backend.snapshot()['tf']['data']['pairs'][0]['static_cache'])
    def submit(self,name):
        wait_for(lambda:self.backend.snapshot()['operations']['submit'])
        return self.backend.submit(dict(task_id=name,mode=1,ordered_colors=[2],timeout_ms=30000,placement_stability_sec=3))
    def test_03_native_three_level_goal_chain_and_full_export(self):
        goal=self.submit('p2-success')
        wait_for(lambda:self.backend.snapshot()['pick_feedback'].get('fresh'),timeout=12)
        wait_for(lambda:len(self.backend.query_events(task_id='p2-success')['trace']['edges'])==2)
        self.assertEqual(self.backend.snapshot()['pick_feedback']['data']['substate'],'TEST_LIFT')
        wait_for(lambda:self.backend.snapshot()['task']['status']=='SUCCEEDED',timeout=12)
        wait_for(lambda:self.backend.snapshot()['pick_result'] is not None)
        wait_for(lambda:self.backend.snapshot()['fjt_result'] is not None)
        record=self.backend.task_record('p2-success');self.assertEqual(len(record['trace']['nodes']),3)
        self.assertTrue(any(e['kind']=='PICK_RESULT' for e in record['events']))
        fjt=self.fixture.motion_id
        trace=self.backend.query_events(goal_id=fjt)['trace'];self.assertEqual(len(trace['nodes']),3)
        self.assertTrue(any(n['goal_id']==goal['goal_id'].replace('-','') for n in trace['nodes']))
    def test_04_cancel_ack_and_native_520_remain_distinct_from_stop(self):
        goal=self.submit('p2-cancel');wait_for(lambda:self.backend.snapshot()['pick_feedback'].get('fresh'))
        wait_for(lambda:self.backend.snapshot()['task']['status'] in ('ACCEPTED','EXECUTING'))
        self.backend.cancel(goal['task_id'],goal['goal_id'])
        wait_for(lambda:self.backend.snapshot()['task']['status']=='ABORTED',timeout=12)
        s=self.backend.snapshot();self.assertTrue(s['task']['cancel_ack']['request_delivered']);self.assertEqual(s['task']['result']['error_code'],520)
        self.assertFalse(s['stop_evidence']['confirmed'])
        record=self.backend.export_session(task_id='p2-cancel');self.assertEqual(record['latest_failure']['error_code'],520)
    def test_05_duplicate_calibration_source_revokes_current_evidence(self):
        extra=self.fixture.node.create_publisher(self.fixture.string_type,'/p2/calibration',10)
        try:wait_for(lambda:not self.backend.snapshot()['calibration']['available'])
        finally:self.fixture.node.destroy_publisher(extra)
        wait_for(lambda:self.backend.snapshot()['calibration'].get('fresh'))
    def test_06_stale_tf_and_calibration_do_not_stay_valid(self):
        self.fixture.emit=False
        wait_for(lambda:not self.backend.snapshot()['calibration'].get('fresh'),timeout=6)
        wait_for(lambda:not self.backend.snapshot()['tf']['data']['available'],timeout=6)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--isolated-fixture',required=True,action='store_true');parser.add_argument('--serve',action='store_true');parser.add_argument('--output-dir',type=Path);args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    if not args.serve:unittest.main(argv=['p2_integration_check'],verbosity=2);return
    from mtc_webui.server import Server
    fixture=P2Fixture();fixture.emit_tf=True;backend=ValidationBackend(configuration(),args.output_dir);RosTransport(backend).start()
    try:
        with Server(('127.0.0.1',8769),backend) as server:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:backend.close();fixture.close()

if __name__=='__main__':main()
