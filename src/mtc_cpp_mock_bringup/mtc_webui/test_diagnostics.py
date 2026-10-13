"""P2 regression: attribution, full history and independent unknown evidence."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.request import urlopen
from urllib.error import HTTPError
from mtc_webui.diagnostics import Journal, trace_sample, calibration_sample, trace_log
from mtc_webui.config import RosConfig
from mtc_webui.ros_backend import RosBackend
from mtc_webui.mock_backend import ApiError
from mtc_webui.server import Server

A='a'*32;B='b'*32;C='c'*32;D='d'*32

class Diagnostics(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.b=RosBackend(output_dir=Path(self.tmp.name)/'session')
        self.b.connection_update(connected=True,reason='fixture')
    def link(self, parent=A, child=B, fjt=C, task='task-1'):
        self.b.declare_trace(dict(execute_goal_id=parent,pick_goal_id=child,fjt_goal_id=fjt,task_id=task),'/fixture')
    def test_full_persistent_query_survives_live_window_and_paginates(self):
        self.link()
        for i in range(650):self.b._event('ROS_LOG',str(i),module='execution',goal_id=C)
        self.assertEqual(len(self.b.events),500)
        q=self.b.query_events(task_id='task-1',module='execution',limit=200)
        self.assertEqual(q['total'],650);self.assertEqual(len(q['events']),200)
        all_ids=[]
        while True:
            all_ids.extend(e['seq'] for e in q['events'])
            if not q['next_before']:break
            q=self.b.query_events(task_id='task-1',module='execution',limit=200,before=q['next_before'])
        self.assertEqual(len(set(all_ids)),650)
        full=self.b.export_session(task_id='task-1',module='execution')
        self.assertEqual(len(full['events']),650);self.assertEqual(full['events'][0]['message'],'0')
        stored=[json.loads(x) for x in (self.b.output_dir/'events.jsonl').read_text().splitlines()]
        self.assertEqual(len(stored),self.b.seq)
        self.assertTrue(full['coverage']['persistent'])
    def test_relationships_arrive_after_result_and_query_uuid_hyphens(self):
        self.b._event('MOTION_RESULT','late result',goal_id=C,module='execution',level='error',error_code=-4)
        self.assertEqual(self.b.query_events(task_id='task-1')['total'],0)
        self.link()
        q=self.b.query_events(task_id='task-1')
        self.assertEqual(q['latest_failure']['error_code'],-4)
        self.assertEqual(len(q['trace']['edges']),2)
        q=self.b.query_events(goal_id='bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb')
        self.assertEqual(q['total'],1)
        self.assertEqual(q['events'][0]['goal_id'],C)
        self.assertEqual(q['events'][0]['related_task_ids'],['task-1'])
    def test_no_temporal_correlation_and_conflicting_parent_quarantines_descendants(self):
        self.b._event('MOTION_RESULT','unrelated',goal_id=C,module='execution')
        self.b.journal.node(A,'ExecuteTask','task-1')
        self.assertEqual(self.b.query_events(task_id='task-1')['total'],0)
        self.link();self.link(parent=D,task='task-2')
        self.assertFalse(any(e['kind']=='MOTION_RESULT' for e in self.b.query_events(task_id='task-1')['events']))
        self.assertTrue(all(e['conflict'] for e in self.b.query_events(task_id='task-1')['trace']['edges']))
    def test_exact_native_trace_log_and_untrusted_logger_does_not_link(self):
        msg=f'MTC_TRACE event=task_start parent={A} task="test-1"'
        self.b.log('wrong_node',20,msg)
        self.assertFalse(self.b.journal.nodes)
        self.b.log('task_executor',20,msg)
        self.b.log('task_executor',20,f'MTC_TRACE event=child_response parent={A} child={B} accepted=1 request="r/attempt-1"')
        self.assertEqual(len(self.b.query_events(task_id='test-1')['trace']['edges']),1)
        self.assertIsNone(trace_log(f'MTC_TRACE event=child_response parent={A} child=none accepted=0 request="r"'))
    def test_nominal_calibration_and_invalid_link_rejected(self):
        raw=dict(schema_version=1,stamp_ns=1,calibration_id='fixture',valid=True,evidence='nominal',reference='fake.yaml')
        with self.assertRaises(ValueError):calibration_sample(raw)
        raw.update(evidence='measured');self.assertTrue(calibration_sample(raw)['valid'])
        for values in [dict(execute_goal_id=A,pick_goal_id=A),dict(execute_goal_id=A,pick_goal_id='bad')]:
            with self.assertRaises(ValueError):trace_sample(dict(schema_version=1,stamp_ns=1,link_id='link',task_id='t',**values))
    def test_optional_topics_require_publisher_and_tf_pairs_validate(self):
        for kind in ('trace','calibration'):
            with self.assertRaises(ValueError):RosConfig(**{kind+'_topic':'/x'})
        with self.assertRaises(ValueError):RosConfig(tf_pairs=[['same','same']])
        self.assertEqual(RosConfig(tf_pairs=[['base','camera']]).tf_pairs,(('base','camera'),))
    def test_stale_observation_does_not_create_new_error_or_link(self):
        self.b.observe('planner',dict(plan_id='p',state='FAILED',reason='old watchdog'),'/fixture',1,9000)
        self.assertIsNone(self.b.query_events()['latest_failure'])
        self.b.observe('trace',dict(execute_goal_id=A,pick_goal_id=B,task_id='t'),'/fixture',1,9000)
        self.assertFalse(self.b.journal.nodes)
    def test_transitions_and_latest_failure_keep_module_and_raw_code(self):
        for stamp,state in [(1,'PLANNING'),(2,'FAILED'),(3,'FAILED')]:
            self.b.observe('planner',dict(plan_id='p',task_id='t',state=state,error_code='EXCEEDS_STATE_WATCHDOG',reason='too long'),'/planner',stamp,0)
        q=self.b.query_events(module='planner',transitions=True)
        self.assertEqual(q['total'],2);self.assertEqual(q['latest_failure']['error_code'],'EXCEEDS_STATE_WATCHDOG')
        self.assertEqual(q['events'][0]['transition']['previous']['state'],'PLANNING')
    def test_pick_feedback_and_result_never_confirm_stop(self):
        self.b.observe_pick_status([dict(goal_id=B,status=3)],'/pick',1,0)
        self.b.observe_pick_feedback(B,dict(substate='CANCELING'),'/pick',2,0)
        self.b.record_pick_result(B,6,dict(success=False,placement_verified=False,error_code=520,message='stop unconfirmed'),'/pick')
        self.assertFalse(self.b.stop_evidence()['confirmed'])
        self.assertEqual(self.b.query_events(module='pickplace')['latest_failure']['error_code'],520)
    def test_log_export_preserves_full_text_beyond_live_preview(self):
        message='完整日志' * 2000
        self.b.log('task_executor',40,message)
        self.assertTrue(self.b.snapshot()['events'][-1]['preview_truncated'])
        self.assertEqual(self.b.query_events(module='task')['events'][0]['message'],message)
        self.assertEqual(self.b.export_session()['events'][-1]['message'],message)

    def test_http_query_validation_and_full_export(self):
        self.link();self.b._event('MOTION_RESULT','failed',goal_id=C,module='execution',level='error',error_code=-4)
        server=Server(('127.0.0.1',0),self.b);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
        base='http://127.0.0.1:'+str(server.server_address[1])
        for path in ('/api/events?task_id=task-1','/api/export?goal_id='+B):
            with urlopen(base+path) as r:self.assertEqual(json.load(r)['latest_failure']['error_code'],-4)
        for query in ('goal_id=garbage','module=evil','limit=1000','errors=perhaps','module=tool&module=robot'):
            with self.assertRaises(HTTPError) as e:urlopen(base+'/api/events?'+query)
            self.assertEqual(e.exception.code,400)

if __name__=='__main__':unittest.main()
