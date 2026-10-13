"""External exact-main HTTP -> C++ Task -> PickPlace delayed-cancel/restart audit.

Only HTTP and the delivered top-level Task CLI send Goals. Child services are
read-only GetResult, never SendGoal or Cancel. No kernel/source modifications.
"""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
import traceback

import rclpy
from ament_index_python.packages import get_package_prefix
from mtc_interfaces.action import PickPlace
from rosidl_runtime_py.convert import message_to_ordereddict
from mtc_cpp_mock_bringup.acceptance import http, until
from mtc_cpp_mock_bringup.client import Recorder

ROOT=Path('/review-evidence/delayed-http');ROOT.mkdir(exist_ok=False)
NS='/monitor_main_mock';PORT=8768
stamp=lambda:datetime.now(timezone.utc).isoformat()
def save(p,value):p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
summary=dict(started_at=stamp(),status='FAIL',rounds=[],scope='Software Mock real parent-child Actions via HTTP/CLI, no browser rendering or hardware')
BIN=Path(get_package_prefix('mtc_cpp_mock_bringup'))/'lib/mtc_cpp_mock_bringup'
for round_name in ['delayed_and_next','restart']:
    folder=ROOT/round_name;folder.mkdir()
    row=dict(name=round_name,started_at=stamp(),goals=[],status='FAIL')
    argv=[str(BIN/'mock_stack'),'--namespace',NS,'--output-dir',str(folder/'stack'),'--port',str(PORT),
          '--profile','success','--result-delay-ms','700']
    row['argv']=argv
    log=(folder/'supervisor.log').open('xb')
    p=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();observer=Recorder(NS,folder/'observations');done=threading.Event()
    def spin():
        while not done.is_set():rclpy.spin_once(observer,timeout_sec=.02)
    thread=threading.Thread(target=spin);thread.start()
    child_client=None
    try:
        def ready():
            assert p.poll() is None,'stack exited before ready'
            try:
                code,value=http(PORT,'GET','/api/snapshot')
                return value if code==200 and value['operations']['submit'] else None
            except (OSError,ConnectionError):return None
        snap=until(ready,30);save(folder/'ready.json',snap);until(observer.ready,10)
        assert snap['mode']=='cpp_mock' and snap['connection']['graph']['valid']
        assert not snap['robot']['available'] and not snap['tool']['available'] and not snap['operations']['stop']
        def graph():
            rows=[dict(node=n,namespace=ns,clients=observer.get_client_names_and_types_by_node(n,ns)) for n,ns in observer.get_node_names_and_namespaces()]
            writers=[x['node'] for x in rows if any(a==NS+'/manipulation/pick_place/_action/send_goal' for a,_ in x['clients'])]
            return rows if writers==['mtc_mock_task'] else None
        rows=until(graph,10);save(folder/'service-clients.json',rows)
        assert not any(x['node'] in ['task_executor','pick_place_server','mtc_task_executor','mtc_pick_place_server'] for x in rows)
        identity=[]
        for child in json.loads((folder/'stack/launch.json').read_text())['processes']:
            if child['label'] not in ['task','pick_place']:continue
            exe=Path(os.readlink('/proc/'+str(child['pid'])+'/exe'))
            assert exe.read_bytes()[:4]==b'\x7fELF'
            identity.append(dict(label=child['label'],pid=child['pid'],executable=str(exe),sha256=hashlib.sha256(exe.read_bytes()).hexdigest()))
        row['binary_identity']=identity;assert len(identity)==2
        child_client=observer.create_client(PickPlace.Impl.GetResultService,NS+'/manipulation/pick_place/_action/get_result')
        assert child_client.wait_for_service(timeout_sec=5)
        if round_name=='delayed_and_next':
            for label,cancel in [('delayed_cancel',True),('same_stack_next',False)]:
                until(ready,10)
                goal=dict(task_id=label,mode=1,ordered_colors=[1],timeout_ms=30000,placement_stability_sec=3)
                sent=observer.event('http_goal_request',goal=goal)
                code,reply=http(PORT,'POST','/api/tasks',goal);assert code==202,reply
                observer.event('http_goal_queued',http_status=code,reply=reply)
                parent=reply['goal_id'].replace('-','');item=dict(task_id=label,parent_uuid=parent,sent=sent)
                if cancel:
                    def moving():
                        s=http(PORT,'GET','/api/snapshot')[1]
                        return s if (s.get('task') or {}).get('substate')=='MOVE_PRE_ALIGN' and s['operations']['cancel'] else None
                    before=until(moving,10);trigger=time.monotonic();save(folder/'before-cancel.json',before)
                    time.sleep(.4)
                    t=time.monotonic();observer.event('http_cancel_requested',goal_id=parent,after_feedback_ms=(t-trigger)*1000)
                    code,response=http(PORT,'POST','/api/tasks/'+label+'/cancel',dict(goal_id=reply['goal_id']))
                    observer.event('http_cancel_queued',http_status=code,response=response)
                    assert code==202 and response['request_delivered'] is None
                    item['cancel_delay_ms']=(t-trigger)*1000
                def finished():
                    record=http(PORT,'GET','/api/tasks/'+label)[1]
                    return record if record['status'] in ['SUCCEEDED','CANCELED','ABORTED','REJECTED'] else None
                task=until(finished,40);save(folder/(label+'-http-result.json'),task)
                observer.event('http_terminal_observed',goal_id=parent,record=task)
                expected=5 if cancel else 4
                assert task['status']==('CANCELED' if cancel else 'SUCCEEDED'),task
                assert task['result']['error_code']==0 and task['completed_count']==(0 if cancel else 1)
                assert task['result']['success']==(not cancel) and any(e['kind']=='FEEDBACK' for e in task['events'])
                if cancel:assert task['cancel_ack']['request_delivered'] is True
                def trace():
                    text=(folder/'stack/task.log').read_text()
                    match=re.search(r'event=child_response parent='+parent+r' child=([0-9a-f]+) accepted=1',text)
                    if match and f'event=child_result parent={parent} child={match[1]} status={expected} error=0' in text:return match[1]
                child=until(trace,5)
                request=PickPlace.Impl.GetResultService.Request();request.goal_id.uuid=list(bytes.fromhex(child))
                future=child_client.call_async(request);until(future.done,5)
                child_result=dict(message_to_ordereddict(future.result()))
                assert child_result['status']==expected and child_result['result']['error_code']==0
                item.update(child_uuid=child,child_result=child_result,http_result=task)
                if cancel:assert 'event=child_cancel child='+child in (folder/'stack/task.log').read_text()
                until(lambda:any(e['kind']=='status' and e['goal_id']==parent and e['status']==expected for e in observer.events),5)
                until(lambda:any(e['kind']=='status' and e['goal_id']==child and e['status']==expected for e in observer.events),5)
                for role,gid in [('parent',parent),('child',child)]:
                    events=[e for e in observer.events if e.get('goal_id')==gid]
                    assert any(e['kind']=='feedback' for e in events)
                    item[role+'_status_changes']=[e for e in events if e['kind']=='status']
                    item[role+'_feedback_count']=sum(e['kind']=='feedback' for e in events)
                save(folder/(label+'-correlation.json'),item);row['goals'].append(item)
            code,export=http(PORT,'GET','/api/export');assert code==200
            assert {x['task_id'] for x in export['tasks']}=={'delayed_cancel','same_stack_next'}
            save(folder/'http-export.json',export)
        else:
            # Same domain/namespace/port after clean shutdown; use shipped top Task CLI.
            goal=dict(task_id='after_restart',mode=1,ordered_colors=[2],timeout_ms=30000,placement_stability_sec=3)
            argv=[str(BIN/'mock_task'),'--namespace',NS,'--output-dir',str(folder/'cli'),'--goal',json.dumps(goal)]
            with (folder/'cli.log').open('xb') as out:q=subprocess.run(argv,stdout=out,stderr=subprocess.STDOUT,timeout=45)
            result=json.loads((folder/'cli/result.json').read_text());assert q.returncode==0 and result['status']==4 and result['result']['success']
            row['cli']=dict(argv=argv,exit_code=q.returncode,result=result)
        row['status']='PASS'
    except Exception as exc:
        row.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        if child_client is not None:observer.destroy_client(child_client)
        done.set();thread.join(timeout=3);assert not thread.is_alive()
        observer.finish();rclpy.shutdown()
        if p.poll() is None:p.send_signal(signal.SIGINT)
        try:p.wait(timeout=25)
        except subprocess.TimeoutExpired:
            row['forced_cleanup']=True;os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
        log.close();row['stack_exit_code']=p.returncode
        cleanup=folder/'stack/cleanup.json';row['cleanup']=json.loads(cleanup.read_text()) if cleanup.exists() else None
        if p.returncode or row.get('forced_cleanup') or not row['cleanup'] or not row['cleanup']['cleanup_ok']:row['status']='CLEANUP_FAIL'
        row['ended_at']=stamp();save(folder/'summary.json',row);summary['rounds'].append(row);save(ROOT/'summary.json',summary)
    if row['status']!='PASS':break
summary['status']='PASS' if len(summary['rounds'])==2 and all(x['status']=='PASS' for x in summary['rounds']) else 'FAIL'
summary['ended_at']=stamp();save(ROOT/'summary.json',summary)
print(json.dumps(dict(status=summary['status'],rounds=[dict(name=x['name'],status=x['status'],error=x.get('error')) for x in summary['rounds']]),ensure_ascii=False))
raise SystemExit(0 if summary['status']=='PASS' else 1)
