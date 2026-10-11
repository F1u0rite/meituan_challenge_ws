"""Explicit local ROS process validation; run with the installed overlay sourced."""
import json
import math
import os
from pathlib import Path
import signal
import sys
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy,LivelinessPolicy,qos_check_compatible
from rosidl_runtime_py.convert import message_to_ordereddict
from mtc_interfaces.msg import BatteryDetectionArray
from mtc_vision_adapter.core import motion_rejection

w=Path(__file__).resolve().parents[3];out=Path(os.environ.get('MTC_VALIDATION_OUT','/tmp/mtc_vision_validation'))
out.mkdir(parents=True,exist_ok=True)
os.environ.setdefault('ROS_LOG_DIR',str(out/'ros_logs'))
fixture=Path(os.environ.get('MTC_REPLAY_FIXTURE',str(w/'src/mtc_vision_adapter/examples/synthetic.jsonl')))
records=[json.loads(l) for l in fixture.read_text().splitlines() if l]
qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE,liveliness=LivelinessPolicy.AUTOMATIC)
rclpy.init();node=Node('vision_sigint_acceptance');received=[];checks=[];endpoints=[];rejections=[]
def equal(a,b):
    if b is None:return isinstance(a,float) and math.isnan(a)
    if isinstance(b,dict):return set(a)==set(b) and all(equal(a[k],v) for k,v in b.items())
    if isinstance(b,list):return len(a)==len(b) and all(equal(x,y) for x,y in zip(a,b))
    return a==b

def callback(msg):
    data=message_to_ordereddict(msg);i=len(received);received.append(data)
    checks.append(i<len(records) and equal(data,records[i]['message']))
    stamp=data['header']['stamp']
    capture=stamp['sec']+stamp['nanosec']*1e-9
    offline=motion_rejection(data,capture+.01,mode=records[i]['mode'])
    live=motion_rejection(data,capture+.01,mode='live')
    assert offline=='REPLAY_NOT_MOTION_INPUT'
    assert live==('RING_INVALID' if data['detections'] else 'EMPTY')
    rejections.append({'offline':offline,'live':live})
    print('native received frame',i,'array length',len(msg.detections),'match',checks[-1],flush=True)

sub=node.create_subscription(BatteryDetectionArray,'/mtc/perception/batteries',callback,qos)
executable=str(w/'install/mtc_vision_adapter/lib/mtc_vision_adapter/replay_publisher')
# Run the installed entry point and observe its created publisher QoS.
# RMW graph discovery may omit history/depth; do not infer them from defaults.
probe = '''
import json, runpy, sys, time
from rclpy.node import Node
original=Node.create_publisher
def observed(self,*args,**kwargs):
    pub=original(self,*args,**kwargs)
    if pub.topic_name!='/mtc/perception/batteries':return pub
    deadline=time.monotonic()+20
    while pub.get_subscription_count()<1 and time.monotonic()<deadline:
        time.sleep(.05)
    assert pub.get_subscription_count()>=1,'subscriber discovery timeout'
    q=pub.qos_profile
    print('CREATED_QOS='+json.dumps(dict(reliability=q.reliability.name,history=q.history.name,depth=q.depth,durability=q.durability.name)),flush=True)
    return pub
Node.create_publisher=observed
entry=sys.argv[1]
sys.argv=sys.argv[1:]
runpy.run_path(entry,run_name='__main__')
'''
log=open(out/'ros_publisher.log','w');proc=subprocess.Popen([sys.executable,'-c',probe,executable,'--ros-args','-p','input_jsonl:='+str(fixture),'-p','period_s:=2.0'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
result={}
try:
    deadline=time.monotonic()+120
    while len(received)<4 and proc.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
    assert len(received)==4 and all(checks),('receive failure',len(received),proc.poll())
    for ep in node.get_publishers_info_by_topic('/mtc/perception/batteries'):
        compat=qos_check_compatible(ep.qos_profile,qos)
        endpoints.append({'node':ep.node_name,'type':ep.topic_type,'reliability':ep.qos_profile.reliability.name,'durability':ep.qos_profile.durability.name,'history':ep.qos_profile.history.name,'depth':ep.qos_profile.depth,'compatibility':compat[0].name,'reason':compat[1]})
    print('discovered QoS:',json.dumps(endpoints),flush=True)
    assert endpoints and all(e['compatibility']=='OK' and e['reliability']=='RELIABLE' and e['durability']=='VOLATILE' for e in endpoints)
    actual=[json.loads(line.split('=',1)[1]) for line in (out/'ros_publisher.log').read_text().splitlines() if line.startswith('CREATED_QOS=')]
    assert actual==[dict(reliability='RELIABLE',history='KEEP_LAST',depth=5,durability='VOLATILE')],actual
    # Await completion log before sending SIGINT, not a speculative startup delay.
    deadline=time.monotonic()+30
    while 'replay complete' not in (out/'ros_publisher.log').read_text() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
    assert 'replay complete' in (out/'ros_publisher.log').read_text()
    os.killpg(proc.pid,signal.SIGINT);code=proc.wait(timeout=30)
    assert code==0,code
    deadline=time.monotonic()+12
    while any(name=='vision_offline_replay' for name,_ in node.get_node_names_and_namespaces()) and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
    remaining=[name for name,_ in node.get_node_names_and_namespaces() if name=='vision_offline_replay']
    assert not remaining,remaining
    result={'received_frames':len(received),'array_lengths':[len(m['detections']) for m in received],'exact_fixture_matches':checks,'qos':endpoints,'publisher_created_qos':actual,'motion_rejections':rejections,'publisher_exit':code,'replay_nodes_remaining':remaining,'scope':'Local ROS replay and subscriber only; no C++ FSM or real hardware'}
    print(json.dumps(result,indent=2),flush=True)
finally:
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=30)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close();node.destroy_node();rclpy.try_shutdown()
    result['subscriber_cleanup_complete']=True
    (out/'ros_results.json').write_text(json.dumps(result,indent=2))
    (out/'ros_received.json').write_text(json.dumps(received,indent=2,allow_nan=True))
