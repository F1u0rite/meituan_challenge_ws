"""An FSM success bit must not override independently observed physics failures."""
import importlib.util
import json
from pathlib import Path
import sys
import pytest

spec=importlib.util.spec_from_file_location('analyze_v1',Path(__file__).parents[1]/'scripts/analyze_v1.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def recorded(tmp_path, *, lift=True, displaced=False, gap=False, collision=False):
    (tmp_path/'scenario.json').write_text(json.dumps({'task':'basic','targets':{'T0':[.3,-.1,.65]},'assumptions':{}}))
    (tmp_path/'result.json').write_text(json.dumps({'success':True,'order':'A','objects':[{'object':'battery_A','slot':'T0','success':True,'placement_verified':True}],
      'contacts':{'total':{'arm / table':{'kind':'unexpected','max_depth_m':.002}} if collision else {}}}))
    rows=[]
    for i in range(61):
        t=i*.1
        if gap and 44<i<50:continue
        xyz=[.3 if not displaced else .34,-.1,.65]
        if i<20 and lift:xyz[2]+=.03*i/20
        rows.append({'sim_time':t,'tcp':[0,0,0],'objects':{'battery_A':{'xyz':xyz,'quat':[0,0,0,1],'speed':0.0,'angular_speed':0.0}}})
    (tmp_path/'physics.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))

def test_measured_lift_and_stability(tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path)
    assert module.analyze(tmp_path)['passed']

@pytest.mark.parametrize('fault',[{'lift':False},{'displaced':True},{'gap':True},{'collision':True}])
def test_success_bit_cannot_mask_physics_failure(tmp_path,monkeypatch,fault):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path,**fault)
    assert not module.analyze(tmp_path)['passed']

@pytest.mark.parametrize('fault',['empty_order','wrong_slot','missing_angular_speed','tool_penetration'])
def test_incomplete_or_inconsistent_evidence_fails(tmp_path,monkeypatch,fault):
    monkeypatch.setitem(sys.modules,'matplotlib',None)
    recorded(tmp_path)
    p=tmp_path/'result.json';result=json.loads(p.read_text())
    if fault=='empty_order':result.update(order='',objects=[])
    if fault=='wrong_slot':result['objects'][0]['slot']='P3'
    if fault=='tool_penetration':result['contacts']['total']={'hook / bar':{'kind':'tool_contact','max_depth_m':.002}}
    if fault=='missing_angular_speed':
        q=tmp_path/'physics.jsonl';rows=[json.loads(s) for s in q.read_text().splitlines()]
        for r in rows:del r['objects']['battery_A']['angular_speed']
        q.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    p.write_text(json.dumps(result))
    assert not module.analyze(tmp_path)['passed']
