#!/usr/bin/env python3
"""Independently check recorded Gazebo poses against the requested task, and plot them."""
import argparse
import json
import math
from pathlib import Path

def analyze(run):
    scenario=json.loads((run/'scenario.json').read_text())
    result=json.loads((run/'result.json').read_text())
    samples=[json.loads(line) for line in (run/'physics.jsonl').read_text().splitlines() if line.strip()]
    order=result.get('order','')
    count=1 if scenario['task']=='basic' else 3
    order_ok=(len(order)==count and len(set(order))==count and all(c in 'ABCD' for c in order)
              and order==scenario.get('requested_order',order)
              and [r.get('object') for r in result['objects']]==['battery_'+c for c in order])
    rows=[]
    for index,letter in enumerate(result['order']):
        key='battery_'+letter
        slot='T0' if scenario['task']=='basic' else 'P'+str(index+1)
        target=scenario['targets'][slot]
        data=[(s['sim_time'],s['objects'][key]) for s in samples if key in s['objects']]
        if not data:
            rows.append({'object':key,'passed':False,'reason':'no physics samples'});continue
        initial=data[0][1]['xyz'];final=data[-1][1]['xyz']
        error=math.hypot(final[0]-target[0],final[1]-target[1])
        max_lift=max(o['xyz'][2]-initial[2] for _,o in data)
        start=data[-1][0];last=start
        for t,o in reversed(data):
            x,y,z=o['xyz'];qx,qy,qz,qw=o['quat']
            upright=1-2*(qx*qx+qy*qy)
            if last-t>.3 or math.hypot(x-target[0],y-target[1])>=.015 or abs(z-target[2])>=.002 or o['speed']>=.003 or o.get('angular_speed',float('inf'))>=.04 or upright<=.995:break
            start=t;last=t
        stable=data[-1][0]-start
        fsm=next((r for r in result['objects'] if r['object']==key),{})
        passed=error<.015 and max_lift>.012 and stable>=3.0-1e-6 and fsm.get('placement_verified',False)
        passed=passed and fsm.get('slot')==slot and fsm.get('success',False)
        rows.append({'object':key,'slot':slot,'passed':passed,'max_lift_m':max_lift,
                     'final_center_error_m':error,'final_xyz':final,'stable_from_trace_s':stable})
    pairs=result.get('contacts',{}).get('total',{})
    unexpected={k:v for k,v in pairs.items() if v['kind']=='unexpected' and v['max_depth_m']>.001}
    penetrating_tool={k:v for k,v in pairs.items() if v['kind']=='tool_contact' and v['max_depth_m']>.001}
    report={'passed':bool(result['success']) and order_ok and bool(rows) and all(r['passed'] for r in rows) and not unexpected and not penetrating_tool,
            'requested_order_verified':order_ok,'tool_contacts_over_1mm':penetrating_tool,
            'objects':rows,'unexpected_contacts_over_1mm':unexpected,
            'acceptance':'15 mm center tolerance, 2 mm base height tolerance, upright, >=3 s stable; not competition scoring',
            'inner_circle_footprint_fits':math.hypot(.07,.08)<=.08,
            'model_limits':scenario['assumptions']}
    (run/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
        for letter,color in zip('ABCD',['#cc3333','#c49a00','#3355bb','#228844']):
            key='battery_'+letter
            data=[(s['sim_time'],s['objects'][key]['xyz']) for s in samples if key in s['objects']]
            if not data:continue
            axes[0].plot([x[1][0] for x in data],[x[1][1] for x in data],color=color,label=letter)
            axes[0].scatter(*data[-1][1][:2],color=color,s=35)
            axes[1].plot([x[0]-samples[0]['sim_time'] for x in data],[(x[1][2]-.65)*1000 for x in data],color=color,label=letter)
        for slot,v in scenario['targets'].items():
            axes[0].scatter(v[0],v[1],marker='x',color='black');axes[0].annotate(slot,v[:2])
        axes[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Measured battery paths',aspect='equal')
        axes[1].set(xlabel='Simulation time (s)',ylabel='Battery base above table (mm)',title='Physical lift and placement')
        for ax in axes:ax.grid(alpha=.2);ax.legend()
        fig.savefig(run/'trajectories.png',dpi=160);plt.close(fig)
    except ImportError:
        pass # Acceptance never depends on optional plotting.
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
    result=analyze(args.run);print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
