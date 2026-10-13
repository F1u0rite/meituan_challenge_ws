"""Bounded V4.1 paired replay; never starts a camera."""
import sys,json,hashlib,argparse
from pathlib import Path
from time import perf_counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from perception.realtime_rgbd import RGBDVisionPipeline
from perception.dual_rate_rgbd import FastPerception
from perception.vision_ui import VisionUI
from real_data.rgbd_recording import load_rgbd_sample

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--phase',choices=['before','after'],required=True);a=parser.parse_args()
 out=ROOT/'outputs/live_rgbd_integration/v41';out.mkdir(exist_ok=True)
 selection=out/'selection.json'
 if not selection.exists():
  base=ROOT/'outputs/live_rgbd_integration/v33_diagnostic'
  audit=json.loads((base/'audit_frames.json').read_text());fit=[json.loads(x) for x in (base/'ring_axis_refined_frames.jsonl').read_text().splitlines()]
  chosen={}
  groups=[('table_failure',[r['sample'] for r in fit if r['fit']['fit_status']=='NO_TABLE_OR_RED_BODY'],3),
          ('geometric_support',[r['sample'] for r in fit if r['fit']['fit_status']=='GEOMETRIC_SUPPORT_ONLY'],5),
          ('weak',[r['sample'] for r in fit if r['fit']['fit_status']=='WEAK_XY_YAW_CONSTRAINT'],6),
          ('blue_background_candidate',[r['sample'] for r in audit if 'blue' in r['offline_colors']],3),
          ('yellow_background_candidate',[r['sample'] for r in audit if 'yellow' in r['offline_colors']],3)]
  for reason,names,limit in groups:
   for i in np.linspace(0,len(names)-1,min(limit,len(names)),dtype=int):chosen.setdefault(names[i],[]).append(reason)
  for r in audit:
   if len(chosen)>=20:break
   chosen.setdefault(r['sample'],[]).append('fill_normal')
  selection.write_text(json.dumps(chosen,indent=2),encoding='utf-8')
 selected=json.loads(selection.read_text());p=RGBDVisionPipeline();fast=FastPerception();ui=VisionUI();rows=[]
 target=out/(a.phase+'.json');assert not target.exists()
 for name in sorted(selected):
  f=load_rgbd_sample(ROOT/'outputs/live_rgbd_integration/dual_90dd28e6'/name)
  start=perf_counter();fr,fv=fast.process(f,name);fast_ms=(perf_counter()-start)*1000
  r,v=p.process(f,force_cad=True);start=perf_counter();ui.render(f,r,v,p.geometry);ui_ms=(perf_counter()-start)*1000
  reds=[(o,x) for o,x in zip(r['objects'],v) if o['color_label']=='red'];o,x=max(reds,key=lambda z:z[1]['mask'].sum(),default=(None,None))
  row=dict(sample=name,selection=selected[name],fast_ms=fast_ms,ui_ms=ui_ms,timing=r['timing_ms'],
   fast_colors=[z['color_label'] for z in fr['objects']],colors=[z['color_label'] for z in r['objects']],
   fit_status=o['fit_status'] if o else 'NO_RED',mask_pixels=int(x['mask'].sum()) if x else 0,
   mask_hash=hashlib.sha256(x['mask'].tobytes()).hexdigest() if x else None,
   body_xyz=o['model_estimated_body_center_xyz'] if o else None,body_yaw=o['body_yaw_candidate'] if o else None,
   ring_center=o['ring_center_candidate'] if o else None,ambiguity=o['ambiguity'] if o else [],
   pose_count=o['pose_quality'].get('reasonable_pose_count',0) if o else 0,
   ring_surface=o.get('ring_surface_support',{}) if o else {},rejections=r['rejections'])
  assert all(not z['body_pose_valid'] and not z['ring_pose_valid'] for z in r['objects'])
  rows.append(row);print(a.phase,len(rows),'/',len(selected),name,row['fit_status'],flush=True)
 def stats(xs):return dict(mean=float(np.mean(xs)),p95=float(np.percentile(xs,95)),throughput_hz=1000/float(np.mean(xs)))
 summary=dict(frames=len(rows),timing={k:stats([r[k] if k in ('fast_ms','ui_ms') else r['timing'][k] for r in rows]) for k in ['fast_ms','ui_ms','segmentation','cad_fit','total']},actual_camera_started=False,position_accuracy='NOT_MEASURED',yaw_accuracy='NOT_MEASURED')
 target.write_text(json.dumps(dict(rows=rows,summary=summary),indent=2),encoding='utf-8');print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
