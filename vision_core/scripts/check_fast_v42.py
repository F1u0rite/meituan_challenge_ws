"""Small candidate-state audit; frozen source RGB-D, no device."""
import sys,json,copy,argparse
from pathlib import Path
from datetime import datetime,timedelta
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from perception.dual_rate_rgbd import FastPerception
from real_data.rgbd_recording import load_rgbd_sample

def main():
 p=argparse.ArgumentParser();p.add_argument('--phase',required=True);args=p.parse_args()
 out=ROOT/'outputs/live_rgbd_integration/v42';out.mkdir(exist_ok=True)
 before=json.loads((ROOT/'outputs/live_rgbd_integration/v41/before.json').read_text())['rows']
 chosen=[r['sample'] for r in before if any(c in ('blue','yellow') for c in r['fast_colors'])]
 chosen+= [r['sample'] for r in before if r['fit_status']=='NO_TABLE_PLANE'][:2]
 chosen+= [r['sample'] for r in before if r['fit_status']=='GEOMETRIC_SUPPORT_ONLY'][:2]
 chosen=list(dict.fromkeys(chosen))[:10]
 planes={r['sample']:r['after']['table_plane'] for r in map(json.loads,(ROOT/'outputs/live_rgbd_integration/v33_diagnostic/paired_frames.jsonl').read_text().splitlines())}
 rows=[]
 for name in chosen:
  frame=load_rgbd_sample(ROOT/'outputs/live_rgbd_integration/dual_90dd28e6'/name);fast=FastPerception();r,_=fast.process(frame,'first')
  stamp=r['capture_timestamp'];fast.set_table_hint(planes[name],stamp,'saved_same_pose_plane')
  f=copy.copy(frame);f.timestamps=dict(frame.timestamps)
  f.timestamps['host_utc']=(datetime.fromisoformat(f.timestamps['host_utc'])+timedelta(seconds=.1)).isoformat()
  for k in ('color_sdk_ms','depth_sdk_ms'):f.timestamps[k]+=100
  r,_=fast.process(f,'second')
  rows.append(dict(sample=name,source='OFFLINE_REPEATED_STATIC_INPUT_WITH_SAVED_PLANE; NOT_TEMPORAL_ACCURACY',
    plane_available=planes[name] is not None,objects=r['objects'],rejections=r['rejections']))
 path=out/(args.phase+'.json');assert not path.exists();path.write_text(json.dumps(rows,indent=2),encoding='utf-8')
 print(args.phase,'frames',len(rows),'objects',[(x['sample'],[(o['color_label'],o['candidate_state'],o['tracking_status']) for o in x['objects']]) for x in rows],flush=True)
if __name__=='__main__':main()
