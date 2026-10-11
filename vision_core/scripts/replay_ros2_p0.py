"""Generate reproducible real replay and synthetic four-color tracking diagnostics."""
import sys,json,argparse
from pathlib import Path
from datetime import datetime
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'src/mtc_vision_adapter')]
from mtc_vision_adapter.core import Tracker,make_array,motion_rejection
from perception.unified_detector import UnifiedBatteryDetector
from real_data.rgbd_recording import load_rgbd_sample
from perception.rgbd_offline import analyze_frame

def candidates(image,detector,depth=None):
    out=[]
    detections=detector.detect_batteries(image)
    if depth is not None:
        report,_,_,_=analyze_frame(depth,detections)
        if report["table_plane"] is None: return [],report
        detections=[d for d,r in zip(detections,report["candidates"]) if r["accepted"]]
        depth=depth.aligned_depth
    else: report={"source":"synthetic", "candidate_filter":"unified RGB geometry/conflict"}
    for d in detections:
        ratio=float(np.mean(depth[d.mask]>0)) if depth is not None and d.mask.any() else None
        if ratio is not None and ratio<.5:continue
        out.append({'color':d.color,'confidence':d.confidence,'pixel_center':list(d.pixel_center),
                    'yaw_deg':d.yaw_deg,'detection_valid':d.detection_valid,
                    'depth_valid_ratio':ratio})
    return out,report

def run(output,session):
    output.mkdir(parents=True,exist_ok=True)
    detector=UnifiedBatteryDetector(); results={}
    for mode in ('replay','synthetic'):
        tracker=Tracker(prefix=mode); records=[]; counts={}
        if mode=='replay':
            inputs=[]
            for folder in sorted(session.glob('sample_*')):
                frame=load_rgbd_sample(folder)
                ts=datetime.fromisoformat(frame.timestamps['host_utc']).timestamp()
                inputs.append((folder.name,frame.color_bgr,frame,ts))
        else:
            inputs=[]
            for index in range(16):
                image=np.full((480,640,3),20,np.uint8)
                for j,(color,hue) in enumerate((('red',0),('blue',110),('yellow',30),('green',60))):
                    if j==0 and index in (5,6,7,8,9):continue
                    bgr=tuple(int(v) for v in cv2.cvtColor(np.uint8([[[hue,230,210]]]),cv2.COLOR_HSV2BGR)[0,0])
                    x=40+j*145+index
                    cv2.rectangle(image,(x,150),(x+90,270),bgr,-1)
                if index==3:cv2.rectangle(image,(40,330),(130,420),(0,0,210),-1)
                if index==12:image[:]=20
                cv2.imencode('.png',image)[1].tofile(str(output/f'synthetic_{index:02d}.png'))
                inputs.append((f'synthetic_{index:02d}',image,None,1700000000+index*.25))
        for name,image,depth,ts in inputs:
            cs,filter_report=candidates(image,detector,depth)
            tracking=tracker.update(cs,ts)
            msg=make_array(tracking,'replay_unregistered_color_optical_frame',ts)
            for d in msg['detections']:counts[str(d['color'])]=counts.get(str(d['color']),0)+1
            records.append({'sample':name,'mode':mode,
              'timestamp_source':'metadata.timestamps.host_utc' if mode=='replay' else 'synthetic_clock',
              'frame_id_source':'NOT_RECORDED; replay placeholder, NOT TF calibrated',
              'coordinate_mode':'IMAGE_PIXEL','pose_units':'m; all unmeasured values null/ROS NaN',
              'candidate_only':True,'body_pose_valid':False,'ring_pose_valid':False,
              'filter_report':filter_report,'tracking':tracking,'message':msg,
              'motion_rejection':motion_rejection(msg,ts,mode=mode)})
            overlay=image.copy()
            for t in tracking['observed']:
                u,v=map(round,t['pixel_center'])
                cv2.circle(overlay,(u,v),5,(255,255,255),-1)
                cv2.putText(overlay,f"{t['object_id']} {t['color']} {t['tracking_status']}",
                            (max(0,u-100),max(20,v-10)),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
            cv2.imencode('.png',overlay)[1].tofile(str(output/f'{mode}_{name}_tracking.png'))
        (output/f'{mode}.jsonl').write_text('\n'.join(json.dumps(r,allow_nan=False) for r in records)+'\n',encoding='utf-8')
        results[mode]={'frames':len(records),'published_candidate_counts_by_enum':counts,
                      'unique_ids':sorted({d['object_id'] for r in records for d in r['message']['detections']}),
                      'rejected_for_motion':sum(r['motion_rejection']!='NO_REJECTION' for r in records),
                      'physical_identity_accuracy':'NOT MEASURED'}
    (output/'summary.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,default=ROOT/'outputs/p0_ros2')
    p.add_argument('--session',type=Path,default=ROOT/'data/rgbd/session_20261011_143230_red_top')
    a=p.parse_args();run(a.output,a.session)
