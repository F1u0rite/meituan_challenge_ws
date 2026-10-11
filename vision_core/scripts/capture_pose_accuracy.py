"""Two-stage authorized preview and user-triggered bounded accuracy capture."""
import sys,json,argparse,time,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import cv2,numpy as np
from input.realsense_rgbd_source import RealSenseRGBDSource
from perception.unified_detector import UnifiedBatteryDetector
from perception.rgbd_offline import analyze_frame,OfflineConfig
from perception.cad_body import fit_body,sample_surface
from cad_geometry.extract import analyze_cad,normalized_mesh
from real_data.rgbd_recording import load_rgbd_sample
from real_data.pose_experiment import save_accuracy_sample,write_json
from evaluation.pose_accuracy import validate_gt
from evaluation.pose_uncertainty import validity_gate
_pipeline=None
def load_estimator():
    from perception.realtime_rgbd import RGBDVisionPipeline
    global _pipeline
    _pipeline=RGBDVisionPipeline()
    return _pipeline.geometry,_pipeline.surface

def predict_frame(frame,detector,geometry,surface,min_depth=.25,max_depth=.8,min_valid_ratio=.6):
    # Current vision prediction never receives GT or fixture calibration.
    from dataclasses import replace
    from perception.realtime_rgbd import RGBDVisionPipeline
    global _pipeline
    if _pipeline is None:_pipeline=RGBDVisionPipeline(geometry=geometry,surface=surface,detector=detector)
    _pipeline.detector=detector
    _pipeline.cfg=replace(_pipeline.cfg,min_depth_m=min_depth,max_depth_m=max_depth)
    result,views=_pipeline.process(frame,force_cad=True)
    pairs=[(o,v) for o,v in zip(result['objects'],views) if o['color_label']=='red']
    selected,payload=max(pairs,key=lambda pair:pair[1]['mask'].sum(),default=(None,None))
    points=payload['points'] if payload else np.empty((0,3))
    rgb=payload['rgb'] if payload else np.empty((0,3),np.uint8)
    depth_ok=bool(selected and selected['depth_valid_ratio']>=min_valid_ratio and len(points)>=100)
    prediction={'object_id':selected['object_id'] if selected else None,'color':'red' if selected else None,
        'unit':'m','coordinate_frame':'camera_color_optical','reference_point':'body_geometry_center',
        'body_pose_candidate':selected['body_pose_candidate'] if selected else None,
        'ring_pose_candidate':selected['ring_pose_candidate'] if selected else None,
        'ring_center_candidate':selected['ring_center_candidate'] if selected else None,
        'ring_direction_candidates':selected['insertion_direction_candidates'] if selected else [],
        'predicted_xyz_m':selected['model_estimated_body_center_xyz'] if selected else None,
        'body_yaw_candidate_rad':selected['body_yaw_candidate'] if selected else None,
        'detected':selected is not None,'depth_valid':depth_ok,
        'multiple_solutions':bool(selected and selected['pose_quality'].get('reasonable_pose_count',0)>1),
        'pose_hypotheses':selected.get('pose_hypotheses',[]) if selected else [],
        'ambiguity':selected['ambiguity'] if selected else ['NOT_DETECTED'],
        'insertion_direction_resolved':False,'body_pose_valid':False,'ring_pose_valid':False,
        'timestamps':frame.timestamps,'capture_timestamp':result['capture_timestamp'],
        'source_frame_id':{'color':frame.timestamps.get('color_frame_number'),'depth':frame.timestamps.get('depth_frame_number')}}
    quality={'depth_valid':depth_ok,'validity_reason':selected['fit_status'] if selected else 'NOT_DETECTED',
        'cad_fit':selected['pose_quality'] if selected else {},'gate':validity_gate(),
        'depth_interval_m':[min_depth,max_depth],'pipeline':'V3.3_EXISTING_RGBDVisionPipeline'}
    image=frame.color_bgr.copy()
    if payload:
        contours,_=cv2.findContours(np.uint8(payload['mask']),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image,contours,-1,(0,255,255),2)
    cv2.putText(image,'UNVALIDATED CANDIDATE | BODY/RING INVALID',(12,35),0,.7,(0,255,255),2)
    return prediction,quality,points,rgb,image

def main(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--plan',type=Path,default=ROOT/'outputs/pose_accuracy/pilot_red_v1/pilot_plan.json')
    p.add_argument('--gt',type=Path,default=ROOT/'outputs/pose_accuracy/pilot_red_v1/manual_gt.json')
    p.add_argument('--fixture-calibration',type=Path,default=ROOT/'outputs/pose_accuracy/pilot_red_v1/fixture_calibration.json')
    p.add_argument('--output',type=Path,default=ROOT/'data/pose_accuracy');p.add_argument('--fps',type=int,default=15)
    p.add_argument('--min-depth-m',type=float,default=.25);p.add_argument('--max-depth-m',type=float,default=.8)
    p.add_argument('--preview-authorized',action='store_true');p.add_argument('--static-confirmed',action='store_true')
    p.add_argument('--offline-sample',type=Path);p.add_argument('--allow-qualitative-without-gt',action='store_true')
    args=p.parse_args(argv)
    if not args.offline_sample and not args.preview_authorized:p.error('Human confirmation required before preview')
    if args.min_depth_m<=0 or args.max_depth_m<=args.min_depth_m:p.error('Invalid depth interval')
    plan=json.loads(args.plan.read_text(encoding='utf-8-sig'));geometry,surface=load_estimator();detector=UnifiedBatteryDetector()
    calibration=json.loads(args.fixture_calibration.read_text(encoding='utf-8-sig'))
    session=args.output/('session_'+uuid.uuid4().hex);case_index=0;remaining=0;next_save=0.;saved=0
    def save(frame,case):
        nonlocal saved
        gt=json.loads(args.gt.read_text(encoding='utf-8-sig')).get(case['case_id'])
        if validate_gt(gt) and not args.allow_qualitative_without_gt:
            print('GT missing/unconfirmed: record independent measurement first; no sample saved',flush=True);return False
        prediction,quality,points,rgb,image=predict_frame(frame,detector,geometry,surface,args.min_depth_m,args.max_depth_m)
        if not session.exists():
            session.mkdir(parents=True);write_json(session/'experiment_plan.json',plan)
        folder=save_accuracy_sample(session,frame,prediction,quality,gt,calibration,case_id=case['case_id'],
            visualization=image,points=points,rgb=rgb,static_confirmed=args.static_confirmed,
            capture_trigger='OFFLINE_REPLAY' if args.offline_sample else 'USER_KEY')
        saved+=1;print('saved',folder,quality.get('validity_reason','UNAVAILABLE'),flush=True);return True
    if args.offline_sample:
        frame=load_rgbd_sample(args.offline_sample);save(frame,plan['cases'][0]);return
    source=RealSenseRGBDSource(fps=args.fps);last=time.perf_counter()
    try:
        profiles=source.start();print('Actual profiles',json.dumps(profiles),flush=True)
        print('Preview only. s save; r record <= case repeats; n next case; q exit. No automatic start.',flush=True)
        while True:
            frame=source.read();now=time.perf_counter();fps=1/max(now-last,1e-9);last=now
            case=plan['cases'][case_index];view=frame.color_bgr.copy();V=cv2.cvtColor(view,cv2.COLOR_BGR2HSV)[:,:,2]
            lines=[f"RED {case['case_id']} FPS {fps:.1f} Vmean {V.mean():.1f} recording {remaining}",
                's save | r finite record/stop | n next after placement | q exit',
                'GT must be independently measured; body/ring invalid']
            for i,line in enumerate(lines):cv2.putText(view,line,(10,30+i*28),0,.6,(0,255,255),2)
            cv2.imshow('D435i accuracy preview',view);key=cv2.waitKey(1)&255
            if key in (27,ord('q')):break
            if key==ord('n'):
                remaining=0;case_index=(case_index+1)%len(plan['cases'])
                print('Prepare next measured pose:',plan['cases'][case_index]['case_id'],'Then press s/r; no automatic saving',flush=True)
            elif key==ord('s'):save(frame,case)
            elif key==ord('r'):
                remaining=0 if remaining else min(case['repeat_frames'],10);next_save=now
            if remaining and now>=next_save:
                success=save(frame,case);remaining=remaining-1 if success else 0;next_save=time.perf_counter()+1.
    finally:source.stop();cv2.destroyAllWindows()
    print('session',session,'saved',saved,flush=True)
if __name__=='__main__':main()
