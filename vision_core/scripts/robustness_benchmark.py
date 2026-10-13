"""Measure current OpenCV vision limits under reproducible RGB domain gaps."""
import argparse,json,sys,time
from collections import Counter,defaultdict
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from synthetic.scene_generator import SceneGenerator,load_config
from synthetic.camera_projection import CameraProjection
from synthetic.robustness_augmentation import RobustnessAugmentation
from synthetic.io import write_png
from perception.board_detector import BoardDetector
from perception.board_rectifier import BoardRectifier
from perception.battery_detector import BatteryDetector
from perception.target_detector import TargetDetector
from evaluation.board_evaluator import evaluate_board
from evaluation.evaluator import Evaluator
from evaluation.failure_classifier import classify_board,classify_battery

def stats(values):
    return {k:(float(f(values)) if values else None) for k,f in [('mean',np.mean),('median',np.median),('p95',lambda x:np.percentile(x,95)),('max',np.max)]}

def sheet(images,cols=5,size=(320,240)):
    if not images: return np.full((size[1],size[0],3),230,np.uint8)
    thumbs=[cv2.resize(x,size) for x in images]; rows=[]
    while len(thumbs)%cols: thumbs.append(np.full_like(thumbs[0],230))
    for i in range(0,len(thumbs),cols): rows.append(np.concatenate(thumbs[i:i+cols],axis=1))
    return np.concatenate(rows,axis=0)

def annotate_board(image,detection):
    out=image.copy()
    if detection.detected:
        pts=np.rint(detection.corners_px).astype(np.int32); cv2.polylines(out,[pts],True,(0,255,255),3)
        for label,p in zip(('TL','TR','BR','BL'),pts): cv2.putText(out,label,tuple(p),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
    return out

def annotate_batteries(image,detections):
    out=image.copy()
    for d in detections:
        cv2.drawContours(out,[d.contour],-1,(0,0,0),2); c=tuple(round(x) for x in d.center_px)
        cv2.putText(out,f'{d.color} {d.x_mm:.1f},{d.y_mm:.1f} {d.yaw_deg:.1f}',c,cv2.FONT_HERSHEY_SIMPLEX,.35,(0,0,0),1)
    return out

def save_failure(root,kind,seed,camera,board_image,rectified,result,metadata):
    d=root/kind/f'seed_{seed}'; d.mkdir(parents=True,exist_ok=True)
    write_png(d/'camera_input.png',camera); write_png(d/'board_detection.png',board_image)
    write_png(d/'rectified.png',rectified); write_png(d/'result.png',result)
    (d/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')

def main():
    p=argparse.ArgumentParser(); p.add_argument('--difficulty',choices=['clean','medium','hard'],required=True)
    p.add_argument('--num-scenes',type=int,default=1000); p.add_argument('--seed',type=int,default=42)
    p.add_argument('--output-root',type=Path,default=Path(__file__).resolve().parents[1]/'outputs'/'robustness')
    a=p.parse_args(); root=a.output_root; run_root=root/a.difficulty; run_root.mkdir(parents=True,exist_ok=True)
    cfg=load_config(); gen=SceneGenerator(cfg); projector=CameraProjection(cfg); bd=BoardDetector(cfg); rectifier=BoardRectifier(cfg)
    battery=BatteryDetector(cfg); target=TargetDetector(cfg); evaluator=Evaluator(cfg)
    corner_errors=[]; xy=[]; yaw=[]; oracle_xy=[]; board_ok=0; detected_count=0; false_pos=0
    color_total=Counter(); color_correct=Counter(); failures=Counter(); bg=defaultdict(lambda:[0,0]); timings=defaultdict(list)
    contacts=[]; worst={}; saved=Counter(); scene_records=[]
    for index in range(a.num_scenes):
        seed=a.seed+index; scene=gen.generate(seed,mode='official'); cam=projector.project(scene.canonical_image,seed,a.difficulty)
        aug=RobustnessAugmentation(cfg,a.difficulty,seed).apply(cam.image,cam.board_corners_gt_px); image=aug.image
        if len(contacts)<25: contacts.append(image.copy())
        t0=time.perf_counter(); board=bd.detect(image); t1=time.perf_counter(); timings['board_detector_ms'].append((t1-t0)*1000)
        board_eval=evaluate_board(board,cam.board_corners_gt_px); bg[aug.metadata['background_type']][1]+=1
        board_image=annotate_board(image,board); rectified=np.full((1080,1400,3),220,np.uint8); detections=[]; rect_ok=False; vision_end=t1
        if board.detected:
            board_ok+=1; bg[aug.metadata['background_type']][0]+=1; corner_errors.extend(board_eval.corner_errors_px)
            try:
                t2=time.perf_counter(); rectified=rectifier.rectify(image,board.corners_px).image; t3=time.perf_counter(); rect_ok=True
                timings['rectification_ms'].append((t3-t2)*1000)
                t4=time.perf_counter(); detections=battery.detect(rectified); t5=time.perf_counter(); timings['battery_detector_ms'].append((t5-t4)*1000)
                t6=time.perf_counter(); target.detect(rectified); t7=time.perf_counter(); timings['target_detector_ms'].append((t7-t6)*1000); vision_end=t7
            except cv2.error: rect_ok=False
        board_fail=classify_board(board,board_eval.corner_errors_px,cfg['evaluation']['corner_threshold_px'],rect_ok)
        result=evaluator.evaluate(scene.ground_truth,detections); battery_fail=classify_battery(result,not rect_ok)
        detected=[c for c in result.cases if c['prediction'] is not None]; detected_count+=len(detected); false_pos+=result.false_positives
        for c in result.cases:
            color_total[c['gt']['color']]+=1
            if c['color_correct']: color_correct[c['gt']['color']]+=1
        xy.extend(c['xy_error_mm'] for c in detected); yaw.extend(c['yaw_error_deg'] for c in detected)
        # Oracle transform is evaluator-only attribution; it never feeds the official prediction.
        oracle=rectifier.rectify(image,cam.board_corners_gt_px).image; oracle_result=evaluator.evaluate(scene.ground_truth,battery.detect(oracle))
        oracle_xy.extend(c['xy_error_mm'] for c in oracle_result.cases if c['prediction'] is not None)
        reasons=board_fail+battery_fail
        mapping={'no_board_detected':'board_failure','wrong_quadrilateral':'board_failure','corner_order_failure':'board_failure','corner_error_too_large':'board_failure','rectification_failure':'board_failure',
                 'wrong_color':'color_failure','xy_error':'xy_failure','yaw_error':'yaw_failure','false_positive':'false_positive','missed_detection':'battery_failure','missed':'battery_failure','upstream_board_failure':'board_failure'}
        for reason in reasons: failures[reason]+=1
        result_image=annotate_batteries(rectified,detections)
        metadata={'scene_seed':seed,'difficulty':a.difficulty,'active_augmentations':aug.metadata['active'],'augmentation_parameters':aug.metadata['parameters'],
                  'background_type':aug.metadata['background_type'],'ground_truth':scene.ground_truth,'prediction':[vars(d)|{'contour':None} for d in detections],
                  'failure_reasons':reasons,'corner_errors_px':board_eval.corner_errors_px}
        for kind in sorted(set(mapping.get(x,'battery_failure') for x in reasons)):
            if saved[kind]<3: save_failure(run_root,kind,seed,image,board_image,rectified,result_image,metadata); saved[kind]+=1
        mean_corner=float(np.mean(board_eval.corner_errors_px)) if board_eval.corner_errors_px else 1e9
        mean_xy=float(np.mean([c['xy_error_mm'] for c in detected])) if detected else 1e9
        mean_yaw=float(np.mean([c['yaw_error_deg'] for c in detected])) if detected else 1e9
        for key,score in [('corner',mean_corner),('xy',mean_xy),('yaw',mean_yaw)]:
            if key not in worst or score>worst[key][0]: worst[key]=(score,result_image.copy())
        timings['total_ms'].append((vision_end-t0)*1000)
    color_accuracy={c:color_correct[c]/color_total[c] for c in ('red','yellow','blue','green')}
    report={'difficulty':a.difficulty,'master_seed':a.seed,'scenes':a.num_scenes,'batteries':a.num_scenes*4,
      'board':{'detection_success_rate':board_ok/a.num_scenes,'corner_error_px':stats(corner_errors),'success_by_background':{k:v[0]/v[1] for k,v in bg.items()}},
      'battery':{'detection_success_rate':detected_count/(a.num_scenes*4),'color_accuracy':sum(color_correct.values())/max(detected_count,1),'color_accuracy_by_class':color_accuracy,'false_positives':false_pos},
      'xy_error_mm':stats(xy),'yaw_error_deg':stats(yaw),'failures':dict(failures),
      'error_attribution':{'full_pipeline_xy_mean_mm':stats(xy)['mean'],'oracle_board_xy_mean_mm':stats(oracle_xy)['mean'],'estimated_board_component_mm':max(0,(stats(xy)['mean'] or 0)-(stats(oracle_xy)['mean'] or 0))},
      'latency_ms':{k:stats(v) for k,v in timings.items()}}
    (root/f'{a.difficulty}_benchmark.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    write_png(root/f'{a.difficulty}_contact_sheet.png',sheet(contacts))
    write_png(run_root/'worst_cases.png',sheet([v[1] for v in worst.values()],cols=3))
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
