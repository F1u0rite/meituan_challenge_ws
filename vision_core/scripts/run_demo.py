"""Save all five stages of the camera-to-board perception demo."""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from synthetic.scene_generator import SceneGenerator,load_config
from synthetic.camera_projection import CameraProjection
from synthetic.io import write_png
from perception.board_detector import BoardDetector
from perception.board_rectifier import BoardRectifier
from perception.battery_detector import BatteryDetector
from evaluation.evaluator import Evaluator
from evaluation.board_evaluator import evaluate_board

def main():
    p=argparse.ArgumentParser(); p.add_argument('--seed',type=int,default=42)
    p.add_argument('--mode',choices=['official','random_stress'],default='official')
    p.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parents[1]/'outputs'/'demo_official')
    a=p.parse_args(); cfg=load_config(); a.output_dir.mkdir(parents=True,exist_ok=True)
    scene=SceneGenerator(cfg).generate(a.seed,mode=a.mode)
    camera=CameraProjection(cfg).project(scene.canonical_image,a.seed)
    board=BoardDetector(cfg).detect(camera.image)
    write_png(a.output_dir/'01_canonical_official.png',scene.canonical_image)
    write_png(a.output_dir/'02_camera.png',camera.image)
    overlay=camera.image.copy(); labels=['TL','TR','BR','BL']
    if board.detected:
        pts=np.rint(board.corners_px).astype(np.int32)
        cv2.polylines(overlay,[pts],True,(0,255,255),4)
        for label,point in zip(labels,pts):
            cv2.circle(overlay,tuple(point),9,(0,0,255),-1)
            cv2.putText(overlay,label,tuple(point+[12,-12]),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
    write_png(a.output_dir/'03_board_detection.png',overlay)
    if not board.detected: raise RuntimeError('Board was not detected in demo')
    rectified=BoardRectifier(cfg).rectify(camera.image,board.corners_px).image
    write_png(a.output_dir/'04_rectified.png',rectified)
    detections=BatteryDetector(cfg).detect(rectified)
    result=Evaluator(cfg).evaluate(scene.ground_truth,detections)
    visual=rectified.copy()
    for item in detections:
        cv2.drawContours(visual,[item.contour],-1,(0,0,0),3); c=tuple(round(v) for v in item.center_px)
        angle=np.deg2rad(item.yaw_deg); end=(round(c[0]-55*np.sin(angle)),round(c[1]+55*np.cos(angle)))
        cv2.arrowedLine(visual,c,end,(0,0,0),3)
        cv2.putText(visual,item.color,(c[0]-48,c[1]-22),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,0,0),1,cv2.LINE_AA)
        cv2.putText(visual,f'{item.x_mm:.1f},{item.y_mm:.1f}  {item.yaw_deg:.1f}deg',
                    (c[0]-62,c[1]+30),cv2.FONT_HERSHEY_SIMPLEX,.32,(0,0,0),1,cv2.LINE_AA)
    write_png(a.output_dir/'05_result.png',visual)
    be=evaluate_board(board,camera.board_corners_gt_px)
    report={'board_detected':board.detected,'corner_errors_px':be.corner_errors_px,'battery_cases':result.cases,
            'board_corners_gt_px':camera.board_corners_gt_px.tolist(),'board_corners_pred_px':board.corners_px.tolist()}
    (a.output_dir/'demo_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(f'board={board.detected} corner_mean={np.mean(be.corner_errors_px):.3f}px batteries={len(detections)} failed={len(result.failed_cases)}')

if __name__=='__main__': main()
