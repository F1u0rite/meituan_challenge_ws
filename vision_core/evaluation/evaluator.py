"""Color-independent matching, per-case errors, and aggregate competition metrics."""
from dataclasses import dataclass, field
from pathlib import Path
import json
import cv2
import numpy as np
from synthetic.geometry import yaw_error
from synthetic.io import write_png
from .metrics import summarize


@dataclass
class Evaluation:
    cases: list[dict]
    false_positives: int

    @property
    def failed_cases(self):
        return [c for c in self.cases if c['failed']]


class Evaluator:
    def __init__(self, config): self.config = config

    def evaluate(self, ground_truth, detections):
        available = list(detections)
        cases = []
        xy_limit = self.config['evaluation']['xy_threshold_mm']
        yaw_limit = self.config['evaluation']['yaw_threshold_deg']
        for gt in ground_truth['batteries']:
            if available:
                index = min(range(len(available)), key=lambda i:
                            (gt['x_mm']-available[i].x_mm)**2+(gt['y_mm']-available[i].y_mm)**2)
                pred = available.pop(index)
                xy = float(np.hypot(gt['x_mm']-pred.x_mm, gt['y_mm']-pred.y_mm))
                # A far-away nearest detection is a miss, not a valid match.
                if xy > max(self.config['battery']['width_mm'],self.config['battery']['height_mm']):
                    available.append(pred)
                    pred = None
            else: pred = None
            if pred is None:
                cases.append({'id':gt['id'],'gt':gt,'prediction':None,'xy_error_mm':None,
                              'yaw_error_deg':None,'color_correct':False,'failed':True,'reasons':['missed']})
                continue
            angle = yaw_error(gt['yaw_deg'], pred.yaw_deg)
            reasons = []
            if gt['color'] != pred.color: reasons.append('wrong_color')
            if xy > xy_limit: reasons.append('xy_error')
            if angle > yaw_limit: reasons.append('yaw_error')
            cases.append({'id':gt['id'],'gt':gt,
                          'prediction':{'color':pred.color,'x_mm':pred.x_mm,'y_mm':pred.y_mm,'yaw_deg':pred.yaw_deg},
                          'xy_error_mm':xy,'yaw_error_deg':angle,
                          'color_correct':gt['color']==pred.color,'failed':bool(reasons),'reasons':reasons})
        return Evaluation(cases, len(available))


def aggregate(evaluations):
    cases = [c for result in evaluations for c in result.cases]
    detected = [c for c in cases if c['prediction'] is not None]
    xy = summarize([c['xy_error_mm'] for c in detected])
    yaw = summarize([c['yaw_error_deg'] for c in detected])
    return {'scenes':len(evaluations),'batteries':len(cases),
            'detection_success_rate':len(detected)/len(cases) if cases else 0,
            'color_accuracy':sum(c['color_correct'] for c in detected)/len(detected) if detected else 0,
            'xy_error_mm':xy,'yaw_error_deg':yaw,
            'failed_cases':sum(c['failed'] for c in cases),
            'false_positives':sum(e.false_positives for e in evaluations)}


def save_failure(scene, evaluation, path, ppm):
    """Save annotated failure image and both truth/prediction records."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    image = scene.image.copy()
    canonical_to_image = np.linalg.inv(scene.image_to_canonical)
    for case in evaluation.failed_cases:
        gt = case['gt']; pred = case['prediction']
        gp = canonical_to_image @ [gt['x_mm']*ppm,gt['y_mm']*ppm,1]
        gp /= gp[2]
        g = (round(gp[0]),round(gp[1]))
        cv2.drawMarker(image,g,(255,0,255),cv2.MARKER_CROSS,25,3)
        cv2.putText(image,'GT '+gt['color'],g,cv2.FONT_HERSHEY_SIMPLEX,.5,(255,0,255),2)
        if pred:
            pp = canonical_to_image @ [pred['x_mm']*ppm,pred['y_mm']*ppm,1]
            pp /= pp[2]
            p = (round(pp[0]),round(pp[1]))
            cv2.drawMarker(image,p,(0,0,0),cv2.MARKER_TILTED_CROSS,25,3)
            cv2.putText(image,'P '+pred['color'],p,cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),2)
    write_png(path.with_suffix('.png'),image)
    path.with_suffix('.json').write_text(json.dumps({'ground_truth':scene.ground_truth,
                             'cases':evaluation.cases},indent=2),encoding='utf-8')
