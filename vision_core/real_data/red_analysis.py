"""Red calibration diagnostics; never reports independent validation accuracy."""
from datetime import datetime, timezone
from pathlib import Path
import json
import shutil
import cv2
import numpy as np
import yaml
from perception.battery_only import PixelBatteryDetector
from synthetic.io import write_png
from .calibration import describe, hue_arc, suggest_ranges, _within
from .dataset import load_raw
from .red_annotation import ROLES, complete, combined_mask, read_regions, selected_annotation_status
from .replay import contour_mask, match_one_to_one


def _pixels(hsv, mask, cap=50000):
    values=hsv[mask]
    return values[np.linspace(0,len(values)-1,cap,dtype=int)] if len(values)>cap else values


def _histogram(pixels):
    canvas=np.full((630,700,3),255,np.uint8)
    for row,(name,bins) in enumerate((('H',180),('S',256),('V',256))):
        hist=np.bincount(pixels[:,row],minlength=bins).astype(float)[:bins]
        if hist.max(): hist/=hist.max()
        y0=195+row*205
        cv2.putText(canvas,f'{name} distribution',(15,y0-175),cv2.FONT_HERSHEY_SIMPLEX,.75,(0,0,0),2)
        for i,height in enumerate(hist):
            x=15+round(i*660/bins)
            cv2.line(canvas,(x,y0),(x,y0-round(height*140)),(0,0,180),2)
    return canvas


def _red_mask(image,ranges):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=np.zeros(hsv.shape[:2],np.uint8)
    for h0,s0,v0,h1,s1,v1 in ranges:
        mask|=cv2.inRange(hsv,(h0,s0,v0),(h1,s1,v1))
    return mask>0


def _overlay(image, full, background, detections, reasons):
    view=image.copy()
    for mask,color in ((full,(0,255,255)),(background,(255,0,0))):
        contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(view,contours,-1,color,3)
    for d in detections:
        color={'red':(0,0,255),'yellow':(0,255,255),'blue':(255,0,0),'green':(0,255,0)}[d.color]
        cv2.drawContours(view,[d.contour],-1,color,2)
    cv2.putText(view,','.join(sorted(set(reasons)))[:90],(12,35),
                cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2,cv2.LINE_AA)
    return view


def _contact(images,path):
    if not images: return
    tiles=[]
    for image in images[:24]:
        tile=cv2.resize(image,(320,180))
        tiles.append(tile)
    while len(tiles)%4: tiles.append(np.full_like(tiles[0],30))
    write_png(path,np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)]))


def run_red_analysis(selection_path, base_config, output_dir):
    selection=json.loads(Path(selection_path).read_text(encoding='utf-8'))
    output_dir=Path(output_dir); output_dir.mkdir(parents=True,exist_ok=True)
    status=selected_annotation_status(selection_path)
    pending=[row['sample_id'] for row in status if not row['complete']]
    if pending:
        report={'status':'ANNOTATION REQUIRED','completed':len(status)-len(pending),
                'selected':len(status),'pending_sample_ids':pending,
                'calibration_diagnostic':'NOT RUN'}
        (output_dir/'annotation_status.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        return report
    provenance={'user_annotated':0,'codex_provisional':0,'other':0}
    for row in selection['selected']:
        kind=read_regions(row['sample_path']).get('annotation_kind','')
        if kind=='codex_visual_provisional': provenance['codex_provisional']+=1
        elif kind=='human_polygon': provenance['user_annotated']+=1
        else: provenance['other']+=1
    (output_dir/'annotation_status.json').write_text(json.dumps({
        'status':'COMPLETE', 'completed':len(status), 'selected':len(status),
        'annotation_provenance':provenance,
        'annotation_review_status':'PENDING USER REVIEW' if provenance['codex_provisional'] else 'USER ANNOTATED',
        'calibration_diagnostic':'RUN'
    },indent=2),encoding='utf-8')
    surface_parts=[]; background_parts=[]
    for row in selection['selected']:
        sample=Path(row['sample_path']); hsv=cv2.cvtColor(load_raw(sample),cv2.COLOR_BGR2HSV)
        surface_parts.append(_pixels(hsv,combined_mask(sample,'color_surface')))
        background_parts.append(_pixels(hsv,combined_mask(sample,'negative_background')))
    surface=np.concatenate(surface_parts); background=np.concatenate(background_parts)
    ranges,screen=suggest_ranges(surface,background)
    stats={'source':'measured_real','status':'provisional',
           'calibration_session':selection['session_id'],
           'annotated_images':len(selection['selected']),
           'annotation_provenance':provenance,
           'annotation_review_status':'PENDING USER REVIEW' if provenance['codex_provisional'] else 'USER ANNOTATED',
           'surface_pixel_count':len(surface),'background_pixel_count':len(background),
           'hsv':{name:describe(surface[:,i]) for i,name in enumerate('hsv')},
           'hue_circular_arc_p05_p95':hue_arc(surface[:,0],5),
           'histograms':{name:np.bincount(surface[:,i],minlength=bins)[:bins].tolist()
                         for i,(name,bins) in enumerate((('h',180),('s',256),('v',256)))},
           'recommended_red_ranges':ranges,'background_pixel_screen':screen,
           'validation_status':'NOT RUN'}
    (output_dir/'red_hsv_statistics.json').write_text(json.dumps(stats,indent=2),encoding='utf-8')
    write_png(output_dir/'red_hsv_histogram.png',_histogram(surface))
    base=yaml.safe_load(Path(base_config).read_text(encoding='utf-8'))
    provisional=dict(base); provisional['hsv_ranges']=dict(base['hsv_ranges'])
    provisional['hsv_ranges']['red']=ranges
    provisional['provenance']={'source':'measured_real','status':'provisional',
                               'validation_status':'NOT RUN','source_session':selection['session_id'],
                               'annotated_images':len(selection['selected']),
                               'annotation_provenance':provenance,
                               'annotation_review_status':'PENDING USER REVIEW' if provenance['codex_provisional'] else 'USER ANNOTATED',
                               'generation_timestamp':datetime.now(timezone.utc).isoformat(),
                               'method':'circular red hue percentiles screened against annotated background ROIs'}
    proposed_path=output_dir/'red_threshold_provisional.yaml'
    proposed_path.write_text(yaml.safe_dump(provisional,sort_keys=False,allow_unicode=True),encoding='utf-8')
    old_detector=PixelBatteryDetector(base_config); new_detector=PixelBatteryDetector(proposed_path)
    selected_paths={row['sample_id']:Path(row['sample_path']) for row in selection['selected']}
    session=Path(selection['selected'][0]['sample_path']).parents[1]
    reports={}; failure_images=[]
    for name,detector in (('original',old_detector),('provisional',new_detector)):
        summary={'candidate_frames':{c:0 for c in ('red','yellow','blue','green')},
                 'annotated_count':0,'red_match':0,'wrong_color_match':0,
                 'red_false_negative':0,'unmatched_candidates':{c:0 for c in ('red','yellow','blue','green')},
                 'confidence_by_color':{c:[] for c in ('red','yellow','blue','green')},
                 'matched_iou':[],'failure_reasons':{},'annotated_frames':[]}
        for sample in sorted((session/'samples').iterdir()):
            if not (sample/'raw.png').exists(): continue
            image=load_raw(sample); detections=detector.detect(image)
            for color in summary['candidate_frames']:
                summary['candidate_frames'][color]+=int(any(d.color==color for d in detections))
            if sample.name not in selected_paths: continue
            summary['annotated_count']+=1
            full=combined_mask(sample,'full_object')
            background=combined_mask(sample,'negative_background')
            surface=combined_mask(sample,'color_surface')
            matches,missing,extra=match_one_to_one([full],detections,image.shape,.3)
            reasons=[]
            matched_colors=[]
            for _gi,pi,iou in matches:
                d=detections[pi]; matched_colors.append(d.color); summary['matched_iou'].append(iou)
                if d.color=='red': summary['red_match']+=1
                else: summary['wrong_color_match']+=1; reasons.append('wrong_color')
            if not matches or matched_colors!=['red']:
                summary['red_false_negative']+=1; reasons.append('false_negative')
            for pi in extra:
                d=detections[pi]; summary['unmatched_candidates'][d.color]+=1
                mask=contour_mask(image.shape,d.contour)
                if mask.sum() and np.logical_and(mask,background).sum()/mask.sum()>.2:
                    reasons.append('background_color_similarity')
                else:
                    reasons.append('unreviewed_area_candidate')
            for d in detections:
                summary['confidence_by_color'][d.color].append(d.confidence)
            red_mask=_red_mask(image,detector.config['hsv_ranges']['red'])
            surface_coverage=float(np.logical_and(red_mask,surface).sum()/max(surface.sum(),1))
            if surface_coverage<.8: reasons.append('HSV_threshold_miss')
            if red_mask[full].sum()>1000 and not any(d.color=='red' and
                  np.logical_and(contour_mask(image.shape,d.contour),full).sum()>0 for d in detections):
                reasons.append('contour_filtering')
            if any(d.color=='red' and np.logical_and(contour_mask(image.shape,d.contour),full).sum()>0
                   for d in detections) and not any(d.color=='red' for _gi,pi,_iou in matches
                                                    for d in [detections[pi]]):
                reasons.append('shape_or_partial_surface_IoU')
            for reason in set(reasons):
                summary['failure_reasons'][reason]=summary['failure_reasons'].get(reason,0)+1
            item={'sample_id':sample.name,'matched':bool(matches),'matched_colors':matched_colors,
                  'matched_iou':[float(x[2]) for x in matches],
                  'surface_red_coverage':surface_coverage,
                  'unmatched_predictions':[detections[i].color for i in extra],
                  'reasons':sorted(set(reasons))}
            summary['annotated_frames'].append(item)
            if reasons:
                folder=output_dir/'failures'/name/sample.name
                folder.mkdir(parents=True,exist_ok=True)
                shutil.copy2(sample/'raw.png',folder/'raw.png')
                overlay=_overlay(image,full,background,detections,reasons)
                write_png(folder/'prediction_and_gt.png',overlay)
                (folder/'metadata.json').write_text(json.dumps(item,indent=2),encoding='utf-8')
                if len(failure_images)<24: failure_images.append(overlay)
        reports[name]=summary
    report={'status':'CALIBRATION DIAGNOSTIC','source_session':selection['session_id'],
            'independent_validation':'NOT RUN','region_annotated_images':len(selection['selected']),
            'annotation_provenance':provenance,
            'annotation_review_status':'PENDING USER REVIEW' if provenance['codex_provisional'] else 'USER ANNOTATED',
            'original':reports['original'],'provisional':reports['provisional'],
            'interpretation':'IoU and candidate counts use calibration-session region annotations; assistant-provisional regions await user review. No independent precision/recall.'}
    (output_dir/'red_ab_diagnostic.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    _contact(failure_images,output_dir/'failure_contact_sheet.png')
    return report
