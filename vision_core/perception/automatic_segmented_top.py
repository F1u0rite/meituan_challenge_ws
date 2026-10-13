"""Opt-in automatic RGB-D -> SAM cap -> green/red centre candidates."""
import copy
from time import perf_counter
import cv2
import numpy as np
from input.rgbd_pair_guard import RGBDPairGuard
from .rgbd_geometry import frame_capture_time
from .simplified_rgbd import BatteryDetection
from .automatic_top_prompt import AutomaticTopPrompt
from .segmented_top_center import SegmentedTopCenter


class AutomaticSegmentedTop:
    def __init__(self,config,geometry,*,predictor,top_span_mm=(70.,25.)):
        self.detector=BatteryDetection(config)
        self.proposer=AutomaticTopPrompt()
        measured=copy.deepcopy(geometry)
        face=next(f for f in measured['battery']['planar_faces'] if f['face_index']==27)
        x0,y0,z0,x1,y1,z1=face['bbox_mm']
        cx,cy=(x0+x1)/2,(y0+y1)/2
        face['bbox_mm']=[cx-top_span_mm[0]/2,cy-top_span_mm[1]/2,z0,
            cx+top_span_mm[0]/2,cy+top_span_mm[1]/2,z1]
        face['area_mm2']=float(np.prod(top_span_mm))
        self.center=SegmentedTopCenter(measured)
        self.predictor=predictor
        self.guard=RGBDPairGuard()
        self.last_timestamp=None
        self.top_span_mm=top_span_mm

    def process(self,frame,source_frame_id):
        start=perf_counter();stamp,stamp_source=frame_capture_time(frame)
        if self.last_timestamp is not None and stamp<=self.last_timestamp:
            raise ValueError('Non-increasing capture time')
        self.last_timestamp=stamp
        pair=self.guard.inspect(frame.timestamps)
        qualified=pair['qualified'] and (frame.alignment_metadata or {}).get('method')=='SDK_DEPTH_TO_COLOR'
        proposals,_,_=self.detector.detect(frame.color_bgr,stamp)
        groups={}
        for p in proposals:
            # Initial RGB association remains diagnostic; it does not turn a
            # single-frame geometry result into confirmed identity or motion.
            if any(r!='RGB_ASSOCIATION_NOT_ESTABLISHED' for r in p['reasons']):continue
            groups.setdefault(p['color'],[]).append(p)
        masks={c:ps[0]['battery_mask'] for c,ps in groups.items() if len(ps)==1}
        prompts=self.proposer.propose(frame,masks,qualified=qualified)
        result=dict(source_frame_id=source_frame_id,capture_timestamp=stamp,
            capture_timestamp_source=stamp_source,timestamps=dict(frame.timestamps),
            coordinate_frame='camera_color_optical',unit_position='m',pair_qualified=qualified,
            manual_prompts=False,top_span_reference_mm=list(self.top_span_mm),
            support_plane=prompts['support_plane'],objects=[],missing_colors=[],
            reasons=prompts['reasons'],body_pose_valid=False,ring_pose_valid=False,motion_usable=False)
        if not qualified:result['reasons']=result['reasons']+['RGBD_PAIR_REJECTED']
        available=[c for c,p in prompts['objects'].items() if p['available']]
        if available:self.predictor.set_image(cv2.cvtColor(frame.color_bgr,cv2.COLOR_BGR2RGB))
        for color in ['red','yellow','blue','green']:
            item=dict(color=color,candidate_available=False,green_point_camera_m=None,
                red_point_camera_m=None,rgb_confirmed=False,geometry_consistent=False,reasons=[])
            result['objects'].append(item)
            if len(groups.get(color,[]))>1:
                item['reasons']=['AMBIGUOUS_MULTIPLE_OBJECTS_OF_SAME_COLOR']
            elif color not in masks:
                item['reasons']=['NO_ELIGIBLE_CURRENT_RGB_OBJECT']
            elif color not in available:
                item['reasons']=prompts['objects'].get(color,{}).get('reasons',result['reasons'] or ['NO_CURRENT_TOP_PROMPT'])
            else:
                p=prompts['objects'][color];seed=p['seed_mask'];object_mask=masks[color]
                item['rgb_confirmed']=not groups[color][0]['reasons']
                item['prompt']={k:v for k,v in p.items() if k!='seed_mask'}
                predicted,scores,_=self.predictor.predict(box=np.array(p['box_px']),
                    point_coords=np.array(p['point_coords_px']),point_labels=np.array(p['point_labels']),
                    multimask_output=True)
                choices=[]
                for i,raw in enumerate(predicted):
                    raw=np.asarray(raw,bool)
                    if raw.shape!=object_mask.shape:raise ValueError('SAM mask shape mismatch')
                    area=int(raw.sum());coverage=float((raw&seed).sum()/max(1,seed.sum()))
                    outside=float((raw&~object_mask).sum()/max(1,area))
                    ratio=area/max(1,seed.sum())
                    if coverage>=.6 and outside<=.2 and .4<=ratio<=4.:
                        choices.append((float(scores[i]),i,raw&object_mask,coverage,outside))
                if not choices:item['reasons']=['NO_SAM_MASK_WITH_CURRENT_CAP_SUPPORT']
                else:
                    _,index,top_mask,coverage,outside=max(choices,key=lambda x:x[0])
                    reference=dict(prompts['support_plane'],
                        color_frame_number=frame.timestamps.get('color_frame_number'),
                        color_sdk_ms=frame.timestamps.get('color_sdk_ms'))
                    geometry=self.center.estimate(frame,top_mask,qualified=qualified,
                        support_plane=reference,plane_seed_mask=seed)
                    item.update(candidate_available=geometry['candidate_available'],
                        green_point_camera_m=geometry['top_center_candidate'],
                        red_point_camera_m=geometry['ring_center_candidate'],
                        geometry_consistent=geometry['geometry_consistent'],geometry=geometry,
                        sam_quality_score=float(scores[index]),sam_selected_candidate=index,
                        sam_seed_coverage=coverage,sam_outside_object_fraction=outside,
                        sam_score_is_not_accuracy=True,reasons=geometry['reasons'])
                    item['top_mask']=top_mask
                    support=prompts['support_plane']
                    if geometry['candidate_available'] and support:
                        n=np.array(support['normal_camera_xyz']);o=np.array(support['point_camera_m'])
                        item['green_height_m']=float((np.array(geometry['top_center_candidate'])-o)@n)
                        item['red_height_m']=float((np.array(geometry['ring_center_candidate'])-o)@n)
            if not item['candidate_available']:result['missing_colors'].append(color)
        result['processing_ms']=(perf_counter()-start)*1000
        return result
