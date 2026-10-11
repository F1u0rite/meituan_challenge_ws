import json
import cv2
import numpy as np
import pytest

from perception.battery_only import PixelBatteryDetection
from perception.unified_detector import UnifiedBatteryDetector, detect_batteries, vision_result
from real_data.dataset import create_session, save_sample
from real_data.red_annotation import (active_regions, annotation_source, read_regions,
                                      replace_role_for_review, review_status,
                                      save_region, set_review_status)


def _candidate(color, vertices, confidence=.9):
    contour=np.asarray(vertices,np.int32).reshape(-1,1,2)
    rect=cv2.minAreaRect(contour)
    return PixelBatteryDetection(color, *rect[0], 0., confidence, contour, cv2.boxPoints(rect))


class FakeCandidates:
    def __init__(self, items):
        self.items=items

    def detect(self, _image):
        return self.items


def test_unified_four_colors_empty_and_pose_flags():
    image=np.full((320,700,3),30,np.uint8)
    for i,bgr in enumerate(((0,0,255),(0,255,255),(255,0,0),(0,255,0))):
        cv2.rectangle(image,(30+i*160,80),(110+i*160,210),bgr,-1)
    result=vision_result(image)
    assert result.coordinate_frame=='IMAGE_PIXEL' and result.valid
    assert {d.color for d in result.batteries if d.detection_valid}=={'red','yellow','blue','green'}
    assert all(d.body_pose is None and not d.body_pose_valid and
               d.ring_pose is None and not d.ring_pose_valid for d in result.batteries)
    assert all(d.mask.shape==image.shape[:2] and d.bounding_box.shape==(4,2)
               for d in result.batteries)
    assert detect_batteries(np.zeros_like(image))==[]
    with pytest.raises(ValueError):
        detect_batteries(np.zeros((30,30),np.uint8))


def test_cross_color_conflict_unknown_and_dominant_color():
    image=np.zeros((240,320,3),np.uint8)
    same=[(50,50),(150,50),(150,170),(50,170)]
    red=_candidate('red',same)
    yellow=_candidate('yellow',same)
    ambiguous=UnifiedBatteryDetector(candidate_detector=FakeCandidates([red,yellow]))
    found=ambiguous.detect_batteries(image)
    assert len(found)==1 and found[0].color=='unknown' and not found[0].detection_valid
    small=[(70,70),(105,70),(105,105),(70,105)]
    dominant=UnifiedBatteryDetector(candidate_detector=FakeCandidates([red,_candidate('yellow',small)]))
    found=dominant.detect_batteries(image)
    assert len(found)==1 and found[0].color=='red' and found[0].detection_valid


def test_background_geometry_filter_and_separate_objects():
    image=np.zeros((240,400,3),np.uint8)
    slender=_candidate('blue',[(5,5),(390,5),(390,14),(5,14)])
    red=_candidate('red',[(40,70),(100,70),(100,160),(40,160)])
    blue=_candidate('blue',[(230,70),(290,70),(290,160),(230,160)])
    found=UnifiedBatteryDetector(candidate_detector=FakeCandidates([slender,red,blue])).detect_batteries(image)
    assert {d.color for d in found}=={'red','blue'}


def test_codex_review_preserves_source_and_user_masks(tmp_path):
    session=create_session(tmp_path,'red','static')
    image=np.full((100,120,3),30,np.uint8)
    cv2.rectangle(image,(30,20),(90,80),(0,0,220),-1)
    sample,_=save_sample(session,image)
    for role,poly in (
        ('full_object',[(30,20),(90,20),(90,80),(30,80)]),
        ('color_surface',[(40,40),(80,40),(80,70),(40,70)]),
        ('negative_background',[(0,0),(20,0),(20,90),(0,90)])):
        save_region(sample,role,poly)
    raw_before=(sample/'raw.png').read_bytes()
    with pytest.raises(ValueError):
        set_review_status(sample,'user_accepted')
    path=sample/'annotation_regions.json'
    data=json.loads(path.read_text())
    data['annotation_kind']='codex_visual_provisional'
    data['annotation_source']='codex_provisional'
    data['review_status']='pending_user_review'
    path.write_text(json.dumps(data))
    assert annotation_source(sample)=='codex_provisional'
    set_review_status(sample,'user_accepted')
    assert review_status(sample)=='user_accepted'
    old_id=active_regions(sample,'color_surface')[0]['annotation_id']
    replace_role_for_review(sample,'color_surface',[(42,42),(78,42),(78,68),(42,68)])
    assert review_status(sample)=='pending_user_review'
    assert len(active_regions(sample,'color_surface'))==1
    assert any(r['annotation_id']==old_id and not r['active'] for r in read_regions(sample)['regions'])
    assert read_regions(sample)['annotation_source']=='codex_provisional'
    set_review_status(sample,'user_rejected')
    assert review_status(sample)=='user_rejected'
    assert (sample/'raw.png').read_bytes()==raw_before
