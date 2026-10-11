"""Independent masked HSV evidence; shared ranges, including red wrap-around."""
import cv2,numpy as np,yaml
from .battery_only import DEFAULT_CONFIG

class ColorClassifier:
    def __init__(self,config_path=DEFAULT_CONFIG,min_purity=.50,min_margin=.15):
        self.ranges=yaml.safe_load(config_path.read_text(encoding='utf-8-sig'))['hsv_ranges']
        self.min_purity=min_purity;self.min_margin=min_margin
    def classify(self,image,mask,*,hsv=None):
        if image is None or image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
            raise ValueError('BGR uint8 required')
        mask=np.asarray(mask,bool)
        if mask.shape!=image.shape[:2]:raise ValueError('Mask shape mismatch')
        count=int(mask.sum())
        if not count:return dict(color_label='unknown',color_confidence=0.,color_evidence={'reason':'EMPTY_MASK','pixel_count':0})
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV) if hsv is None else hsv
        votes={}
        for color,ranges in self.ranges.items():
            selected=np.zeros(mask.shape,np.uint8)
            for h0,s0,v0,h1,s1,v1 in ranges:selected|=cv2.inRange(hsv,(h0,s0,v0),(h1,s1,v1))
            votes[color]=float(np.count_nonzero((selected>0)&mask)/count)
        ranked=sorted(votes,key=votes.get,reverse=True);best=ranked[0]
        margin=votes[best]-votes[ranked[1]]
        certain=votes[best]>=self.min_purity and margin>=self.min_margin
        return dict(color_label=best if certain else 'unknown',
            color_confidence=votes[best] if certain else 0.,
            color_evidence=dict(pixel_count=count,mask_pixel_fractions=votes,
                dominance_margin=margin,reason='MASK_HSV_DOMINANCE' if certain else 'LOW_PURITY_OR_OVERLAP'))
