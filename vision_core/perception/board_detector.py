"""Traditional image-only competition board quadrilateral detector."""
from dataclasses import dataclass
import cv2
import numpy as np


def order_corners(points):
    """Return TL, TR, BR, BL for a convex four-point set."""
    pts=np.asarray(points,dtype=np.float32).reshape(4,2)
    center=pts.mean(axis=0)
    angles=np.arctan2(pts[:,1]-center[1],pts[:,0]-center[0])
    pts=pts[np.argsort(angles)]
    start=int(np.argmin(pts[:,0]+pts[:,1]))
    pts=np.roll(pts,-start,axis=0)
    first=pts[1]-pts[0]; second=pts[2]-pts[1]
    if first[0]*second[1]-first[1]*second[0] < 0:
        pts=pts[[0,3,2,1]]
    return pts.astype(np.float32)


@dataclass
class BoardDetection:
    detected: bool
    corners_px: np.ndarray | None
    confidence: float
    contour: np.ndarray | None = None
    debug_mask: np.ndarray | None = None


class BoardDetector:
    def __init__(self,config): self.config=config

    def detect(self,image):
        cfg=self.config['board_detection']
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        _,mask=cv2.threshold(gray,int(cfg['threshold']),255,cv2.THRESH_BINARY)
        k=int(cfg['close_kernel_px'])|1
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((k,k),np.uint8))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        image_area=image.shape[0]*image.shape[1]
        candidates=[]
        expected=self.config['board']['width_mm']/self.config['board']['height_mm']
        for contour in contours:
            area=cv2.contourArea(contour)
            if area < cfg['min_area_fraction']*image_area: continue
            hull=cv2.convexHull(contour)
            peri=cv2.arcLength(hull,True)
            poly=cv2.approxPolyDP(hull,cfg['approximation_epsilon_fraction']*peri,True)
            if len(poly)!=4 or not cv2.isContourConvex(poly): continue
            corners=order_corners(poly[:,0,:])
            top=np.linalg.norm(corners[1]-corners[0]); bottom=np.linalg.norm(corners[2]-corners[3])
            left=np.linalg.norm(corners[3]-corners[0]); right=np.linalg.norm(corners[2]-corners[1])
            ratio=((top+bottom)/2)/((left+right)/2)
            ratio_error=abs(ratio-expected)/expected
            if ratio_error > cfg['expected_aspect_ratio_tolerance']: continue
            rectangularity=area/max(cv2.contourArea(hull),1)
            score=(area/image_area)*max(0.0,1-ratio_error)*rectangularity
            candidates.append((score,corners,contour))
        if not candidates: return BoardDetection(False,None,0.0,debug_mask=mask)
        score,corners,contour=max(candidates,key=lambda item:item[0])
        return BoardDetection(True,corners,float(min(1.0,score/0.55)),contour,mask)
