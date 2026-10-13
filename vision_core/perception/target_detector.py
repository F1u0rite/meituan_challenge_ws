"""Locate circular target rings in the image and report millimetre ROIs."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class TargetDetection:
    name: str
    x_mm: float
    y_mm: float
    inner_radius_mm: float
    outer_radius_mm: float


class TargetDetector:
    def __init__(self, config): self.config = config

    def detect(self, image, image_to_canonical=None):
        ppm = self.config['render']['pixels_per_mm']
        H = np.eye(3) if image_to_canonical is None else image_to_canonical
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 1)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=120*ppm,
                                  param1=100, param2=40,
                                  minRadius=round(60*ppm), maxRadius=round(90*ppm))
        if circles is None: return []
        candidates = []
        for u,v,r in circles[0]:
            p = H @ [u,v,1]
            p /= p[2]
            candidates.append((p[0]/ppm,p[1]/ppm,r/ppm))
        # Names are assigned by configured approximate reference positions, not ground truth.
        result = []
        for name, center in self.config['targets']['centers_mm'].items():
            if not candidates: break
            index = min(range(len(candidates)), key=lambda i: (candidates[i][0]-center[0])**2+(candidates[i][1]-center[1])**2)
            x,y,r = candidates.pop(index)
            result.append(TargetDetection(name,float(x),float(y),self.config['targets']['inner_size_mm']/2,float(r)))
        return result
