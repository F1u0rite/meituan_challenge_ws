"""HSV segmentation and contour based battery detection; accepts image only."""
from dataclasses import dataclass
import cv2
import numpy as np
from synthetic.geometry import mm_to_px
from .pose_estimator import estimate_pose


@dataclass
class BatteryDetection:
    color: str
    center_px: tuple[float, float]
    x_mm: float
    y_mm: float
    yaw_deg: float
    confidence: float
    contour: np.ndarray


class BatteryDetector:
    def __init__(self, config):
        self.config = config

    def detect(self, image, image_to_canonical=None):
        ppm = self.config['render']['pixels_per_mm']
        H = np.eye(3) if image_to_canonical is None else image_to_canonical
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        found = []
        min_area = self.config['battery']['width_mm']*self.config['battery']['height_mm']*ppm*ppm*.45
        max_area = self.config['battery']['width_mm']*self.config['battery']['height_mm']*ppm*ppm*1.5
        kernel = np.ones((3,3), np.uint8)
        for color, ranges in self.config['color_models']['hsv_ranges'].items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo_h,lo_s,lo_v,hi_h,hi_s,hi_v in ranges:
                mask |= cv2.inRange(hsv, (lo_h,lo_s,lo_v), (hi_h,hi_s,hi_v))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if not min_area <= area <= max_area:
                    continue
                (x,y), yaw, _ = estimate_pose(contour, H, ppm)
                p = np.array([x*ppm,y*ppm,1.0])
                raw = np.linalg.inv(H) @ p
                raw /= raw[2]
                found.append(BatteryDetection(color, (float(raw[0]),float(raw[1])), x,y,yaw,
                                              min(1.0, area/(min_area/.45)), contour))
        return sorted(found, key=lambda d: d.color)
