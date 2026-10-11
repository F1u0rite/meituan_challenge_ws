"""Image-pixel battery detection without board geometry or millimetre coordinates."""
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
import cv2
import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'realsense_color.yaml'


@dataclass
class PixelBatteryDetection:
    color: str
    u: float
    v: float
    yaw_deg: float
    confidence: float
    contour: np.ndarray
    box_px: np.ndarray


class PixelBatteryDetector:
    def __init__(self, config_path=DEFAULT_CONFIG):
        self.config = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
        self._validate()

    def _validate(self):
        config = self.config
        if set(config['hsv_ranges']) != {'red', 'yellow', 'blue', 'green'}:
            raise ValueError('RealSense color config must define red, yellow, blue, green')
        for ranges in config['hsv_ranges'].values():
            for bounds in ranges:
                if len(bounds) != 6 or any(not 0 <= v <= maximum for v, maximum in
                                           zip(bounds, (179, 255, 255, 179, 255, 255))):
                    raise ValueError(f'Invalid HSV bounds: {bounds}')
        if config['min_contour_area_px'] <= 0 or not 0 < config['max_contour_area_fraction'] <= 1:
            raise ValueError('Invalid contour area limits')

    def detect(self, image, *, timing_ms=None, hsv=None):
        if image is None or image.size == 0 or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('Battery-only input must be HxWx3 BGR uint8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) if hsv is None else hsv
        config = self.config
        kernel_size = int(config['morphology_kernel_px'])
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        image_area = image.shape[0] * image.shape[1]
        detections = []
        for color, ranges in config['hsv_ranges'].items():
            color_start = perf_counter() if timing_ms is not None else None
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for h0, s0, v0, h1, s1, v1 in ranges:
                mask |= cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if not config['min_contour_area_px'] <= area <= config['max_contour_area_fraction'] * image_area:
                    continue
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                edges = np.roll(box, -1, axis=0) - box
                axis = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
                yaw = float(np.degrees(np.arctan2(-axis[0], axis[1])) % 180)
                fill = area / max(rect[1][0] * rect[1][1], 1)
                detections.append(PixelBatteryDetection(color, *map(float, rect[0]), yaw,
                                                         float(np.clip(fill, 0, 1)), contour, box))
            if timing_ms is not None:
                timing_ms[color] = (perf_counter()-color_start)*1000
        return sorted(detections, key=lambda d: (d.color, -cv2.contourArea(d.contour)))
