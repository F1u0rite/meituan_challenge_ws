"""Conservative RGB-only multiclass detector; leaves the existing live modes intact."""
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import cv2
import numpy as np
import yaml
from .battery_only import DEFAULT_CONFIG, PixelBatteryDetector

DEFAULT_FUSION_CONFIG = Path(__file__).resolve().parents[1]/'config'/'unified_detector.yaml'
COLORS = ('red', 'yellow', 'blue', 'green', 'unknown')


@dataclass
class BatteryResult:
    object_id: str
    color: str
    confidence: float
    pixel_center: tuple[float, float]
    yaw_deg: float
    contour: np.ndarray
    bounding_box: np.ndarray
    mask: np.ndarray
    detection_valid: bool
    body_pose: None = None
    body_pose_valid: bool = False
    ring_pose: None = None
    ring_pose_valid: bool = False


@dataclass
class VisionResult:
    timestamp: str
    coordinate_frame: str
    batteries: list[BatteryResult]
    targets: list
    valid: bool


def _mask(shape, contour):
    out = np.zeros(shape[:2], np.uint8)
    cv2.drawContours(out, [contour], -1, 1, -1)
    return out.astype(bool)


def _box_iou(a, b):
    x1, y1, w1, h1 = cv2.boundingRect(a.contour)
    x2, y2, w2, h2 = cv2.boundingRect(b.contour)
    ix = max(0, min(x1+w1, x2+w2)-max(x1, x2))
    iy = max(0, min(y1+h1, y2+h2)-max(y1, y2))
    intersection = ix*iy
    union = w1*h1+w2*h2-intersection
    return intersection/union if union else 0.0


class UnifiedBatteryDetector:
    def __init__(self, color_config=None, fusion_config=DEFAULT_FUSION_CONFIG,
                 candidate_detector=None):
        self.candidate_detector = candidate_detector or PixelBatteryDetector(color_config or DEFAULT_CONFIG)
        self.config = yaml.safe_load(Path(fusion_config).read_text(encoding='utf-8'))

    def _geometry_ok(self, candidate):
        w, h = cv2.minAreaRect(candidate.contour)[1]
        if min(w, h) <= 0:
            return False
        return (candidate.confidence >= self.config['min_fill_ratio'] and
                max(w, h)/min(w, h) <= self.config['max_aspect_ratio'])

    def _conflict(self, a, b, masks, areas=None):
        if a.color == b.color:
            return False
        ma, mb = masks[id(a)], masks[id(b)]
        x1,y1,w1,h1=cv2.boundingRect(a.contour);x2,y2,w2,h2=cv2.boundingRect(b.contour)
        left,top=max(x1,x2),max(y1,y2);right,bottom=min(x1+w1,x2+w2),min(y1+h1,y2+h2)
        if left>=right or top>=bottom:return False
        denominator=min(areas[id(a)],areas[id(b)]) if areas is not None else min(ma.sum(),mb.sum())
        overlap=np.logical_and(ma[top:bottom,left:right],mb[top:bottom,left:right]).sum()/max(denominator,1)
        return (overlap >= self.config['conflict_mask_overlap_small'] or
                _box_iou(a, b) >= self.config['conflict_box_iou'])

    def detect_batteries(self, image_bgr, *, hsv=None):
        if (image_bgr is None or not isinstance(image_bgr, np.ndarray) or
                image_bgr.dtype != np.uint8 or image_bgr.ndim != 3 or
                image_bgr.shape[2] != 3 or image_bgr.size == 0):
            raise ValueError('Expected nonempty HxWx3 BGR uint8 image')
        candidates = [d for d in (self.candidate_detector.detect(image_bgr, hsv=hsv) if isinstance(self.candidate_detector, PixelBatteryDetector) else self.candidate_detector.detect(image_bgr))
                      if self._geometry_ok(d)]
        masks = {id(d): _mask(image_bgr.shape, d.contour) for d in candidates}
        areas={key:int(mask.sum()) for key,mask in masks.items()}
        parent = list(range(len(candidates)))

        def root(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for i, first in enumerate(candidates):
            for j in range(i+1, len(candidates)):
                if self._conflict(first, candidates[j], masks,areas):
                    parent[root(j)] = root(i)
        groups = {}
        for i in range(len(candidates)):
            groups.setdefault(root(i), []).append(candidates[i])
        results = []
        for group in groups.values():
            # Large coherent colored regions outrank small cross-color patches.
            ranked = sorted(group, key=lambda d: areas[id(d)]*d.confidence,
                            reverse=True)
            best = ranked[0]
            ambiguous = (len(ranked) > 1 and
                         areas[id(best)]*best.confidence <
                         self.config['classification_margin_ratio']*
                         areas[id(ranked[1])]*ranked[1].confidence)
            color = 'unknown' if ambiguous else best.color
            results.append(BatteryResult(
                object_id=f'battery_{len(results)}', color=color,
                confidence=float(best.confidence),
                pixel_center=(float(best.u), float(best.v)),
                yaw_deg=float(best.yaw_deg), contour=best.contour,
                bounding_box=best.box_px, mask=masks[id(best)],
                detection_valid=not ambiguous))
        return results


_DEFAULT_DETECTOR = None


def detect_batteries(image_bgr):
    """Unified image-only API; coordinates and contour remain in input pixels."""
    global _DEFAULT_DETECTOR
    if _DEFAULT_DETECTOR is None:
        _DEFAULT_DETECTOR = UnifiedBatteryDetector()
    return _DEFAULT_DETECTOR.detect_batteries(image_bgr)


def vision_result(image_bgr, timestamp=None, detector=None):
    detector = detector or UnifiedBatteryDetector()
    found = detector.detect_batteries(image_bgr)
    return VisionResult(timestamp or datetime.now(timezone.utc).isoformat(),
                        'IMAGE_PIXEL', found, [], True)
