"""Reviewed object masks: rectangles, polygons, or hand-painted binary masks."""
from datetime import datetime, timezone
from pathlib import Path
import json
import uuid
import cv2
import numpy as np
from synthetic.io import write_png
from .dataset import LABELS, load_raw


def roi_mask(shape, mode, *, vertices=None, binary_mask=None):
    height, width = shape[:2]
    mask = np.zeros((height, width), np.uint8)
    if mode in ('rectangle', 'polygon'):
        points = np.asarray(vertices, dtype=np.int32)
        minimum = 2 if mode == 'rectangle' else 3
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < minimum:
            raise ValueError('ROI has too few vertices')
        if np.any(points[:, 0] < 0) or np.any(points[:, 0] >= width) or np.any(points[:, 1] < 0) or np.any(points[:, 1] >= height):
            raise ValueError('ROI extends beyond image')
        if mode == 'rectangle':
            cv2.rectangle(mask, tuple(points[0]), tuple(points[1]), 255, -1)
        else:
            cv2.fillPoly(mask, [points], 255)
    elif mode == 'mask':
        if binary_mask is None or np.asarray(binary_mask).shape != (height, width):
            raise ValueError('Binary mask size does not match image')
        mask = (np.asarray(binary_mask) > 0).astype(np.uint8)*255
    else:
        raise ValueError('mode must be rectangle, polygon, or mask')
    if cv2.countNonZero(mask) == 0:
        raise ValueError('ROI is empty')
    return mask


def annotate(sample, color, mode, *, vertices=None, binary_mask=None,
             yaw_gt_deg=None, yaw_gt_source=None):
    if color not in LABELS[:4]:
        raise ValueError('Object color must be red, yellow, blue, or green')
    if yaw_gt_deg is not None:
        if not 0 <= float(yaw_gt_deg) < 180 or not yaw_gt_source:
            raise ValueError('Yaw ground truth needs a 0..180 degree value and measured source')
    sample = Path(sample)
    image = load_raw(sample)
    mask = roi_mask(image.shape, mode, vertices=vertices, binary_mask=binary_mask)
    path = sample/'annotation.json'
    data = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {
        'sample_id': sample.name, 'raw_image': 'raw.png', 'reviewed': True, 'objects': []}
    annotation_id = uuid.uuid4().hex
    mask_name = f'mask_{annotation_id}.png'
    write_png(sample/mask_name, mask)
    data['objects'].append({'annotation_id': annotation_id, 'color': color, 'mode': mode,
                            'vertices_px': np.asarray(vertices).tolist() if vertices is not None else None,
                            'mask_path': mask_name,
                            'yaw_gt_deg': float(yaw_gt_deg) if yaw_gt_deg is not None else None,
                            'yaw_gt_source': yaw_gt_source,
                            'annotated_at': datetime.now(timezone.utc).isoformat()})
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')
    return data['objects'][-1]


def mark_negative(sample):
    sample = Path(sample)
    metadata = json.loads((sample/'metadata.json').read_text(encoding='utf-8'))
    if metadata['capture_label'] != 'negative':
        raise ValueError('Only a negative capture can be marked as reviewed empty ground truth')
    data = {'sample_id': sample.name, 'raw_image': 'raw.png', 'reviewed': True,
            'objects': [], 'annotated_at': datetime.now(timezone.utc).isoformat()}
    path = sample/'annotation.json'
    if path.exists():
        raise FileExistsError(path)
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')
    return data


def read_annotation(sample):
    path = Path(sample)/'annotation.json'
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding='utf-8'))
    if not data.get('reviewed'):
        return None
    return data


def read_mask(sample, item):
    encoded = np.fromfile(str(Path(sample)/item['mask_path']), np.uint8)
    mask = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise ValueError('Cannot read annotation mask')
    return mask > 0
