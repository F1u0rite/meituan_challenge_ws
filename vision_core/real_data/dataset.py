"""Immutable real captures and session-level calibration/validation allocation."""
from datetime import datetime, timezone
from pathlib import Path
import json
import uuid
import cv2
import numpy as np
from synthetic.io import write_png
from .quality import diagnose

LABELS = ('red', 'yellow', 'blue', 'green', 'negative', 'unknown')
SPLITS = ('calibration', 'validation')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def create_session(root, label, capture_mode, *, notes='', session_id=None):
    if label not in LABELS or capture_mode not in ('static', 'handheld'):
        raise ValueError('Invalid label or capture mode')
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    session_id = session_id or f'session_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}_{label}'
    folder = root/session_id
    folder.mkdir(exist_ok=False)
    (folder/'samples').mkdir()
    info = {'session_id': session_id, 'capture_label': label, 'capture_mode': capture_mode,
            'created_at': utc_now(), 'notes': notes, 'split': 'UNASSIGNED'}
    (folder/'session.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
    return folder


def save_sample(session, image, *, timestamp=None, frame_number=None, camera_metadata=None,
                quality_config=None, overlay=None):
    session = Path(session)
    info = json.loads((session/'session.json').read_text(encoding='utf-8'))
    if image is None or image.size == 0 or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected nonempty BGR uint8 capture')
    sample_id = f'{info["session_id"]}_{uuid.uuid4().hex[:12]}'
    folder = session/'samples'/sample_id
    folder.mkdir(exist_ok=False)
    write_png(folder/'raw.png', image)
    if overlay is not None:
        write_png(folder/'overlay.png', overlay)
    metadata = {'sample_id': sample_id, 'session_id': info['session_id'],
                'capture_label': info['capture_label'], 'capture_mode': info['capture_mode'],
                'timestamp': timestamp or utc_now(), 'frame_number': frame_number,
                'image_width': image.shape[1], 'image_height': image.shape[0],
                'pixel_format': 'BGR8', 'camera': camera_metadata or {},
                'image_quality': diagnose(image, quality_config),
                'ground_truth_annotation': 'NOT ANNOTATED', 'model_prediction': 'NOT RUN'}
    (folder/'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    return folder, metadata


def load_raw(sample):
    raw = np.fromfile(str(Path(sample)/'raw.png'), np.uint8)
    image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'Cannot read raw image in {sample}')
    return image


def assign_split(session, split):
    if split not in SPLITS:
        raise ValueError('Split must be calibration or validation')
    path = Path(session)/'session.json'
    info = json.loads(path.read_text(encoding='utf-8'))
    if info['split'] not in ('UNASSIGNED', split):
        raise ValueError('Session already assigned to the other split')
    info['split'] = split
    path.write_text(json.dumps(info, indent=2), encoding='utf-8')
    return info


def iter_sessions(root, split=None):
    for path in sorted(Path(root).glob('session_*/session.json')):
        info = json.loads(path.read_text(encoding='utf-8'))
        if split is None or info['split'] == split:
            yield path.parent, info


def iter_samples(root, split=None):
    for session, info in iter_sessions(root, split):
        for sample in sorted((session/'samples').iterdir()):
            if (sample/'metadata.json').exists():
                yield sample, info
