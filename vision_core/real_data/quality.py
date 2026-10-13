"""Configurable image quality diagnostics for unmodified BGR frames."""
from pathlib import Path
import cv2
import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1]/'config'/'real_quality.yaml'


def load_quality_config(path=DEFAULT_CONFIG):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def diagnose(image, config=None):
    if image is None or image.size == 0 or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected nonempty HxWx3 BGR uint8 image')
    config = config or load_quality_config()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    metrics = {
        'value_mean': float(v.mean()), 'value_median': float(np.median(v)),
        'value_p05': float(np.percentile(v, 5)), 'value_p95': float(np.percentile(v, 95)),
        'low_value_fraction': float(np.mean(v < config['low_value_cutoff'])),
        'near_white_fraction': float(np.mean((v >= config['white_value_cutoff']) &
                                              (s <= config['white_saturation_ceiling']))),
        'laplacian_variance': float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    }
    reasons = []
    level = 'ACCEPTED'
    for threshold, bad, severity, reason in (
        ('reject_mean_v', metrics['value_mean'] < config['reject_mean_v'], 'REJECTED', 'too_dark'),
        ('reject_dark_fraction', metrics['low_value_fraction'] > config['reject_dark_fraction'], 'REJECTED', 'mostly_dark'),
        ('reject_white_fraction', metrics['near_white_fraction'] > config['reject_white_fraction'], 'REJECTED', 'mostly_white'),
        ('reject_laplacian_variance', metrics['laplacian_variance'] < config['reject_laplacian_variance'], 'REJECTED', 'severely_blurred'),
        ('warning_mean_v', metrics['value_mean'] < config['warning_mean_v'], 'WARNING', 'dim'),
        ('warning_dark_fraction', metrics['low_value_fraction'] > config['warning_dark_fraction'], 'WARNING', 'dark_pixels'),
        ('warning_white_fraction', metrics['near_white_fraction'] > config['warning_white_fraction'], 'WARNING', 'white_saturation'),
        ('warning_laplacian_variance', metrics['laplacian_variance'] < config['warning_laplacian_variance'], 'WARNING', 'blurred'),
    ):
        if bad:
            reasons.append(reason)
            if severity == 'REJECTED' or level == 'ACCEPTED':
                level = severity
    return {'status': level, 'metrics': metrics, 'reasons': reasons,
            'rule_source': 'engineering_initial', 'thresholds': config}
