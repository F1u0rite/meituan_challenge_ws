"""Periodic yaw and summary statistics."""
import numpy as np
from synthetic.geometry import yaw_error


def summarize(values):
    return {'mean': float(np.mean(values)) if values else None,
            'median': float(np.median(values)) if values else None,
            'p95': float(np.percentile(values, 95)) if values else None}
