"""Image-pixel center estimators for a single predicted color contour.

None of these methods uses a ground-truth mask or estimates a 3D body pose.
"""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class RectangleEstimate:
    center: tuple[float, float]
    yaw_deg: float
    corners: np.ndarray


@dataclass
class VisibleGeometryEstimate:
    """2D proxy only; validity does not imply a calibrated 3D body pose."""
    center_px: tuple[float, float]
    center_method: str
    yaw_deg: float
    yaw_valid: bool
    yaw_reason: str
    body_pose_valid: bool = False
    ring_pose_valid: bool = False


def mask_centroid(mask):
    ys, xs = np.nonzero(mask)
    if not len(xs):
        raise ValueError('Empty mask has no center')
    return float(xs.mean()), float(ys.mean())


def contour_centroid(contour):
    moments = cv2.moments(contour)
    if moments['m00'] <= 0:
        raise ValueError('Degenerate contour has no area centroid')
    return float(moments['m10']/moments['m00']), float(moments['m01']/moments['m00'])


def yaw_from_axis(axis):
    """Long-axis yaw relative to image vertical, modulo 180 degrees."""
    return float(np.degrees(np.arctan2(-axis[0], axis[1])) % 180)


def yaw_delta_180(a, b):
    return float(abs((float(a)-float(b)+90) % 180-90))


def min_area_rectangle(contour):
    rect = cv2.minAreaRect(contour)
    if min(rect[1]) <= 0:
        raise ValueError('Degenerate rectangle')
    corners = cv2.boxPoints(rect)
    edges = np.roll(corners, -1, axis=0)-corners
    axis = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
    return RectangleEstimate(tuple(map(float, rect[0])), yaw_from_axis(axis), corners)


def robust_rectangle_fit(contour, trim=0.10):
    """PCA rectangle whose extent uses trimmed pixel projections.

    The 10/90-percentile span is fixed before evaluation. It reduces leverage
    of small contour protrusions, but can miss genuine body edges.
    """
    if not 0 <= trim < .5:
        raise ValueError('trim must be in [0, 0.5)')
    x, y, w, h = cv2.boundingRect(contour)
    if w < 2 or h < 2:
        raise ValueError('Degenerate contour')
    local = contour.copy()
    local[:, 0, 0] -= x
    local[:, 0, 1] -= y
    filled = np.zeros((h, w), np.uint8)
    cv2.drawContours(filled, [local], -1, 1, -1)
    yy, xx = np.nonzero(filled)
    if len(xx) < 4:
        raise ValueError('Too few contour pixels')
    points = np.column_stack((xx+x, yy+y)).astype(float)
    origin = points.mean(axis=0)
    covariance = np.cov(points-origin, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major = eigenvectors[:, int(np.argmax(eigenvalues))]
    minor = np.array([-major[1], major[0]])
    coordinates = np.column_stack(((points-origin)@major, (points-origin)@minor))
    low = np.quantile(coordinates, trim, axis=0)
    high = np.quantile(coordinates, 1-trim, axis=0)
    midpoint = (low+high)/2
    center = origin+major*midpoint[0]+minor*midpoint[1]
    corners = np.array([center+major*s*(high[0]-low[0])/2+minor*t*(high[1]-low[1])/2
                        for s,t in ((-1,-1),(1,-1),(1,1),(-1,1))], dtype=np.float32)
    return RectangleEstimate((float(center[0]), float(center[1])),
                             yaw_from_axis(major), corners)


def estimate_visible_geometry(contour, *, min_yaw_aspect_ratio=1.25,
                              max_yaw_method_disagreement_deg=15):
    """Use area centroid for a visible-color silhouette; gate its axis estimate.

    The result is an RGB silhouette proxy, not the physical body center.
    """
    center = contour_centroid(contour)
    minimum = min_area_rectangle(contour)
    robust = robust_rectangle_fit(contour)
    sides = cv2.minAreaRect(contour)[1]
    ratio = max(sides)/max(min(sides), 1e-9)
    disagreement = yaw_delta_180(minimum.yaw_deg, robust.yaw_deg)
    if ratio < min_yaw_aspect_ratio:
        valid, reason = False, 'NEAR_SQUARE_SILHOUETTE'
    elif disagreement > max_yaw_method_disagreement_deg:
        valid, reason = False, 'AXIS_METHOD_DISAGREEMENT'
    else:
        valid, reason = True, 'GEOMETRICALLY_CONSISTENT_ONLY'
    return VisibleGeometryEstimate(center, 'detected_contour_centroid',
                                   robust.yaw_deg, valid, reason)
