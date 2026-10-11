"""Board geometry: origin top left, x right, y down, positive yaw clockwise."""
import math
import numpy as np


def mm_to_px(point, ppm):
    return tuple(float(v) * ppm for v in point)


def px_to_mm(point, ppm):
    return tuple(float(v) / ppm for v in point)


def normalize_yaw(degrees):
    return float(degrees) % 180.0


def yaw_error(a, b):
    d = abs(normalize_yaw(a) - normalize_yaw(b))
    return min(d, 180.0 - d)


def rectangle_corners(cx, cy, width, height, yaw):
    theta = math.radians(yaw)
    c, s = math.cos(theta), math.sin(theta)
    points = np.array([[-width/2, -height/2], [width/2, -height/2],
                       [width/2, height/2], [-width/2, height/2]], dtype=float)
    return points @ np.array([[c, s], [-s, c]]) + [cx, cy]


def overlaps(a, b, gap=0.0):
    """Separating axis test for two four-corner polygons."""
    for polygon in (a, b):
        for edge in np.roll(polygon, -1, axis=0) - polygon:
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() + gap <= pb.min() or pb.max() + gap <= pa.min():
                return False
    return True
