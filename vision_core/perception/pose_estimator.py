"""OpenCV rectangle orientation converted to project clockwise yaw."""
import cv2
import numpy as np
from synthetic.geometry import normalize_yaw


def estimate_pose(contour, image_to_canonical, ppm):
    points = contour.reshape(-1, 2).astype(np.float32)
    canonical = cv2.perspectiveTransform(points[None,:,:], image_to_canonical.astype(np.float64))[0]
    rect = cv2.minAreaRect(canonical)
    box = cv2.boxPoints(rect)
    edges = np.roll(box, -1, axis=0) - box
    lengths = np.linalg.norm(edges, axis=1)
    # Width is 70 mm and height 80 mm: the long axis is the local y axis.
    axis = edges[int(np.argmax(lengths))]
    yaw = normalize_yaw(np.degrees(np.arctan2(-axis[0], axis[1])))
    center = tuple(float(v) for v in rect[0])
    return (center[0]/ppm, center[1]/ppm), yaw, box
