"""Reproducible planar camera view synthesis with the complete board in frame."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class CameraView:
    image: np.ndarray
    board_corners_gt_px: np.ndarray
    canonical_to_camera: np.ndarray


class CameraProjection:
    def __init__(self, config):
        self.config = config

    def project(self, canonical_image, seed=42, difficulty='clean'):
        cfg = self.config['camera_projection']
        rng = np.random.default_rng(seed)
        h, w = canonical_image.shape[:2]
        cw, ch = int(cfg['canvas_width_px']), int(cfg['canvas_height_px'])
        scale = float(rng.uniform(*cfg['scale_range']))
        bw, bh = w * scale, h * scale
        angle = np.deg2rad(rng.uniform(-cfg['rotation_deg'], cfg['rotation_deg']))
        base = np.array([[-bw/2,-bh/2],[bw/2,-bh/2],[bw/2,bh/2],[-bw/2,bh/2]],np.float32)
        rotation = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]],np.float32)
        corners = base @ rotation.T
        multiplier=1.0 if difficulty=='clean' else self.config['robustness'][difficulty]['perspective_multiplier']
        perspective=cfg['perspective_fraction']*multiplier
        jitter = rng.uniform(-perspective,perspective,(4,2)).astype(np.float32)
        corners += jitter * np.array([bw,bh],np.float32)
        extent = np.max(np.abs(corners),axis=0)
        margin = 12.0
        free_x = max(0.0, cw/2-extent[0]-margin)
        free_y = max(0.0, ch/2-extent[1]-margin)
        limit = float(cfg['translation_fraction'])
        tx = rng.uniform(-min(free_x,cw*limit),min(free_x,cw*limit))
        ty = rng.uniform(-min(free_y,ch*limit),min(free_y,ch*limit))
        corners += [cw/2+tx,ch/2+ty]
        src = np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
        H = cv2.getPerspectiveTransform(src,corners.astype(np.float32))
        image = cv2.warpPerspective(canonical_image,H,(cw,ch),
                                    borderValue=tuple(cfg['background_bgr']))
        return CameraView(image,corners.astype(np.float32),H)
