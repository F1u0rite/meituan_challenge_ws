"""Independently selectable camera-like perturbations with image homography."""
import cv2
import numpy as np


def augment(image, options, rng):
    out = image.copy()
    h, w = out.shape[:2]
    H = np.eye(3, dtype=np.float64)
    def on(name): return bool(options.get(name, {}).get('enabled', False))
    if on('brightness'):
        d = rng.uniform(-options['brightness']['max_delta'], options['brightness']['max_delta'])
        out = np.clip(out.astype(float) + d, 0, 255).astype(np.uint8)
    if on('contrast'):
        f = options['contrast']['max_fraction']
        out = np.clip((out.astype(float) - 127.5) * rng.uniform(1-f, 1+f) + 127.5, 0, 255).astype(np.uint8)
    if on('color_shift'):
        d = options['color_shift']['max_delta']
        out = np.clip(out.astype(float) + rng.uniform(-d, d, 3), 0, 255).astype(np.uint8)
    if on('shadow'):
        strength = options['shadow']['strength']
        xx = np.linspace(0, 1, w)[None, :, None]
        out = np.clip(out * (1 - strength * xx), 0, 255).astype(np.uint8)
    if on('gaussian_noise'):
        out = np.clip(out.astype(float) + rng.normal(0, options['gaussian_noise']['sigma'], out.shape), 0, 255).astype(np.uint8)
    if on('gaussian_blur'):
        k = int(options['gaussian_blur']['kernel']) | 1
        out = cv2.GaussianBlur(out, (k, k), 0)
    if on('perspective'):
        f = options['perspective']['corner_fraction']
        src = np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
        dst = (src + rng.uniform(-f, f, (4,2)) * [w,h]).astype(np.float32)
        P = cv2.getPerspectiveTransform(src, dst)
        out = cv2.warpPerspective(out, P, (w,h), borderValue=(255,255,255))
        H = P @ H
    if on('rotation'):
        angle = rng.uniform(-options['rotation']['max_deg'], options['rotation']['max_deg'])
        A = cv2.getRotationMatrix2D((w/2,h/2), angle, 1)
        out = cv2.warpAffine(out, A, (w,h), borderValue=(255,255,255))
        H = np.vstack([A,[0,0,1]]) @ H
    return out, H
