"""Unicode-safe OpenCV image writing on Windows and Linux."""
from pathlib import Path
import cv2


def write_png(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode('.png', image)
    if not ok:
        raise RuntimeError(f'Could not encode {path}')
    encoded.tofile(str(path))
