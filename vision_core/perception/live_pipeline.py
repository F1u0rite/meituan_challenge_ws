"""Frame-by-frame BGR vision pipeline shared by live camera and offline tests."""
from dataclasses import dataclass
from time import perf_counter
import cv2
import numpy as np
from .board_detector import BoardDetector, BoardDetection
from .board_rectifier import BoardRectifier
from .battery_detector import BatteryDetector, BatteryDetection


@dataclass
class FrameResult:
    board: BoardDetection
    board_valid: bool
    rectified: np.ndarray | None
    detections: list[BatteryDetection]
    latency_ms: float


class LiveVisionPipeline:
    """Runs board detection and homography afresh for every input frame."""

    def __init__(self, config, min_board_confidence=0.0):
        self.board_detector = BoardDetector(config)
        self.rectifier = BoardRectifier(config)
        self.battery_detector = BatteryDetector(config)
        self.min_board_confidence = float(min_board_confidence)

    def process(self, image: np.ndarray) -> FrameResult:
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('Vision input must be a HxWx3 BGR uint8 image')
        start = perf_counter()
        board = self.board_detector.detect(image)
        valid = bool(board.detected and board.corners_px is not None and
                     board.confidence >= self.min_board_confidence and
                     np.isfinite(board.corners_px).all())
        if not valid:
            return FrameResult(board, False, None, [], (perf_counter() - start) * 1000)
        try:
            rectified = self.rectifier.rectify(image, board.corners_px).image
            detections = self.battery_detector.detect(rectified)
        except (cv2.error, ValueError, np.linalg.LinAlgError):
            return FrameResult(board, False, None, [], (perf_counter() - start) * 1000)
        return FrameResult(board, True, rectified, detections, (perf_counter() - start) * 1000)
