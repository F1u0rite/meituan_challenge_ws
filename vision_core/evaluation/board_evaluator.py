"""Board corner metrics kept separate from perception."""
from dataclasses import dataclass
import numpy as np
from .metrics import summarize


@dataclass
class BoardEvaluation:
    detected: bool
    corner_errors_px: list[float]


def evaluate_board(detection,gt_corners):
    if not detection.detected: return BoardEvaluation(False,[])
    errors=np.linalg.norm(np.asarray(detection.corners_px)-np.asarray(gt_corners),axis=1)
    return BoardEvaluation(True,[float(x) for x in errors])


def aggregate_boards(evaluations):
    errors=[x for result in evaluations for x in result.corner_errors_px]
    return {'detection_success_rate':sum(e.detected for e in evaluations)/len(evaluations) if evaluations else 0,
            'corner_error_px':summarize(errors)}
