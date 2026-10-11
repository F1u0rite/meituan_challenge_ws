"""Stable failure labels for robustness benchmark reporting."""
def classify_board(detection,corner_errors=None,corner_limit=8.0,rectification_ok=True):
    if not detection.detected: return ['no_board_detected']
    failures=[]
    if corner_errors and max(corner_errors)>corner_limit: failures.append('corner_error_too_large')
    if not rectification_ok: failures.append('rectification_failure')
    return failures

def classify_battery(evaluation,upstream=False):
    if upstream: return ['upstream_board_failure']
    failures=[]
    for case in evaluation.cases:
        if case['prediction'] is None: failures.append('missed_detection')
        failures.extend('missed_detection' if reason=='missed' else reason for reason in case['reasons'] if reason!='missed' or case['prediction'] is not None)
    if evaluation.false_positives: failures.extend(['false_positive']*evaluation.false_positives)
    return failures
