"""Resolve hole-axis sign with a same-frame robot-to-battery reference.

The reference is in camera_color_optical. It must be recomputed when the camera
moves; sign selection never validates metrology, transforms or robot motion.
"""
import numpy as np


def select_insertion_direction(top, reference, source_frame_id, capture_timestamp):
    out=dict(top, insertion_direction_selected=None, direction_unique=False)
    diagnostic=dict(status='UNRESOLVED',reason='CURRENT_FRAME_ROBOT_SIDE_REFERENCE_REQUIRED')
    out['direction_selection']=diagnostic
    if not top.get('top_geometry_valid') or not top.get('ring_center_geometry_valid'):
        diagnostic['reason']='SUPPORTED_TOP_AND_RING_CENTER_REQUIRED'
        return out
    if reference is None:
        return out
    if not isinstance(reference,dict):
        diagnostic['reason']='INVALID_INSERTION_REFERENCE'
        return out
    if reference.get('source_frame_id')!=source_frame_id:
        diagnostic['reason']='INSERTION_REFERENCE_FRAME_MISMATCH'
        return out
    if reference.get('coordinate_frame')!='camera_color_optical':
        diagnostic['reason']='INSERTION_REFERENCE_COORDINATE_FRAME_MISMATCH'
        return out
    try:
        stamp=float(reference['capture_timestamp'])
        current=float(capture_timestamp)
        vector=np.asarray(reference['robot_to_battery_camera'],dtype=float)
    except (KeyError,TypeError,ValueError):
        diagnostic['reason']='INVALID_INSERTION_REFERENCE'
        return out
    if not np.isfinite([stamp,current]).all() or abs(stamp-current)>1e-6:
        diagnostic['reason']='INSERTION_REFERENCE_TIME_MISMATCH'
        return out
    provenance=reference.get('source')
    if not isinstance(provenance,str) or not provenance.strip():
        diagnostic['reason']='INSERTION_REFERENCE_SOURCE_REQUIRED'
        return out
    if vector.shape!=(3,) or not np.isfinite(vector).all() or np.linalg.norm(vector)<1e-9:
        diagnostic['reason']='INVALID_INSERTION_REFERENCE'
        return out
    axes=np.asarray(top.get('insertion_direction_candidates',[]),dtype=float)
    if axes.shape!=(2,3) or not np.isfinite(axes).all():
        diagnostic['reason']='INVALID_OBSERVED_HOLE_AXIS'
        return out
    lengths=np.linalg.norm(axes,axis=1)
    if np.any(lengths<1e-9):
        diagnostic['reason']='INVALID_OBSERVED_HOLE_AXIS'
        return out
    axes=axes/lengths[:,None]
    if not np.allclose(axes[0],-axes[1],atol=1e-6):
        diagnostic['reason']='HOLE_AXIS_CANDIDATES_NOT_OPPOSITE'
        return out
    vector=vector/np.linalg.norm(vector)
    scores=axes@vector;index=int(np.argmax(scores))
    diagnostic.update(reference_source=provenance,source_frame_id=source_frame_id,
        capture_timestamp=stamp,reference_unit_vector=vector.tolist(),
        alignment_cosine=float(scores[index]),minimum_alignment_cosine=.5)
    if scores[index]<=.5:
        diagnostic['reason']='ROBOT_SIDE_REFERENCE_NOT_ALIGNED_WITH_HOLE_AXIS'
        return out
    diagnostic.update(status='SELECTED',reason='ROBOT_SIDE_TO_BATTERY_CONSTRAINT',
        selected_candidate_index=index)
    out.update(direction_unique=True,insertion_direction_selected=axes[index].tolist(),
        reasons=[r for r in top.get('reasons',[]) if r!='INSERTION_SIDE_180_DEG_AMBIGUITY'])
    return out
