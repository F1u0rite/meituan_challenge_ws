"""Ubuntu-only simulated sender fixtures; not a D435i or Windows implementation."""
import time


def battery(identity='battery-1', color='RED', **changes):
    result = dict(object_id=identity, color=color, confidence=.9,
                  detection_status='CONFIRMED', surface_depth_m=None,
                  body_position_candidate_m=None, body_orientation_candidate=None,
                  body_yaw_candidate_rad=None, ring_position_candidate_m=None,
                  ring_orientation_candidate=None, insertion_direction_candidates=None,
                  ambiguity=False, stale=False, body_pose_valid=False, ring_pose_valid=False)
    result.update(changes)
    return result


def frame(sequence=0, objects=None, **changes):
    now = time.time_ns()
    result = dict(protocol_version=1, sequence_id=sequence, mode='test',
                  camera_frame_id='test_color_optical_frame', capture_timestamp_raw=float(sequence),
                  timestamp_domain='simulated',
                  windows_capture_wall_time={'sec':now//1000000000,'nanosec':now%1000000000},
                  source_frame_id='test_color_optical_frame',
                  objects=[battery()] if objects is None else objects)
    result.update(changes)
    return result


def installed_executable(workspace, name):
    """Resolve explicit, isolated or conventional colcon install without fake binaries."""
    import os
    from pathlib import Path
    workspace = Path(workspace)
    explicit = os.environ.get('MTC_VISION_INSTALL_BASE')
    base = Path(explicit).expanduser().resolve() if explicit else (
        workspace/'install/yuze' if (workspace/'install/yuze').is_dir() else workspace/'install')
    isolated = base/'mtc_vision_adapter/lib/mtc_vision_adapter'/name
    merged = base/'lib/mtc_vision_adapter'/name
    return merged if not isolated.is_file() and merged.is_file() else isolated
