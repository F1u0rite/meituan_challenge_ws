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
