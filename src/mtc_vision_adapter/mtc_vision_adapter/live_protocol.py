"""Version 1 untrusted wire validation, session ordering and safe ROS data."""
import json
import math
import re

MAX_BYTES = 256*1024
MAX_OBJECTS = 128
VERSION = 1
COLORS = {'UNKNOWN': 0, 'RED': 1, 'BLUE': 2, 'YELLOW': 3, 'GREEN': 4}
DOMAINS = {'hardware_clock', 'system_time', 'global_time', 'simulated', 'unknown'}
STATUS = {'CONFIRMED', 'RAW_CANDIDATE', 'AMBIGUOUS', 'REJECTED'}
FRAME_KEYS = {'protocol_version', 'sequence_id', 'mode', 'camera_frame_id',
              'capture_timestamp_raw', 'timestamp_domain', 'windows_capture_wall_time',
              'source_frame_id', 'objects'}
OBJECT_KEYS = {'object_id', 'color', 'confidence', 'detection_status', 'surface_depth_m',
               'body_position_candidate_m', 'body_orientation_candidate',
               'body_yaw_candidate_rad', 'ring_position_candidate_m',
               'ring_orientation_candidate', 'insertion_direction_candidates',
               'ambiguity', 'stale', 'body_pose_valid', 'ring_pose_valid'}


class ProtocolError(ValueError):
    pass


def require(condition, reason):
    if not condition:
        raise ProtocolError(reason)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def exact_keys(value, keys):
    require(type(value) is dict and set(value) == keys, 'FIELDS')


def vector(value, size, unit=False):
    if value is None:
        return
    require(type(value) is list and len(value) == size and all(number(v) for v in value),
            'VECTOR')
    if unit:
        require(abs(sum(v*v for v in value)-1) <= 1e-3, 'UNIT_VECTOR')


def frame_id(value):
    require(type(value) is str and len(value) <= 128 and
            re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', value), 'FRAME_ID')


def wall_ns(stamp):
    if stamp is None:
        return None
    return stamp['sec']*1000000000+stamp['nanosec']


def validate(message):
    exact_keys(message, FRAME_KEYS)
    require(type(message['protocol_version']) is int and message['protocol_version'] == VERSION,
            'VERSION')
    seq = message['sequence_id']
    require(type(seq) is int and 0 <= seq <= 9007199254740991, 'SEQUENCE_TYPE')
    require(type(message['mode']) is str and message['mode'] in ('live', 'test'), 'MODE')
    frame_id(message['camera_frame_id'])
    frame_id(message['source_frame_id'])
    domain = message['timestamp_domain']
    require(type(domain) is str and domain in DOMAINS, 'TIME_DOMAIN')
    raw = message['capture_timestamp_raw']
    require((raw is None and domain == 'unknown') or (number(raw) and raw >= 0), 'RAW_TIME')
    require(domain != 'simulated' or message['mode'] == 'test', 'SIMULATED_LIVE')
    stamp = message['windows_capture_wall_time']
    if stamp is not None:
        exact_keys(stamp, {'sec', 'nanosec'})
        require(type(stamp['sec']) is int and 0 < stamp['sec'] <= 2147483647 and
                type(stamp['nanosec']) is int and 0 <= stamp['nanosec'] < 1000000000,
                'WALL_TIME')
    objects = message['objects']
    require(type(objects) is list and len(objects) <= MAX_OBJECTS, 'OBJECTS')
    ids = set()
    for obj in objects:
        exact_keys(obj, OBJECT_KEYS)
        identity = obj['object_id']
        require(type(identity) is str and
                re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,95}', identity), 'OBJECT_ID')
        require(identity not in ids, 'DUPLICATE_ID')
        ids.add(identity)
        require(type(obj['color']) is str and obj['color'] in COLORS, 'COLOR')
        require(number(obj['confidence']) and 0 <= obj['confidence'] <= 1, 'CONFIDENCE')
        require(type(obj['detection_status']) is str and
                obj['detection_status'] in STATUS, 'STATUS')
        for key in ('ambiguity', 'stale', 'body_pose_valid', 'ring_pose_valid'):
            require(type(obj[key]) is bool, 'BOOL')
        require(obj['body_pose_valid'] is False and obj['ring_pose_valid'] is False,
                'POSE_VALID_FORBIDDEN')
        depth = obj['surface_depth_m']
        require(depth is None or (number(depth) and depth > 0), 'DEPTH')
        yaw = obj['body_yaw_candidate_rad']
        require(yaw is None or (number(yaw) and -math.pi <= yaw <= math.pi), 'YAW')
        vector(obj['body_position_candidate_m'], 3)
        vector(obj['ring_position_candidate_m'], 3)
        vector(obj['body_orientation_candidate'], 4, unit=True)
        vector(obj['ring_orientation_candidate'], 4, unit=True)
        directions = obj['insertion_direction_candidates']
        require(directions is None or (type(directions) is list and len(directions) <= 4),
                'DIRECTIONS')
        for direction in directions or []:
            require(direction is not None, 'DIRECTIONS')
            vector(direction, 3, unit=True)
    return message


def decode(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'DUPLICATE_KEY')
            result[key] = value
        return result
    def constant(_):
        raise ProtocolError('NONFINITE')
    try:
        require(0 < len(payload) <= MAX_BYTES, 'LENGTH')
        message = json.loads(payload.decode('utf-8'), object_pairs_hook=pairs,
                             parse_constant=constant)
        return validate(message)
    except ProtocolError:
        raise
    except (ValueError, TypeError, UnicodeError, RecursionError, OverflowError):
        raise ProtocolError('JSON_OR_TYPE') from None


class Session:
    """One TCP connection; no global device-clock or motion-freshness trust."""
    def __init__(self):
        self.sequence = None
        self.identity = None
        self.raw = None

    def observe(self, message, receive_wall_ns, max_age_s=5., max_future_s=1.):
        seq = message['sequence_id']
        if self.sequence is None and seq != 0:
            return 'SEQUENCE_START'
        if self.sequence is not None and seq <= self.sequence:
            return 'SEQUENCE_ORDER'
        self.sequence = seq
        identity = (message['mode'], message['camera_frame_id'], message['source_frame_id'],
                    message['timestamp_domain'])
        if self.identity is not None and self.identity != identity:
            return 'SOURCE_CHANGED'
        self.identity = identity
        raw = message['capture_timestamp_raw']
        if raw is not None and self.raw is not None and raw <= self.raw:
            return 'CAPTURE_ORDER'
        self.raw = raw
        stamp = wall_ns(message['windows_capture_wall_time'])
        if message['timestamp_domain'] == 'unknown' or raw is None or stamp is None:
            return 'TIME_UNKNOWN'
        # This is an UNTRUSTED clock-skew/age sanity check, not a clock mapping
        # or evidence of sensor-to-ROS latency or motion freshness.
        age = (receive_wall_ns-stamp)*1e-9
        if age > max_age_s:
            return 'WALL_OLD'
        if age < -max_future_s:
            return 'WALL_FUTURE'
        return None


def diagnostic_array(message):
    """Ignore all candidate XYZ/quaternions and emit NaN through core.to_ros."""
    stamp = message['windows_capture_wall_time']
    header = {'frame_id': message['source_frame_id'],
              'stamp': dict(stamp) if stamp is not None else {'sec': 0, 'nanosec': 0}}
    detections = []
    for obj in message['objects']:
        if (obj['detection_status'] != 'CONFIRMED' or obj['ambiguity'] or obj['stale'] or
                obj['color'] == 'UNKNOWN' or obj['confidence'] < .2):
            continue
        invalid = {'header': header, 'pose': {
            'position': dict.fromkeys(('x', 'y', 'z')),
            'orientation': dict.fromkeys(('x', 'y', 'z', 'w'))}}
        detections.append(dict(object_id=obj['object_id'], color=COLORS[obj['color']],
                               confidence=float(obj['confidence']), body_pose=invalid,
                               ring_pose=invalid, ring_pose_valid=False))
    return {'header': header, 'detections': detections}
