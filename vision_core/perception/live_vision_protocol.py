"""Strict LIVE_VISION_PROTOCOL_V1 Windows encoding; no ROS imports."""
import json
import math
import re
import struct
from datetime import datetime, timezone

MAX_PAYLOAD = 262144
MAX_SEQUENCE = 9007199254740991
FRAME_PATTERN = re.compile(r'[A-Za-z][A-Za-z0-9_/]{0,127}\Z')
ID_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,95}\Z')
COLORS = {0: 'UNKNOWN', 1: 'RED', 2: 'BLUE', 3: 'YELLOW', 4: 'GREEN'}
STATUSES = {'CONFIRMED', 'RAW_CANDIDATE', 'AMBIGUOUS', 'REJECTED'}
DOMAINS = {'hardware_clock', 'system_time', 'global_time', 'simulated', 'unknown'}
TOP_KEYS = {'protocol_version', 'sequence_id', 'mode', 'camera_frame_id',
            'capture_timestamp_raw', 'timestamp_domain', 'windows_capture_wall_time',
            'source_frame_id', 'objects'}
OBJECT_KEYS = {'object_id', 'color', 'confidence', 'detection_status', 'surface_depth_m',
               'body_position_candidate_m', 'body_orientation_candidate', 'body_yaw_candidate_rad',
               'ring_position_candidate_m', 'ring_orientation_candidate',
               'insertion_direction_candidates', 'ambiguity', 'stale',
               'body_pose_valid', 'ring_pose_valid'}


class ProtocolError(ValueError):
    pass


def require(condition, reason):
    if not condition:
        raise ProtocolError(reason)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value, size, unit=False):
    require(type(value) is list and len(value) == size and all(number(v) for v in value), 'VECTOR')
    if unit:
        require(abs(sum(v*v for v in value)-1) <= .001, 'UNIT_VECTOR_OR_QUATERNION')


def validate_message(message):
    require(type(message) is dict and set(message) == TOP_KEYS, 'TOP_FIELDS')
    require(type(message['protocol_version']) is int and message['protocol_version'] == 1, 'VERSION')
    require(type(message['sequence_id']) is int and 0 <= message['sequence_id'] <= MAX_SEQUENCE, 'SEQUENCE')
    require(message['mode'] in ('live', 'test'), 'MODE')
    for key in ('camera_frame_id', 'source_frame_id'):
        require(type(message[key]) is str and FRAME_PATTERN.fullmatch(message[key]) is not None, 'FRAME_ID')
    domain = message['timestamp_domain']
    require(type(domain) is str and domain in DOMAINS, 'DOMAIN')
    require(domain != 'simulated' or message['mode'] == 'test', 'SIMULATED_LIVE')
    raw = message['capture_timestamp_raw']
    if raw is None:
        require(domain == 'unknown', 'NULL_RAW_DOMAIN')
    else:
        require(number(raw) and raw >= 0, 'RAW_TIMESTAMP')
    wall = message['windows_capture_wall_time']
    if wall is not None:
        require(type(wall) is dict and set(wall) == {'sec', 'nanosec'}, 'WALL_FIELDS')
        require(type(wall['sec']) is int and 1 <= wall['sec'] <= 2147483647, 'WALL_SEC')
        require(type(wall['nanosec']) is int and 0 <= wall['nanosec'] <= 999999999, 'WALL_NANOSEC')
    objects = message['objects']
    require(type(objects) is list and len(objects) <= 128, 'OBJECT_COUNT')
    ids = set()
    for obj in objects:
        require(type(obj) is dict and set(obj) == OBJECT_KEYS, 'OBJECT_FIELDS')
        ident = obj['object_id']
        require(type(ident) is str and ID_PATTERN.fullmatch(ident) is not None, 'OBJECT_ID')
        require(ident not in ids, 'DUPLICATE_ID'); ids.add(ident)
        require(type(obj['color']) is str and obj['color'] in COLORS.values(), 'COLOR')
        require(number(obj['confidence']) and 0 <= obj['confidence'] <= 1, 'CONFIDENCE')
        require(type(obj['detection_status']) is str and obj['detection_status'] in STATUSES, 'STATUS')
        for key in ('ambiguity', 'stale', 'body_pose_valid', 'ring_pose_valid'):
            require(type(obj[key]) is bool, 'BOOLEAN')
        require(not obj['body_pose_valid'] and not obj['ring_pose_valid'], 'POSE_VALIDITY_FORBIDDEN')
        depth = obj['surface_depth_m']
        require(depth is None or (number(depth) and depth > 0), 'SURFACE_DEPTH')
        yaw = obj['body_yaw_candidate_rad']
        require(yaw is None or (number(yaw) and -math.pi <= yaw <= math.pi), 'YAW')
        for key in ('body_position_candidate_m', 'ring_position_candidate_m'):
            if obj[key] is not None: vector(obj[key], 3)
        for key in ('body_orientation_candidate', 'ring_orientation_candidate'):
            if obj[key] is not None: vector(obj[key], 4, True)
        directions = obj['insertion_direction_candidates']
        if directions is not None:
            require(type(directions) is list and len(directions) <= 4, 'DIRECTION_COUNT')
            for direction in directions: vector(direction, 3, True)
    return message


def encode_message(message):
    validate_message(message)
    payload = json.dumps(message, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    require(1 <= len(payload) <= MAX_PAYLOAD, 'PAYLOAD_SIZE')
    return struct.pack('!I', len(payload)) + payload


def strict_loads(text):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'DUPLICATE_JSON_KEY'); result[key] = value
        return result
    def constant(value):
        raise ProtocolError('NON_FINITE_JSON')
    return json.loads(text, object_pairs_hook=pairs, parse_constant=constant)


def capture_wall(timestamps):
    """Only preserved capture metadata, never current sending wall time."""
    if 'host_wall_ns' in timestamps:
        ns = timestamps['host_wall_ns']
        require(type(ns) is int and ns > 0, 'HOST_WALL_NS')
        return dict(sec=ns // 1000000000, nanosec=ns % 1000000000)
    text = timestamps.get('host_utc')
    if text is None: return None
    try:
        instant = datetime.fromisoformat(text.replace('Z', '+00:00'))
    except (TypeError, ValueError) as exc:
        raise ProtocolError('HOST_UTC') from exc
    require(instant.tzinfo is not None, 'HOST_UTC_TIMEZONE')
    delta = instant.astimezone(timezone.utc) - datetime(1970, 1, 1, tzinfo=timezone.utc)
    return dict(sec=delta.days*86400 + delta.seconds, nanosec=delta.microseconds*1000)


def quaternion(rotation):
    if rotation is None: return None
    import numpy as np
    from scipy.spatial.transform import Rotation
    array = np.asarray(rotation, dtype=float)
    if array.shape == (4,):
        value = array.tolist(); vector(value, 4, True); return value
    require(array.shape == (3, 3) and np.isfinite(array).all(), 'ROTATION_MATRIX')
    require(np.allclose(array.T @ array, np.eye(3), atol=1e-6, rtol=0)
            and abs(np.linalg.det(array)-1) <= 1e-6, 'ROTATION_NOT_RIGID')
    return Rotation.from_matrix(array).as_quat().tolist()  # x,y,z,w, body/ring -> camera


def vision_message(result, *, camera_frame_id, namespace, mode, timestamps=None):
    """Map existing core candidates only; sequence is assigned by TCP worker."""
    require(type(result) is dict, 'VISION_RESULT')
    require(type(namespace) is str and re.fullmatch(r'[A-Za-z0-9_.:-]+', namespace) is not None, 'NAMESPACE')
    require(result.get('body_pose_valid', False) is False and result.get('ring_pose_valid', False) is False,
            'POSE_VALIDITY_FORBIDDEN')
    times = result.get('timestamps', {}) if timestamps is None else timestamps
    require(type(times) is dict, 'TIMESTAMPS')
    domain = times.get('color_timestamp_domain', 'unknown')
    require(type(domain) is str, 'DOMAIN')
    domain = domain.removeprefix('timestamp_domain.')
    raw = times.get('color_sdk_ms')
    if raw is None: domain = 'unknown'
    source = result.get('coordinate_frame', camera_frame_id)
    objects = []
    for item in result.get('objects', []):
        require(type(item) is dict, 'VISION_OBJECT')
        require(item.get('body_pose_valid', False) is False and item.get('ring_pose_valid', False) is False,
                'POSE_VALIDITY_FORBIDDEN')
        require(item.get('unit_position', 'm') == 'm' and item.get('unit_angle', 'rad') == 'rad', 'UNITS')
        require(item.get('frame_id', item.get('coordinate_frame', source)) == source, 'MIXED_SPATIAL_FRAMES')
        color = item.get('color_label', item.get('color', 'UNKNOWN'))
        if type(color) is int: color = COLORS.get(color)
        if type(color) is str: color = color.upper()
        state = item.get('detection_status', item.get('candidate_state', 'RAW_CANDIDATE'))
        if state == 'CONFIRMED_OBJECT': state = 'CONFIRMED'
        ident = item.get('object_id')
        require(type(ident) is str and ID_PATTERN.fullmatch(ident) is not None, 'OBJECT_ID')
        separated='detection_stale' in item and 'detection_ambiguity' in item
        if separated:
            require(type(item['detection_ambiguity']) is bool, 'DETECTION_AMBIGUITY')
            require(item['detection_stale'] is None or type(item['detection_stale']) is bool, 'DETECTION_STALE')
        stale = (item['detection_stale'] is not False) if separated else item.get('stale_status') == 'STALE'
        if 'stale' in item:
            require(type(item['stale']) is bool, 'BOOLEAN'); stale = stale or item['stale']
        ambiguity = item['detection_ambiguity'] if separated else item.get('ambiguity', True)
        require(type(ambiguity) in (bool, list), 'AMBIGUITY')
        body = item.get('body_center_candidate_xyz', item.get('model_estimated_body_center_xyz'))
        body_rotation = item.get('body_orientation_candidate')
        if body_rotation is None and item.get('body_pose_candidate') is not None:
            import numpy as np
            matrix = np.asarray(item['body_pose_candidate'], dtype=float)
            require(matrix.shape == (4, 4) and np.isfinite(matrix).all()
                    and np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6, rtol=0), 'BODY_POSE_MATRIX')
            body_rotation = matrix[:3, :3].tolist()
        obj = dict(object_id=namespace+'-'+ident, color=color, confidence=item.get('confidence'),
            detection_status=state, surface_depth_m=item.get('surface_depth_z'),
            body_position_candidate_m=body, body_orientation_candidate=quaternion(body_rotation),
            body_yaw_candidate_rad=item.get('body_yaw_candidate'),
            ring_position_candidate_m=item.get('ring_center_candidate'),
            ring_orientation_candidate=quaternion(item.get('ring_orientation_candidate')),
            insertion_direction_candidates=item.get('hook_insertion_direction_candidates', item.get('insertion_direction_candidates')),
            ambiguity=bool(ambiguity), stale=stale, body_pose_valid=False, ring_pose_valid=False)
        # Old camera-frame candidates never become current network spatial data.
        if stale or (separated and (item.get('cad_stale') or item.get('pose_ambiguity'))):
            for key in ('body_position_candidate_m', 'body_orientation_candidate', 'body_yaw_candidate_rad',
                        'ring_position_candidate_m', 'ring_orientation_candidate', 'insertion_direction_candidates'):
                obj[key] = None
        objects.append(obj)
    message = dict(protocol_version=1, sequence_id=0, mode=mode, camera_frame_id=camera_frame_id,
        capture_timestamp_raw=raw, timestamp_domain=domain, windows_capture_wall_time=capture_wall(times),
        source_frame_id=source, objects=objects)
    encode_message(message)  # preflight size and complete schema
    return message
