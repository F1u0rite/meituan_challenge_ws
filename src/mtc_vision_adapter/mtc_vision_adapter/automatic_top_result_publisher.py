"""Send current automatic top/CAD point candidates; never generate motion poses."""
import argparse
import json
import math
import re
import time
import uuid
from pathlib import Path

from .core import COLORS, to_ros

COLOR_ORDER = ('red', 'blue', 'yellow', 'green')
BATTERIES_TOPIC = '/mtc/perception/batteries'
CANDIDATES_TOPIC = '/mtc/perception/top_hole_candidates'
SCHEMA = 'mtc.top_hole_candidates.v1'


def _number(value, name):
    if isinstance(value, bool):
        raise ValueError(name + ' must be finite numeric')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(name + ' must be finite numeric')
    return value


def _xyz(value, name):
    if value is None or len(value) != 3:
        raise ValueError(name + ' requires three coordinates')
    return [_number(v, name) for v in value]


def candidate_packet(result, *, session_id, sequence, mode):
    """Pure conversion. Four colour slots; unavailable slots contain null, never history."""
    if mode not in ('live', 'replay', 'offline'):
        raise ValueError('invalid source mode')
    if result.get('unit_position') != 'm':
        raise ValueError('position unit must explicitly be m')
    frame = result.get('coordinate_frame')
    if not isinstance(frame, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', frame):
        raise ValueError('invalid coordinate frame')
    sid = result.get('source_frame_id')
    if not isinstance(sid, str) or not sid:
        raise ValueError('source frame ID required')
    capture = _number(result['capture_timestamp'], 'capture timestamp')
    if capture <= 0:
        raise ValueError('capture timestamp must be positive')
    if any(result.get(k) for k in ('ring_pose_valid', 'body_pose_valid', 'motion_usable')):
        raise ValueError('candidate output cannot contain valid motion poses')
    objects = {}
    source_times = result.get('timestamps')
    for obj in result['objects']:
        color = obj['color']
        if color not in COLOR_ORDER or color in objects:
            raise ValueError('duplicate or unknown colour')
        if any(obj.get(k) for k in ('ring_pose_valid', 'body_pose_valid', 'motion_usable')):
            raise ValueError('candidate output cannot contain valid motion poses')
        times = obj.get('geometry', {}).get('source_timestamps')
        if times:
            if source_times is None:
                source_times = times
            elif times != source_times:
                raise ValueError('geometry/source timestamp mismatch')
        objects[color] = obj
    # Sensor serial numbers, raw masks, CAD assets and private paths are not exported.
    allowed_times = ('host_utc', 'color_sdk_ms', 'depth_sdk_ms', 'color_frame_number',
                     'depth_frame_number', 'color_timestamp_domain',
                     'depth_timestamp_domain', 'queued_frames_discarded')
    times = {k: v for k, v in (source_times or {}).items() if k in allowed_times}
    packet = dict(schema=SCHEMA, session_id=session_id, sequence=sequence,
        source_mode=mode, replay=mode != 'live', source_frame_id=sid,
        coordinate_frame=frame, unit_position='m', capture_timestamp_s=capture,
        capture_timestamp_source=result.get('capture_timestamp_source',
            'RECORDED_SOURCE_LABEL_UNAVAILABLE'), timestamps=times,
        ros_capture_clock_verified=False,
        ros_header_policy='ZERO_UNMAPPED_CAMERA_CLOCK',
        body_pose_valid=False, ring_pose_valid=False, motion_usable=False,
        processing_ms=_number(result.get('processing_ms', 0), 'processing time'),
        objects=[], missing_colors=[])
    for color in COLOR_ORDER:
        obj = objects.get(color, {})
        available = obj.get('candidate_available') is True and result.get('pair_qualified') is True
        reasons = list(obj.get('reasons', ['NO_CURRENT_CANDIDATE']))
        if result.get('pair_qualified') is not True:
            reasons.append('RGBD_PAIR_REJECTED')
        top = _xyz(obj.get('green_point_camera_m'), 'top point') if available else None
        hole = _xyz(obj.get('red_point_camera_m'), 'hole point') if available else None
        entry = dict(object_id=f'{session_id}-{color}', identity_kind='UNIQUE_COLOR_SLOT',
            color=color, color_code=COLORS[color], candidate_available=available,
            status='CANDIDATE' if available else 'MISSING',
            green_point_camera_m=top, red_point_camera_m=hole,
            green_point_semantics='OBSERVED_TOP_CENTER',
            red_point_semantics='CAD_DERIVED_HOLE_CENTER',
            rgb_confirmed=obj.get('rgb_confirmed') is True,
            geometry_consistent=obj.get('geometry_consistent') is True,
            reasons=reasons, ring_pose_valid=False, body_pose_valid=False,
            absolute_accuracy='NOT_MEASURED')
        for key in ('green_height_m', 'red_height_m'):
            entry[key] = _number(obj[key], key) if available and obj.get(key) is not None else None
        entry['coordinate_variant'] = obj.get('coordinate_variant', 'RAW_UNCORRECTED')
        if entry['coordinate_variant'] not in ('RAW_UNCORRECTED','HEIGHT_CONSTRAINED'):
            raise ValueError('invalid coordinate variant')
        entry['height_constraint'] = obj.get('height_constraint', dict(enabled=False,applied=False))
        raw_available = obj.get('raw_candidate_available') is True and result.get('pair_qualified') is True
        entry['raw_candidate_available'] = raw_available
        for key in ('raw_green_point_camera_m','raw_red_point_camera_m'):
            entry[key] = _xyz(obj.get(key),key) if raw_available else None
        for key in ('raw_green_height_m','raw_red_height_m'):
            entry[key] = _number(obj[key],key) if raw_available and obj.get(key) is not None else None
        if entry['coordinate_variant'] == 'HEIGHT_CONSTRAINED':
            if not available or not raw_available or entry['height_constraint'].get('applied') is not True:
                raise ValueError('height-constrained point requires raw source and applied status')
            entry['green_point_semantics'] = 'HEIGHT_CONSTRAINED_TOP_CENTER'
            entry['red_point_semantics'] = 'HEIGHT_CONSTRAINED_CAD_HOLE_CENTER'
        packet['objects'].append(entry)
        if not available:
            packet['missing_colors'].append(color)
    # Validate the whole packet before either ROS publication.
    targets = []
    for target in result.get('placement_targets', []):
        if target.get('target_id') != 'T0' or targets:
            raise ValueError('duplicate or unsupported placement target')
        if target.get('coordinate_frame') != frame or target.get('unit_position') != 'm':
            raise ValueError('placement target frame/unit mismatch')
        if target.get('ring_pose_valid') or target.get('motion_usable'):
            raise ValueError('placement target remains a candidate')
        entry = dict(target)
        available = target.get('candidate_available') is True and result.get('pair_qualified') is True
        entry['candidate_available'] = available
        for key in ('table_center_camera_m', 'target_hole_center_camera_m'):
            entry[key] = _xyz(target.get(key), key) if available else None
        targets.append(entry)
    packet['placement_targets'] = targets
    packet['height_constraint'] = result.get('height_constraint',dict(enabled=False))
    if result.get('scene_reference') is not None:
        from perception.scene_frame import scene_coordinates
        reference = result['scene_reference']
        if reference.get('schema') != 'mtc.scene_reference.v1' or type(reference.get('ready')) is not bool:
            raise ValueError('invalid scene reference schema/readiness')
        if reference.get('input_coordinate_frame') != frame or reference.get('unit_position') != 'm':
            raise ValueError('scene reference frame/unit mismatch')
        if reference.get('motion_usable') or reference.get('robot_transform_available'):
            raise ValueError('scene reference is not a robot calibration')
        packet['scene_coordinates'] = scene_coordinates(packet, reference)
        packet['scene_reference'] = reference
    json.dumps(packet, allow_nan=False)
    return packet


def battery_array(packet):
    """Existing IDL: hole XYZ only; top is not the body centre; orientation unknown."""
    header = dict(frame_id=packet['coordinate_frame'], stamp=dict(sec=0, nanosec=0))
    def pose(xyz=None):
        return dict(header=header, pose=dict(
            position=dict(zip(('x', 'y', 'z'), xyz)) if xyz else dict.fromkeys(('x', 'y', 'z')),
            orientation=dict.fromkeys(('x', 'y', 'z', 'w'))))
    detections = [dict(object_id=o['object_id'], color=o['color_code'], confidence=0.,
        body_pose=pose(), ring_pose=pose(o['red_point_camera_m']), ring_pose_valid=False)
        for o in packet['objects'] if o['candidate_available']]
    return dict(header=header, detections=detections)


class AutomaticTopResultPublisher:
    """Reusable sink for live inference, offline inference and explicitly labelled replay."""
    def __init__(self, wait_for_subscriber_s=0.):
        if not math.isfinite(wait_for_subscriber_s) or wait_for_subscriber_s < 0:
            raise ValueError('invalid subscriber wait')
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from mtc_interfaces.msg import BatteryDetectionArray, BatteryDetection
        from std_msgs.msg import String
        self.rclpy = rclpy
        self.context = Context()
        self.node = self.executor = None
        self.closed = False
        self.sequence = 0
        self.session_id = 'yuze-' + uuid.uuid4().hex[:12]
        self.Array, self.Detection, self.String = BatteryDetectionArray, BatteryDetection, String
        rclpy.init(context=self.context)
        try:
            self.node = Node('yuze_automatic_top_coordinates', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
            self.batteries = self.node.create_publisher(BatteryDetectionArray, BATTERIES_TOPIC, qos)
            self.candidates = self.node.create_publisher(String, CANDIDATES_TOPIC, qos)
            if wait_for_subscriber_s:
                deadline = time.monotonic() + wait_for_subscriber_s
                while not (self.batteries.get_subscription_count() and self.candidates.get_subscription_count()):
                    if time.monotonic() >= deadline:
                        raise TimeoutError('both coordinate subscribers must match')
                    self.executor.spin_once(timeout_sec=.05)
        except BaseException:
            self._destroy()
            raise

    def publish_result(self, result, *, mode):
        if self.closed:
            raise RuntimeError('coordinate publisher closed')
        packet = candidate_packet(result, session_id=self.session_id,
            sequence=self.sequence + 1, mode=mode)
        stamp = self.node.get_clock().now().to_msg()
        packet['ros_dispatch_stamp'] = dict(sec=stamp.sec, nanosec=stamp.nanosec)
        packet['dispatch_clock_is_not_capture_clock'] = True
        encoded = json.dumps(packet, allow_nan=False, separators=(',', ':'))
        array = to_ros(battery_array(packet), self.Array, self.Detection)
        self.batteries.publish(array)
        self.candidates.publish(self.String(data=encoded))
        self.executor.spin_once(timeout_sec=0.)
        self.sequence += 1
        return packet

    def _destroy(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        self.rclpy.try_shutdown(context=self.context)

    def close(self):
        if self.closed:
            return
        self.closed = True
        from rclpy.duration import Duration
        try:
            acked = [p.wait_for_all_acked(Duration(seconds=2.))
                for p in (self.batteries, self.candidates)]
            if not all(acked):
                raise TimeoutError('coordinate samples not acknowledged')
        finally:
            self._destroy()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main():
    parser = argparse.ArgumentParser(description='Replay automatic point rows; never opens camera.')
    parser.add_argument('--rows', required=True, type=Path)
    parser.add_argument('--period-s', type=float, default=.2)
    parser.add_argument('--wait-subscriber-s', type=float, default=0.)
    args = parser.parse_args()
    if not math.isfinite(args.period_s) or args.period_s < 0:
        parser.error('period must be finite and nonnegative')
    rows = json.loads(args.rows.read_text())
    # Validate every row before establishing an externally visible publisher.
    for row in rows:
        candidate_packet(row, session_id='preflight', sequence=1, mode='replay')
    with AutomaticTopResultPublisher(args.wait_subscriber_s) as sink:
        for i, row in enumerate(rows):
            packet = sink.publish_result(row, mode='replay')
            print(json.dumps(dict(sequence=packet['sequence'],
                source_frame_id=packet['source_frame_id'], missing_colors=packet['missing_colors'])), flush=True)
            if i + 1 < len(rows):
                time.sleep(args.period_s)


if __name__ == '__main__':
    main()
