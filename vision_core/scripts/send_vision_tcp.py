"""No-camera V1 encoder / opt-in network test. Default is dry-run."""
import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception.live_vision_protocol import vision_message, encode_message, strict_loads
from perception.tcp_vision_sender import TCPVisionSender


def demo_results():
    for index in range(4):
        # Explicitly simulated observations, not real capture or real color verification.
        yield dict(coordinate_frame='test_color_optical_frame',
            timestamps=dict(host_utc=datetime.now(timezone.utc).isoformat(),
                            color_sdk_ms=float(index), color_timestamp_domain='simulated'),
            objects=[dict(object_id='synthetic-'+color, color=color, confidence=.9,
                          detection_status='CONFIRMED_OBJECT', ambiguity=False,
                          body_pose_valid=False, ring_pose_valid=False)
                     for color in ('RED', 'BLUE', 'YELLOW', 'GREEN')])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, help='Existing core detections.jsonl; preserves original capture time')
    parser.add_argument('--connect', action='store_true', help='Explicitly connect Windows loopback after bridge/tunnel confirmation')
    parser.add_argument('--config', type=Path, default=ROOT/'config/tcp_sender.yaml')
    parser.add_argument('--output', type=Path, help='Optional new protocol JSONL, refuses overwrite')
    parser.add_argument('--max-frames', type=int, default=4)
    parser.add_argument('--interval-s', type=float, default=.1)
    args = parser.parse_args(argv)
    if args.max_frames <= 0 or not 0 < args.interval_s < float('inf'):
        parser.error('Frame limit and interval must be positive finite')
    import yaml
    config = yaml.safe_load(args.config.read_text(encoding='utf-8-sig'))
    if not args.input: config['camera_frame_id'] = 'test_color_optical_frame'
    sender = TCPVisionSender(**config)
    stream = args.input.open(encoding='utf-8-sig') if args.input else None
    rows = (strict_loads(line) for line in stream if line.strip()) if stream else demo_results()
    output = None; count = 0
    try:
        if args.output: output = args.output.open('x', encoding='utf-8')
        if args.connect: sender.start()
        for index, result in enumerate(rows):
            if index >= args.max_frames: break
            message = vision_message(result, camera_frame_id=config['camera_frame_id'],
                                     namespace=sender.namespace, mode='test')
            message['sequence_id'] = index  # dry-run example only; worker assigns actual connection sequence
            wire = encode_message(message)
            if output: output.write(json.dumps(message, allow_nan=False)+'\n')
            print(json.dumps(dict(mode='test', dry_run=not args.connect, frame=index,
                payload_bytes=len(wire)-4, timestamp_domain=message['timestamp_domain'],
                objects=len(message['objects']), raw_capture_ms=message['capture_timestamp_raw'])))
            if args.connect:
                sender.offer(message); time.sleep(args.interval_s)
            count += 1
        if args.connect: time.sleep(min(1., config.get('send_timeout_s', .5)))
    finally:
        sender.close()
        if stream: stream.close()
        if output: output.close()
    print(json.dumps(dict(frames_encoded=count, camera_started=False, sender=sender.snapshot()), indent=2))
    return 0 if not args.connect or sender.snapshot()['sent'] > 0 else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
