#!/usr/bin/env python3
"""Validate a JSON TF/pose snapshot; stdout is JSON, exit 0 diagnostic OK, 2 rejected."""
import argparse
import json
from pathlib import Path

from contract import FrameGraph, Pose, transform_pose
from geometry import CalibrationError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path, help='schema v1 JSON case; not a ROS parameter file')
    parser.add_argument('--output', type=Path, help='save JSON diagnostics in addition to stdout')
    args = parser.parse_args()
    try:
        d = json.loads(args.case.read_text(encoding='utf-8-sig'))
        result = transform_pose(FrameGraph(d), Pose.from_dict(d.get('pose')),
                                d.get('target_frame'), now_ns=d.get('now_ns'),
                                max_age_ns=d.get('max_age_ns'), time_domain=d.get('time_domain'))
        exit_code = 0
    except (CalibrationError, OSError, ValueError, TypeError) as error:
        result = {'status': 'REJECTED', 'code': getattr(error, 'code', 'INVALID_INPUT'),
                  'message': str(error), 'motion_authorized': False}
        exit_code = 2
    output = json.dumps(result, ensure_ascii=True, indent=2, allow_nan=False)
    if args.output:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output + '\n', encoding='utf-8')
        except OSError as error:
            print(json.dumps({'status': 'REJECTED', 'code': 'OUTPUT_ERROR', 'message': str(error)}))
            return 2
    print(output)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
