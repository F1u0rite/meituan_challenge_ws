"""Initialize a current T0 scene frame; no SAM, robot motion or cached transforms."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'vision_core'), str(ROOT / 'src/mtc_vision_adapter')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--input', type=Path, help='Recorded frame_* directory; no camera opened')
    source.add_argument('--live', action='store_true', help='Explicitly open/release RealSense; never stop another owner')
    direction = parser.add_mutually_exclusive_group()
    direction.add_argument('--positive-x-pixel', nargs=2, type=float, metavar=('U', 'V'),
        help='Current-view pixel near the positive long end of T0; must be reselected after moving camera')
    direction.add_argument('--aruco-id', type=int, help='Fixed table marker, corner 0->1 along T0 +X')
    parser.add_argument('--aruco-dictionary', default='DICT_4X4_50')
    parser.add_argument('--geometry', type=Path, required=True, help='Read-only CAD geometry JSON')
    parser.add_argument('--frames', type=int, default=10, help='Maximum distinct samples, minimum 2')
    parser.add_argument('--stable-frames', type=int, default=5)
    parser.add_argument('--out', type=Path, help='New output directory under this workspace log/ only')
    parser.add_argument('--coordinates', type=Path, help='Optional saved current coordinate packets/rows, matched by full source identity')
    args = parser.parse_args()
    if args.frames < 2 or args.stable_frames < 2:
        parser.error('frames and stable-frames must be at least 2')
    if args.coordinates and args.live:
        parser.error('saved coordinate packets are for recorded input only')
    # All validation precedes opening hardware. Saved outputs are evidence, not
    # reusable calibration: no load-calibration or cached fallback is provided.
    import cv2
    import numpy as np
    import yaml
    from input.realsense_rgbd_source import RGBDFrame, RealSenseRGBDSource
    from input.rgbd_pair_guard import RGBDPairGuard
    from perception.rgbd_geometry import CameraIntrinsics, frame_capture_time
    from perception.simplified_rgbd import BatteryDetection
    from perception.automatic_top_prompt import AutomaticTopPrompt
    from perception.placement_target import T0PlacementTarget
    from perception.scene_frame import SceneFrameInitializer, scene_coordinates
    initializer = SceneFrameInitializer(positive_x_pixel=args.positive_x_pixel,
        aruco_id=args.aruco_id, aruco_dictionary=args.aruco_dictionary,
        min_stable_frames=args.stable_frames)
    geometry = json.loads(args.geometry.read_text())
    detector = BatteryDetection(yaml.safe_load((ROOT / 'vision_core/config/simplified_rgbd.yaml').read_text()))
    proposer = AutomaticTopPrompt()
    placement = T0PlacementTarget(geometry, physical_hole_height_m=.060)
    guard = RGBDPairGuard()
    folders = sorted(args.input.glob('frame_*')) if args.input else None
    if folders is not None and len(folders) < 2:
        parser.error('recorded input requires at least two frame_* directories')
    packets = {}
    if args.coordinates:
        loaded = json.loads(args.coordinates.read_text())
        if not isinstance(loaded, list):
            parser.error('coordinates must be a JSON list of packets/rows')
        for packet in loaded:
            sid = packet['source_frame_id']
            if sid in packets:
                parser.error('duplicate source frame in coordinates')
            packets[sid] = packet
    out = args.out or ROOT / 'log' / ('scene_init_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
    out = out.expanduser().resolve()
    log = (ROOT / 'log').resolve()
    if not out.is_relative_to(log) or out == log:
        parser.error('output must be a new directory inside the Yuze workspace log/')
    if out.exists():
        parser.error('output directory already exists; choose a new path')
    out.mkdir(parents=True)
    rows = []
    camera = None
    frame = None
    accepted_frame = None
    error = None
    try:
        if args.live:
            camera = RealSenseRGBDSource()
            camera.start()
            for _ in range(20):
                camera.read(latest=True)
        for index in range(args.frames):
            if camera is not None:
                frame = camera.read(latest=True)
            else:
                if index >= len(folders):
                    break
                folder = folders[index]
                meta = json.loads((folder / 'metadata.json').read_text())
                with np.load(folder / 'depth.npz') as archive:
                    depth = archive['depth'].copy()
                frame = RGBDFrame(cv2.imread(str(folder / 'color.png')), depth,
                    CameraIntrinsics(**meta['intrinsics']), meta['depth_scale'],
                    meta['timestamps'], {}, alignment_metadata=meta['alignment'])
            stamp, source_label = frame_capture_time(frame)
            sid = f"color_{frame.timestamps['color_frame_number']}_depth_{frame.timestamps['depth_frame_number']}"
            proposals, _, _ = detector.detect(frame.color_bgr, stamp)
            # Every RGB mask excludes foreground from the background hypothesis.
            masks = {str(i): p['battery_mask'] for i, p in enumerate(proposals)}
            pair = guard.inspect(frame.timestamps)
            qualified = pair['qualified'] and (frame.alignment_metadata or {}).get('method') == 'SDK_DEPTH_TO_COLOR'
            prompt = proposer.propose(frame, masks, qualified=qualified)
            result = dict(source_frame_id=sid, timestamps=dict(frame.timestamps),
                capture_timestamp=stamp, capture_timestamp_source=source_label,
                coordinate_frame='camera_color_optical', unit_position='m',
                pair_qualified=qualified, support_plane=prompt['support_plane'], objects=[])
            result['placement_targets'] = [placement.estimate(frame, result)]
            reference = initializer.process(frame, result)
            reference.update(source_mode='live' if camera else 'offline',
                intrinsics=asdict(frame.intrinsics), depth_scale_m=frame.depth_scale,
                rgb_alignment=(frame.alignment_metadata or {}).get('method'),
                robot_started=False, camera_capture_started=bool(camera))
            # Use matched inference points if supplied, otherwise only T0 target.
            current_points = packets.get(sid, result)
            reference['scene_coordinates'] = scene_coordinates(current_points, reference)
            reference['matching_battery_packet_present'] = sid in packets
            rows.append(reference)
            accepted_frame = frame
            print(json.dumps(dict(source_frame_id=sid, status=reference['status'],
                stable_frames=reference['stable_frames'], depth_valid_ratio=reference['depth_valid_ratio']), allow_nan=False), flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        error = type(exc).__name__ + ': ' + str(exc)
    finally:
        if camera is not None:
            camera.stop()
        (out / 'frames.json').write_text(json.dumps(rows, indent=2, allow_nan=False))
        latest = rows[-1] if rows else None
        summary = dict(schema='mtc.scene_initialization.v1',
            ready=bool(error is None and latest and latest['ready']),
            status='ERROR' if error else latest['status'] if latest else 'NO_SAMPLES',
            error=error, processed_frames=len(rows), latest_reference=latest,
            camera_released=True, robot_started=False,
            reference_reuse_allowed=False, absolute_accuracy='NOT_MEASURED')
        (out / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
        if accepted_frame is not None:
            frame = accepted_frame
            image = frame.color_bgr.copy()
            if latest and latest.get('t0_border_corners_px'):
                border = np.rint(latest['t0_border_corners_px']).astype(np.int32)
                cv2.polylines(image, [border], True, (255, 255, 0), 2)
                for pixel in border:
                    cv2.putText(image, f'{pixel[0]},{pixel[1]}', tuple(pixel),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 0), 1)
            if latest and latest.get('origin_camera_m') and latest.get('axes_camera_xyz'):
                points = np.array([latest['origin_camera_m']] + [
                    (np.array(latest['origin_camera_m']) + .06 * np.array(latest['axes_camera_xyz'][axis])).tolist()
                    for axis in ('x', 'y', 'z')])
                k = frame.intrinsics
                pixels = np.column_stack((points[:, 0] / points[:, 2] * k.fx + k.ppx,
                    points[:, 1] / points[:, 2] * k.fy + k.ppy)).astype(int)
                for j, (label, colour) in enumerate(zip(('X', 'Y', 'Z'), ((0,0,255), (0,255,0), (255,0,0))), 1):
                    cv2.arrowedLine(image, tuple(pixels[0]), tuple(pixels[j]), colour, 3)
                    cv2.putText(image, label, tuple(pixels[j]), cv2.FONT_HERSHEY_SIMPLEX, .7, colour, 2)
            cv2.putText(image, summary['status'], (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 0, 255), 2)
            cv2.imwrite(str(out / 'preview.png'), image)
        print(json.dumps(dict(output=str(out), ready=summary['ready'], status=summary['status'], error=error)), flush=True)
    return 0 if summary['ready'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
