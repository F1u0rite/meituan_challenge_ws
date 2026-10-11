"""Handheld D435i RGB preview and pose stability capture."""
import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluation.temporal_stability import StabilityRecorder
from evaluation.pixel_stability import PixelStabilityRecorder
from input.realsense_source import RealSenseSource
from perception.battery_only import PixelBatteryDetector
from perception.live_pipeline import LiveVisionPipeline
from synthetic.io import write_png
from synthetic.scene_generator import load_config


def label(image, text, y=32, color=(255, 255, 255)):
    cv2.putText(image, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, .72, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, .72, color, 2, cv2.LINE_AA)


def panels(frame, result, fps):
    raw = frame.copy()
    board = frame.copy()
    status = 'BOARD FOUND' if result.board_valid else 'BOARD LOST - NO COORDINATES'
    status_color = (0, 220, 0) if result.board_valid else (0, 0, 255)
    label(raw, f'CAMERA  {fps:.1f} FPS  {result.latency_ms:.0f} ms')
    label(board, status, color=status_color)
    if result.board_valid:
        points = np.rint(result.board.corners_px).astype(np.int32)
        cv2.polylines(board, [points], True, (0, 255, 255), 3)
        for name, point in zip(('TL', 'TR', 'BR', 'BL'), points):
            cv2.circle(board, tuple(point), 6, (0, 0, 255), -1)
            cv2.putText(board, name, tuple(point + [8, -8]), cv2.FONT_HERSHEY_SIMPLEX,
                        .55, (0, 0, 255), 2, cv2.LINE_AA)
        label(board, f'BOARD FOUND  confidence {result.board.confidence:.2f}', 65, status_color)
        rectified = result.rectified.copy()
        visual = rectified.copy()
        for item in result.detections:
            cv2.drawContours(visual, [item.contour], -1, (0, 0, 0), 3)
            center = tuple(int(round(v)) for v in item.center_px)
            cv2.circle(visual, center, 5, (0, 0, 0), -1)
            cv2.putText(visual, f'{item.color} ({item.x_mm:.1f},{item.y_mm:.1f}) '
                        f'{item.yaw_deg:.1f} deg  c={item.confidence:.2f}',
                        (max(5, center[0]-100), max(25, center[1]-18)),
                        cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 0), 2, cv2.LINE_AA)
    else:
        rectified = np.full((540, 700, 3), 55, np.uint8)
        visual = rectified.copy()
        label(rectified, status, 70, status_color)
        label(visual, status, 70, status_color)
    return raw, board, rectified, visual


def mosaic(images):
    tiles = []
    for image in images:
        h, w = image.shape[:2]
        scale = min(640/w, 360/h)
        resized = cv2.resize(image, (round(w*scale), round(h*scale)))
        tile = np.full((380, 660, 3), 25, np.uint8)
        y, x = (360-resized.shape[0])//2+10, (640-resized.shape[1])//2+10
        tile[y:y+resized.shape[0], x:x+resized.shape[1]] = resized
        tiles.append(tile)
    return np.vstack((np.hstack(tiles[:2]), np.hstack(tiles[2:])))


def save_debug(root, color_frame, result, images):
    folder = root / datetime.now().strftime('frame_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    write_png(folder / 'raw.png', color_frame.image)
    for name, image in zip(('board_detection', 'rectified', 'result'), images[1:]):
        write_png(folder / f'{name}.png', image)
    metadata = {
        'timestamp': color_frame.timestamp, 'frame_number': color_frame.frame_number,
        'board_detected': result.board_valid, 'board_confidence': result.board.confidence,
        'board_corners_px': result.board.corners_px.tolist() if result.board_valid else None,
        'latency_ms': result.latency_ms,
        'detections': [{'color': d.color, 'x_mm': d.x_mm, 'y_mm': d.y_mm,
                        'yaw_deg': d.yaw_deg, 'confidence': d.confidence}
                       for d in result.detections] if result.board_valid else [],
    }
    (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    return folder


def pixel_overlay(frame, detections, fps, latency_ms):
    visual = frame.copy()
    label(visual, f'coordinate_mode = IMAGE_PIXEL  {fps:.1f} FPS  {latency_ms:.0f} ms')
    palette = {'red': (0, 0, 255), 'yellow': (0, 255, 255),
               'blue': (255, 0, 0), 'green': (0, 255, 0)}
    for item in detections:
        color = palette[item.color]
        cv2.drawContours(visual, [item.contour], -1, color, 2)
        cv2.polylines(visual, [np.rint(item.box_px).astype(np.int32)], True, (255, 255, 255), 2)
        center = (round(item.u), round(item.v))
        cv2.circle(visual, center, 5, color, -1)
        angle = np.deg2rad(item.yaw_deg)
        end = (round(item.u-55*np.sin(angle)), round(item.v+55*np.cos(angle)))
        cv2.arrowedLine(visual, center, end, color, 3)
        label(visual, f'{item.color} u={item.u:.1f} v={item.v:.1f} '
              f'yaw={item.yaw_deg:.1f} c={item.confidence:.2f}',
              max(65, min(frame.shape[0]-10, center[1]-18)), color)
    return visual


def save_pixel_debug(root, color_frame, detections, detection_image, latency_ms):
    folder = root / datetime.now().strftime('battery_only_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    write_png(folder/'raw.png', color_frame.image)
    write_png(folder/'detection.png', detection_image)
    metadata = {'timestamp': color_frame.timestamp, 'frame_number': color_frame.frame_number,
                'coordinate_mode': 'IMAGE_PIXEL', 'latency_ms': latency_ms,
                'detections': [{'color': d.color, 'u': d.u, 'v': d.v,
                                'yaw_deg': d.yaw_deg, 'confidence': d.confidence,
                                'contour_px': d.contour.reshape(-1, 2).tolist(),
                                'rotated_box_px': d.box_px.tolist()}
                               for d in detections]}
    (folder/'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    return folder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial')
    parser.add_argument('--fps', type=int, default=30, help='Color FPS; default 30')
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parents[1]/'outputs'/'realsense')
    parser.add_argument('--min-board-confidence', type=float, default=0.0)
    parser.add_argument('--probe', action='store_true', help='Capture and process one RGB frame without GUI')
    parser.add_argument('--battery-only', action='store_true', help='Detect real batteries in image pixels without a board')
    parser.add_argument('--color-config', type=Path,
                        default=Path(__file__).resolve().parents[1]/'config'/'realsense_color.yaml')
    for color in ('red', 'yellow', 'blue', 'green'):
        parser.add_argument(f'--{color}-slot', choices=list('ABCD'))
    args = parser.parse_args()
    config = load_config()
    slot_map = {color: getattr(args, f'{color}_slot') for color in ('red', 'yellow', 'blue', 'green')
                if getattr(args, f'{color}_slot')}
    pipeline = PixelBatteryDetector(args.color_config) if args.battery_only else LiveVisionPipeline(config, args.min_board_confidence)
    recorder = None
    prior = perf_counter()
    try:
        with RealSenseSource(serial=args.serial, fps=args.fps) as source:
            if args.probe:
                for _ in range(10):
                    source.read()  # Allow auto-exposure to settle before the saved probe frame.
            while True:
                frame = source.read()
                now = perf_counter()
                fps = 1/(now-prior) if now > prior else 0
                prior = now
                if args.battery_only:
                    start = perf_counter()
                    detections = pipeline.detect(frame.image)
                    latency_ms = (perf_counter()-start)*1000
                    visual = pixel_overlay(frame.image, detections, fps, latency_ms)
                    if args.probe:
                        folder = save_pixel_debug(args.output_dir, frame, detections, visual, latency_ms)
                        print(json.dumps({'coordinate_mode': 'IMAGE_PIXEL',
                                          'detections': [d.color for d in detections],
                                          'saved': str(folder)}, ensure_ascii=False))
                        break
                    if recorder is not None:
                        recorder.record(frame.timestamp, frame.frame_number, detections)
                    cv2.imshow('D435i Battery Only | q quit | s save | r record', visual)
                    key = cv2.waitKey(1) & 0xff
                    if key == ord('q'):
                        break
                    if key == ord('s'):
                        print(f'Saved {save_pixel_debug(args.output_dir, frame, detections, visual, latency_ms)}')
                    if key == ord('r'):
                        if recorder is None:
                            args.output_dir.mkdir(parents=True, exist_ok=True)
                            index = 1
                            while (args.output_dir/f'battery_only_session_{index:03d}.csv').exists():
                                index += 1
                            recorder = PixelStabilityRecorder(
                                args.output_dir/f'battery_only_session_{index:03d}.csv',
                                pipeline.config['tracking_match_radius_px'])
                            print(f'Recording {recorder.csv_path}')
                        else:
                            path, summary = recorder.stop()
                            print(json.dumps({'summary_path': str(path), **summary}, ensure_ascii=False, indent=2))
                            recorder = None
                    continue
                result = pipeline.process(frame.image)
                images = panels(frame.image, result, fps)
                if args.probe:
                    folder = save_debug(args.output_dir, frame, result, images)
                    print(json.dumps({'board_detected': result.board_valid,
                                      'detections': [d.color for d in result.detections],
                                      'saved': str(folder)}, ensure_ascii=False))
                    break
                if recorder is not None:
                    recorder.record(frame.timestamp, frame.frame_number, result)
                cv2.imshow('D435i Handheld RGB | q quit | s save | r record', mosaic(images))
                key = cv2.waitKey(1) & 0xff
                if key == ord('q'):
                    break
                if key == ord('s'):
                    print(f'Saved {save_debug(args.output_dir, frame, result, images)}')
                if key == ord('r'):
                    if recorder is None:
                        recordings = args.output_dir
                        recordings.mkdir(parents=True, exist_ok=True)
                        index = 1
                        while (recordings/f'session_{index:03d}.csv').exists():
                            index += 1
                        recorder = StabilityRecorder(recordings/f'session_{index:03d}.csv',
                                                     slot_map, config['battery_slots'])
                        print(f'Recording {recorder.csv_path}')
                    else:
                        path, summary = recorder.stop()
                        print(json.dumps({'summary_path': str(path), **summary}, ensure_ascii=False, indent=2))
                        recorder = None
    finally:
        if recorder is not None:
            path, summary = recorder.stop()
            print(json.dumps({'summary_path': str(path), **summary}, ensure_ascii=False, indent=2))
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
