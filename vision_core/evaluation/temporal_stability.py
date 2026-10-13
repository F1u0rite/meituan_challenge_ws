"""CSV recording and 180-degree periodic pose stability statistics."""
import csv
import json
import math
from pathlib import Path
import numpy as np

COLORS = ('red', 'yellow', 'blue', 'green')
CSV_FIELDS = ('timestamp', 'frame_number', 'board_detected', 'color', 'detected',
              'x_mm', 'y_mm', 'yaw_deg', 'confidence', 'slot', 'slot_xy_error_mm')


def orientation_stats(yaw_degrees):
    """Circular mean/std for unoriented axes with a 180 degree period."""
    if not yaw_degrees:
        return {'mean_deg': None, 'std_deg': None}
    theta = np.deg2rad(np.asarray(yaw_degrees, dtype=float) * 2)
    mean_sin, mean_cos = np.sin(theta).mean(), np.cos(theta).mean()
    mean = math.degrees(math.atan2(mean_sin, mean_cos)) / 2 % 180
    resultant = min(1.0, math.hypot(mean_sin, mean_cos))
    std = math.degrees(math.sqrt(-2 * math.log(max(resultant, 1e-12)))) / 2
    return {'mean_deg': mean, 'std_deg': std}


def linear_stats(values):
    if not values:
        return {'mean': None, 'std': None, 'min': None, 'max': None}
    arr = np.asarray(values, dtype=float)
    return {key: float(value) for key, value in
            (('mean', arr.mean()), ('std', arr.std()), ('min', arr.min()), ('max', arr.max()))}


class StabilityRecorder:
    def __init__(self, csv_path: Path, slots_by_color: dict[str, str] | None = None,
                 slot_centers: dict | None = None):
        self.csv_path = Path(csv_path)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.slots_by_color = slots_by_color or {}
        self.slot_centers = slot_centers or {}
        self._file = self.csv_path.open('w', encoding='utf-8-sig', newline='')
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self.rows: list[dict] = []
        self.frame_count = 0
        self.board_found_count = 0

    def record(self, timestamp, frame_number, result):
        self.frame_count += 1
        self.board_found_count += int(result.board_valid)
        best = {}
        if result.board_valid:
            for detection in result.detections:
                if detection.color not in best or detection.confidence > best[detection.color].confidence:
                    best[detection.color] = detection
        for color in COLORS:
            detection = best.get(color)
            slot = self.slots_by_color.get(color, '')
            distance = ''
            if detection is not None and slot in self.slot_centers:
                center = self.slot_centers[slot]['center_mm']
                distance = float(np.hypot(detection.x_mm-center[0], detection.y_mm-center[1]))
            row = {'timestamp': timestamp, 'frame_number': frame_number,
                   'board_detected': int(result.board_valid), 'color': color,
                   'detected': int(detection is not None),
                   'x_mm': detection.x_mm if detection else '',
                   'y_mm': detection.y_mm if detection else '',
                   'yaw_deg': detection.yaw_deg if detection else '',
                   'confidence': detection.confidence if detection else '',
                   'slot': slot, 'slot_xy_error_mm': distance}
            self._writer.writerow(row)
            self.rows.append(row)
        self._file.flush()

    def summarize(self):
        by_color = {}
        for color in COLORS:
            rows = [r for r in self.rows if r['color'] == color]
            seen = [r for r in rows if r['detected']]
            slot_errors = [r['slot_xy_error_mm'] for r in seen if r['slot_xy_error_mm'] != '']
            entry = {'detection_rate': len(seen) / self.frame_count if self.frame_count else 0,
                     'detected_frames': len(seen),
                     'x_mm': linear_stats([r['x_mm'] for r in seen]),
                     'y_mm': linear_stats([r['y_mm'] for r in seen]),
                     'yaw': orientation_stats([r['yaw_deg'] for r in seen])}
            if color in self.slots_by_color:
                entry['slot'] = self.slots_by_color[color]
                entry['slot_xy_error_mm'] = {
                    'mean': float(np.mean(slot_errors)) if slot_errors else None,
                    'median': float(np.median(slot_errors)) if slot_errors else None,
                    'p95': float(np.percentile(slot_errors, 95)) if slot_errors else None}
            by_color[color] = entry
        return {'frames': self.frame_count,
                'board_detection_rate': self.board_found_count / self.frame_count if self.frame_count else 0,
                'colors': by_color}

    def stop(self):
        if not self._file.closed:
            self._file.close()
        summary = self.summarize()
        path = self.csv_path.with_name(self.csv_path.stem + '_summary.json')
        path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
        return path, summary
