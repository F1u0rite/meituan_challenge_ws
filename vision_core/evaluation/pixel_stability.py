"""Battery-only temporal statistics in image pixels, never board millimetres."""
import csv
import json
from pathlib import Path
import numpy as np
from .temporal_stability import COLORS, linear_stats, orientation_stats

FIELDS = ('timestamp', 'frame_number', 'color', 'detected', 'u', 'v', 'yaw_deg', 'confidence')


class PixelStabilityRecorder:
    def __init__(self, csv_path, match_radius_px=80):
        self.csv_path = Path(csv_path)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.csv_path.open('w', encoding='utf-8-sig', newline='')
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self.rows = []
        self.frames = 0
        self.previous = []
        self.match_radius_px = match_radius_px
        self.matched = {color: 0 for color in COLORS}
        self.consistent = {color: 0 for color in COLORS}

    def record(self, timestamp, frame_number, detections):
        self.frames += 1
        current = list(detections)
        used = set()
        for detection in current:
            candidates = [(float(np.hypot(detection.u-old.u, detection.v-old.v)), index, old)
                          for index, old in enumerate(self.previous) if index not in used]
            if candidates:
                distance, index, old = min(candidates, key=lambda item: item[0])
                if distance <= self.match_radius_px:
                    used.add(index)
                    self.matched[detection.color] += 1
                    self.consistent[detection.color] += int(detection.color == old.color)
            row = {'timestamp': timestamp, 'frame_number': frame_number,
                   'color': detection.color, 'detected': 1, 'u': detection.u, 'v': detection.v,
                   'yaw_deg': detection.yaw_deg, 'confidence': detection.confidence}
            self._writer.writerow(row)
            self.rows.append(row)
        for color in COLORS:
            if not any(d.color == color for d in current):
                row = {'timestamp': timestamp, 'frame_number': frame_number, 'color': color,
                       'detected': 0, 'u': '', 'v': '', 'yaw_deg': '', 'confidence': ''}
                self._writer.writerow(row)
                self.rows.append(row)
        self.previous = current
        self._file.flush()

    def summarize(self):
        colors = {}
        for color in COLORS:
            seen = [r for r in self.rows if r['color'] == color and r['detected']]
            unique_frames = len({r['frame_number'] for r in seen})
            colors[color] = {
                'detection_rate': unique_frames/self.frames if self.frames else 0,
                'detected_frames': unique_frames,
                'color_consistency': self.consistent[color]/self.matched[color]
                                     if self.matched[color] else None,
                'matched_frame_pairs': self.matched[color],
                'u_px': linear_stats([r['u'] for r in seen]),
                'v_px': linear_stats([r['v'] for r in seen]),
                'yaw': orientation_stats([r['yaw_deg'] for r in seen]),
            }
        return {'coordinate_mode': 'IMAGE_PIXEL', 'frames': self.frames, 'colors': colors,
                'color_consistency_definition':
                'Fraction of nearest spatial matches in consecutive frames retaining the same color label; null when no matches.'}

    def stop(self):
        if not self._file.closed:
            self._file.close()
        path = self.csv_path.with_name(self.csv_path.stem + '_summary.json')
        summary = self.summarize()
        path.write_text(json.dumps(summary, indent=2), encoding='utf-8')
        return path, summary
