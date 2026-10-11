"""Session-isolated HSV summaries and provisional threshold suggestions."""
from datetime import datetime, timezone
from pathlib import Path
import csv
import json
import cv2
import numpy as np
import yaml
from .annotation import read_annotation, read_mask
from .dataset import iter_samples, load_raw

COLORS = ('red', 'yellow', 'blue', 'green')


def hue_arc(hues, percentile=5):
    """Small circular percentile arc on OpenCV's 180-step hue wheel."""
    values = np.asarray(hues, dtype=float)
    if values.size == 0:
        raise ValueError('Empty hue sample')
    angles = values*2*np.pi/180
    center = (np.arctan2(np.sin(angles).mean(), np.cos(angles).mean()) * 180/(2*np.pi)) % 180
    unwrapped = ((values-center+90) % 180)-90
    low = int(np.floor((center+np.percentile(unwrapped, percentile)) % 180))
    high = int(np.ceil((center+np.percentile(unwrapped, 100-percentile)) % 180)) % 180
    if low <= high:
        return [(low, high)]
    return [(0, high), (low, 179)]


def describe(values):
    if len(values) == 0:
        return None
    arr = np.asarray(values)
    return {name: float(value) for name, value in zip(
        ('min', 'max', 'median', 'p01', 'p05', 'p10', 'p90', 'p95', 'p99'),
        (arr.min(), arr.max(), np.median(arr), *[np.percentile(arr, q) for q in (1, 5, 10, 90, 95, 99)]))}


def _sample_pixels(hsv, mask, cap=30000):
    pixels = hsv[mask]
    if len(pixels) > cap:
        pixels = pixels[np.linspace(0, len(pixels)-1, cap, dtype=int)]
    return pixels


def calibration_pixels(root):
    by_color = {color: [] for color in COLORS}
    negatives = []
    sessions = set()
    counts = {color: {'objects': 0, 'images': set(), 'pixels': 0} for color in COLORS}
    for sample, session in iter_samples(root, 'calibration'):
        annotation = read_annotation(sample)
        if annotation is None:
            continue
        metadata = json.loads((sample/'metadata.json').read_text(encoding='utf-8'))
        if metadata['image_quality']['status'] == 'REJECTED':
            continue
        hsv = cv2.cvtColor(load_raw(sample), cv2.COLOR_BGR2HSV)
        sessions.add(session['session_id'])
        if session['capture_label'] == 'negative' and not annotation['objects']:
            negatives.append(_sample_pixels(hsv, np.ones(hsv.shape[:2], bool)))
        for item in annotation['objects']:
            color = item['color']
            pixels = _sample_pixels(hsv, read_mask(sample, item))
            if len(pixels):
                by_color[color].append(pixels)
                counts[color]['objects'] += 1
                counts[color]['images'].add(sample.name)
                counts[color]['pixels'] += len(pixels)
    return by_color, negatives, counts, sorted(sessions)


def _within(hsv, ranges):
    found = np.zeros(len(hsv), bool)
    for h0, s0, v0, h1, s1, v1 in ranges:
        found |= ((hsv[:, 0] >= h0) & (hsv[:, 0] <= h1) &
                  (hsv[:, 1] >= s0) & (hsv[:, 1] <= s1) &
                  (hsv[:, 2] >= v0) & (hsv[:, 2] <= v1))
    return found


def suggest_ranges(positive, other):
    """Choose a quantile candidate by pixel F1 against other colors/background."""
    if len(positive) == 0:
        return None, None
    best = None
    for trim in (1, 5, 10):
        arcs = hue_arc(positive[:, 0], trim)
        s0 = int(np.floor(np.percentile(positive[:, 1], trim)))
        v0 = int(np.floor(np.percentile(positive[:, 2], trim)))
        ranges = [[h0, s0, v0, h1, 255, 255] for h0, h1 in arcs]
        tp = int(_within(positive, ranges).sum())
        fp = int(_within(other, ranges).sum()) if len(other) else 0
        fn = len(positive)-tp
        f1 = 2*tp/max(2*tp+fp+fn, 1)
        candidate = (f1, -fp, ranges, {'trim_percentile': trim, 'calibration_pixel_f1': f1,
                                       'positive_covered': tp, 'other_pixels_accepted': fp,
                                       'other_pixel_count': len(other)})
        if best is None or candidate[:2] > best[:2]:
            best = candidate
    return best[2], best[3]


def _histogram_image(values, bins, title):
    canvas = np.full((270, 550, 3), 255, np.uint8)
    hist = np.bincount(values.astype(np.uint8), minlength=bins).astype(float)[:bins]
    if hist.max() > 0:
        hist /= hist.max()
    for index, fraction in enumerate(hist):
        x = 20 + round(index*510/bins)
        cv2.line(canvas, (x, 235), (x, 235-round(fraction*195)), (90, 75, 35), 2)
    cv2.putText(canvas, title, (20, 25), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 0), 2)
    return canvas


def run_calibration(root, output_dir, base_config):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    by_color, negatives, counts, sessions = calibration_pixels(root)
    arrays = {color: np.concatenate(items) if items else np.empty((0, 3), np.uint8)
              for color, items in by_color.items()}
    background = np.concatenate(negatives) if negatives else np.empty((0, 3), np.uint8)
    report = {'status': 'PROVISIONAL' if any(len(a) for a in arrays.values()) else 'INSUFFICIENT DATA',
              'source_dataset': str(Path(root).resolve()), 'session_ids': sessions,
              'generated_at': datetime.now(timezone.utc).isoformat(), 'colors': {}}
    base = yaml.safe_load(Path(base_config).read_text(encoding='utf-8'))
    suggested = dict(base)
    suggested['hsv_ranges'] = dict(base['hsv_ranges'])
    for color, pixels in arrays.items():
        if len(pixels) == 0:
            report['colors'][color] = {'status': 'NOT MEASURED', 'sample_count': 0}
            continue
        others = [array for name, array in arrays.items() if name != color and len(array)]
        if len(background):
            others.append(background)
        other = np.concatenate(others) if others else np.empty((0, 3), np.uint8)
        ranges, score = suggest_ranges(pixels, other)
        suggested['hsv_ranges'][color] = ranges
        report['colors'][color] = {'status': 'PROVISIONAL',
                                   'sample_count': counts[color]['objects'],
                                   'image_count': len(counts[color]['images']),
                                   'pixel_count': counts[color]['pixels'],
                                   'hsv': {channel: describe(pixels[:, i]) for i, channel in enumerate('hsv')},
                                   'hue_circular_arc_p05_p95': hue_arc(pixels[:, 0], 5),
                                   'histograms': {channel: np.bincount(pixels[:, i], minlength=bins)[:bins].tolist()
                                                  for i, (channel, bins) in enumerate((('h', 180), ('s', 256), ('v', 256)))},
                                   'suggested_ranges': ranges, 'calibration_pixel_screen': score}
        for i, (channel, bins) in enumerate((('h', 180), ('s', 256), ('v', 256))):
            from synthetic.io import write_png
            write_png(output_dir/f'{color}_{channel}_hist.png', _histogram_image(pixels[:, i], bins, f'{color.upper()} {channel.upper()}'))
    suggested['provenance'] = {'config_version': report['generated_at'],
                               'status': 'provisional', 'validation_status': 'NOT RUN',
                               'source_dataset': report['source_dataset'], 'session_ids': sessions,
                               'sample_counts': {color: counts[color]['objects'] for color in COLORS},
                               'method': 'circular hue percentile candidates scored against other annotated colors and reviewed negative background'}
    (output_dir/'hsv_statistics.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (output_dir/'hsv_statistics.csv').open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        writer.writerow(('color', 'channel', 'min', 'max', 'median', 'p01', 'p05', 'p10', 'p90', 'p95', 'p99', 'pixel_count'))
        for color, item in report['colors'].items():
            if item['status'] == 'NOT MEASURED':
                continue
            for channel, stats in item['hsv'].items():
                writer.writerow((color, channel, *[stats[key] for key in ('min','max','median','p01','p05','p10','p90','p95','p99')], item['pixel_count']))
    (output_dir/'suggested_realsense_color.yaml').write_text(yaml.safe_dump(suggested, sort_keys=False, allow_unicode=True), encoding='utf-8')
    return report
