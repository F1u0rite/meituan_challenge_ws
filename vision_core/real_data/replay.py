"""Independent session-level replay with one-to-one mask IoU assignment."""
from pathlib import Path
import json
import shutil
import cv2
import numpy as np
from perception.battery_only import PixelBatteryDetector
from synthetic.io import write_png
from .annotation import read_annotation, read_mask
from .dataset import iter_samples, iter_sessions, load_raw

COLORS = ('red', 'yellow', 'blue', 'green')


def contour_mask(shape, contour):
    mask = np.zeros(shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 1, -1)
    return mask.astype(bool)


def mask_iou(a, b):
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum()/union) if union else 0.0


def yaw_error_180(predicted, measured):
    return abs((float(predicted)-float(measured)+90) % 180-90)


def mask_center(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        raise ValueError('Ground-truth object mask is empty')
    return float(xs.mean()), float(ys.mean())


def match_one_to_one(ground_truth_masks, predictions, shape, threshold=0.3):
    """Global greedy assignment by descending IoU, independent of predicted color."""
    predicted_masks = [contour_mask(shape, item.contour) for item in predictions]
    pairs = [(mask_iou(gt, pred), gi, pi)
             for gi, gt in enumerate(ground_truth_masks)
             for pi, pred in enumerate(predicted_masks)]
    matched, used_gt, used_pred = [], set(), set()
    for iou, gi, pi in sorted(pairs, reverse=True):
        if iou < threshold:
            break
        if gi not in used_gt and pi not in used_pred:
            matched.append((gi, pi, iou))
            used_gt.add(gi); used_pred.add(pi)
    return matched, set(range(len(ground_truth_masks)))-used_gt, set(range(len(predictions)))-used_pred


def overlays(image, objects, masks, predictions):
    gt_image, pred_image = image.copy(), image.copy()
    for item, mask in zip(objects, masks):
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(gt_image, contours, -1, (0, 255, 255), 2)
        if contours:
            x, y, _, _ = cv2.boundingRect(max(contours, key=cv2.contourArea))
            cv2.putText(gt_image, item['color'], (x, max(y-8, 20)), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 0), 2)
    for item in predictions:
        cv2.drawContours(pred_image, [item.contour], -1, (0, 0, 255), 2)
        cv2.putText(pred_image, f'{item.color} {item.confidence:.2f}',
                    (round(item.u), max(round(item.v)-10, 20)), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 2)
    return gt_image, pred_image


def _save_failure(root, sample, image, objects, masks, predictions, reasons, matches):
    folder = Path(root)/sample.parent.parent.name/sample.name
    folder.mkdir(parents=True, exist_ok=True)
    shutil.copy2(sample/'raw.png', folder/'raw.png')
    gt_image, pred_image = overlays(image, objects, masks, predictions)
    write_png(folder/'ground_truth_overlay.png', gt_image)
    write_png(folder/'prediction_overlay.png', pred_image)
    (folder/'metadata.json').write_text(json.dumps({
        'sample_id': sample.name, 'failure_reasons': sorted(set(reasons)),
        'matches': [{'ground_truth_index': g, 'prediction_index': p, 'iou': iou} for g,p,iou in matches],
        'predictions': [{'color': d.color, 'u': d.u, 'v': d.v, 'yaw_deg': d.yaw_deg,
                         'confidence': d.confidence} for d in predictions]}, indent=2), encoding='utf-8')
    return pred_image


def _contact_sheet(images, path):
    if not images:
        return
    thumbs = []
    for image in images:
        scale = min(320/image.shape[1], 240/image.shape[0])
        thumb = cv2.resize(image, (round(image.shape[1]*scale), round(image.shape[0]*scale)))
        tile = np.full((250, 330, 3), 240, np.uint8)
        tile[:thumb.shape[0], :thumb.shape[1]] = thumb
        thumbs.append(tile)
    rows = []
    for index in range(0, len(thumbs), 3):
        row = thumbs[index:index+3]
        while len(row) < 3:
            row.append(np.full_like(thumbs[0], 240))
        rows.append(np.hstack(row))
    write_png(path, np.vstack(rows))


def run_replay(data_root, output_dir, color_config, *, iou_threshold=.3, low_confidence=.5,
               min_validation_sessions=2, min_annotated_objects=20):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    config = Path(color_config)
    import yaml
    provenance = yaml.safe_load(config.read_text(encoding='utf-8')).get('provenance', {})
    calibration_ids = {info['session_id'] for _, info in iter_sessions(data_root, 'calibration')}
    source_ids = set(provenance.get('session_ids', []))
    validation_ids = {info['session_id'] for _, info in iter_sessions(data_root, 'validation')}
    if calibration_ids & validation_ids or source_ids & validation_ids:
        raise ValueError('Calibration and validation session overlap')
    if source_ids and not source_ids <= calibration_ids:
        raise ValueError('Suggested config source sessions are not assigned to calibration')
    detector = PixelBatteryDetector(config)
    counts = {c: {'tp': 0, 'fp': 0, 'fn': 0, 'confidences': []} for c in COLORS}
    geometry = {c: {'iou': [], 'center_error_px': [], 'yaw_error_deg': []} for c in COLORS}
    confusion = {actual: {predicted: 0 for predicted in (*COLORS, 'missed')} for actual in COLORS}
    confusion['background'] = {predicted: 0 for predicted in (*COLORS, 'missed')}
    reviewed_images = positive_objects = 0
    failures = []
    contact_images = []
    for sample, session in iter_samples(data_root, 'validation'):
        annotation = read_annotation(sample)
        if annotation is None:
            continue
        image = load_raw(sample)
        objects = annotation['objects']
        masks = [read_mask(sample, item) for item in objects]
        predictions = detector.detect(image)
        matches, unmatched_gt, unmatched_pred = match_one_to_one(masks, predictions, image.shape, iou_threshold)
        reviewed_images += 1
        positive_objects += len(objects)
        reasons = []
        for gi, pi, _iou in matches:
            true, predicted = objects[gi]['color'], predictions[pi].color
            confusion[true][predicted] += 1
            cx, cy = mask_center(masks[gi])
            geometry[true]['iou'].append(float(_iou))
            geometry[true]['center_error_px'].append(float(np.hypot(predictions[pi].u-cx,
                                                                  predictions[pi].v-cy)))
            if objects[gi].get('yaw_gt_deg') is not None:
                yaw_error=yaw_error_180(predictions[pi].yaw_deg,objects[gi]['yaw_gt_deg'])
                geometry[true]['yaw_error_deg'].append(yaw_error)
                if yaw_error > 15:
                    reasons.append('yaw_failure')
            if true == predicted:
                counts[true]['tp'] += 1
            else:
                counts[true]['fn'] += 1
                counts[predicted]['fp'] += 1
                reasons.append('wrong_color')
        for gi in unmatched_gt:
            color = objects[gi]['color']
            counts[color]['fn'] += 1
            confusion[color]['missed'] += 1
            reasons.extend(('false_negative','missed_detection'))
        for pi in unmatched_pred:
            color = predictions[pi].color
            counts[color]['fp'] += 1
            confusion['background'][color] += 1
            reasons.append('false_positive')
            if any(np.logical_and(contour_mask(image.shape,predictions[pi].contour),mask).any()
                   for mask in masks):
                reasons.append('bad_contour')
        for item in predictions:
            counts[item.color]['confidences'].append(item.confidence)
            if item.confidence < low_confidence:
                reasons.append('low_confidence')
        if reasons:
            contact_images.append(_save_failure(output_dir/'failures', sample, image, objects,
                                                masks, predictions, reasons, matches))
            failures.append({'sample_id': sample.name, 'session_id': session['session_id'],
                             'reasons': sorted(set(reasons))})
    sufficient = (calibration_ids and len(validation_ids) >= min_validation_sessions and
                  positive_objects >= min_annotated_objects)
    status = 'VALIDATION RUN' if sufficient else 'INSUFFICIENT DATA'
    metrics = {}
    for color, count in counts.items():
        tp, fp, fn = count['tp'], count['fp'], count['fn']
        g=geometry[color]
        def distribution(values):
            return {'mean':float(np.mean(values)),'p95':float(np.percentile(values,95))} if values else None
        metrics[color] = {**count,
                          'precision': tp/(tp+fp) if status == 'VALIDATION RUN' and tp+fp else None,
                          'recall': tp/(tp+fn) if status == 'VALIDATION RUN' and tp+fn else None,
                          'iou':distribution(g['iou']) if status == 'VALIDATION RUN' else None,
                          'center_error_px':distribution(g['center_error_px']) if status == 'VALIDATION RUN' else None,
                          'yaw_error_deg':distribution(g['yaw_error_deg']) if status == 'VALIDATION RUN' else None}
    report = {'status': status, 'calibration_sessions': sorted(calibration_ids),
              'validation_sessions': sorted(validation_ids), 'reviewed_images': reviewed_images,
              'annotated_objects': positive_objects, 'iou_threshold': iou_threshold,
              'sufficiency_rule': {'min_validation_sessions': min_validation_sessions,
                                   'min_annotated_objects': min_annotated_objects},
              'matching': 'one-to-one global greedy descending mask IoU, irrespective of color',
              'metrics': metrics, 'confusion_matrix': confusion, 'false_positive_count': sum(v['fp'] for v in counts.values()),
              'failures': failures,
              'yaw_accuracy': 'MEASURED' if status=='VALIDATION RUN' and any(
                  geometry[c]['yaw_error_deg'] for c in COLORS) else 'NOT MEASURED: no reliable yaw ground truth'}
    (output_dir/'replay_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    _contact_sheet(contact_images, output_dir/'failure_contact_sheet.png')
    return report
