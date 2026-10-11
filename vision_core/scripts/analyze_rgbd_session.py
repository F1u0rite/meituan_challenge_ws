"""Replay one recorded session without opening RealSense or changing raw data."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from real_data.rgbd_recording import load_rgbd_sample, save_ply
from perception.unified_detector import UnifiedBatteryDetector
from perception.rgbd_offline import OfflineConfig, analyze_frame


def _write_png(path, image):
    ok, buf = cv2.imencode('.png', image)
    if not ok:
        raise IOError(path)
    buf.tofile(str(path))


def _panel(image, title, size=(640, 360)):
    out = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    cv2.rectangle(out, (0, 0), (size[0], 32), (25, 25, 25), -1)
    cv2.putText(out, title, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, .62, (255, 255, 255), 2)
    return out


def _cloud_view(points, top_count, geometry, size=(640, 360)):
    out = np.full((size[1], size[0], 3), 18, np.uint8)
    if not len(points):
        return out
    # Isometric camera-frame preview. The PLY retains full metric coordinates.
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    horiz = x-y
    vert = .4*(x+y)-z
    h_min, h_span = float(horiz.min()), max(float(np.ptp(horiz)), 1e-9)
    v_min, v_span = float(vert.min()), max(float(np.ptp(vert)), 1e-9)
    def project(xyz):
        h = xyz[0]-xyz[1]
        v = .4*(xyz[0]+xyz[1])-xyz[2]
        return (int((h-h_min)/h_span*(size[0]-50)+25),
                int((v-v_min)/v_span*(size[1]-50)+25))
    uv = np.asarray([project(p) for p in points])
    ix = np.linspace(0, len(points)-1, min(7000, len(points)), dtype=int)
    for i in ix:
        px, py = uv[i]
        out[np.clip(py, 0, size[1]-1), np.clip(px, 0, size[0]-1)] = (
            (30, 210, 80) if z[i] <= np.percentile(z, 25) else (230, 160, 35))
    centre = np.median(points, axis=0)
    for key, color in [('pca_axis_camera_xyz', (255, 0, 255)),
                       ('obb_axis_camera_xyz', (0, 255, 255))]:
        axis = geometry.get(key)
        if axis is not None:
            axis = np.asarray(axis)
            cv2.arrowedLine(out, project(centre), project(centre+.04*axis), color, 2,
                            tipLength=.15)
    cv2.putText(out, 'Isometric XYZ (m) | green=near depth', (8, size[1]-12),
                cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1)
    return out


def render_diagnostic(frame, report, maps, points):
    color = frame.color_bgr
    raw = color.copy()
    obj = maps['object']
    contours, _ = cv2.findContours(np.uint8(obj), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(raw, contours, -1, (0, 255, 255), 2)
    depth = frame.aligned_depth.astype(np.float32)*frame.depth_scale
    depth_view = cv2.applyColorMap(np.uint8(np.clip((depth-.15)/.45, 0, 1)*255), cv2.COLORMAP_TURBO)
    depth_view[frame.aligned_depth == 0] = 0
    table = np.zeros_like(color)
    table[maps['table_roi']] = (100, 100, 100)
    plane = report['table_plane']
    if plane:
        inliers = maps['table_roi'] & (np.abs(maps['height_m']) <= .006)
        table[inliers] = (0, 180, 0)
        table[obj] = (0, 0, 220)
    foreground = np.zeros_like(color)
    foreground[maps['foreground']] = (0, 170, 200)
    foreground[obj] = (0, 0, 255)
    bands = np.zeros_like(color)
    bands[obj] = (230, 160, 35)
    bands[maps['top_band']] = (30, 210, 80)
    geom = report['body_geometry']
    cloud = _cloud_view(points, geom.get('near_camera_band_point_count', 0), geom)
    before = sum(report['candidate_counts_before'].values())
    after = sum(report['candidate_counts_after'].values())
    titles = [('Raw RGB + extracted red', raw), ('Aligned Z16 view', depth_view),
              ('Table ROI / plane inliers', table),
              (f'Foreground / candidate {before}->{after}', foreground),
              ('Near-depth band / remainder', bands), ('Battery cloud + axes candidate', cloud)]
    panels = [_panel(im, title) for title, im in titles]
    return np.vstack([np.hstack(panels[:3]), np.hstack(panels[3:])])


def _audit_sample(frame, metadata, report):
    depth = frame.aligned_depth
    scale = frame.depth_scale
    z = depth.astype(np.float64)*scale
    valid = depth > 0
    object_z = z[report.get('_object_mask', np.zeros(valid.shape, bool))]
    return {'original_depth_profile': metadata['stream_profiles']['depth'],
            'original_color_profile': metadata['stream_profiles']['color'],
            'aligned_depth_shape': list(depth.shape), 'depth_dtype': str(depth.dtype),
            'depth_scale_m_per_unit': scale,
            'intrinsics': asdict(frame.intrinsics),
            'color_depth_timestamp_delta_ms': abs(frame.timestamps['color_sdk_ms']-frame.timestamps['depth_sdk_ms']),
            'disparity_shift': 'UNAVAILABLE_IN_RECORDED_METADATA',
            'depth_preset': 'UNAVAILABLE_IN_RECORDED_METADATA',
            'postprocessing_settings': 'UNAVAILABLE_IN_RECORDED_METADATA',
            'whole_frame_zero_fraction': float(np.mean(~valid)),
            'whole_frame_nonzero_below_175mm_fraction': float(np.mean(valid & (z < .175))),
            'extracted_object_below_200mm_fraction': float(np.mean(object_z < .2)) if len(object_z) else None,
            'whole_frame_depth_p05_p50_p95_m': np.percentile(z[valid], [5, 50, 95]).tolist() if valid.any() else None,
            'alignment_assessment': 'SIZE_AND_VISUAL_CORRESPONDENCE_ONLY; PIXEL_ACCURACY_NOT_CALIBRATED',
            'table_fit_status': report['table_failure'] or report['table_plane']['status']}


def run(session, output, cfg=OfflineConfig()):
    samples = sorted(Path(session).glob('sample_*'))
    if not samples:
        raise ValueError('No recorded samples')
    output = Path(output)
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite previous analysis: {output}')
    output.mkdir(parents=True)
    detector = UnifiedBatteryDetector()
    rows, tiles = [], []
    for sample in samples:
        frame = load_rgbd_sample(sample)
        metadata = json.loads((sample/'metadata.json').read_text(encoding='utf-8'))
        detections = detector.detect_batteries(frame.color_bgr)
        report, maps, points, rgb = analyze_frame(frame, detections, cfg)
        report['_object_mask'] = maps['object']
        report['sample_id'] = sample.name
        report['data_source'] = str(sample)
        report['audit'] = _audit_sample(frame, metadata, report)
        del report['_object_mask']
        report['failure_reasons'] = [c['reason'] for c in report['candidates'] if c['reason']]
        sub = output/sample.name
        sub.mkdir()
        (sub/'diagnostic.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
        if len(points):
            save_ply(sub/'battery_points.ply', points, rgb)
        diag = render_diagnostic(frame, report, maps, points)
        _write_png(sub/'diagnostic.png', diag)
        tile = cv2.resize(diag, (768, 288), interpolation=cv2.INTER_AREA)
        cv2.putText(tile, sample.name, (10, 280), cv2.FONT_HERSHEY_SIMPLEX,
                    .58, (255, 255, 255), 2)
        tiles.append(tile)
        rows.append(report)
        print(sample.name, 'plane', report['table_failure'] or 'candidate',
              'points', len(points), 'candidates',
              report['candidate_counts_before'], '->', report['candidate_counts_after'], flush=True)
    cols = 2
    if len(tiles) % cols:
        tiles.append(np.full_like(tiles[0], 20))
    sheet = np.vstack([np.hstack(tiles[i:i+cols]) for i in range(0, len(tiles), cols)])
    _write_png(output/'contact_sheet.png', sheet)
    before, after, reasons = Counter(), Counter(), Counter()
    reasons_by_color = {}
    for row in rows:
        before.update(row['candidate_counts_before'])
        after.update(row['candidate_counts_after'])
        reasons.update(row['failure_reasons'])
        for candidate in row['candidates']:
            if candidate['reason']:
                reasons_by_color.setdefault(candidate['color'], Counter()).update([candidate['reason']])
    good = [r for r in rows if r['table_plane'] is not None and r['object_point_count'] > 0]
    planes = [r['table_plane'] for r in rows if r['table_plane'] is not None]
    top_gaps = [r['body_geometry']['near_band_pca_axis_difference_deg'] for r in good
                if r['body_geometry']['near_band_pca_axis_difference_deg'] is not None]
    summary = {'session': str(session), 'sample_count': len(samples),
               'table_candidate_count': len(planes), 'battery_cloud_count': len(good),
               'candidate_counts_before': dict(before), 'candidate_counts_after': dict(after),
               'failure_reasons': dict(reasons),
               'failure_reasons_by_color': {k: dict(v) for k, v in reasons_by_color.items()},
               'table_inlier_ratio_median': float(np.median([p['inlier_ratio'] for p in planes])) if planes else None,
               'table_residual_m_median': float(np.median([p['mean_residual_m'] for p in planes])) if planes else None,
               'table_normal_camera_xyz_median': np.median([p['normal_camera_xyz'] for p in planes], axis=0).tolist() if planes else None,
               'object_point_count_median': float(np.median([r['object_point_count'] for r in good])) if good else None,
               'depth_growth_accepted_count': sum(r.get('depth_growth', {}).get('accepted', False) for r in rows),
               'frames_joining_disconnected_components': sum(r.get('depth_growth', {}).get('joined_disconnected_components', 0) > 0 for r in rows),
               'max_depth_growth_ratio': max(r.get('depth_growth', {}).get('growth_ratio') or 0 for r in rows),
               'visible_surface_center_m_median': np.median([r['body_geometry']['visible_surface_center_m'] for r in good], axis=0).tolist() if good else None,
               'visible_surface_center_m_min': np.min([r['body_geometry']['visible_surface_center_m'] for r in good], axis=0).tolist() if good else None,
               'visible_surface_center_m_max': np.max([r['body_geometry']['visible_surface_center_m'] for r in good], axis=0).tolist() if good else None,
               'pca_obb_axis_difference_deg_median': float(np.median([r['body_geometry']['pca_obb_axis_difference_deg'] for r in good])) if good else None,
               'near_band_pca_difference_over_70deg_count': sum(x > 70 for x in top_gaps),
               'whole_frame_zero_fraction_median': float(np.median([r['audit']['whole_frame_zero_fraction'] for r in rows])),
               'frames_with_majority_extracted_points_below_200mm': sum(r['audit']['extracted_object_below_200mm_fraction'] > .5 for r in rows),
               'original_depth_profile': rows[0]['audit']['original_depth_profile'],
               'depth_scale_m_per_unit': rows[0]['depth_scale_m_per_unit'],
               'settings_not_recorded': ['disparity_shift', 'depth_preset', 'postprocessing_settings'],
               'intrinsics_consistent': len({json.dumps(r['camera_intrinsics'], sort_keys=True) for r in rows}) == 1,
               'stream_profiles_consistent': len({json.dumps(r['audit']['original_depth_profile'], sort_keys=True) for r in rows}) == 1,
               'depth_scale_consistent': len({r['depth_scale_m_per_unit'] for r in rows}) == 1,
               'body_pose_valid': False, 'ring_pose_valid': False,
               'position_accuracy': 'NOT_MEASURED', 'yaw_accuracy': 'NOT_MEASURED',
               'config': asdict(cfg)}
    (output/'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
    report_text = [
        '# RGB-D 红色正俯视离线诊断', '',
        f'- 样本：{len(samples)}；桌面候选平面：{len(planes)}；红色点云：{len(good)}。',
        f'- 原始深度流：{summary["original_depth_profile"]}；对齐图：1280×720 Z16。',
        f'- 颜色候选：{dict(before)} → 几何过滤后：{dict(after)}。',
        f'- 分颜色淘汰原因：{summary["failure_reasons_by_color"]}。',
        f'- 平面内点率中位数：{summary["table_inlier_ratio_median"]}；拟合残差中位数：{summary["table_residual_m_median"]} m。',
        '- 这些是对当前采集场景的候选拟合指标，不代表深度测量精度或独立验证准确率。',
        '- 640×480 的 D435i 标称 Min-Z 约 0.175 m；当前部分可见面仅约 0.18–0.21 m，接近边界。',
        '- Min-Z 来源：https://www.realsenseai.com/download/21345/ （D400 Series Product Family Datasheet）。',
        '- Disparity Shift、Depth Preset 与后处理开关未记录；实际 Min-Z 与绝对误差无法从文件确定。',
        '- 对齐目前只验证了尺寸、时间戳与可视对应，未用标定靶验证逐像素误差。',
        '- near-depth band 是顶部假设；剩余点不等同可靠侧面分割。模型主体中心不具抓取有效性。',
        f'- 深度前景扩展：{summary["depth_growth_accepted_count"]}/{len(rows)} 帧通过；{summary["frames_joining_disconnected_components"]} 帧拼接了分离表面，属有界几何假设。',
        f'- PCA 与 OBB 轴差中位数：{summary["pca_obb_axis_difference_deg_median"]:.1f}°；最近深度带与 PCA 轴差超过 70°：{summary["near_band_pca_difference_over_70deg_count"]}/{len(good)} 帧。',
        '- 相机是否移动未得到独立记录，帧间 XYZ 变化不解释为定位误差。',
        '- 3D position accuracy = NOT MEASURED；Yaw accuracy = NOT MEASURED。',
        '', '## 逐帧结果', '',
        '| 样本 | 平面 | 红点云 | 候选前→后 | 可见 Z (m) |',
        '|---|---:|---:|---:|---:|']
    for row in rows:
        z = row['body_geometry'].get('visible_surface_center_m')
        report_text.append(f'| {row["sample_id"]} | {"OK" if row["table_plane"] else row["table_failure"]} | '
                           f'{row.get("object_point_count", 0)} | '
                           f'{sum(row["candidate_counts_before"].values())}→{sum(row["candidate_counts_after"].values())} | '
                           f'{z[2]:.3f} |' if z else f'| {row["sample_id"]} | failed | 0 | - | - |')
    (output/'diagnostic_report.md').write_text('\n'.join(report_text)+'\n', encoding='utf-8')
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, default=ROOT/'data'/'rgbd'/'session_20261011_143230_red_top')
    parser.add_argument('--output', type=Path, default=ROOT/'outputs'/'rgbd_red_top_offline_v1')
    args = parser.parse_args()
    print(json.dumps(run(args.session, args.output), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
