"""Human-triggered RGB-D preview. Importing or --help never opens the camera."""
import argparse
import json
import sys
from pathlib import Path
from time import perf_counter
from datetime import datetime
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
os.environ.setdefault("OMP_NUM_THREADS","1")
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from input.realsense_rgbd_source import RealSenseRGBDSource
from perception.unified_detector import UnifiedBatteryDetector
from perception.rgbd_geometry import analyze_rgbd
from real_data.rgbd_recording import save_rgbd_sample


def _diagnose(frame, detector, source, min_depth, max_depth):
    start = perf_counter()
    detections = detector.detect_batteries(frame.color_bgr)
    red = [d for d in detections if d.color == 'red' and d.detection_valid]
    chosen = max(red, key=lambda d: d.mask.sum()) if red else None
    mask = chosen.mask if chosen else np.zeros(frame.aligned_depth.shape, bool)
    geometry, points, rgb = analyze_rgbd(frame.color_bgr, frame.aligned_depth, mask,
        frame.intrinsics, frame.depth_scale, min_depth_m=min_depth,
        max_depth_m=max_depth, sdk_deproject=source.deproject)
    geometry['color'] = chosen.color if chosen else None
    geometry['rgb_confidence'] = chosen.confidence if chosen else None
    geometry['image_yaw_deg'] = chosen.yaw_deg if chosen else None
    geometry['candidate_count'] = len(detections)
    geometry['processing_ms'] = (perf_counter()-start)*1000
    depth_m = frame.aligned_depth.astype(np.float32)*frame.depth_scale
    display_depth = np.clip((depth_m-min_depth)/(max_depth-min_depth), 0, 1)
    depth_view = cv2.applyColorMap(np.uint8(display_depth*255), cv2.COLORMAP_TURBO)
    depth_view[frame.aligned_depth == 0] = 0
    valid_overlay = mask & (frame.aligned_depth > 0)
    if valid_overlay.any():
        depth_view[valid_overlay] = cv2.addWeighted(
            depth_view[valid_overlay], .4,
            np.full_like(depth_view[valid_overlay], (0, 255, 0)), .6, 0)
    color_view = frame.color_bgr.copy()
    if mask.any():
        color_view[mask] = cv2.addWeighted(color_view[mask], .55,
                                          np.full_like(color_view[mask], (0, 0, 255)), .45, 0)
    if chosen:
        cv2.drawContours(color_view, [chosen.contour], -1, (0, 255, 255), 2)
        cv2.circle(color_view, tuple(map(int, chosen.pixel_center)), 5, (255, 255, 255), -1)
    status = f"RED | depth {geometry['depth_valid_ratio']:.1%} | points {geometry['point_count']} | {geometry['processing_ms']:.0f} ms"
    cv2.putText(color_view, status, (12, 35), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
    cv2.putText(color_view, 'BODY INVALID  RING INVALID  s=save q=quit',
                (12, 65), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
    xyz = geometry['visible_surface_position_m']
    xyz_text = ('visible XYZ m: '+', '.join(f'{x:.3f}' for x in xyz)) if xyz else 'visible XYZ: INVALID'
    cv2.putText(color_view, xyz_text, (12, 95), cv2.FONT_HERSHEY_SIMPLEX,
                .55, (255, 255, 255), 2)
    cv2.putText(depth_view, 'Aligned raw Z16 (visualized only)', (12, 35),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
    plane = geometry['plane_candidate']
    plane_text = (f'Plane candidate residual {plane["mean_residual_m"]:.4f} m'
                  if plane else 'Plane candidate: INVALID')
    cv2.putText(depth_view, plane_text, (12, 65), cv2.FONT_HERSHEY_SIMPLEX,
                .55, (255, 255, 255), 2)
    visualization = np.hstack((color_view, depth_view))
    return geometry, points, rgb, visualization


def main(argv=None):
    parser = argparse.ArgumentParser(description='D435i RGB-D preview; no automatic recording')
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--color-size', type=int, nargs=2, default=(1280, 720), metavar=('W', 'H'))
    parser.add_argument('--depth-size', type=int, nargs=2, default=(640, 480), metavar=('W', 'H'))
    parser.add_argument('--min-depth-m', type=float, default=.15)
    parser.add_argument('--max-depth-m', type=float, default=2.0)
    parser.add_argument('--scene', choices=('top', 'tilted', 'side'), default='top')
    parser.add_argument('--output', type=Path, default=ROOT/'data'/'rgbd')
    parser.add_argument('--tcp-send', action='store_true', help='Opt-in V1 loopback TCP diagnostics; requires dual-rate')
    parser.add_argument('--tcp-config', type=Path, default=ROOT/'config/tcp_sender.yaml')
    parser.add_argument('--dual-rate', action='store_true', help='Bounded capture, fast detection and slower CAD worker')
    parser.add_argument('--continuous-max-seconds', type=float, default=30.)
    parser.add_argument('--continuous-max-frames', type=int, default=450)
    parser.add_argument('--full-vision', action='store_true',
                        help='Existing RGB-D segmentation + CAD + tracking diagnostic preview')
    parser.add_argument('--replay', type=Path, help='Offline session; never starts a camera')
    parser.add_argument('--live', action='store_true', help='Native camera UI; selects existing full-vision dual-rate pipeline')
    parser.add_argument('--pair-manifest',type=Path,help='Approved original native NPY pairs; requires offline dual-rate replay')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--preview-authorized', action='store_true',
                        help='Only use after explicit human camera preparation confirmation')
    parser.add_argument('--vision-config', type=Path, default=ROOT/'config/realtime_rgbd.yaml')
    parser.add_argument('--run-output', '--replay-output', dest='replay_output', type=Path,
                        help='New results directory; must not exist (legacy replay alias retained)')
    parser.add_argument('--ui-smoke', action='store_true', help='Finite offline GUI check only')
    parser.add_argument('--max-frames', type=int, default=0)
    args = parser.parse_args(argv)
    if args.live:
        if args.replay or args.pair_manifest:
            parser.error('--live cannot be combined with offline replay inputs')
        args.full_vision = args.dual_rate = True
    if not args.replay and not args.preview_authorized:
        parser.error('Human confirmation required: 准备好了，启动预览')
    if args.pair_manifest and not (args.replay and args.dual_rate and args.full_vision):
        parser.error('--pair-manifest requires --replay --full-vision --dual-rate')
    if not args.headless and os.name!='nt' and not os.environ.get('DISPLAY'):
        parser.error('No DISPLAY: run in Ubuntu graphical desktop Terminal, or use --headless for actual offline rendering')
    if args.tcp_send and not (args.dual_rate and args.full_vision):
        parser.error('--tcp-send requires --full-vision --dual-rate')
    if args.ui_smoke and not args.replay:
        parser.error('--ui-smoke is offline only')
    if args.replay and not args.full_vision:
        parser.error('--replay requires --full-vision')
    if args.dual_rate and not args.full_vision:
        parser.error('--dual-rate requires --full-vision')
    if args.dual_rate and (args.continuous_max_seconds <= 0 or args.continuous_max_frames <= 0):
        parser.error('Continuous recording limits must be positive')
    if args.full_vision:
        if not args.replay and not args.preview_authorized:
            parser.error('Human confirmation required: 准备好了，启动预览')
        if args.dual_rate:
            from scripts.dual_rate_runner import run_dual_rate
            return run_dual_rate(args)
        return run_full_vision(args)
    if args.min_depth_m <= 0 or args.max_depth_m <= args.min_depth_m:
        parser.error('Invalid depth range')
    source = RealSenseRGBDSource(preferred_color=tuple(args.color_size),
                                 preferred_depth=tuple(args.depth_size), fps=args.fps)
    detector = UnifiedBatteryDetector()
    session = f"session_{datetime.now():%Y%m%d_%H%M%S}_red_{args.scene}"
    count = 0
    from input.rgbd_pair_guard import RGBDPairGuard
    gate=RGBDPairGuard();consecutive_bad=0
    try:
        profiles = source.start()
        print('Actual queried stream selection:', json.dumps(profiles, ensure_ascii=False))
        while True:
            frame = source.read()
            report=gate.inspect(frame.timestamps)
            if not report['qualified']:
                consecutive_bad+=1
                print('RGBD_REJECTED',report,flush=True)
                if consecutive_bad>=30:raise RuntimeError('No qualified RGB-D in 30 SDK observations')
                continue
            consecutive_bad=0
            geometry, points, rgb, visualization = _diagnose(frame, detector, source,
                args.min_depth_m, args.max_depth_m)
            cv2.imshow('RGB-D RED preview (s save, q quit)', visualization)
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                count += 1
                folder = args.output/session/f'sample_{count:04d}'
                save_rgbd_sample(folder, frame, geometry, points, rgb, visualization,
                                 scene_label=f'red_{args.scene}')
                print(f'Saved {folder}; depth valid {geometry["depth_valid_ratio"]:.1%}; '
                      f'visible surface valid {geometry["visible_surface_valid"]}; body pose invalid')
    finally:
        source.stop()
        cv2.destroyAllWindows()
    print(f'RGB-D samples saved: {count}')


def run_full_vision(args):
    """Shared core, stateful UI, user-controlled saves; no device calls in replay."""
    import hashlib,uuid,time
    from perception.realtime_rgbd import RGBDVisionPipeline
    from perception.vision_ui import VisionUI
    from real_data.rgbd_recording import load_rgbd_sample,save_vision_sample
    pipeline=RGBDVisionPipeline(args.vision_config);ui=VisionUI()
    source=None;rows=[];thumbs=[];hashes={};saved=0;last=time.perf_counter()
    recording_count=0;record_deadline=0.;window_called=False
    cad_events=[];rate_start=perf_counter()
    window='Meituan Battery RGB-D | OFFLINE' if args.replay else 'Meituan Battery RGB-D | LIVE UNVERIFIED'
    output=args.replay_output or ROOT/'outputs/live_rgbd_integration'/('run_'+uuid.uuid4().hex[:8])
    if output.exists():raise FileExistsError('Refuse overwriting results directory')
    output.mkdir(parents=True)
    if args.replay:
        folders=sorted(args.replay.glob('sample_*'))
        if args.max_frames:folders=folders[:args.max_frames]
        if not folders:raise ValueError('No replay samples')
        for folder in folders:
            for name in ('raw_color.png','aligned_depth.png','metadata.json','camera_intrinsics.json'):
                path=folder/name;hashes[str(path.resolve())]=hashlib.sha256(path.read_bytes()).hexdigest()
        frames=((folder.name,load_rgbd_sample(folder)) for folder in folders)
    else:
        source=RealSenseRGBDSource(preferred_color=tuple(args.color_size),preferred_depth=tuple(args.depth_size),fps=args.fps)
        from input.rgbd_pair_guard import RGBDPairGuard
        gate=RGBDPairGuard()
        def stream():
            consecutive_bad=0
            while True:
                frame=source.read(latest=True)
                report=gate.inspect(frame.timestamps)
                if not report['qualified']:
                    consecutive_bad+=1
                    print('RGBD_REJECTED',report,flush=True)
                    if consecutive_bad>=30:raise RuntimeError('No qualified RGB-D in 30 SDK observations')
                    continue
                consecutive_bad=0
                yield 'live',frame
        frames=stream()
    def save_current(frame,result,views,visualization,trigger):
        nonlocal saved
        saved+=1
        result=dict(result,capture_trigger=trigger)
        folder=output/'user_saves'/f'sample_{saved:04d}'
        save_vision_sample(folder,frame,result,views,visualization)
        print('Saved current sample',folder,flush=True)
    try:
        if source:
            print('Actual streams:',json.dumps(source.start()),flush=True)
            print('Preview only; s save; r bounded recording (max30 samples/30s); q exit',flush=True)
        for frame_index,(name,frame) in enumerate(frames):
            record_saved_this_frame=False
            cycle=perf_counter()
            result,views=pipeline.process(frame,sdk_deproject=source.deproject if source else None,force_cad=bool(args.replay),collect_diagnostics=ui.show_candidates)
            cad_now=perf_counter()
            if any(o['cad_fresh'] and o['body_pose_candidate'] is not None for o in result['objects']):cad_events.append(cad_now)
            cad_events=[t for t in cad_events if cad_now-t<=5]
            result['cad_pose_update_rate_hz']=len(cad_events)/max(min(cad_now-rate_start,5),1e-6) if source else None
            result.update(mode='REAL_RED_RGBD_OFFLINE' if args.replay else 'LIVE_DEVICE_UNVERIFIED',sample_id=name)
            now=perf_counter();fps=1/max(now-last,1e-9);last=now
            render_start=perf_counter()
            visualization=ui.render(frame,result,views,pipeline.geometry,source.project if source else None,fps if source else None)
            result['timing_ms']['render']=(perf_counter()-render_start)*1000
            result['timing_ms']['process_and_render']=(perf_counter()-cycle)*1000
            if args.replay:
                save_vision_sample(output/name,frame,result,views,visualization)
                thumbnail=cv2.resize(visualization,(640,360));thumbs.append(thumbnail);rows.append(result)
                print(name,len(result['objects']),[o['fit_status'] for o in result['objects']],f"{result['timing_ms']['process_and_render']:.0f}ms",flush=True)
            quit_requested=False
            if not args.headless:
                cv2.namedWindow(window,cv2.WINDOW_NORMAL);cv2.resizeWindow(window,1600,900)
                while True:
                    cv2.imshow(window,visualization);window_called=True
                    key=cv2.waitKey(30 if ui.paused else 1)&255
                    ui.handle_key(key)
                    if key in (ord('q'),27):quit_requested=True;break
                    if key==ord('s'):save_current(frame,result,views,visualization,'USER_KEY_S')
                    if key==ord('r'):
                        ui.recording=not ui.recording;recording_count=0;record_deadline=perf_counter()+30
                    if ui.recording and perf_counter()>=record_deadline:ui.recording=False
                    if ui.recording and not record_saved_this_frame:
                        record_saved_this_frame=True
                        save_current(frame,result,views,visualization,'USER_TRIGGERED_BOUNDED_RECORDING')
                        recording_count+=1
                        if recording_count>=30 or perf_counter()>=record_deadline:ui.recording=False
                    if key==ord('n'):break
                    visualization=ui.render(frame,result,views,pipeline.geometry,source.project if source else None,fps if source else None)
                    if not ui.paused or args.ui_smoke:break
            if quit_requested:break
            if not args.replay and args.max_frames and frame_index+1>=args.max_frames:break
        if args.replay:
            if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in hashes.items()):raise RuntimeError('Raw input modified')
            while len(thumbs)%3:thumbs.append(np.zeros((360,640,3),np.uint8))
            if thumbs:cv2.imencode('.png',np.vstack([np.hstack(thumbs[i:i+3]) for i in range(0,len(thumbs),3)]))[1].tofile(str(output/'contact_sheet.png'))
            fields=('detection','depth_filter','point_cloud','segmentation','cad_fit','tracking','total','render','process_and_render')
            timing={field:{'mean_ms':float(np.mean([r['timing_ms'][field] for r in rows])),
                       'p95_ms':float(np.percentile([r['timing_ms'][field] for r in rows],95))} for field in fields} if rows else {}
            summary=dict(frames_processed=len(rows),frames_expected=len(folders),
                frames_with_objects=sum(bool(r['objects']) for r in rows),
                frames_with_body_candidates=sum(any(o['body_pose_candidate'] is not None for o in r['objects']) for r in rows),
                frames_with_ring_candidates=sum(any(o['ring_center_candidate'] is not None for o in r['objects']) for r in rows),
                weak_constraint_count=sum('WEAK_XY_YAW_CONSTRAINT' in o['ambiguity'] for r in rows for o in r['objects']),
                timing=timing,offline_processing_throughput_hz=1000/timing['process_and_render']['mean_ms'] if rows else None,
                throughput_excludes_io_acquisition=True,actual_camera_started=False,new_real_frames=0,
                raw_inputs_unchanged=True,body_pose_valid=False,ring_pose_valid=False,
                position_accuracy='NOT_MEASURED',yaw_accuracy='NOT_MEASURED',
                gui_window_display_called=window_called,window_interaction='NOT RUN (automated)' if args.ui_smoke else ('AVAILABLE' if window_called else 'NOT RUN'),
                live_device_test='NOT RUN',live_status='DEVICE_TEST_PENDING')
            (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
            (output/'raw_input_sha256.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
            print('RESULT',output,json.dumps(summary,indent=2),flush=True)
    finally:
        if source:source.stop()
        if not args.headless:cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
