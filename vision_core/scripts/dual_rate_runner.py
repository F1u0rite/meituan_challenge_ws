"""Opt-in dual-rate RGB-D preview and offline replay. Camera stays closed in replay."""
import hashlib
import json
import uuid
from pathlib import Path
from threading import Lock
from time import monotonic, perf_counter, time
import cv2
import numpy as np
from input.realsense_rgbd_source import RealSenseRGBDSource
from perception.dual_rate_rgbd import CaptureWorker, CADWorker, FastPerception, cad_for_display, frame_id
from perception.vision_ui import VisionUI
from real_data.continuous_rgbd import ContinuousRGBDRecorder
from real_data.rgbd_recording import load_rgbd_sample, save_vision_sample


def _stats(values):
    return dict(mean_ms=float(np.mean(values)),p95_ms=float(np.percentile(values,95))) if values else None


def merge_cad_history(history, messages):
    """Late/out-of-order CAD jobs cannot replace a newer observation for one ID."""
    accepted=[];rejected=0
    for message in messages:
        for object_id,obj in message['matches'].items():
            old=history.get(object_id)
            if old and old['capture_timestamp'] >= message['capture_timestamp']:
                rejected+=1;continue
            history[object_id]=dict(source_frame_id=message['source_frame_id'],
                capture_timestamp=message['capture_timestamp'],result_timestamp=message['result_timestamp'],capture_timestamp_source=message.get('capture_timestamp_source','UNAVAILABLE'),
                timestamps=message.get('timestamps',{}),object=obj)
            accepted.append(dict(object_id=object_id,source_frame_id=message['source_frame_id'],
                capture_timestamp=message['capture_timestamp'],result_timestamp=message['result_timestamp'],
                processing_ms=message['processing_ms'],fit_status=obj.get('fit_status'),
                body_xyz_candidate=obj.get('model_estimated_body_center_xyz'),
                body_yaw_candidate=obj.get('body_yaw_candidate'),
                ring_center_candidate=obj.get('ring_center_candidate'),
                insertion_direction_candidates=obj.get('insertion_direction_candidates'),
                ambiguity=obj.get('ambiguity'),rejection_reason=obj.get('rejection_reason'),
                body_pose_valid=False,ring_pose_valid=False))
    return accepted,rejected


def run_dual_rate(args):
    from scripts.live_rgbd_preview import ROOT
    from perception.realtime_rgbd import RGBDVisionPipeline
    from perception.observation import ObservationMonitor
    observation=ObservationMonitor()
    fast=FastPerception();cad=CADWorker(RGBDVisionPipeline(args.vision_config));ui=VisionUI()
    source=None;capture=None;recorder=None;record_lock=Lock();record_index=0
    output=args.replay_output or ROOT/'outputs/live_rgbd_integration'/('dual_'+uuid.uuid4().hex[:8])
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True)
    hashes={};folders=[];rows=[];cad_rows=[];views_log=[];screens=[];history={}
    capture_count=0;cad_rejected=0;last_cad_schedule=-1e9;cad_interval=float(cad.pipeline.config.get('cad_interval_s',1.0))
    total_start=perf_counter();window='Battery RGB-D Dual Rate | OFFLINE' if args.replay else 'Battery RGB-D Dual Rate | LIVE UNVERIFIED'
    if args.replay:
        folders=sorted(args.replay.glob('sample_*'))
        if args.max_frames:folders=folders[:args.max_frames]
        if not folders:raise ValueError('No samples')
        for folder in folders:
            for name in ('raw_color.png','aligned_depth.png','metadata.json','camera_intrinsics.json'):
                p=folder/name;hashes[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest()
        frames=((frame_id(f,i),f,folder.name) for i,folder in enumerate(folders,1)
                for f in [load_rgbd_sample(folder)])
    else:
        # Construction and start are reachable only after --preview-authorized in the entry point.
        source=RealSenseRGBDSource(preferred_color=tuple(args.color_size),
            preferred_depth=tuple(args.depth_size),fps=args.fps)
        def record_frame(identity,frame):
            with record_lock:
                if recorder is not None:recorder.offer(identity,frame)
        def live_frames():
            index=0
            while True:
                packet=capture.slot.get(.3)
                if packet is None:
                    if capture.error:raise RuntimeError(capture.error)
                    if capture.slot.closed:break
                    continue
                index+=1
                yield packet[0],packet[1],f'live_{index:06d}'
        frames=live_frames()
    def finish_record():
        nonlocal recorder
        with record_lock:
            old=recorder;recorder=None
        if old is not None:
            summary=old.close(capture.dropped if capture else 0)
            print('Continuous recording:',old.folder,json.dumps(summary),flush=True)
            ui.recording=False
    try:
        if source:
            print('Actual streams:',json.dumps(source.start()),flush=True)
            capture=CaptureWorker(source,on_frame=record_frame)
        cad.start()
        if capture:
            capture.start()
            print('s save current diagnostic; r begin/end continuous RGB-D (bounded); q exit',flush=True)
        for identity,frame,name in frames:
            capture_count+=1;cycle=perf_counter()
            result,object_views=fast.process(frame,identity)
            result['mode']='REAL_RED_RGBD_OFFLINE_DUAL_RATE' if args.replay else 'LIVE_DUAL_RATE_UNVERIFIED'
            result['sample_id']=name
            received=cad.drain()
            for message in received:
                fast.set_table_hint(message.get('table_plane'),message['capture_timestamp'],message['source_frame_id'])
            merged,rejected=merge_cad_history(history,received);cad_rejected+=rejected;cad_rows.extend(merged)
            now=monotonic()
            if result['objects'] and now-last_cad_schedule>=cad_interval:
                cad.submit(frame,identity,result['objects']);last_cad_schedule=now
            result['observation']=observation.update(result,received)
            cad_for_display(result,identity,history,now=result['capture_timestamp'] if args.replay else None)
            result['camera_fps']=capture.rate.hz() if capture else None
            result['detection_fps']=fast.rate.hz() if capture else None
            result['cad_fps']=cad.rate.hz() if capture else None
            result['capture_dropped_frames']=capture.dropped if capture else 0
            result['cad_queue_dropped']=cad.slot.dropped
            result['cad_result_age_ms']=max((o['historical_cad']['result_age_ms'] for o in result['objects']
                if 'historical_cad' in o),default=None)
            result['result_age_ms']=None if args.replay else max(0,(time()-result['capture_timestamp'])*1000)
            image=ui.render(frame,result,object_views,cad.pipeline.geometry,source.project if source else None,
                           capture.rate.hz() if capture else None)
            result['timing_ms']['render']=ui.last_timing_ms['depth_render']+ui.last_timing_ms['ui_overlay']
            result['timing_ms']['process_and_render']=(perf_counter()-cycle)*1000
            rows.append(result)
            if args.replay and (capture_count in (1,18,29,len(folders))):
                target=output/f'frame_{capture_count:04d}.png'
                cv2.imencode('.png',image)[1].tofile(str(target))
                screens.append(cv2.resize(image,(800,450)))
            if not args.headless:
                cv2.namedWindow(window,cv2.WINDOW_NORMAL);cv2.resizeWindow(window,1600,900)
                cv2.imshow(window,image)
                key=cv2.waitKey(1 if not ui.paused else 30)&255
                result['display_timestamp']=time()
                result['capture_to_display_call_ms']=max(0,(result['display_timestamp']-result['capture_timestamp'])*1000) if capture else None
                result['display_latency_definition']='After imshow/waitKey call; physical monitor scanout not measured'
                if recorder is not None:recorder.add_detection(identity,result)
                ui.handle_key(key)
                if key in (ord('q'),27):break
                if key==ord('s'):
                    folder=output/'user_saves'/f'sample_{capture_count:04d}'
                    save_vision_sample(folder,frame,result,object_views,image)
                    print('Saved diagnostic',folder,flush=True)
                if key==ord('r') and capture:
                    if recorder is not None:finish_record()
                    else:
                        record_index+=1
                        target=output/f'continuous_{record_index:03d}'
                        new=ContinuousRGBDRecorder(target,args.continuous_max_seconds,args.continuous_max_frames)
                        new.start()
                        with record_lock:recorder=new
                        ui.recording=True
                        print('Recording complete RGB-D frames to',target,flush=True)
                while ui.paused and not args.replay:
                    k=cv2.waitKey(30)&255
                    if k in (ord('q'),27):break
                    ui.handle_key(k)
            if recorder is not None and recorder.closed:finish_record()
            if capture and args.max_frames and capture_count>=args.max_frames:break
        cad.close()
        merged,rejected=merge_cad_history(history,cad.drain());cad_rejected+=rejected;cad_rows.extend(merged)
        if args.replay:
            if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in hashes.items()):
                raise RuntimeError('Original RGB-D input changed')
        if screens:
            while len(screens)<4:screens.append(np.zeros_like(screens[0]))
            sheet=np.vstack((np.hstack(screens[:2]),np.hstack(screens[2:])))
            cv2.imencode('.png',sheet)[1].tofile(str(output/'ui_contact_sheet.png'))
        elapsed=perf_counter()-total_start
        summary=dict(mode='offline' if args.replay else 'live_unverified',frames_processed=len(rows),
            actual_camera_started=bool(capture),input_frames=len(folders) if args.replay else capture_count,
            fast_timing=_stats([r['processing_latency_ms'] for r in rows]),
            cad_timing=_stats([c['processing_ms'] for c in cad_rows]),
            render_timing=_stats([r['timing_ms']['render'] for r in rows]),
            offline_wall_throughput_hz=len(rows)/elapsed if args.replay else None,
            throughput_is_not_camera_fps=True,
            fast_color_counts={c:sum(o['color_label']==c for r in rows for o in r['objects'])
                               for c in ('red','blue','yellow','green')},
            cad_jobs_completed=len(cad_rows),cad_worker_error=cad.error,
            cad_queue_dropped=cad.slot.dropped,cad_out_of_order_rejected=cad_rejected,
            capture_dropped_frames=capture.dropped if capture else 0,
            fast_body_pose_valid_count=sum(o['body_pose_valid'] for r in rows for o in r['objects']),
            fast_ring_pose_valid_count=sum(o['ring_pose_valid'] for r in rows for o in r['objects']),
            body_pose_valid=False,ring_pose_valid=False,position_accuracy='NOT_MEASURED',
            yaw_accuracy='NOT_MEASURED',live_test='NOT RUN' if args.replay else 'LIVE_SESSION',
            raw_inputs_unchanged=True if args.replay else None)
        with (output/'detections.jsonl').open('w',encoding='utf-8') as f:
            for row in rows:f.write(json.dumps(row,allow_nan=False,default=str)+'\n')
        with (output/'cad_results.jsonl').open('w',encoding='utf-8') as f:
            for row in cad_rows:f.write(json.dumps(row,allow_nan=False,default=str)+'\n')
        (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        if hashes:(output/'raw_input_sha256.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
        print('DUAL_RATE_RESULT',output,json.dumps(summary,indent=2),flush=True)
        return summary
    finally:
        finish_record()
        if capture:capture.close()
        elif source:source.stop()
        if cad.thread and cad.thread.is_alive():cad.close()
        if not args.headless:cv2.destroyAllWindows()
