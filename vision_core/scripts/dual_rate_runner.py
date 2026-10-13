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


def consume_cad_results(fast,cad):
    """Every consumer preserves table evidence, including paused/key refreshes."""
    messages=(cad.drain_table() if hasattr(cad,'drain_table') else [])+cad.drain()
    for message in messages:
        fast.set_table_hint(message.get('table_plane'),message['capture_timestamp'],message['source_frame_id'],
            plane_generated_host_s=message.get('plane_generated_host_s'),
            cad_completed_host_s=message.get('result_timestamp') if message.get('kind')!='TABLE_READY' else None,
            cad_submitted_host_s=message.get('submitted_host_s'),cad_started_host_s=message.get('started_host_s'))
    return messages


def pipeline_diagnostics(capture,cad,result):
    sync=capture.pair_guard.snapshot() if capture else None
    return dict(usb_packet_loss=None,usb_packet_loss_status='NOT_MEASURED',
        sdk_observed_frames=capture.observed_frames if capture else 0,
        sdk_observed_id_gaps=sync['observed_id_gaps'] if sync else {},
        sdk_observed_repeats=sync['observed_repeats'] if sync else {},
        sdk_queue_drained=capture.sdk_dropped if capture else 0,
        capture_latest_queue_replacements=capture.slot.dropped if capture else 0,
        sync_rejected=sync['rejected'] if sync else 0,
        sync_rejection_reasons=sync['reasons'] if sync else {},
        qualified_pairs=sync['accepted'] if sync else None,
        cad_latest_queue_replacements=cad.slot.dropped,
        cad_jobs_completed=cad.completed_jobs,cad_worker_error=cad.error,
        historical_stale_cad_targets=sum(o.get('historical_cad',{}).get('status')=='STALE' for o in result['objects']),
        cad_result_age_ms=result.get('cad_result_age_ms'),
        note='SDK gaps/draining, processing queue replacement and sync rejection are distinct; none proves USB packet loss')


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


def interactive_frame(ui,window,image,on_save,on_record=None,on_render=None):
    """Existing OpenCV controls; q exits even while paused, n advances one frame."""
    while True:
        cv2.imshow(window,image)
        key=cv2.waitKey(30 if ui.paused else 1)&255
        if key in (ord('q'),27):return True,image
        ui.handle_key(key)
        if on_render is not None and (key!=255 or ui.paused):image=on_render()
        if key==ord('s'):on_save(image)
        if key==ord('r') and on_record is not None:on_record()
        if key==ord('n') or not ui.paused:return False,image


def run_dual_rate(args):
    from scripts.live_rgbd_preview import ROOT
    from perception.realtime_rgbd import RGBDVisionPipeline
    from perception.observation import ObservationMonitor
    observation=ObservationMonitor()
    simplified=getattr(args,'perception_pipeline','legacy')=='simplified'
    if simplified:
        from perception.simplified_rgbd import SimplifiedPerception,DisabledFullCAD
        fast=SimplifiedPerception(geometry_path=getattr(args,'cad_geometry',None))
        cad=DisabledFullCAD(fast.geometry)
    else:fast=FastPerception()
    if not simplified:cad=CADWorker(RGBDVisionPipeline(args.vision_config))
    ui=VisionUI()
    source=None;capture=None;recorder=None;record_lock=Lock();record_index=0
    display_channel=getattr(args,'_display_channel',None)
    if display_channel:display_channel.geometry=cad.pipeline.geometry
    tcp_sender=None;ros_publisher=None
    if getattr(args,'tcp_send',False):
        import yaml
        from perception.tcp_vision_sender import TCPVisionSender
        tcp_sender=TCPVisionSender(**yaml.safe_load(args.tcp_config.read_text(encoding='utf-8-sig')))
    output=args.replay_output or ROOT/'outputs/live_rgbd_integration'/('dual_'+uuid.uuid4().hex[:8])
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True)
    hashes={};folders=[];rows=[];cad_rows=[];cad_messages=[];views_log=[];screens=[];history={}
    manual_save_index=0;window_called=False;expected_frames=0
    capture_count=0;cad_rejected=0;last_cad_schedule=-1e9;cad_interval=float(cad.pipeline.config.get('cad_interval_s',1.0))
    total_start=perf_counter();window='Battery RGB-D Dual Rate | OFFLINE' if args.replay else 'Battery RGB-D Dual Rate | LIVE UNVERIFIED'
    if args.replay:
        folders=sorted(args.replay.glob('sample_*'))
        if not folders:
            folders=sorted(p for p in args.replay.glob('frame_*') if p.is_dir() and (p/'metadata.json').exists())
        if args.max_frames:folders=folders[:args.max_frames]
        if not folders and not getattr(args,'pair_manifest',None):raise ValueError('No samples')
        for folder in folders:
            for name in ('raw_color.png','aligned_depth.png','metadata.json','camera_intrinsics.json'):
                p=folder/name;hashes[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest()
        if getattr(args,'pair_manifest',None):
            from real_data.native_array_recording import qualified_native_pairs
            indices=json.loads(args.pair_manifest.read_text())['accepted_original_indices']
            expected_frames=min(len(indices),args.max_frames) if args.max_frames else len(indices)
            originals=json.loads((args.pair_manifest.parent/'original_sha256.json').read_text())
            for index in (indices[:args.max_frames] if args.max_frames else indices):
                for name in ('metadata.json','color_bgr.npy','raw_depth_z16.npy','aligned_depth_z16.npy'):
                    relative='frames/'+f'{index:06d}/'+name
                    hashes[str(args.replay.resolve()/relative)]=originals[relative]
            def original_frames():
                for i,(identity,f,folder) in enumerate(qualified_native_pairs(args.replay,args.pair_manifest)):
                    if args.max_frames and i>=args.max_frames:break
                    yield identity,f,folder.name
            frames=original_frames()
        else:
            expected_frames=len(folders)
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
                if display_channel and display_channel.stop.is_set():break
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
            if display_channel:display_channel.recording=False
    try:
        if getattr(args,'ros_publish',False):
            from mtc_vision_adapter.native_result_publisher import NativeResultPublisher
            ros_publisher=NativeResultPublisher(getattr(args,'ros_wait_subscriber_s',0.))
        if source:
            print('Actual streams:',json.dumps(source.start()),flush=True)
            capture=CaptureWorker(source,on_frame=record_frame,
                on_preview=display_channel.preview if display_channel else None)
            if display_channel:display_channel.capture=capture
        cad.start()
        if tcp_sender: tcp_sender.start()
        if capture:
            capture.start()
            print('s save current diagnostic; r begin/end continuous RGB-D (bounded); q exit',flush=True)
        for identity,frame,name in frames:
            capture_count+=1;cycle=perf_counter()
            # Consume completed evidence before the current confirmation gate.
            received=consume_cad_results(fast,cad);cad_messages.extend(received)
            result,object_views=fast.process(frame,identity)
            result['mode']='REAL_RED_RGBD_OFFLINE_DUAL_RATE' if args.replay else 'LIVE_DUAL_RATE_UNVERIFIED'
            result['sample_id']=name
            merged,rejected=merge_cad_history(history,received);cad_rejected+=rejected;cad_rows.extend(merged)
            now=monotonic()
            if not simplified and result['objects'] and now-last_cad_schedule>=cad_interval:
                cad.submit(frame,identity,result['objects']);last_cad_schedule=now
            result['observation']=observation.update(result,[m for m in received if m.get('kind')!='TABLE_READY'])
            cad_for_display(result,identity,history,now=result['capture_timestamp'] if args.replay else None)
            result['camera_fps']=capture.stream_rates['color'].hz() if capture else None
            result['camera_depth_fps']=capture.stream_rates['depth'].hz() if capture else None
            result['qualified_pair_fps']=capture.rate.hz() if capture else None
            result['detection_fps']=fast.rate.hz()
            result['cad_fps']=cad.rate.hz()
            result['sync_diagnostics']=capture.pair_guard.snapshot() if capture else {'qualified_manifest_replay':bool(getattr(args,'pair_manifest',None))}
            result['cad_clock_policy']='RECORDED_CAPTURE_TIMELINE' if args.replay else 'CURRENT_WALL_TIME'
            result['capture_dropped_frames']=capture.dropped if capture else 0
            result['cad_queue_dropped']=cad.slot.dropped
            result['cad_result_age_ms']=max((o['historical_cad']['result_age_ms'] for o in result['objects']
                if 'historical_cad' in o),default=None)
            result['result_age_ms']=None if args.replay else max(0,(time()-result['capture_timestamp'])*1000)
            result['pipeline_diagnostics']=pipeline_diagnostics(capture,cad,result)
            if ros_publisher:
                result['ros2_dispatch']=ros_publisher.publish_result(result,
                    mode='offline' if args.replay else 'live',pipeline_diagnostics=result['pipeline_diagnostics'])
            if tcp_sender:
                tcp_sender.offer_vision(result,mode='test' if args.replay else 'live',timestamps=frame.timestamps)
                result['tcp_sender']=tcp_sender.snapshot()
            if display_channel:
                image=None;result['timing_ms']['render']=0.
            else:
                image=ui.render(frame,result,object_views,cad.pipeline.geometry,source.project if source else None,
                               capture.rate.hz() if capture else None)
                result['timing_ms']['render']=ui.last_timing_ms['depth_render']+ui.last_timing_ms['ui_overlay']
            result['timing_ms']['process_and_render']=(perf_counter()-cycle)*1000
            rows.append(result)
            if display_channel:display_channel.publish(result,object_views)
            if display_channel and display_channel.record_request.is_set():
                display_channel.record_request.clear()
                if recorder is not None:finish_record()
                else:
                    record_index+=1
                    new=ContinuousRGBDRecorder(output/f'continuous_{record_index:03d}',args.continuous_max_seconds,args.continuous_max_frames)
                    new.start()
                    with record_lock:recorder=new
                    display_channel.recording=True
            if args.replay and (capture_count in (1,18,29,expected_frames)):
                target=output/f'frame_{capture_count:04d}.png'
                cv2.imencode('.png',image)[1].tofile(str(target))
                screens.append(cv2.resize(image,(800,450)))
                (output/f'frame_{capture_count:04d}_display.json').write_text(json.dumps(ui.last_display_data,default=str,indent=2))
            if not args.headless:
                if not window_called:
                    cv2.namedWindow(window,cv2.WINDOW_NORMAL);cv2.resizeWindow(window,1600,900)
                def save_display(current_image):
                    nonlocal manual_save_index
                    manual_save_index+=1
                    folder=output/'user_saves'/f'sample_{manual_save_index:04d}'
                    save_vision_sample(folder,frame,result,object_views,current_image)
                    print('Saved diagnostic',folder,flush=True)
                def record_toggle():
                    nonlocal record_index,recorder
                    if capture is None:return
                    if recorder is not None:finish_record()
                    else:
                        record_index+=1
                        target=output/f'continuous_{record_index:03d}'
                        new=ContinuousRGBDRecorder(target,args.continuous_max_seconds,args.continuous_max_frames)
                        new.start()
                        with record_lock:recorder=new
                        ui.recording=True
                paused_cad_requested=False
                def refresh_display():
                    nonlocal paused_cad_requested,cad_rejected
                    if args.replay and ui.paused and not paused_cad_requested and result['objects']:
                        if not simplified:cad.submit(frame,identity,result['objects'])
                        paused_cad_requested=True
                    fresh=consume_cad_results(fast,cad);cad_messages.extend(fresh)
                    result['detection_fps']=fast.rate.hz();result['cad_fps']=cad.rate.hz()
                    merged,late=merge_cad_history(history,fresh);cad_rejected+=late;cad_rows.extend(merged)
                    cad_for_display(result,identity,history,now=result['capture_timestamp'] if args.replay else None)
                    return ui.render(frame,result,object_views,cad.pipeline.geometry,source.project if source else None)
                quit_requested,image=interactive_frame(ui,window,image,save_display,record_toggle,refresh_display)
                window_called=True
                result['display_timestamp']=time()
                result['capture_to_display_call_ms']=max(0,(result['display_timestamp']-result['capture_timestamp'])*1000) if capture else None
                result['display_latency_definition']='After imshow/waitKey call; physical monitor scanout not measured'
                if recorder is not None:recorder.add_detection(identity,result)
                if quit_requested:break
            if recorder is not None and recorder.closed:finish_record()
            if capture and args.max_frames and capture_count>=args.max_frames:break
        cad.close()
        remaining=consume_cad_results(fast,cad);cad_messages.extend(remaining)
        merged,rejected=merge_cad_history(history,remaining);cad_rejected+=rejected;cad_rows.extend(merged)
        if args.replay:
            if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in hashes.items()):
                raise RuntimeError('Original RGB-D input changed')
        if screens:
            while len(screens)<4:screens.append(np.zeros_like(screens[0]))
            sheet=np.vstack((np.hstack(screens[:2]),np.hstack(screens[2:])))
            cv2.imencode('.png',sheet)[1].tofile(str(output/'ui_contact_sheet.png'))
        if tcp_sender: tcp_sender.close()
        elapsed=perf_counter()-total_start
        summary=dict(mode='offline' if args.replay else 'live_unverified',frames_processed=len(rows),
            actual_camera_started=bool(capture),input_frames=expected_frames if args.replay else capture_count,
            gui_window_display_called=window_called,
            camera_fps=None if args.replay else (capture.stream_rates['color'].hz() if capture else None),
            detection_fps=fast.rate.hz(),cad_fps=cad.rate.hz(),
            sync_diagnostics=capture.pair_guard.snapshot() if capture else {'qualified_manifest_replay':bool(getattr(args,'pair_manifest',None))},
            fast_timing=_stats([r['processing_latency_ms'] for r in rows]),
            cad_timing=_stats([c['processing_ms'] for c in cad_messages if c.get('kind')!='TABLE_READY']),
            render_timing=_stats([r['timing_ms']['render'] for r in rows]),
            offline_wall_throughput_hz=len(rows)/elapsed if args.replay else None,
            throughput_is_not_camera_fps=True,
            fast_color_counts={c:sum(o['color_label']==c for r in rows for o in r['objects'])
                               for c in ('red','blue','yellow','green')},
            cad_jobs_completed=cad.completed_jobs,cad_matched_object_observations=len(cad_rows),cad_worker_error=cad.error,
            cad_queue_dropped=cad.slot.dropped,cad_out_of_order_rejected=cad_rejected,
            capture_dropped_frames=capture.dropped if capture else 0,
            fast_body_pose_valid_count=sum(o['body_pose_valid'] for r in rows for o in r['objects']),
            fast_ring_pose_valid_count=sum(o['ring_pose_valid'] for r in rows for o in r['objects']),
            body_pose_valid=False,ring_pose_valid=False,position_accuracy='NOT_MEASURED',
            yaw_accuracy='NOT_MEASURED',live_test='NOT RUN' if args.replay else 'LIVE_SESSION',
            raw_inputs_unchanged=True if args.replay else None,
            tcp_sender=tcp_sender.snapshot() if tcp_sender else None)
        summary['ros2_dispatch']=ros_publisher.snapshot() if ros_publisher else None
        summary['pipeline_diagnostics']=pipeline_diagnostics(capture,cad,rows[-1] if rows else {'objects':[]})
        with (output/'detections.jsonl').open('w',encoding='utf-8') as f:
            for row in rows:f.write(json.dumps(row,allow_nan=False,default=str)+'\n')
        with (output/'cad_results.jsonl').open('w',encoding='utf-8') as f:
            for row in cad_rows:f.write(json.dumps(row,allow_nan=False,default=str)+'\n')
        with (output/'cad_lifecycle.jsonl').open('w',encoding='utf-8') as f:
            for row in cad_messages:f.write(json.dumps(row,allow_nan=False,default=str)+'\n')
        (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        if hashes:(output/'raw_input_sha256.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
        print('DUAL_RATE_RESULT',output,json.dumps(summary,indent=2),flush=True)
        return summary
    finally:
        # Run every cleanup even if a recorder/source cleanup raises.
        from contextlib import ExitStack
        with ExitStack() as cleanup:
            if ros_publisher:cleanup.callback(ros_publisher.close)
            if not args.headless:cleanup.callback(cv2.destroyAllWindows)
            if cad.thread and cad.thread.is_alive():cleanup.callback(cad.close)
            if capture:cleanup.callback(capture.close)
            elif source:cleanup.callback(source.stop)
            cleanup.callback(finish_record)
            if tcp_sender:cleanup.callback(tcp_sender.close)
