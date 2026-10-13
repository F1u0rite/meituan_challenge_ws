"""Main-thread fixed-size RGB preview; existing perception runs in one worker."""
import copy,json
from pathlib import Path
from threading import Event,Lock,Thread
from time import monotonic,time
import cv2,numpy as np
from perception.dual_rate_rgbd import LatestSlot,RateMeter,capture_time
from perception.vision_ui import VisionUI
from real_data.rgbd_recording import save_vision_sample


class DisplayChannel:
    def __init__(self):
        self.slot=LatestSlot();self.stop=Event();self.lock=Lock();self.latest=None
        self.geometry={};self.capture=None;self.error=None;self.summary=None
        self.record_request=Event();self.recording=False
    def preview(self,identity,frame,report):self.slot.put((identity,frame,dict(report)))
    def publish(self,result,views):
        with self.lock:self.latest=(copy.deepcopy(result),views)
    def snapshot(self):
        with self.lock:return self.latest


def display_snapshot(frame,identity,report,snapshot,now=None):
    """Never project an old detection/geometry onto a newer or rejected RGB pair."""
    now=time() if now is None else now
    if snapshot:
        original,views=snapshot;result=copy.deepcopy(original)
        age=max(0.,(now-original['capture_timestamp'])*1000)
        current=(report['qualified'] and identity==original['source_frame_id'] and
            now-original['capture_timestamp']>=0 and age<=original.get('max_detection_age_s',2.5)*1000)
        source=original['source_frame_id']
        if not current:
            views=[];result['candidate_diagnostics']=[]
            for obj in result['objects']:
                obj.update(candidate_state='HISTORY '+obj['candidate_state'],detection_stale=True,
                    cad_fresh=False,cad_stale=True,body_pose_candidate=None,ring_pose_candidate=None,
                    body_pose_valid=False,ring_pose_valid=False)
    else:
        source=None;age=None;current=False;views=[]
        result=dict(objects=[],timestamp=capture_time(frame),capture_timestamp=capture_time(frame),
            source_frame_id=identity,timing_ms={'total':0.},processing_latency_ms=0.,candidate_diagnostics=[])
    for obj in result['objects']:obj.update(body_pose_valid=False,ring_pose_valid=False)
    result.update(mode='LIVE RGB PREVIEW',preview_source_frame_id=identity,
        annotation_source_frame_id=source,annotation_age_ms=age,annotation_is_current=current,
        preview_rgbd_qualified=report['qualified'],preview_sync_report=report,
        body_pose_valid=False,ring_pose_valid=False)
    return result,views


def run_live_display(args):
    from scripts.dual_rate_runner import run_dual_rate
    from scripts.live_rgbd_preview import ROOT
    import uuid
    output=args.replay_output or ROOT/'outputs/live_rgbd_integration'/('live_'+uuid.uuid4().hex[:8])
    if output.exists():raise FileExistsError(output)
    print('LIVE_DISPLAY_OUTPUT',output,flush=True)
    channel=DisplayChannel();worker_args=copy.copy(args)
    worker_args.headless=True;worker_args.replay_output=output;worker_args._display_channel=channel
    def process():
        try:channel.summary=run_dual_rate(worker_args)
        except BaseException as exc:channel.error=exc
        finally:channel.slot.close()
    worker=Thread(target=process,name='live-fast-dispatch')
    ui=VisionUI();rate=RateMeter();times=[];rows=[];last_rgb=None;last=None;saved=0
    window='Battery RGB-D | LIVE RGB PREVIEW | POSE INVALID';created=False
    def record_show(image,result,report):
        rate.add();stamp=monotonic();times.append(stamp)
        rows.append(dict(display_monotonic_s=stamp,canvas_shape=list(image.shape),
            preview_source_frame_id=result['preview_source_frame_id'],annotation_source_frame_id=result['annotation_source_frame_id'],
            annotation_age_ms=result['annotation_age_ms'],annotation_is_current=result['annotation_is_current'],
            preview_rgbd_qualified=report['qualified'],camera_rgb_fps=result.get('camera_fps'),
            camera_depth_fps=result.get('camera_depth_fps'),ui_fps=rate.hz(),
            fast_fps=result.get('detection_fps'),cad_fps=result.get('cad_fps')))
    worker.start()
    try:
        while True:
            packet=channel.slot.get(.01)
            if packet is not None and not ui.paused:
                identity,frame,report=packet
                rgb_id=frame.timestamps.get('color_frame_number')
                if rgb_id!=last_rgb or (last is not None and report['qualified']!=last[4]['qualified']):
                    last_rgb=rgb_id
                    result,views=display_snapshot(frame,identity,report,channel.snapshot())
                    capture=channel.capture
                    result.update(camera_fps=capture.stream_rates['color'].hz() if capture else None,
                        camera_depth_fps=capture.stream_rates['depth'].hz() if capture else None,
                        ui_fps=rate.hz(),detection_fps=result.get('detection_fps'),cad_fps=result.get('cad_fps'))
                    ui.recording=channel.recording
                    image=ui.render(frame,result,views,channel.geometry)
                    if not created:
                        cv2.namedWindow(window,cv2.WINDOW_NORMAL);cv2.resizeWindow(window,1600,900);created=True
                    cv2.imshow(window,image);record_show(image,result,report)
                    last=(frame,result,views,image,report)
            # A held/paused image must not retain a frozen CURRENT label past expiry.
            if last and last[1]['annotation_is_current'] and time()-last[1]['capture_timestamp']>2.5:
                frame,result,views,image,report=last
                result,views=display_snapshot(frame,result['preview_source_frame_id'],report,(result,views))
                image=ui.render(frame,result,views,channel.geometry);cv2.imshow(window,image)
                record_show(image,result,report)
                last=(frame,result,views,image,report)
            key=cv2.waitKey(1)&255 if created else 255
            if key in (ord('q'),27):break
            ui.handle_key(key)
            if key==ord('r'):channel.record_request.set()
            if key==ord('s') and last:
                frame,result,views,image,report=last
                if report['qualified']:
                    saved+=1;folder=output/'user_saves'/f'preview_{saved:04d}'
                    save_vision_sample(folder,frame,result,views,image)
                    print('Saved preview with explicit annotation provenance',folder,flush=True)
                else:print('Save refused: unqualified RGB-D; RGB preview remains available',flush=True)
            if not worker.is_alive() and channel.slot.closed:break
        if channel.error:raise channel.error
    finally:
        channel.stop.set();worker.join(30)
        if created:cv2.destroyAllWindows()
        intervals=np.diff(times)*1000
        summary=dict(ui_display_frames=len(times),ui_fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None,
            interval_mean_ms=float(np.mean(intervals)) if len(intervals) else None,
            interval_p95_ms=float(np.percentile(intervals,95)) if len(intervals) else None,
            interval_max_ms=float(max(intervals)) if len(intervals) else None,
            canvas_shapes=[list(s) for s in sorted({tuple(r['canvas_shape']) for r in rows})],
            resize_calls=int(created),window_initializations=int(created),preview_queue_replacements=channel.slot.dropped,
            rejected_rgbd_preview_frames=sum(not r['preview_rgbd_qualified'] for r in rows),
            body_pose_valid=False,ring_pose_valid=False,physical_monitor_fps='NOT_MEASURED')
        if output.exists():
            (output/'display_frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (output/'display_summary.json').write_text(json.dumps(summary,indent=2))
        if worker.is_alive():raise RuntimeError('Live processing did not stop; check camera worker cleanup')
    if channel.error:raise channel.error
    return dict(channel.summary or {},display=summary)
