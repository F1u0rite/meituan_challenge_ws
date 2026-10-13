"""Bounded asynchronous lossless RGB-D writer with explicit drop/integrity logs."""
from collections import deque
from dataclasses import asdict
from pathlib import Path
from threading import Condition,Thread
from time import monotonic,time
import hashlib,json
import cv2,numpy as np
from perception.rgbd_geometry import validate_frame

class ContinuousRGBDRecorder:
    def __init__(self,folder,max_seconds=30.,max_frames=450,queue_capacity=16,writer_workers=2,png_compression=0):
        if max_seconds<=0 or max_frames<=0 or queue_capacity<=0 or writer_workers not in (1,2,3,4) or not 0<=png_compression<=9:
            raise ValueError('Invalid recorder bounds or PNG settings')
        self.folder=Path(folder);self.max_seconds=max_seconds;self.max_frames=max_frames
        self.capacity=queue_capacity;self.worker_count=writer_workers;self.png_compression=png_compression
        self.cv=Condition();self.queue=deque();self.thread=None;self.threads=[]
        self.started=None;self.closed=False;self.offered=0;self.saved=0;self.dropped=0;self.error=None
        self.detected=0;self.records=[];self.manifest=[];self.drop_events=[];self.queue_peak=0;self.finalized=None
    def start(self):
        if self.folder.exists():raise FileExistsError(self.folder)
        self.folder.mkdir(parents=True);self.started=monotonic()
        self.threads=[Thread(target=self._run,name=f'rgbd-writer-{i}') for i in range(self.worker_count)]
        self.thread=self.threads[0]
        for worker in self.threads:worker.start()
    def offer(self,source_frame_id,frame):
        with self.cv:
            if self.closed:return False
            if monotonic()-self.started>=self.max_seconds or self.offered>=self.max_frames:
                self.closed=True;self.cv.notify_all();return False
            self.offered+=1
            if len(self.queue)>=self.capacity:
                sequence,identity,_,_=self.queue.popleft();self.dropped+=1
                self.drop_events.append(dict(sequence=sequence,source_frame_id=identity,reason='WRITER_QUEUE_FULL',timestamp=time()))
            self.queue.append((self.offered,source_frame_id,frame,monotonic()))
            self.queue_peak=max(self.queue_peak,len(self.queue));self.cv.notify_all();return True
    def add_detection(self,source_frame_id,result):
        record=dict(source_frame_id=source_frame_id,capture_timestamp=result['capture_timestamp'],
            capture_timestamp_source=result.get('capture_timestamp_source','UNAVAILABLE'),
            result_timestamp=result['result_timestamp'],processing_latency_ms=result['processing_latency_ms'],
            capture_to_display_call_ms=result.get('capture_to_display_call_ms'),
            camera_motion_proxy=result.get('camera_motion_proxy'),observation=result.get('observation'),
            capture_dropped_frames=result.get('capture_dropped_frames'),cad_queue_dropped=result.get('cad_queue_dropped'),
            objects=[{k:o.get(k) for k in ('object_id','color_label','confidence','bbox','pixel_center','surface_depth_z','candidate_state','tracking_status')} for o in result['objects']],
            tracking_status=result['tracking_status'])
        with self.cv:
            if self.finalized is None:self.records.append(record);self.detected+=1
    def _write(self,sequence,identity,frame,queued_at):
        started=monotonic();validate_frame(frame.color_bgr,frame.aligned_depth,frame.intrinsics,frame.depth_scale)
        temporary=self.folder/f'.partial_{sequence:04d}';temporary.mkdir()
        images={'raw_color.png':frame.color_bgr,'aligned_depth.png':frame.aligned_depth}
        if frame.raw_depth is not None:
            if frame.raw_depth.dtype!=np.uint16:raise ValueError('Raw depth must be uint16')
            images['raw_depth.png']=frame.raw_depth
        checksums={};encoding_ms=0.;writing_ms=0.
        for name,im in images.items():
            start=monotonic();ok,data=cv2.imencode('.png',im,[cv2.IMWRITE_PNG_COMPRESSION,self.png_compression])
            if not ok:raise IOError('PNG encode failed: '+name)
            encoding_ms+=(monotonic()-start)*1000
            checksums[name]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=int(data.nbytes),dtype=str(im.dtype),shape=list(im.shape))
            start=monotonic();data.tofile(str(temporary/name));writing_ms+=(monotonic()-start)*1000
            if (temporary/name).stat().st_size!=data.nbytes:raise IOError('Incomplete PNG write: '+name)
        metadata=dict(source_frame_id=identity,sequence=sequence,depth_scale_m_per_unit=frame.depth_scale,
            timestamps=frame.timestamps,stream_profiles=frame.stream_profiles,alignment_metadata=frame.alignment_metadata,
            camera_configuration=frame.camera_configuration,
            raw_depth_intrinsics=asdict(frame.raw_depth_intrinsics) if frame.raw_depth_intrinsics else None,
            pixel_format='BGR8',aligned_depth_format='Z16_RAW_ALIGNED_TO_COLOR',png_compression=self.png_compression)
        for name,value in [('camera_intrinsics.json',asdict(frame.intrinsics)),('metadata.json',metadata)]:
            encoded=json.dumps(value,separators=(',',':'),allow_nan=False).encode('utf-8');(temporary/name).write_bytes(encoded)
            checksums[name]=dict(sha256=hashlib.sha256(encoded).hexdigest(),bytes=len(encoded))
        (temporary/'integrity.json').write_text(json.dumps(dict(complete=True,files=checksums),indent=2),encoding='utf-8')
        target=self.folder/f'frame_{sequence:04d}';temporary.rename(target)
        return dict(sequence=sequence,source_frame_id=identity,directory=target.name,status='COMPLETE',
            queue_wait_ms=(started-queued_at)*1000,encoding_ms=encoding_ms,file_write_ms=writing_ms,
            total_write_ms=(monotonic()-started)*1000,encoded_bytes=sum(v['bytes'] for v in checksums.values()))
    def _run(self):
        while True:
            with self.cv:
                while not self.queue and not self.closed:self.cv.wait(.2)
                if not self.queue:return
                sequence,identity,frame,queued_at=self.queue.popleft()
            try:
                record=self._write(sequence,identity,frame,queued_at)
                with self.cv:self.saved+=1;self.manifest.append(record)
            except Exception as exc:
                with self.cv:
                    self.error=f'{type(exc).__name__}: {exc}';self.dropped+=1
                    self.drop_events.append(dict(sequence=sequence,source_frame_id=identity,reason='WRITE_FAILED',error=self.error,timestamp=time()))
    def close(self,capture_dropped=0,timeout=30):
        with self.cv:
            if self.finalized is not None:return self.finalized
            self.closed=True;self.cv.notify_all()
        deadline=monotonic()+timeout
        for worker in self.threads:
            worker.join(max(0,deadline-monotonic()))
            if worker.is_alive():raise RuntimeError('Recorder did not stop')
        for name,rows in [('detections.jsonl',self.records),('frame_manifest.jsonl',sorted(self.manifest,key=lambda x:x['sequence'])),('dropped_frames.jsonl',self.drop_events)]:
            with (self.folder/name).open('w',encoding='utf-8') as stream:
                for row in rows:stream.write(json.dumps(row,allow_nan=False)+'\n')
        def stats(key):
            values=[r[key] for r in self.manifest]
            return dict(mean_ms=float(np.mean(values)),p95_ms=float(np.percentile(values,95))) if values else None
        saved_ids={r['source_frame_id'] for r in self.manifest};detected_ids={r['source_frame_id'] for r in self.records}
        summary=dict(offered=self.offered,saved=self.saved,recorder_dropped=self.dropped,
            capture_dropped=capture_dropped,detection_rows=self.detected,
            saved_frames_without_detection=len(saved_ids-detected_ids),saved_frames_without_detection_status='NOT_PROCESSED',
            max_seconds=self.max_seconds,max_frames=self.max_frames,queue_capacity=self.capacity,queue_peak=self.queue_peak,
            writer_workers=self.worker_count,png_compression=self.png_compression,lossless=True,
            bytes_written=sum(r['encoded_bytes'] for r in self.manifest),
            queue_wait=stats('queue_wait_ms'),encoding=stats('encoding_ms'),file_write=stats('file_write_ms'),total_write=stats('total_write_ms'),
            accounting_complete=self.offered==self.saved+self.dropped,
            error=self.error,capture_is_complete_framesets=True,detection_join_key='source_frame_id',
            body_pose_valid=False,ring_pose_valid=False)
        (self.folder/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8');self.finalized=summary
        return summary
