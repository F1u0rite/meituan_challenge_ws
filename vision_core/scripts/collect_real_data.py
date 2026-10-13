"""Human-triggered D435i preview and bounded, session-based BGR collection."""
import argparse
import sys
import uuid
from datetime import datetime
from pathlib import Path
from time import monotonic
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from input.realsense_source import RealSenseSource
from real_data.dataset import create_session, save_sample
from real_data.quality import diagnose, load_quality_config

KEY_LABELS = {ord('1'): 'red', ord('2'): 'yellow', ord('3'): 'blue',
              ord('4'): 'green', ord('0'): 'negative'}


def planned_session_id(label):
    return f'session_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}_{label}'


def draw_preview(frame, label, mode, session_id, fps, quality, camera, message):
    view = frame.copy()
    lines = [f'PREVIEW / HUMAN TRIGGER  label={label.upper()}  mode={mode}',
             f'session={session_id or "new on first save"}  FPS={fps:.1f}',
             f'exposure={camera["exposure"]} gain={camera["gain"]} auto={camera["auto_exposure_enabled"]}',
             f'quality={quality["status"]} Vmean={quality["metrics"]["value_mean"]:.1f} '
             f'dark={quality["metrics"]["low_value_fraction"]:.2f} '
             f'blur={quality["metrics"]["laplacian_variance"]:.1f}',
             '1-4 color  0 negative  s single save  r bounded record  q quit', message]
    for index, line in enumerate(lines):
        y = 30 + index*31
        cv2.putText(view, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(view, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    (0, 255, 255) if index == 3 and quality['status'] != 'ACCEPTED' else (255, 255, 255),
                    2, cv2.LINE_AA)
    return view


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fps', type=int, default=15)
    p.add_argument('--serial')
    p.add_argument('--label', choices=('red', 'yellow', 'blue', 'green', 'negative'), default='red')
    p.add_argument('--mode', choices=('static', 'handheld'), default='static')
    p.add_argument('--data-root', type=Path, default=Path(__file__).resolve().parents[1]/'data'/'real')
    p.add_argument('--quality-config', type=Path, default=Path(__file__).resolve().parents[1]/'config'/'real_quality.yaml')
    p.add_argument('--countdown', type=float, default=0, help='Seconds between save/record key and first saved frame')
    p.add_argument('--max-frames', type=int, default=60)
    p.add_argument('--max-seconds', type=float, default=20)
    args = p.parse_args()
    if args.countdown < 0 or args.max_frames < 1 or args.max_seconds <= 0:
        p.error('countdown must be >=0, max-frames >=1, max-seconds >0')
    config = load_quality_config(args.quality_config)
    label = args.label
    session_id = planned_session_id(label)
    session = None
    recording = False
    pending_single = False
    start_at = 0.0
    saved_count = 0
    prior = monotonic()
    print('Preview starts now. No image is saved until you press s or r.')
    with RealSenseSource(fps=args.fps, serial=args.serial) as source:
        try:
            while True:
                frame = source.read()
                now = monotonic()
                fps = 1/max(now-prior, 1e-6)
                prior = now
                quality = diagnose(frame.image, config)
                camera = source.camera_metadata()
                active = (recording or pending_single) and now >= start_at
                message = ''
                if (recording or pending_single) and not active:
                    message = f'COUNTDOWN {start_at-now:.1f}s'
                if recording and (saved_count >= args.max_frames or now-start_at >= args.max_seconds):
                    recording = False
                    message = f'RECORD LIMIT REACHED ({saved_count} frames)'
                if active and (recording or pending_single):
                    if session is None:
                        session = create_session(args.data_root, label, args.mode, session_id=session_id)
                        print(f'Session {session.name}; label={label}. Raw frames are immutable.')
                    overlay = draw_preview(frame.image, label, args.mode, session.name,
                                           fps, quality, camera, 'SAVED BY USER TRIGGER')
                    folder, metadata = save_sample(session, frame.image, timestamp=frame.timestamp,
                                                   frame_number=frame.frame_number,
                                                   camera_metadata=camera, quality_config=config, overlay=overlay)
                    print(f'{folder.name}: {metadata["image_quality"]["status"]}')
                    saved_count += 1
                    pending_single = False
                    if saved_count >= args.max_frames:
                        recording = False
                view = draw_preview(frame.image, label, args.mode, session_id,
                                    fps, quality, camera, message)
                cv2.imshow('D435i REAL DATA PREVIEW', view)
                key = cv2.waitKey(1) & 0xff
                if key == ord('q'):
                    break
                if key in KEY_LABELS:
                    recording = pending_single = False
                    label = KEY_LABELS[key]
                    session = None
                    session_id = planned_session_id(label)
                    saved_count = 0
                    print(f'Prepare {label.upper()} object/background; inspect preview, then press s or r to save.')
                elif key == ord('s'):
                    recording = False
                    pending_single = True
                    start_at = monotonic() + args.countdown
                elif key == ord('r'):
                    if recording:
                        recording = False
                        print(f'Recording stopped at {saved_count} samples.')
                    else:
                        pending_single = False
                        recording = True
                        saved_count = 0
                        start_at = monotonic() + args.countdown
                        print(f'Recording armed: <= {args.max_frames} frames, <= {args.max_seconds}s.')
        finally:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
