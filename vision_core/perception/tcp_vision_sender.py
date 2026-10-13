"""Opt-in loopback TCP client, one pending latest frame, no camera dependencies."""
import json
import select
import socket
import threading
import time
import uuid
from .live_vision_protocol import encode_message, vision_message, ProtocolError, MAX_SEQUENCE, FRAME_PATTERN


class TCPVisionSender:
    def __init__(self, *, port=8765, connect_timeout_s=1., send_timeout_s=.5,
                 reconnect_delay_s=.25, max_pending_age_s=.5, camera_frame_id='camera_color_optical',
                 namespace=None):
        for value in (connect_timeout_s, send_timeout_s, reconnect_delay_s, max_pending_age_s):
            if type(value) not in (int, float) or not 0 < value < float('inf'):
                raise ValueError('TCP time limits must be positive finite numbers')
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('Invalid local port')
        if type(camera_frame_id) is not str or FRAME_PATTERN.fullmatch(camera_frame_id) is None:
            raise ValueError('Invalid declared capture-camera optical frame ID')
        self.port = port; self.connect_timeout = connect_timeout_s; self.send_timeout = send_timeout_s
        self.reconnect_delay = reconnect_delay_s; self.max_age = max_pending_age_s
        self.camera_frame_id = camera_frame_id; self.namespace = namespace or ('boot'+uuid.uuid4().hex[:12])
        self.cv = threading.Condition(); self.stop = threading.Event(); self.pending = None
        self.sock = None; self.thread = None; self.started_at = None
        self.stats = dict(offered=0, sent=0, superseded=0, queue_old=0, invalid=0,
                          order_dropped=0, failed_frames=0, connections=0, connection_errors=0,
                          discarded_on_close=0, last_error=None, connected=False)

    def start(self):
        with self.cv:
            if self.stop.is_set() or self.thread is not None:
                raise RuntimeError('Sender cannot be restarted')
            self.started_at = time.monotonic()
            self.thread = threading.Thread(target=self._run, name='vision-tcp-sender')
            self.thread.start()

    def _count(self, key, amount=1, error=None):
        with self.cv:
            self.stats[key] += amount
            if error: self.stats['last_error'] = error

    def offer_vision(self, result, *, mode, timestamps=None):
        try:
            message = vision_message(result, camera_frame_id=self.camera_frame_id,
                                     namespace=self.namespace, mode=mode, timestamps=timestamps)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self._count('invalid', error=f'{type(exc).__name__}: {exc}'); return False
        return self.offer(message)

    def offer(self, message):
        """No network calls on the producer/UI thread. Take an independent snapshot."""
        try:
            encode_message(message)
            snapshot = json.loads(json.dumps(message, allow_nan=False))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self._count('invalid', error=f'{type(exc).__name__}: {exc}'); return False
        with self.cv:
            if self.stop.is_set(): return False
            self.stats['offered'] += 1
            if self.pending is not None: self.stats['superseded'] += 1
            self.pending = (snapshot, time.monotonic()); self.cv.notify_all()
        return True

    def _disconnect(self):
        with self.cv:
            sock, self.sock = self.sock, None; self.stats['connected'] = False
        if sock:
            try: sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            sock.close()

    def _run(self):
        sequence = 0; identity = None; previous_raw = None
        try:
            while not self.stop.is_set():
                with self.cv:
                    if self.pending is None: self.cv.wait(.05)
                    available = self.pending is not None
                    sock = self.sock
                if self.stop.is_set(): break
                if sock:
                    try:
                        readable, _, _ = select.select([sock], [], [], 0)
                        if readable and sock.recv(1, socket.MSG_PEEK) == b'':
                            raise ConnectionResetError('Receiver EOF')
                    except (OSError, ValueError) as exc:
                        self._count('connection_errors', error=str(exc)); self._disconnect(); continue
                if not available: continue
                if sock is None:
                    try:
                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        sock.settimeout(self.connect_timeout)
                        with self.cv: self.sock = sock  # allows close() to interrupt connect
                        sock.connect(('127.0.0.1', self.port)); sock.settimeout(self.send_timeout)
                        with self.cv: self.stats['connected'] = True
                        self._count('connections'); sequence = 0; identity = None; previous_raw = None
                    except OSError as exc:
                        self._count('connection_errors', error=str(exc)); self._disconnect()
                        self.stop.wait(self.reconnect_delay); continue
                with self.cv:
                    packet, self.pending = self.pending, None
                if packet is None: continue
                message, queued_at = packet
                if time.monotonic()-queued_at > self.max_age:
                    self._count('queue_old'); continue
                current = tuple(message[k] for k in ('mode', 'camera_frame_id', 'source_frame_id', 'timestamp_domain'))
                if identity is not None and current != identity:
                    # Change sources only on a new session; retain only the newest unsent frame.
                    self._disconnect()
                    with self.cv:
                        if self.pending is None: self.pending = packet
                        else: self.stats['superseded'] += 1
                    continue
                raw = message['capture_timestamp_raw']
                if raw is not None and previous_raw is not None and raw <= previous_raw:
                    self._count('order_dropped'); continue
                if sequence > MAX_SEQUENCE:
                    self._count('failed_frames', error='SEQUENCE_EXHAUSTED'); self._disconnect(); continue
                message['sequence_id'] = sequence
                try:
                    sock.sendall(encode_message(message))
                except (OSError, ProtocolError) as exc:
                    self._count('failed_frames'); self._count('connection_errors', error=str(exc))
                    self._disconnect(); self.stop.wait(self.reconnect_delay); continue
                sequence += 1; identity = current; previous_raw = raw
                self._count('sent')
        finally:
            self._disconnect()

    def snapshot(self):
        with self.cv:
            report = dict(self.stats, pending=int(self.pending is not None))
        elapsed = time.monotonic()-self.started_at if self.started_at is not None else 0
        report['send_rate_hz'] = report['sent']/elapsed if elapsed > 0 else None
        report['dropped_frames'] = sum(report[k] for k in ('superseded', 'queue_old', 'order_dropped',
                                                         'failed_frames', 'discarded_on_close'))
        report['ros_delivery_verified'] = False
        report['end_to_end_capture_latency'] = 'NOT VERIFIED'
        return report

    def close(self, timeout=3.):
        self.stop.set()
        with self.cv:
            if self.pending is not None: self.stats['discarded_on_close'] += 1
            self.pending = None; self.cv.notify_all()
        self._disconnect()
        if self.thread:
            self.thread.join(timeout)
            if self.thread.is_alive(): raise RuntimeError('TCP sender shutdown timeout')
