"""Bounded localhost-only TCP framing worker; no ROS or control operations."""
from collections import Counter
import queue
import socket
import struct
import threading
import time

from .live_protocol import MAX_BYTES, ProtocolError, Session, decode


class TCPReceiver:
    def __init__(self, port=8765, max_age_s=5., max_future_s=1.):
        self.max_age_s, self.max_future_s = max_age_s, max_future_s
        self.pending = queue.Queue(maxsize=1)
        self.counts = Counter()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.client = None
        self.listener = socket.socket()
        try:
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener.bind(('127.0.0.1', port))
            self.listener.listen(1)
            self.listener.settimeout(.2)
        except BaseException:
            self.listener.close()
            raise
        self.port = self.listener.getsockname()[1]
        self.thread = threading.Thread(target=self._run, name='vision_tcp', daemon=True)
        self.thread.start()

    def count(self, key, amount=1):
        with self.lock:
            self.counts[key] += amount

    def stats(self):
        with self.lock:
            return dict(self.counts)

    def latest(self):
        try:
            return self.pending.get_nowait()
        except queue.Empty:
            return None

    def _enqueue(self, item):
        try:
            self.pending.put_nowait(item)
        except queue.Full:
            self.latest()
            self.count('queue_superseded')
            self.pending.put_nowait(item)

    def _read(self, conn, size, idle=False):
        data = bytearray()
        deadline = None if idle else time.monotonic()+2.
        while len(data) < size and not self.stop.is_set():
            if deadline is not None and time.monotonic() > deadline:
                raise ProtocolError('PARTIAL_TIMEOUT')
            try:
                chunk = conn.recv(size-len(data))
            except socket.timeout:
                continue
            if not chunk:
                if data:
                    raise ProtocolError('TRUNCATED')
                raise EOFError
            if deadline is None:
                deadline = time.monotonic()+2.
            data.extend(chunk)
        if self.stop.is_set():
            raise EOFError
        return bytes(data)

    def _run(self):
        while not self.stop.is_set():
            try:
                conn, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.client = conn
            self.count('connections')
            session = Session()
            conn.settimeout(.2)
            try:
                while not self.stop.is_set():
                    size = struct.unpack('!I', self._read(conn, 4, idle=True))[0]
                    if size == 0 or size > MAX_BYTES:
                        self.count('framing_LENGTH')
                        break
                    payload = self._read(conn, size)
                    receive_wall_ns, receive_mono_ns = time.time_ns(), time.monotonic_ns()
                    self.count('received')
                    try:
                        message = decode(payload)
                    except ProtocolError as error:
                        self.count('invalid_'+str(error))
                        continue
                    reason = session.observe(message, receive_wall_ns,
                                             self.max_age_s, self.max_future_s)
                    if reason:
                        self.count('dropped_'+reason)
                    else:
                        self.count('validated')
                    self._enqueue(dict(message=message, reason=reason,
                                       receive_wall_ns=receive_wall_ns,
                                       receive_monotonic_ns=receive_mono_ns))
            except ProtocolError as error:
                self.count('framing_'+str(error))
            except (OSError, EOFError):
                pass
            finally:
                conn.close()
                self.client = None
                self.count('disconnects')

    def close(self):
        self.stop.set()
        self.listener.close()
        client = self.client
        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(timeout=3.)
        if self.thread.is_alive():
            raise RuntimeError('TCP worker did not stop')
