"""Bounded buffering and worker lifetime using real localhost sockets."""
import json
import socket
import struct
import time
from mtc_vision_adapter.live_transport import TCPReceiver
from live_test_support import frame


def test_fast_sender_cannot_create_unbounded_history():
    receiver=TCPReceiver(0)
    try:
        with socket.create_connection(('127.0.0.1',receiver.port),timeout=2.) as client:
            for sequence in range(100):
                payload=json.dumps(frame(sequence)).encode()
                client.sendall(struct.pack('!I',len(payload))+payload)
            deadline=time.monotonic()+3.
            while receiver.stats().get('queue_superseded',0)<99:
                assert time.monotonic()<deadline
                time.sleep(.01)
            assert receiver.pending.qsize()==1
            assert receiver.latest()['message']['sequence_id']==99
            assert receiver.stats()['queue_superseded']==99
    finally:
        receiver.close()
    assert not receiver.thread.is_alive()


def test_shutdown_unblocks_partial_header():
    receiver=TCPReceiver(0)
    client=socket.create_connection(('127.0.0.1',receiver.port),timeout=2.)
    try:
        client.sendall(b'\x00')
        deadline=time.monotonic()+2.
        while receiver.stats().get('connections',0)<1:
            assert time.monotonic()<deadline
            time.sleep(.01)
        receiver.close()
        assert not receiver.thread.is_alive()
    finally:
        client.close()
        if receiver.thread.is_alive():receiver.close()
