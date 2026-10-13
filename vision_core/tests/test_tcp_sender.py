"""Pure Windows localhost contract tests, never SSH/ROS/camera."""
import copy
import json
import socket
import struct
import threading
import time
from pathlib import Path
import numpy as np
import pytest
from perception.live_vision_protocol import (vision_message, encode_message, validate_message,
    strict_loads, ProtocolError, MAX_PAYLOAD)
from perception.tcp_vision_sender import TCPVisionSender


def core():
    return dict(coordinate_frame='camera_color_optical',
        timestamps=dict(host_utc='2026-10-12T04:05:06.123456+00:00', color_sdk_ms=123.25,
                        color_timestamp_domain='timestamp_domain.hardware_clock'),
        objects=[dict(object_id='battery-1', color_label='red', confidence=.9,
            detection_status='CONFIRMED_OBJECT', ambiguity=['180_DEG_DIRECTION'],
            visible_surface_center_xyz=[.01,.02,.3], surface_depth_z=.3,
            body_center_candidate_xyz=[.01,.02,.335], body_orientation_candidate=np.eye(3).tolist(),
            body_yaw_candidate=0., ring_center_candidate=[.01,.02,.37],
            ring_orientation_candidate=np.eye(3).tolist(), geometry_source='CAD_PREDICTED',
            hook_insertion_direction_candidates=[[1,0,0],[-1,0,0]],
            body_pose_valid=False, ring_pose_valid=False)])


def message(raw=123.25):
    result = vision_message(core(), camera_frame_id='camera_color_optical', namespace='testboot', mode='test')
    result['capture_timestamp_raw'] = raw
    return result


def wait_until(predicate, seconds=3):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        if predicate(): return
        time.sleep(.01)
    assert predicate(), 'Timed out waiting for test event'


class LocalReceiver:
    def __init__(self):
        self.listener = socket.socket(); self.listener.bind(('127.0.0.1',0)); self.listener.listen(2)
        self.port = self.listener.getsockname()[1]; self.listener.settimeout(.1)
        self.stop = threading.Event(); self.messages = []; self.clients = []; self.connection_count = 0
        self.thread = threading.Thread(target=self.run); self.thread.start()
    def read(self, client, count):
        data = b''
        while len(data) < count and not self.stop.is_set():
            try: part = client.recv(min(7,count-len(data)))  # deliberately fragmented reads
            except socket.timeout: continue
            if not part: return None
            data += part
        return data if len(data)==count else None
    def run(self):
        while not self.stop.is_set():
            try: client,_ = self.listener.accept()
            except socket.timeout: continue
            except OSError: return
            self.clients.append(client); self.connection_count += 1; client.settimeout(.1)
            try:
                while not self.stop.is_set():
                    prefix = self.read(client,4)
                    if prefix is None: break
                    length = struct.unpack('!I',prefix)[0]
                    assert 0 < length <= MAX_PAYLOAD
                    data = self.read(client,length)
                    if data is None: break
                    obj = strict_loads(data.decode('utf-8')); validate_message(obj)
                    self.messages.append((self.connection_count,obj))
            except OSError: pass
            finally: client.close()
    def disconnect(self):
        if self.clients:
            try: self.clients[-1].shutdown(socket.SHUT_RDWR)
            except OSError: pass
    def close(self):
        self.stop.set(); self.disconnect(); self.listener.close(); self.thread.join(3)
        assert not self.thread.is_alive()


def test_exact_schema_encoding_units_and_capture_provenance():
    m = message(); wire = encode_message(m)
    assert struct.unpack('!I',wire[:4])[0] == len(wire[4:])
    assert json.loads(wire[4:].decode('utf-8')) == m
    o = m['objects'][0]
    assert o['color']=='RED' and o['object_id']=='testboot-battery-1' and o['detection_status']=='CONFIRMED'
    assert o['body_position_candidate_m'] != core()['objects'][0]['visible_surface_center_xyz']
    assert o['body_orientation_candidate'] == [0,0,0,1]  # conversion of supplied complete rotation, no placeholder
    assert len(o['insertion_direction_candidates'])==2 and o['ambiguity']
    assert m['timestamp_domain']=='hardware_clock' and m['capture_timestamp_raw']==123.25
    assert m['windows_capture_wall_time']['nanosec']==123456000
    assert m['source_frame_id']=='camera_color_optical'
    assert not o['body_pose_valid'] and not o['ring_pose_valid']


@pytest.mark.parametrize('color,value', [('RED',1),('BLUE',2),('YELLOW',3),('GREEN',4),('UNKNOWN',0)])
def test_numeric_core_colors_and_same_color_ids(color,value):
    r=core();r['objects'][0].pop('color_label');r['objects'][0]['color']=value
    second=copy.deepcopy(r['objects'][0]);second['object_id']='battery-2';r['objects'].append(second)
    m=vision_message(r,camera_frame_id='camera_color_optical',namespace='boot',mode='test')
    assert [o['color'] for o in m['objects']]==[color,color]
    assert len({o['object_id'] for o in m['objects']})==2


@pytest.mark.parametrize('state', ['RAW_CANDIDATE','AMBIGUOUS','REJECTED'])
def test_unconfirmed_empty_null_and_unknown_time(state):
    r=core();r['timestamps']={};r['objects']=[dict(object_id='id',color='UNKNOWN',confidence=.1,
        detection_status=state,ambiguity=False)]
    m=vision_message(r,camera_frame_id='camera_color_optical',namespace='boot',mode='test')
    assert m['capture_timestamp_raw'] is None and m['timestamp_domain']=='unknown'
    assert m['windows_capture_wall_time'] is None
    assert m['objects'][0]['body_orientation_candidate'] is None
    assert m['objects'][0]['body_position_candidate_m'] is None
    r['objects']=[]
    assert not vision_message(r,camera_frame_id='camera_color_optical',namespace='boot',mode='test')['objects']


@pytest.mark.parametrize('change', [lambda m:m.update(extra=0),lambda m:m.update(sequence_id=True),
    lambda m:m.update(protocol_version=2),lambda m:m.update(camera_frame_id='color-1'),
    lambda m:m['objects'][0].update(confidence=float('nan')),
    lambda m:m['objects'][0].update(body_position_candidate_m=[0,0,float('inf')]),
    lambda m:m['objects'][0].update(ring_pose_valid=True),
    lambda m:m['objects'][0].update(body_pose_valid=True),
    lambda m:m['objects'][0].update(body_orientation_candidate=[0,0,0,0]),
    lambda m:m['objects'].append(copy.deepcopy(m['objects'][0])),
    lambda m:m.update(capture_timestamp_raw=None,timestamp_domain='hardware_clock'),
    lambda m:m.update(timestamp_domain='simulated',mode='live')])
def test_invalid_fields_rejected(change):
    m=message();change(m)
    with pytest.raises((ProtocolError,ValueError,OverflowError)): encode_message(m)


def test_duplicate_json_keys_and_nonfinite_constants():
    for text in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}'):
        with pytest.raises(ProtocolError): strict_loads(text)


def test_size_limit_byte_count_and_unknown_extras():
    m=message();m['objects']=[dict(m['objects'][0],object_id=str(i)) for i in range(128)]
    # V1 maximum vectors/IDs still fit; custom huge number representation exceeds wire limit.
    m['objects'][0]['body_position_candidate_m']=[1.,2.,3.]
    assert len(encode_message(m)) <= MAX_PAYLOAD+4
    import perception.live_vision_protocol as protocol
    old=protocol.MAX_PAYLOAD
    try:
        protocol.MAX_PAYLOAD=16
        with pytest.raises(ProtocolError,match='PAYLOAD_SIZE'): encode_message(message())
    finally: protocol.MAX_PAYLOAD=old


def test_stale_candidates_removed_and_invalid_pose_input_refused():
    r=core();r['objects'][0]['stale_status']='STALE'
    m=vision_message(r,camera_frame_id='camera_color_optical',namespace='boot',mode='test')
    assert m['objects'][0]['stale'] and m['objects'][0]['body_position_candidate_m'] is None
    assert m['objects'][0]['ring_position_candidate_m'] is None
    r['objects'][0]['ring_pose_valid']=True
    with pytest.raises(ProtocolError): vision_message(r,camera_frame_id='camera_color_optical',namespace='boot',mode='test')


@pytest.mark.skipif(not (Path(__file__).resolve().parents[1]/'data/rgbd/session_20261011_143230_red_top').is_dir(), reason='Private Windows RGB-D session unavailable on Ubuntu')
def test_real_saved_result_conversion_no_camera():
    root=Path(__file__).resolve().parents[1]
    folders=sorted((root/'data/rgbd/session_20261011_143230_red_top').glob('sample_*'))
    assert folders
    from real_data.rgbd_recording import load_rgbd_sample
    from perception.dual_rate_rgbd import FastPerception
    frame=load_rgbd_sample(folders[0]);result,_=FastPerception().process(frame,'real_frame')
    m=vision_message(result,camera_frame_id='camera_color_optical',namespace='realtest',mode='test',timestamps=frame.timestamps)
    assert m['capture_timestamp_raw']==frame.timestamps['color_sdk_ms']
    assert m['windows_capture_wall_time'] is not None and m['objects']
    encode_message(m)


def test_latest_slot_superseded_snapshot_and_expiry():
    sender=TCPVisionSender(max_pending_age_s=.02)
    for raw in range(20): assert sender.offer(message(raw))
    assert sender.snapshot()['pending']==1 and sender.snapshot()['superseded']==19
    r=LocalReceiver()
    try:
        sender.port=r.port;time.sleep(.04);sender.start()
        wait_until(lambda: sender.snapshot()['queue_old']==1)
        assert not r.messages
    finally: sender.close();r.close()
    assert sender.sock is None and not sender.thread.is_alive()


def test_tcp_order_disconnect_reconnect_and_resource_cleanup():
    r=LocalReceiver();sender=TCPVisionSender(port=r.port,reconnect_delay_s=.02)
    try:
        sender.start()
        for raw in (10.,11.,12.,13.):
            before=len(r.messages);assert sender.offer(message(raw))
            wait_until(lambda: len(r.messages)>before)
        assert [m['sequence_id'] for _,m in r.messages]==[0,1,2,3]
        assert sender.offer(message(13.))
        wait_until(lambda: sender.snapshot()['order_dropped']==1)
        r.disconnect();wait_until(lambda: not sender.snapshot()['connected'])
        assert sender.offer(message(1.))  # new connection may reset the raw clock
        wait_until(lambda: len(r.messages)==5)
        assert r.messages[-1][1]['sequence_id']==0 and r.messages[-1][0]==2
    finally: sender.close();r.close()
    sender.close()
    assert not sender.thread.is_alive() and sender.sock is None
    assert not sender.offer(message(20))


def test_unavailable_network_does_not_raise_and_shutdown_interrupts_retry():
    probe=socket.socket();probe.bind(('127.0.0.1',0));port=probe.getsockname()[1];probe.close()
    sender=TCPVisionSender(port=port,connect_timeout_s=.1,reconnect_delay_s=.02)
    sender.start()
    try:
        assert sender.offer(message())
        wait_until(lambda: sender.snapshot()['connection_errors']>=1)
        invalid=message();invalid['objects'][0]['confidence']=float('nan')
        assert not sender.offer(invalid) and sender.snapshot()['invalid']==1
    finally: sender.close()
    assert not sender.thread.is_alive() and sender.sock is None


def test_source_change_opens_new_session():
    r=LocalReceiver();sender=TCPVisionSender(port=r.port,reconnect_delay_s=.02)
    try:
        sender.start();sender.offer(message(5));wait_until(lambda:len(r.messages)==1)
        m=message(6);m['timestamp_domain']='system_time';sender.offer(m)
        wait_until(lambda:len(r.messages)==2)
        assert r.messages[-1][0]==2 and r.messages[-1][1]['sequence_id']==0
    finally:sender.close();r.close()


def test_slow_socket_keeps_ui_nonblocking_and_only_newest_pending(monkeypatch):
    from types import SimpleNamespace
    import perception.tcp_vision_sender as module
    entered=threading.Event();release=threading.Event();wires=[]
    class SlowSocket:
        def settimeout(self,value): pass
        def connect(self,address): assert address[0]=='127.0.0.1'
        def sendall(self,wire):
            if not wires:
                entered.set(); assert release.wait(2)
            wires.append(strict_loads(wire[4:].decode()))
        def shutdown(self,how): release.set()
        def close(self): pass
    monkeypatch.setattr(module,'socket',SimpleNamespace(AF_INET=2,SOCK_STREAM=1,SHUT_RDWR=2,socket=lambda *a:SlowSocket()))
    monkeypatch.setattr(module,'select',SimpleNamespace(select=lambda *a:([],[],[])))
    sender=TCPVisionSender(max_pending_age_s=2.)
    try:
        sender.start();sender.offer(message(0));assert entered.wait(2)
        start=time.monotonic()
        for raw in range(1,20):sender.offer(message(raw))
        assert time.monotonic()-start < .5
        assert sender.snapshot()['superseded']==18 and sender.snapshot()['pending']==1
        release.set();wait_until(lambda:len(wires)==2)
        assert [m['capture_timestamp_raw'] for m in wires]==[0,19]
        assert [m['sequence_id'] for m in wires]==[0,1]
    finally:sender.close()


def test_cli_defaults_no_socket_and_no_camera(tmp_path,monkeypatch):
    from scripts import send_vision_tcp as app
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('Dry run opened a socket'))
    path=tmp_path/'demo.jsonl'
    assert app.main(['--output',str(path)])==0
    rows=[strict_loads(line) for line in path.read_text().splitlines()]
    assert len(rows)==4 and all(m['mode']=='test' for m in rows)
    assert [m['sequence_id'] for m in rows]==list(range(4))


def test_preview_tcp_opt_in_requires_human_camera_authorization(monkeypatch):
    from scripts import live_rgbd_preview as app
    monkeypatch.setattr(app.RealSenseRGBDSource,'start',lambda *a:pytest.fail('Unauthorized camera'))
    with pytest.raises(SystemExit):app.main(['--full-vision','--dual-rate','--tcp-send'])
    with pytest.raises(SystemExit):app.main([])


def test_invalid_or_missing_orientation_and_mixed_units():
    r=core();r['objects'][0]['body_orientation_candidate']=None
    m=vision_message(r,camera_frame_id='camera_color_optical',namespace='boot',mode='test')
    assert m['objects'][0]['body_orientation_candidate'] is None
    r['objects'][0]['unit_position']='mm'
    with pytest.raises(ProtocolError,match='UNITS'):
        vision_message(r,camera_frame_id='camera_color_optical',namespace='boot',mode='test')


@pytest.mark.skipif(not (Path(__file__).resolve().parents[1]/'data/rgbd/session_20261011_143230_red_top').is_dir(), reason='Private Windows RGB-D session unavailable on Ubuntu')
def test_optional_dual_rate_sender_replays_real_frame_without_camera(tmp_path,monkeypatch):
    from scripts import live_rgbd_preview as app
    root=Path(__file__).resolve().parents[1]
    session=root/'data/rgbd/session_20261011_143230_red_top'
    receiver=LocalReceiver()
    config=tmp_path/'tcp.yaml';config.write_text('port: '+str(receiver.port)+'\n')
    monkeypatch.setattr(app.RealSenseRGBDSource,'start',lambda *a:pytest.fail('Replay started camera'))
    try:
        app.main(['--full-vision','--dual-rate','--tcp-send','--tcp-config',str(config),
                  '--replay',str(session),'--headless','--max-frames','1',
                  '--replay-output',str(tmp_path/'replay')])
        wait_until(lambda: len(receiver.messages)==1)
        report=json.loads((tmp_path/'replay/summary.json').read_text())
        assert not report['actual_camera_started'] and report['tcp_sender']['sent']==1
        m=receiver.messages[0][1]
        assert m['mode']=='test' and m['sequence_id']==0
        assert m['source_frame_id']=='camera_color_optical'
        from real_data.rgbd_recording import load_rgbd_sample
        frame=load_rgbd_sample(sorted(session.glob('sample_*'))[0])
        assert m['capture_timestamp_raw']==frame.timestamps['color_sdk_ms']
        assert not any(o['body_pose_valid'] or o['ring_pose_valid'] for o in m['objects'])
    finally:receiver.close()


def test_invalid_core_inputs_do_not_escape_sender():
    s=TCPVisionSender()
    assert not s.offer_vision([],mode='test')
    assert not s.offer_vision(dict(objects=[None]),mode='test')
    assert s.snapshot()['invalid']==2
    s.close()
