import json
import sys
import types
from unittest.mock import Mock
import pytest
from mtc_vision_adapter.publisher import main

class ExternalShutdownException(Exception):pass

@pytest.fixture
def fake_ros(monkeypatch,tmp_path):
    path=tmp_path/'replay.jsonl';path.write_text(json.dumps({'mode':'synthetic'})+'\n')
    node=Mock();node.get_parameter.side_effect=lambda key:types.SimpleNamespace(value=str(path) if key=='input_jsonl' else 1.)
    ros=types.ModuleType('rclpy');ros.init=Mock();ros.spin=Mock();ros.try_shutdown=Mock()
    monkeypatch.setitem(sys.modules,'rclpy',ros)
    monkeypatch.setitem(sys.modules,'rclpy.node',types.SimpleNamespace(Node=Mock(return_value=node)))
    monkeypatch.setitem(sys.modules,'rclpy.executors',types.SimpleNamespace(ExternalShutdownException=ExternalShutdownException))
    monkeypatch.setitem(sys.modules,'rclpy.qos',types.SimpleNamespace(QoSProfile=lambda **kw:kw,ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1),HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),DurabilityPolicy=types.SimpleNamespace(VOLATILE=1)))
    monkeypatch.setitem(sys.modules,'mtc_interfaces.msg',types.SimpleNamespace(BatteryDetectionArray=object,BatteryDetection=object))
    return ros,node,path

@pytest.mark.parametrize('error',[KeyboardInterrupt,ExternalShutdownException,None])
def test_signal_or_normal_spin_cleanup(fake_ros,error):
    ros,node,_=fake_ros
    if error:ros.spin.side_effect=error()
    assert main() is None
    node.destroy_node.assert_called_once();ros.try_shutdown.assert_called_once()

def test_invalid_input_still_cleans_resources(fake_ros):
    ros,node,path=fake_ros;path.write_text('{"mode":"live"}\n')
    with pytest.raises(ValueError,match='explicit replay'):main()
    node.destroy_node.assert_called_once();ros.try_shutdown.assert_called_once()

def test_destroy_error_still_shuts_context(fake_ros):
    ros,node,_=fake_ros;node.destroy_node.side_effect=RuntimeError('destroy failed')
    with pytest.raises(RuntimeError,match='destroy failed'):main()
    ros.try_shutdown.assert_called_once()
