"""Native ROS mapping keeps detection freshness independent from CAD pose state."""
import math
import pytest
from mtc_interfaces.msg import BatteryDetectionArray, BatteryDetection
from mtc_vision_adapter.native_rgbd import native_array
from mtc_vision_adapter.core import to_ros


def confirmed(**changes):
    item=dict(object_id='red',color='red',confidence=.9,detection_status='CONFIRMED_OBJECT',
              detection_ambiguity=False,detection_stale=False,body_pose_valid=False,ring_pose_valid=False)
    item.update(changes)
    return dict(coordinate_frame='camera_color_optical',objects=[item])


@pytest.mark.parametrize('changes',[
    dict(cad_stale=True,stale_status='STALE'),
    dict(pose_ambiguity=True,pose_ambiguity_reasons=['180_DEG_DIRECTION']),
    dict(cad_stale=True,pose_ambiguity=True,stale_status='STALE'),
])
def test_fresh_confirmed_detection_survives_old_or_ambiguous_cad(changes):
    data=native_array(confirmed(**changes))
    message=to_ros(data,BatteryDetectionArray,BatteryDetection)
    assert len(message.detections)==1
    item=message.detections[0]
    assert item.object_id=='red' and not item.ring_pose_valid
    for pose in (item.body_pose,item.ring_pose):
        assert all(math.isnan(x) for x in (pose.pose.position.x,pose.pose.position.y,
            pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,
            pose.pose.orientation.z,pose.pose.orientation.w))


@pytest.mark.parametrize('changes',[
    dict(detection_stale=True),dict(detection_stale=None),dict(detection_ambiguity=True),
    dict(detection_status='AMBIGUOUS'),dict(detection_status='REJECTED'),
])
def test_stale_unknown_or_unconfirmed_detection_is_not_published(changes):
    assert native_array(confirmed(**changes))['detections']==[]


def test_legacy_stale_mapping_remains_rejected():
    result=confirmed(stale_status='STALE')
    item=result['objects'][0]
    del item['detection_stale'];del item['detection_ambiguity']
    assert native_array(result)['detections']==[]


@pytest.mark.parametrize('changes',[
    dict(ambiguity=['180_DEG_DIRECTION']),
    dict(cad_stale=True,stale_status='STALE',ambiguity=[]),
    dict(ambiguity=['NO_INDEPENDENT_3D_VALIDATION']),
])
def test_new_encoder_is_compatible_with_unchanged_ubuntu_bridge(changes):
    import json
    from perception.vision_interface import finalize_interface
    from perception.live_vision_protocol import vision_message
    from mtc_vision_adapter.live_protocol import decode,diagnostic_array
    result=dict(result_timestamp=10.1,capture_timestamp=10,source_frame_id='current',
        capture_timestamp_source='HOST_UTC',coordinate_frame='camera_color_optical',
        timestamps=dict(host_wall_ns=10000000000,color_sdk_ms=100.,color_timestamp_domain='hardware_clock'),
        objects=[dict(object_id='A',color_label='red',confidence=.9,
            candidate_state='CONFIRMED_OBJECT',confirmation=dict(reasons=[]),**changes)])
    wire=vision_message(finalize_interface(result),camera_frame_id='camera_color_optical',namespace='test',mode='test')
    data=diagnostic_array(decode(json.dumps(wire).encode()))
    assert len(data['detections'])==1
    assert data['detections'][0]['object_id']=='test-A'
    assert not data['detections'][0]['ring_pose_valid']
    for name in ('body_pose','ring_pose'):
        assert all(v is None for group in data['detections'][0][name]['pose'].values() for v in group.values())
