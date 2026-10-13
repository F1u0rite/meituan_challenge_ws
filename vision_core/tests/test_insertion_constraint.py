"""Analytic constraints and read-only CAD tests; no device or private assets."""
import copy
import hashlib
import json
import numpy as np
import pytest
import cv2
from types import SimpleNamespace
from scipy.spatial.transform import Rotation
from perception.insertion_constraint import select_insertion_direction
from perception.simplified_rgbd import SimplifiedPerception
from test_top_bar_pose import geometry,rendered
from datetime import datetime,timezone
from time import time


def reference(vector,**changes):
    value=dict(source_frame_id='frame1',capture_timestamp=100.,
        coordinate_frame='camera_color_optical',robot_to_battery_camera=vector,
        source='ANALYTIC_CURRENT_FRAME_STATION')
    value.update(changes)
    return value


def supported(axis):
    return dict(top_geometry_valid=True,ring_center_geometry_valid=True,
        insertion_direction_candidates=[axis.tolist(),(-axis).tolist()],
        ring_center_candidate=[.01,.02,.4],direction_unique=False,
        body_pose_valid=False,ring_pose_valid=False,motion_usable=False,
        reasons=['INSERTION_SIDE_180_DEG_AMBIGUITY','NO_INDEPENDENT_POSITION_ACCURACY_VALIDATION'])


@pytest.mark.parametrize('angles',[(0,0,0),(.4,-.7,.3),(-.8,.5,1.4)])
@pytest.mark.parametrize('reverse',[False,True])
def test_same_physical_direction_under_camera_rotation_and_candidate_order(angles,reverse):
    axis=Rotation.from_euler('xyz',angles).apply([0,1,0])
    top=supported(axis);before=copy.deepcopy(top)
    if reverse:top['insertion_direction_candidates'].reverse()
    out=select_insertion_direction(top,reference((axis*3).tolist()),'frame1',100.)
    assert out['direction_unique'] and np.allclose(out['insertion_direction_selected'],axis)
    assert out['ring_center_candidate']==before['ring_center_candidate']
    assert not out['motion_usable'] and not out['ring_pose_valid']
    assert 'NO_INDEPENDENT_POSITION_ACCURACY_VALIDATION' in out['reasons']
    assert not top['direction_unique']


@pytest.mark.parametrize('changes,reason',[
    ({'source_frame_id':'old'},'INSERTION_REFERENCE_FRAME_MISMATCH'),
    ({'capture_timestamp':99.},'INSERTION_REFERENCE_TIME_MISMATCH'),
    ({'coordinate_frame':'base_link'},'INSERTION_REFERENCE_COORDINATE_FRAME_MISMATCH'),
    ({'source':''},'INSERTION_REFERENCE_SOURCE_REQUIRED'),
    ({'robot_to_battery_camera':[0,0,0]},'INVALID_INSERTION_REFERENCE'),
    ({'robot_to_battery_camera':[0,float('nan'),0]},'INVALID_INSERTION_REFERENCE'),
    ({'robot_to_battery_camera':[1,0,0]},'ROBOT_SIDE_REFERENCE_NOT_ALIGNED_WITH_HOLE_AXIS'),
])
def test_invalid_or_uninformative_reference_never_selects_direction(changes,reason):
    out=select_insertion_direction(supported(np.array([0.,1.,0.])),
        reference([0,1,0],**changes),'frame1',100.)
    assert not out['direction_unique'] and out['insertion_direction_selected'] is None
    assert out['direction_selection']['reason']==reason


def test_missing_reference_and_rejected_geometry_stay_unresolved():
    top=supported(np.array([0.,1.,0.]))
    assert not select_insertion_direction(top,None,'frame1',100.)['direction_unique']
    top['ring_center_geometry_valid']=False
    assert not select_insertion_direction(top,reference([0,1,0]),'frame1',100.)['direction_unique']


def test_external_cad_is_read_only_and_missing_explicit_path_fails(tmp_path):
    cad=tmp_path/'geometry.json';cad.write_text(json.dumps(geometry()))
    before=cad.read_bytes();pipeline=SimplifiedPerception(geometry_path=cad)
    assert cad.read_bytes()==before
    assert pipeline.top.spec is not None
    assert pipeline.geometry_provenance['sha256']==hashlib.sha256(before).hexdigest()
    with pytest.raises(FileNotFoundError):SimplifiedPerception(geometry_path=tmp_path/'missing.json')
    cad.write_text('{}')
    with pytest.raises(ValueError):SimplifiedPerception(geometry_path=cad)


def test_pipeline_propagates_selected_axis_without_authorizing_pose(tmp_path,monkeypatch):
    cad=tmp_path/'geometry.json';cad.write_text(json.dumps(geometry()))
    p=SimplifiedPerception(geometry_path=cad)
    f,mask=rendered(40.);stamp=time()
    # This analytic fixture renders disconnected faces; inject its known object
    # mask at detection input so the integration test exercises real geometry,
    # timing, reference selection and result propagation rather than RGB admission.
    yy,xx=np.where(mask)
    contour=cv2.convexHull(np.column_stack((xx,yy)).astype(np.int32))
    detection=SimpleNamespace(color='red',contour=contour,mask=mask,
        confidence=1.,pixel_center=[float(xx.mean()),float(yy.mean())])
    monkeypatch.setattr(p.detection.detector,'detect_batteries',lambda *a,**k:[detection])
    f.timestamps=dict(host_utc=datetime.fromtimestamp(stamp,timezone.utc).isoformat(),
        color_sdk_ms=stamp*1000,depth_sdk_ms=stamp*1000,
        color_frame_number=1,depth_frame_number=1,
        color_timestamp_domain='global_time',depth_timestamp_domain='global_time')
    axis=[0,float(np.cos(np.deg2rad(40))),float(-np.sin(np.deg2rad(40)))]
    current_reference=reference(axis,capture_timestamp=stamp)
    out,_=p.process(f,'frame1',insertion_reference=current_reference)
    chosen=[o for o in out['objects'] if o['insertion_direction_unique']]
    assert chosen
    for o in chosen:
        assert np.dot(o['insertion_direction_selected'],axis)>.99
        assert not o['ring_pose_valid'] and not o['motion_usable']
    # Even when source_frame_id is accidentally reused, a different timestamp
    # must not reuse the old camera reference.
    f.timestamps.update(host_utc=datetime.fromtimestamp(stamp+1.,timezone.utc).isoformat(),
        color_sdk_ms=(stamp+1)*1000,depth_sdk_ms=(stamp+1)*1000,color_frame_number=2,depth_frame_number=2)
    out,_=p.process(f,'frame1',insertion_reference=current_reference)
    assert all(not o['insertion_direction_unique'] for o in out['objects'])


def test_inconsistent_local_aperture_invalidates_selected_direction():
    top=supported(np.array([0.,1.,0.]))
    first=select_insertion_direction(top,reference([0,1,0]),'frame1',100.)
    assert first['direction_unique']
    first['ring_center_geometry_valid']=False
    rejected=select_insertion_direction(first,reference([0,1,0]),'frame1',100.)
    assert not rejected['direction_unique'] and rejected['insertion_direction_selected'] is None
