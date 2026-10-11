"""Visual hypothesis confirmation is independent of track confirmation and pose validity."""
import numpy as np
import pytest
from perception.dual_rate_rgbd import FastPerception

@pytest.mark.parametrize('color',['red','blue','yellow','green'])
def test_same_cad_confirmation_for_all_colors(color):
 f=FastPerception();f.body_dimensions=dict(long=80,width=70,height=50);f.table_hint={'plane':dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5])}
 x,y=np.meshgrid(np.linspace(-.04,.04,20),np.linspace(-.035,.035,20))
 points=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,.45)])
 p=dict(points=points,color=dict(color_label=color,color_confidence=.9),contour_fill=.95,depth_valid_ratio=1.)
 assert f.confirmation_evidence(p,True)['eligible']
 assert not f.confirmation_evidence(p,False)['eligible']
 p['color']['color_label']='unknown'
 assert 'COLOR_UNCERTAIN' in f.confirmation_evidence(p,True)['reasons']
 p['color']['color_label']=color;p['contour_fill']=.4
 assert 'INCOMPLETE_CONTOUR_SUPPORT' in f.confirmation_evidence(p,True)['reasons']


def test_small_background_footprint_not_confirmed():
 f=FastPerception();f.body_dimensions=dict(long=80,width=70,height=50);f.table_hint={'plane':dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5])}
 x,y=np.meshgrid(np.linspace(-.02,.02,20),np.linspace(-.012,.012,20))
 p=dict(points=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,.45)]),color=dict(color_label='blue'),contour_fill=.95,depth_valid_ratio=1.)
 assert 'INSUFFICIENT_NOMINAL_BODY_FOOTPRINT' in f.confirmation_evidence(p,True)['reasons']


def test_motion_interval_after_table_capture_only():
 f=FastPerception();f.set_table_hint(dict(normal_camera_xyz=[0,0,1],point_camera_m=[0,0,.5]),1,'source')
 f.motion_history.extend([(1,False),(1.1,True)])
 assert f._hint_usable(1.1,dict(stable=True))
 f.motion_history.append((1.2,False))
 assert not f._hint_usable(1.3,dict(stable=True))
 assert not f._hint_usable(4,dict(stable=True))
