"""Continuous straight insertion certification using complete STEP solids."""
import numpy as np
from . import step_backend as occ
from .frames import validate_transform
def millimetres(T):
    T=validate_transform(T).copy();T[:3,3]*=1000
    return T
def normalized_solids(data,battery,hook):
    B=occ.transform_shape(battery,millimetres(data['battery']['T_battery_from_cad_m']))
    H=occ.transform_shape(hook,millimetres(data['hook']['T_hook_tcp_from_cad_m']))
    return B,H
def hook_transform(data,x_mm,lateral_mm=0,vertical_mm=0,yaw_deg=0):
    T=np.asarray(data['battery']['T_battery_from_ring_m']).copy()
    angle=np.radians(yaw_deg);c,s=np.cos(angle),np.sin(angle)
    local=np.eye(4);local[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
    local[:3,3]=np.array([x_mm,lateral_mm,vertical_mm])/1000
    return T@local
def insertion_path(data,battery,hook,*,start_mm=-25,end_mm=30,lateral_mm=0,vertical_mm=0,yaw_deg=0,resolution_mm=.25):
    """Gap is 1-Lipschitz under translation. Midpoint gap > half interval
    certifies every pose in that interval. Unresolved intervals => UNKNOWN."""
    if end_mm<=start_mm or resolution_mm<=0:raise ValueError('Invalid path')
    cache={}
    def check(x):
        if x not in cache:
            moved=occ.transform_shape(hook,millimetres(hook_transform(data,x,lateral_mm,vertical_mm,yaw_deg)))
            cache[x]=occ.interference(battery,moved)|{'x_mm':x}
        return cache[x]
    unresolved=[];collision=None;stack=[(start_mm,end_mm)]
    for x in [start_mm,end_mm]:
        result=check(x)
        if result['status']=='COLLISION':collision=result;break
    while stack and collision is None:
        a,b=stack.pop();mid=(a+b)/2;r=check(mid)
        if r['status']=='COLLISION':collision=r;break
        if r['status']=='NOMINAL_CLEARANCE' and r['distance_mm']>(b-a)/2+1e-5:continue
        if b-a<=resolution_mm:
            unresolved.append([a,b]);continue
        stack.extend([(a,mid),(mid,b)])
    status='COLLISION' if collision else ('UNKNOWN' if unresolved else 'NOMINAL_CLEARANCE')
    distances=[r.get('distance_mm',float('inf')) for r in cache.values()]
    return {'status':status,'start_mm':start_mm,'end_mm':end_mm,'lateral_mm':lateral_mm,
            'vertical_mm':vertical_mm,'yaw_deg':yaw_deg,'evaluated_poses':len(cache),
            'minimum_sampled_gap_mm':min(distances),'collision':collision,
            'unresolved_intervals_mm':unresolved,'method':'FULL_BREP_DISTANCE_BOOLEAN_ADAPTIVE_LIPSCHITZ',
            'real_world_safety':'NOT_VALIDATED','path_kind':'STRAIGHT_FIXED_ORIENTATION_CAD_ONLY'}


class HookGeometryValidator:
    """Stateful offline wrapper around the exact complete-solid validator."""
    def __init__(self,geometry,battery_solid,hook_solid):
        self.geometry=geometry
        self.battery,self.hook=normalized_solids(geometry,battery_solid,hook_solid)
    def evaluate_path(self,**parameters):
        result=insertion_path(self.geometry,self.battery,self.hook,**parameters)
        result['cad_sources']={name:self.geometry[name]['source'] for name in ['battery','hook']}
        return result
