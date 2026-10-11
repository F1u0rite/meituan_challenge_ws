"""Geometry extraction from the actual solid faces and aperture sections."""
from pathlib import Path
import json
import numpy as np
from . import step_backend as occ

def _T(rows,origin_mm):
    R=np.asarray(rows,float);origin=np.asarray(origin_mm,float)
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=-R@origin/1000
    if not np.isclose(np.linalg.det(R),1):raise ValueError('Improper CAD frame')
    return T

def analyze_cad(battery_path,hook_path):
    battery,bmeta=occ.load_step(battery_path)
    hook,hmeta=occ.load_step(hook_path)
    bf=occ.planar_faces(battery);hf=occ.planar_faces(hook)
    # This feature matcher is deliberately restricted to this axis-aligned CAD
    # family. Its inputs are measured faces; mismatch fails instead of guessing.
    horizontal=[f for f in bf if abs(f['normal'][2])>.999 and f['area_mm2']>2000]
    if len(horizontal)!=2:raise ValueError('Battery body delimiting face pair not uniquely identifiable')
    z_values=sorted(f['location_mm'][2] for f in horizontal)
    body_low,body_high=z_values
    body=occ.clip_box(battery,[-500,-500,body_low+1e-5],[500,500,body_high-1e-5])
    bb=occ.bounds(body);overall=bmeta['bbox_mm']
    if not overall[2]<body_low:raise ValueError('Ring does not extend opposite the body')
    body_origin=[(bb[0]+bb[3])/2,(bb[1]+bb[4])/2,(body_low+body_high)/2]
    body_dims=np.array(bb[3:])-np.array(bb[:3]);body_dims[2]=body_high-body_low
    if body_dims[1]<=body_dims[0]:raise ValueError('Body long axis is not CAD +Y')
    Tb=_T([[0,1,0],[1,0,0],[0,0,-1]],body_origin)
    holes=occ.section_holes(battery,1,(bb[1]+bb[4])/2)
    holes=[h for h in holes if h['bbox_mm'][5]<=body_low+1e-4]
    if len(holes)!=1:raise ValueError('Ring aperture not uniquely found in solid section')
    aperture=holes[0];ab=aperture['bbox_mm']
    ring_side=[f for f in bf if abs(f['normal'][1])>.999 and f['bbox_mm'][2]<body_low-1
               and f['bbox_mm'][5]<body_low+1e-4]
    ring_y=sorted(set(round(f['location_mm'][1],6) for f in ring_side))
    if len(ring_y)!=2:raise ValueError('Ring thickness faces not uniquely identifiable')
    ring_center=np.asarray(aperture['centroid_mm'])
    ring_in_body=(Tb[:3,:3]@ring_center/1000+Tb[:3,3])
    Tbr=np.eye(4);Tbr[:3,3]=ring_in_body
    sections=[]
    for y in np.linspace(ring_y[0]+1,ring_y[1]-1,7):
        for hole in occ.section_holes(battery,1,float(y)):
            if hole['bbox_mm'][5]<=body_low+1e-4:
                sections.append({'cad_y_mm':float(y),**hole})
    hb=hmeta['bbox_mm']
    beam_faces=[f for f in hf if abs(f['normal'][2])>.999 and f['area_mm2']>1000 and f['location_mm'][2]>0]
    if len(beam_faces)!=1:raise ValueError('Hook shaft/tongue separation face not identified')
    beam_z=beam_faces[0]['location_mm'][2]
    distal=occ.clip_box(hook,[-500,-500,beam_z+1e-4],[500,500,hb[5]+1])
    distal_bounds=occ.bounds(distal)
    tip=occ.clip_box(hook,[-500,-500,hb[5]-5],[500,500,hb[5]+1])
    tip_bounds=occ.bounds(tip)
    distal=occ.clip_box(hook,[-500,tip_bounds[1],beam_z+1e-4],[500,500,hb[5]+1])
    distal_bounds=occ.bounds(distal)
    tcp_origin=[(tip_bounds[0]+tip_bounds[3])/2,(tip_bounds[1]+tip_bounds[4])/2,hb[5]]
    Th=_T([[0,0,1],[-1,0,0],[0,-1,0]],tcp_origin)
    battery_data={'source':bmeta,'planar_faces':bf,'body_dimensions_mm':{
        'long':float(body_dims[1]),'width':float(body_dims[0]),'height':float(body_dims[2])},
        'overall_height_mm':float(overall[5]-overall[2]),
        'ring_protrusion_mm':float(body_low-overall[2]),'ring_thickness_mm':ring_y[1]-ring_y[0],
        'aperture':{'width_mm':ab[3]-ab[0],'height_mm':ab[5]-ab[2],
                    'central_section':aperture,'sections':sections,
                    'normal_cad':[0,1,0],'ring_center_cad_mm':ring_center.tolist()},
        'T_battery_from_cad_m':Tb.tolist(),'T_battery_from_ring_m':Tbr.tolist(),
        'frame_definition':{'origin':'body bounding-box centre','X':'CAD +Y, body long axis',
                            'Y':'CAD +X, body width / ring horizontal axis','Z':'CAD -Z, up from body toward ring'},
        'body_yaw_axis_battery':[1,0,0],'ring_axis_battery':[0,1,0],
        'ring_insertion_axis_candidates_battery':[[1,0,0],[-1,0,0]],
        'frame_units':'m','body_to_ring_rotation_det':float(np.linalg.det(Tbr[:3,:3]))}
    hook_data={'source':hmeta,'planar_faces':hf,'overall_dimensions_cad_mm':(np.array(hb[3:])-hb[:3]).tolist(),
        'distal_bounds_cad_mm':distal_bounds,'tip_bounds_cad_mm':tip_bounds,
        'entering_envelope_mm':{'width':distal_bounds[3]-distal_bounds[0],
                                'height':distal_bounds[4]-distal_bounds[1]},
        'beam_positive_z_face_mm':beam_z,'tcp_origin_cad_mm':tcp_origin,
        'T_hook_tcp_from_cad_m':Th.tolist(),
        'frame_definition':{'origin':'leading tip section centre at maximum CAD Z',
                            'X':'CAD +Z, proposed insertion direction','Y':'CAD -X, width',
                            'Z':'CAD -Y, upward toward flange'},
        'tcp_status':'CAD_NOMINAL_ONLY; TOOL0_TCP_NOT_CALIBRATED'}
    return {'battery':battery_data,'hook':hook_data,
            'calibration_status':{'camera_to_tool0':'NOT_CALIBRATED','tool0_to_hook_tcp':'NOT_CALIBRATED',
                                  'camera_to_base':'NOT_CALIBRATED'}},battery,hook

def normalized_mesh(shape,T_m):
    triangles=occ.mesh(shape)/1000
    return triangles@np.asarray(T_m)[:3,:3].T+np.asarray(T_m)[:3,3]

def save_analysis(data,battery,hook,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    (output/'geometry.json').write_text(json.dumps(data,indent=2,ensure_ascii=False),encoding='utf-8')
    np.savez_compressed(output/'cad_meshes.npz',
        battery=normalized_mesh(battery,data['battery']['T_battery_from_cad_m']),
        hook=normalized_mesh(hook,data['hook']['T_hook_tcp_from_cad_m']))
