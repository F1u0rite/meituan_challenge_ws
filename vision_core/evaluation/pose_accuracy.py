"""Independent manual GT, explicit frame conversion and diagnostic accuracy."""
from collections import Counter,defaultdict
import numpy as np
from cad_geometry.frames import validate_transform,periodic_error
def _finite(values):return np.isfinite(np.asarray(values,float)).all()
def validate_gt(gt):
    if not gt:return 'GT_MISSING'
    if not isinstance(gt,dict):return 'GT_INVALID'
    required=['xy_mm','body_yaw_deg','measurement_source','measurement_resolution_mm','xy_uncertainty_mm','yaw_uncertainty_deg']
    if any(gt.get(k) is None for k in required):return 'GT_INCOMPLETE'
    if not isinstance(gt.get('measurement_source'),str) or not gt['measurement_source'].strip():return 'GT_INCOMPLETE'
    if not gt.get('independently_measured') or gt.get('status')!='human_measured_confirmed':return 'GT_NOT_VERIFIED'
    if gt.get('coordinate_frame')!='fixture' or gt.get('unit')!='mm':return 'GT_FRAME_OR_UNIT_INVALID'
    if gt.get('reference_point')!='body_center_projection':return 'GT_REFERENCE_POINT_MISMATCH'
    try:
        if len(gt['xy_mm'])!=2 or not _finite([*gt['xy_mm'],gt['body_yaw_deg'],gt['measurement_resolution_mm'],gt['xy_uncertainty_mm'],gt['yaw_uncertainty_deg']]):return 'GT_INVALID'
        if min(gt['measurement_resolution_mm'],gt['xy_uncertainty_mm'],gt['yaw_uncertainty_deg'])<=0:return 'GT_UNCERTAINTY_UNAVAILABLE'
    except (TypeError,ValueError):return 'GT_INVALID'
    return None
def validate_calibration(calibration):
    if not calibration:return 'FRAME_TRANSFORM_UNAVAILABLE'
    if not isinstance(calibration,dict):return 'FRAME_TRANSFORM_INVALID'
    if calibration.get('T_fixture_from_camera_m') is None:return 'FRAME_TRANSFORM_UNAVAILABLE'
    if (calibration.get('status')!='independently_verified' or calibration.get('from_frame')!='camera_color_optical' or
        calibration.get('to_frame')!='fixture' or calibration.get('unit')!='m' or not calibration.get('source')):
        return 'CALIBRATION_NOT_VERIFIED'
    try:validate_transform(calibration['T_fixture_from_camera_m'])
    except (ValueError,TypeError):return 'FRAME_TRANSFORM_INVALID'
    return None
def compare_pose(prediction,gt,calibration,*,xy_limit_mm=1,yaw_limit_deg=1):
    result={'status':'NOT_MEASURED','reasons':[],'xy_error_mm':None,'yaw_error_deg':None,
            'body_pose_valid':False,'ring_pose_valid':False,'robot_execution_allowed':False}
    if not isinstance(prediction,dict) or not prediction or prediction.get('detected') is False:
        result['reasons']=['NOT_DETECTED'];return result
    if prediction.get('depth_valid') is not True:
        result['reasons']=['INVALID_DEPTH'];return result
    if prediction.get('body_pose_candidate') is None:
        result['reasons']=['POSE_UNAVAILABLE'];return result
    errors=[e for e in [validate_gt(gt),validate_calibration(calibration)] if e]
    if prediction.get('coordinate_frame')!='camera_color_optical':errors.append('PREDICTION_FRAME_INVALID')
    if prediction.get('unit')!='m':errors.append('PREDICTION_UNIT_INVALID')
    if prediction.get('reference_point','body_geometry_center')!='body_geometry_center':errors.append('PREDICTION_REFERENCE_POINT_MISMATCH')
    if isinstance(gt,dict) and gt.get('body_center_z_mm') is not None:
        if not _finite([gt['body_center_z_mm']]):errors.append('GT_Z_INVALID')
        if not isinstance(gt.get('z_uncertainty_mm'),(int,float)) or not np.isfinite(gt['z_uncertainty_mm']) or gt['z_uncertainty_mm']<=0:errors.append('GT_Z_UNCERTAINTY_UNAVAILABLE')
    if errors:result['reasons']=errors;return result
    try:T=validate_transform(calibration['T_fixture_from_camera_m'])@validate_transform(prediction['body_pose_candidate'])
    except (ValueError,TypeError):result['reasons']=['PREDICTION_POSE_INVALID'];return result
    result['prediction_fixture_xyz_mm']=(T[:3,3]*1000).tolist()
    result['z_error_mm']=None if gt.get('body_center_z_mm') is None else float(T[2,3]*1000-gt['body_center_z_mm'])
    result['z_accuracy']='NOT_MEASURED' if result['z_error_mm'] is None else 'DIAGNOSTIC'
    axis=T[:3,0]
    if np.linalg.norm(axis[:2])<1e-6:result['reasons']=['YAW_PROJECTION_INVALID'];return result
    xy=T[:2,3]*1000;diff=xy-np.array(gt['xy_mm'])
    yaw=float(np.degrees(np.arctan2(axis[1],axis[0]))%180)
    yaw_error=periodic_error(yaw,gt['body_yaw_deg'])
    insertion_error=None
    if gt.get('insertion_yaw_deg') is not None and prediction.get('insertion_direction_resolved') is True:
        direction=prediction.get('insertion_direction_camera')
        if direction is not None:
            axis=np.array(calibration['T_fixture_from_camera_m'])[:3,:3]@np.asarray(direction)
            angle=np.degrees(np.arctan2(axis[1],axis[0]))
            insertion_error=float((angle-gt['insertion_yaw_deg']+180)%360-180)
    if prediction.get('multiple_solutions'):result['reasons'].append('POSE_MULTIPLE_SOLUTIONS')
    if not prediction.get('insertion_direction_resolved'):result['reasons'].append('INSERTION_DIRECTION_AMBIGUITY')
    if np.linalg.norm(diff)>xy_limit_mm or abs(yaw_error)>yaw_limit_deg:result['reasons'].append('ACCURACY_TARGET_EXCEEDED')
    result.update(status='DIAGNOSTIC_ACCURACY',prediction_fixture_xy_mm=xy.tolist(),
        prediction_fixture_body_yaw_deg=yaw,x_error_mm=float(diff[0]),y_error_mm=float(diff[1]),
        xy_error_mm=float(np.linalg.norm(diff)),yaw_error_deg=float(yaw_error),abs_yaw_error_deg=abs(float(yaw_error)),
        insertion_direction_error_360_deg=insertion_error,
        insertion_direction_accuracy='NOT_MEASURED' if insertion_error is None else 'DIAGNOSTIC',
        gt_reference_uncertainty={'xy_mm':gt['xy_uncertainty_mm'],'yaw_deg':gt['yaw_uncertainty_deg']},
        calibration_source=calibration['source'],calibration_uncertainty=calibration.get('uncertainty','UNAVAILABLE'))
    return result
def statistics(values):
    if not values:return None
    a=np.asarray(values,float)
    return {'mean':float(a.mean()),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())}
def circular_bias_std(angles,period=180):
    a=np.radians(np.asarray(angles)*360/period);c=np.mean(np.cos(a));s=np.mean(np.sin(a))
    r=min(1.,float(np.hypot(c,s)))
    return {'bias_deg':float(np.degrees(np.arctan2(s,c))*period/360),
            'circular_std_deg':float(np.sqrt(max(0,-2*np.log(max(r,1e-15))))*period/(2*np.pi))}
def aggregate(records):
    measured=[r for r in records if r['evaluation']['status']=='DIAGNOSTIC_ACCURACY']
    result={'total_samples':len(records),'measured_samples':len(measured),'not_measured_samples':len(records)-len(measured),
        'failures':dict(Counter(reason for r in records for reason in r['evaluation']['reasons'])),
        'position_accuracy':'NOT_MEASURED' if not measured else 'DIAGNOSTIC_WITH_REFERENCE_UNCERTAINTY',
        'yaw_accuracy':'NOT_MEASURED' if not measured else 'DIAGNOSTIC_WITH_REFERENCE_UNCERTAINTY',
        'body_pose_valid':False,'ring_pose_valid':False,'robot_execution_allowed':False}
    if measured:
        for key in ['xy_error_mm','abs_yaw_error_deg']:
            result[key]=statistics([r['evaluation'][key] for r in measured])
        z=[r['evaluation']['z_error_mm'] for r in measured if r['evaluation'].get('z_error_mm') is not None]
        result['z_accuracy']='NOT_MEASURED' if not z else 'DIAGNOSTIC_WITH_REFERENCE_UNCERTAINTY'
        result['z_abs_error_mm']=statistics([abs(v) for v in z])
        result['xy_bias_mm']={axis:float(np.mean([r['evaluation'][axis+'_error_mm'] for r in measured])) for axis in ['x','y']}
        result['yaw_signed_error_deg']=statistics([r['evaluation']['yaw_error_deg'] for r in measured])
        result['x_abs_error_mm']=statistics([abs(r['evaluation']['x_error_mm']) for r in measured])
        result['y_abs_error_mm']=statistics([abs(r['evaluation']['y_error_mm']) for r in measured])
        result['yaw_bias_and_spread']=circular_bias_std([r['evaluation']['yaw_error_deg'] for r in measured])
        result['x_signed_error_mm']=statistics([r['evaluation']['x_error_mm'] for r in measured])
        result['y_signed_error_mm']=statistics([r['evaluation']['y_error_mm'] for r in measured])
        groups=defaultdict(list)
        for r in measured:groups[r.get('case_id','unknown')].append(r)
        repeat={}
        for name,group in groups.items():
            if len(group)<2 or not all(r.get('relative_static_confirmed') is True for r in group):
                repeat[name]={'status':'NOT_MEASURED','reason':'STATIC_RELATIVE_POSE_UNCONFIRMED_OR_TOO_FEW'};continue
            xy=np.array([r['evaluation']['prediction_fixture_xy_mm'] for r in group])
            repeat[name]={'status':'REPEATABILITY_ONLY','count':len(group),'xy_std_mm':np.std(xy,axis=0,ddof=1).tolist(),
                'yaw_circular_std_deg':circular_bias_std([r['evaluation']['prediction_fixture_body_yaw_deg'] for r in group])['circular_std_deg']}
        result['repeatability']=repeat
    return result
