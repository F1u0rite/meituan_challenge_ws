"""Rigid calibration from independent fixture points; no battery prediction input."""
import sys,json,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from cad_geometry.frames import validate_transform,transform_points
from real_data.pose_experiment import write_json,now
def calibrate(data,*,max_heldout_error_mm,operator_confirmed=False):
    camera=np.asarray(data['camera_points_m'],float);fixture=np.asarray(data['fixture_points_m'],float)
    if camera.ndim!=2 or camera.shape!=fixture.shape or camera.shape[1]!=3 or len(camera)<4 or not np.isfinite(camera).all() or not np.isfinite(fixture).all():
        raise ValueError('Need >=4 paired finite calibration points in m')
    if np.linalg.matrix_rank(camera-camera.mean(axis=0),tol=1e-7)<2:raise ValueError('Collinear calibration points')
    A=camera-camera.mean(axis=0);B=fixture-fixture.mean(axis=0)
    U,_,Vt=np.linalg.svd(A.T@B);R=Vt.T@U.T
    if np.linalg.det(R)<0:Vt[-1]*=-1;R=Vt.T@U.T
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=fixture.mean(axis=0)-R@camera.mean(axis=0);validate_transform(T)
    heldcam=np.asarray(data.get('heldout_camera_points_m'),float);heldgt=np.asarray(data.get('heldout_fixture_points_m'),float)
    errors=None
    if heldcam.ndim==2 and heldcam.shape==heldgt.shape and heldcam.shape[1]==3 and len(heldcam)>=2:
        if not np.isfinite(heldcam).all() or not np.isfinite(heldgt).all():raise ValueError('Invalid heldout points')
        # Heldout coordinates must differ from calibration coordinates.
        if any(np.min(np.linalg.norm(camera-p,axis=1))<1e-7 for p in heldcam):raise ValueError('Calibration/heldout overlap')
        errors=np.linalg.norm(transform_points(heldcam,T)-heldgt,axis=1)*1000
    uncertainty=data.get('measurement_uncertainty_mm')
    verified=bool(operator_confirmed and errors is not None and errors.max()<=max_heldout_error_mm and
        data.get('source') and isinstance(uncertainty,(int,float)) and np.isfinite(uncertainty) and uncertainty>0)
    return {'from_frame':'camera_color_optical','to_frame':'fixture','unit':'m','T_fixture_from_camera_m':T.tolist(),
        'source':data.get('source'),'status':'independently_verified' if verified else 'calibration_candidate',
        'timestamp':now(),'heldout_errors_mm':errors.tolist() if errors is not None else None,
        'uncertainty':{'reference_measurement_mm':uncertainty,'noise_model':'NOT_VALIDATED'},
        'same_camera_depth_correlation':data.get('same_camera_depth_correlation',True),
        'verification_scope':'HELDOUT_FIXTURE_POINTS_AND_OPERATOR_REFERENCE_CONFIRMATION; NOT_FULL_NOISE_VALIDATION',
        'robot_execution_allowed':False}
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--points',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--max-heldout-error-mm',type=float,required=True);p.add_argument('--confirm-reference-measured',action='store_true')
    a=p.parse_args()
    if a.output.exists():p.error('Output exists; retain history and choose a new file')
    write_json(a.output,calibrate(json.loads(a.points.read_text(encoding='utf-8-sig')),
        max_heldout_error_mm=a.max_heldout_error_mm,operator_confirmed=a.confirm_reference_measured))
