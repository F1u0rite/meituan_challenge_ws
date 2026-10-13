"""Experimental CAD fit to observed camera points. Original detector untouched."""
from dataclasses import dataclass,asdict
import numpy as np
import cv2
from time import perf_counter
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from cad_geometry.frames import validate_transform,transform_points,axis_yaw
@dataclass
class CADPoseCandidate:
    coordinate_frame:str='camera_color_optical'
    pose_source:str='CAD_MODEL_FIT_CANDIDATE'
    body_pose:object=None
    ring_pose:object=None
    body_yaw:object=None
    ring_axis_yaw:object=None
    insertion_yaw:object=None
    body_pose_valid:bool=False
    ring_pose_valid:bool=False
    direction_ambiguity_deg:int=180
    calibration_status:str='CAMERA_BASE_AND_TOOL0_TCP_NOT_CALIBRATED'
    reason:str='NO_INDEPENDENT_3D_VALIDATION'
    def to_dict(self):return asdict(self)
class RingPoseEstimator:
    def __init__(self,geometry):self.geometry=geometry
    def estimate(self,T_camera_body,body_support=False,observation_quality=None,calibration_status=None):
        if T_camera_body is None:return CADPoseCandidate(reason='NO_BODY_CANDIDATE').to_dict()
        T=validate_transform(T_camera_body)
        ring=T@validate_transform(self.geometry['battery']['T_battery_from_ring_m'])
        x=T[:3,0];y=T[:3,1]
        result=CADPoseCandidate(body_pose=T.tolist(),ring_pose=ring.tolist(),
            body_yaw=axis_yaw(x),ring_axis_yaw=axis_yaw(y),
            insertion_yaw=[float(np.degrees(np.arctan2(x[1],x[0]))%360),
                           float((np.degrees(np.arctan2(x[1],x[0]))+180)%360)]).to_dict()
        result.update(insertion_direction_candidates_camera=[x.tolist(),(-x).tolist()],
            ring_center_source='BODY_FIT_PLUS_NOMINAL_CAD_TRANSFORM',
            ring_observed=False,body_geometric_support=bool(body_support),
            yaw_reference='AXIS_PROJECTED_IN_CAMERA_OPTICAL_XY; NOT_TABLE_OR_BASE_YAW')
        result.update(body_pose_candidate=result['body_pose'],ring_pose_candidate=result['ring_pose'],
            hook_insertion_yaw=result['insertion_yaw'],invalid_reason=result['reason'],
            ring_axis_direction_camera=y.tolist(),quality_metrics=observation_quality or {},
            geometry_source=self.geometry['battery']['source'])
        if calibration_status is not None:result['calibration_status']=calibration_status
        return result
def ring_surface_evidence(points_camera,T_camera_body,geometry):
    """Measure elevated surface support; do not infer a measured aperture pose."""
    info=dict(ring_pose_valid=False,ring_aperture_observed=False,
        ring_center_source='BODY_FIT_PLUS_NOMINAL_CAD_TRANSFORM',
        reason='NO_INDEPENDENT_APERTURE_BOUNDARIES_OR_ASSEMBLY_CALIBRATION')
    points=np.asarray(points_camera,float)
    if T_camera_body is None or points.ndim!=2 or points.shape[1]!=3 or not len(points):
        return dict(info,surface_observed=False,point_count=0,support_status='NO_BODY_OR_POINTS')
    T=validate_transform(T_camera_body);local=(points-T[:3,3])@T[:3,:3]
    battery=geometry['battery'];body_top=battery['body_dimensions_mm']['height']/2000
    top=battery['overall_height_mm']/1000-body_top
    elevated=(local[:,2]>body_top+.005)&(local[:,2]<=top+.015)
    elevated_points=points[elevated];q=local[elevated]
    info.update(point_count=len(q),surface_observed=False)
    if len(q)<100:return dict(info,support_status='INSUFFICIENT_ELEVATED_SURFACE')
    covariance=np.cov(q[:,:2].T);eigen,axes=np.linalg.eigh(covariance)
    ratio=float(eigen[-1]/max(eigen[-2],1e-12));axis=T[:3,:2]@axes[:,-1]
    axis/=max(np.linalg.norm(axis),1e-12)
    gap=float(np.rad2deg(np.arccos(np.clip(abs(np.dot(axis,T[:3,1])),0,1))))
    span=np.percentile(q[:,:2],95,axis=0)-np.percentile(q[:,:2],5,axis=0)
    expected=np.array([battery['ring_thickness_mm'],battery['body_dimensions_mm']['width']])/1000
    support=bool(ratio>=2. and (span>=expected*.5).all() and (span<=expected*1.6).all() and gap<=15.)
    info.update(surface_observed=support,axis_eigenvalue_ratio=ratio,
        observed_surface_center_xyz_m=np.median(elevated_points,axis=0).tolist(),
        observed_surface_axis_candidates_camera=[axis.tolist(),(-axis).tolist()],
        observed_surface_span_body_xy_m=span.tolist(),body_axis_disagreement_deg=gap,
        support_status='PARTIAL_ELEVATED_SURFACE_SUPPORT' if support else 'RING_SURFACE_SHAPE_OR_AXIS_INCONSISTENT',
        limitations=['SURFACE_CENTROID_IS_NOT_APERTURE_CENTER','180_DEG_AXIS_AMBIGUITY','BODY_FIT_DEPENDENT_SELECTION'])
    return info

def sample_surface(triangles,count=60000,seed=2026):
    triangles=np.asarray(triangles)
    areas=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)/2
    if areas.sum()<=0:raise ValueError('Empty mesh surface')
    rng=np.random.default_rng(seed);tri=triangles[rng.choice(len(triangles),count,p=areas/areas.sum())]
    uv=rng.random((count,2));flip=uv.sum(axis=1)>1;uv[flip]=1-uv[flip]
    return tri[:,0]+uv[:,0,None]*(tri[:,1]-tri[:,0])+uv[:,1,None]*(tri[:,2]-tri[:,0])
def fit_body(points,plane,geometry,surface_m,*,max_points=2500,max_nfev=100,surface_tree=None,analytic_jacobian=False,validation_points=None,use_body_boundary=False,use_ring_axis=False):
    points=np.asarray(points,float)
    if plane is None or points.ndim!=2 or points.shape[1]!=3 or len(points)<100 or not np.isfinite(points).all():
        return {'fit_status':'INVALID_INPUT','pose':RingPoseEstimator(geometry).estimate(None)}
    n=np.array(plane['normal_camera_xyz'],float);n/=np.linalg.norm(n);up=-n
    o=np.array(plane['point_camera_m'])
    ex=np.array([1.,0.,0.])-up*up[0];ex/=np.linalg.norm(ex);ey=np.cross(up,ex)
    basis=np.column_stack([ex,ey,up])
    local=(points-o)@basis
    bodyh=geometry['battery']['body_dimensions_mm']['height']/1000
    bodyband=(local[:,2]>.012)&(local[:,2]<bodyh+.005)
    if bodyband.sum()<100:
        return {'fit_status':'INSUFFICIENT_BODY_BAND','pose':RingPoseEstimator(geometry).estimate(None)}
    xy=local[bodyband,:2];rect=cv2.minAreaRect(xy.astype(np.float32))
    box=cv2.boxPoints(rect);edges=np.roll(box,-1,axis=0)-box
    axis=edges[np.argmax(np.linalg.norm(edges,axis=1))];theta=np.arctan2(axis[1],axis[0])
    boundary=None;boundary_info=dict(enabled=False,reason='NOT_REQUESTED')
    if use_body_boundary:
        dims=geometry['battery']['body_dimensions_mm']
        nominal=np.array([dims['long'],dims['width']],float)/1000
        span=np.sort(np.asarray(rect[1]))[::-1]
        hull=cv2.convexHull(xy.astype(np.float32)).reshape(-1,2)
        solidity=float(cv2.contourArea(hull)/max(np.prod(rect[1]),1e-12))
        boundary_info=dict(enabled=False,reason='INCOMPLETE_OR_NONRECTANGULAR_BODY_BAND',
            observed_span_m=span.tolist(),nominal_span_m=np.sort(nominal)[::-1].tolist(),solidity=solidity)
        if (span>=np.sort(nominal)[::-1]*.7).all() and (span<=np.sort(nominal)[::-1]*1.25).all() and solidity>=.8:
            pieces=[]
            for a,b in zip(hull,np.roll(hull,-1,axis=0)):
                count=max(2,int(np.ceil(np.linalg.norm(b-a)/.002)))
                pieces.append(a+(b-a)*np.linspace(0,1,count,endpoint=False)[:,None])
            boundary=np.concatenate(pieces)
            boundary=boundary[np.linspace(0,len(boundary)-1,min(160,len(boundary)),dtype=int)]
            half=nominal/2
            model_edge=np.concatenate([np.column_stack((np.linspace(-half[0],half[0],20),np.full(20,sign*half[1]))) for sign in (-1,1)]+
                                     [np.column_stack((np.full(20,sign*half[0]),np.linspace(-half[1],half[1],20))) for sign in (-1,1)])
            boundary_tree=cKDTree(boundary)
            boundary_info.update(enabled=True,reason=None,observed_boundary_points=len(boundary),
                source='VISIBLE_BODY_BAND_CONVEX_HULL_WITH_CAD_RECTANGLE_PRIOR',
                occlusion_and_shape_assumption='FULL_VISIBLE_BODY_OUTLINE_HYPOTHESIS; NOT_INDEPENDENT_GT')
    ring_axis_info=dict(enabled=False,reason='NOT_REQUESTED')
    ring_angle=None;ring_lever=geometry['battery']['body_dimensions_mm']['width']/2000
    if use_ring_axis:
        elevated=local[(local[:,2]>bodyh+.005)&(local[:,2]<geometry['battery']['overall_height_mm']/1000+.010)]
        ring_axis_info=dict(enabled=False,reason='INSUFFICIENT_OR_INCONSISTENT_RING_SURFACE',point_count=len(elevated))
        if len(elevated)>=100:
            eigen,axes=np.linalg.eigh(np.cov(elevated[:,:2].T));ratio=float(eigen[-1]/max(eigen[-2],1e-12))
            ring_rect=cv2.minAreaRect(elevated[:,:2].astype(np.float32));span=np.sort(np.asarray(ring_rect[1]))[::-1]
            expected=np.sort(np.array([geometry['battery']['body_dimensions_mm']['width'],geometry['battery']['ring_thickness_mm']])/1000)[::-1]
            if ratio>=2 and (span>=expected*.5).all() and (span<=expected*1.6).all():
                ring_angle=float(np.arctan2(axes[1,-1],axes[0,-1]))
                ring_axis_info.update(enabled=True,reason=None,axis_eigenvalue_ratio=ratio,observed_span_m=span.tolist(),
                    measured_axis_camera=(basis[:,:2]@axes[:,-1]).tolist(),direction_ambiguity_deg=180,
                    source='PCA_OF_MEASURED_ELEVATED_SURFACE; NOT_MEASURED_APERTURE_POSE')
    selected=local[np.linspace(0,len(local)-1,min(max_points,len(local)),dtype=int)]
    tree=surface_tree if surface_tree is not None else cKDTree(surface_m)
    def rotation(yaw):
        c,s=np.cos(yaw),np.sin(yaw)
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    cache={}
    def evaluate(p):
        if 'p' not in cache or not np.array_equal(cache['p'],p):
            R=rotation(p[3]);v=selected-p[:3];inv=v@R
            distance,index=tree.query(inv);delta=inv-tree.data[index]
            unit=np.divide(delta,distance[:,None],out=np.zeros_like(delta),where=distance[:,None]>1e-12)
            c,s=np.cos(p[3]),np.sin(p[3])
            derivative=np.array([[-s,-c,0],[c,-s,0],[0,0,0]])
            jac=np.column_stack((-unit@R.T,np.sum(unit*(v@derivative),axis=1)))
            cache.update(p=p.copy(),distance=distance,jac=jac)
        return cache
    def boundary_terms(p):
        if boundary is None:return np.empty(0),np.empty((0,4))
        R=rotation(p[3])[:2,:2];c,s=np.cos(p[3]),np.sin(p[3])
        derivative=np.array([[-s,-c],[c,-s]])
        v=boundary-p[:2];inv=v@R;q=np.abs(inv)-half
        outside=np.maximum(q,0);norm=np.linalg.norm(outside,axis=1)
        signed=norm+np.minimum(np.max(q,axis=1),0)
        grad=np.zeros_like(inv)
        outside_mask=norm>1e-12
        grad[outside_mask]=outside[outside_mask]/norm[outside_mask,None]*np.sign(inv[outside_mask])
        inside=np.where(~outside_mask)[0];which=np.argmax(q[inside],axis=1)
        grad[inside,which]=np.sign(inv[inside,which]);grad*=np.sign(signed)[:,None]
        jac=np.column_stack((-grad@R.T,np.zeros(len(boundary)),np.sum(grad*(v@derivative),axis=1)))
        prediction=model_edge@R.T+p[:2]
        reverse,index=boundary_tree.query(prediction)
        delta=prediction-boundary[index]
        unit=np.divide(delta,reverse[:,None],out=np.zeros_like(delta),where=reverse[:,None]>1e-12)
        reverse_jac=np.column_stack((unit,np.zeros(len(prediction)),np.sum(unit*(model_edge@derivative.T),axis=1)))
        return np.concatenate((np.abs(signed),reverse)),np.vstack((jac,reverse_jac))
    def axis_terms(p):
        if ring_angle is None:return np.empty(0),np.empty((0,4))
        # The measured roof axis is pi-periodic. Repeating it balances the existing
        # sampled perimeter residuals; it cannot select an insertion sign.
        angle=p[3]+np.pi/2-ring_angle
        residual=np.full(60,ring_lever*np.sin(angle))
        jac=np.zeros((60,4));jac[:,3]=ring_lever*np.cos(angle)
        return residual,jac
    def distances(p):return np.concatenate((evaluate(p)['distance'],boundary_terms(p)[0],axis_terms(p)[0]))
    def jacobian(p):return np.vstack((evaluate(p)['jac'],boundary_terms(p)[1],axis_terms(p)[1]))
    def objective_score(p,surface_residual):
        edges=boundary_terms(p)[0]
        score=float(np.median(surface_residual))
        if len(edges):score=max(score,float(np.sqrt(np.mean(edges**2))) if use_ring_axis else float(np.median(edges)))
        axes=axis_terms(p)[0]
        if len(axes):score=max(score,float(abs(axes[0])))
        return score
    fit_start=perf_counter();fits=[]
    for offset in [0,np.pi/2]:
        initial=np.array([*rect[0],bodyh/2,theta+offset],float)
        delta=np.array([.02,.02,.005,np.radians(30)])
        result=least_squares(distances,initial,bounds=(initial-delta,initial+delta),
                             loss='soft_l1',f_scale=.003,max_nfev=max_nfev,diff_step=1e-4,
                             jac=jacobian if analytic_jacobian else '2-point')
        residual=evaluate(result.x)['distance'].copy()
        fits.append((objective_score(result.x,residual),result.x.copy(),residual,result.success))
    optimize_ms=(perf_counter()-fit_start)*1000
    if validation_points is not None:
        validation=local[np.linspace(0,len(local)-1,min(validation_points,len(local)),dtype=int)]
        validated=[]
        for _,parameters,_,success in fits:
            validation_residual=tree.query((validation-parameters[:3])@rotation(parameters[3]))[0]
            validated.append((objective_score(parameters,validation_residual),parameters,validation_residual,success))
        fits=validated
    fits.sort(key=lambda f:f[0]);score,p,residual,converged=fits[0]
    median=float(np.median(residual));boundary_residual=boundary_terms(p)[0]
    boundary_gate=not len(boundary_residual) or (np.median(boundary_residual)<.004 and np.percentile(boundary_residual,95)<.012)
    T=np.eye(4);T[:3,:3]=basis@rotation(p[3]);T[:3,3]=o+basis@p[:3]
    observed_size=np.ptp(xy,axis=0)
    surface_support=bool(converged and median<.004 and np.percentile(residual,95)<.012 and boundary_gate)
    local_fit=((validation if validation_points is not None else selected)-p[:3])@rotation(p[3])
    perturbations=[]
    for axis in [0,1]:
        for sign in [-1,1]:
            changed=local_fit.copy();changed[:,axis]-=sign*.005
            changed_p=p.copy();changed_p[axis]+=sign*.005
            med=objective_score(changed_p,tree.query(((validation if validation_points is not None else selected)-changed_p[:3])@rotation(changed_p[3]))[0])
            perturbations.append({'axis':axis,'delta_m':sign*.005,'median_increase_m':med-float(score)})
    for sign in [-1,1]:
        changed_p=p.copy();changed_p[3]+=np.radians(sign*5)
        changed_residual=tree.query(((validation if validation_points is not None else selected)-changed_p[:3])@rotation(changed_p[3]))[0]
        med=objective_score(changed_p,changed_residual)
        perturbations.append({'axis':'yaw','delta_deg':sign*5,'median_increase_m':med-float(score)})
    weak=any(item['median_increase_m']<.001 for item in perturbations)
    support=surface_support and not weak
    ring_start=perf_counter();candidate=RingPoseEstimator(geometry).estimate(T,support)
    ring_ms=(perf_counter()-ring_start)*1000
    if weak:
        candidate['reason']=candidate['invalid_reason']='WEAK_XY_YAW_CONSTRAINT; NO_INDEPENDENT_3D_VALIDATION'
    status='WEAK_XY_YAW_CONSTRAINT' if surface_support and weak else ('GEOMETRIC_SUPPORT_ONLY' if support else 'CAD_OBSERVATION_MISMATCH')
    hypotheses=[]
    observations=validation if validation_points is not None else selected
    for _,parameters,_,success in fits:
        for offset in (0.,np.pi):
            alternative=parameters.copy();alternative[3]+=offset
            alternative_residual=tree.query((observations-alternative[:3])@rotation(alternative[3]))[0]
            candidate_score=objective_score(alternative,alternative_residual)
            if candidate_score<=score+.001:
                transform=np.eye(4);transform[:3,:3]=basis@rotation(alternative[3]);transform[:3,3]=o+basis@alternative[:3]
                hypotheses.append(dict(body_pose_candidate=transform.tolist(),objective_score_m=candidate_score,
                    converged=bool(success),direction_offset_rad=offset,body_pose_valid=False,ring_pose_valid=False))

    return {'timing_ms':{'multistart_optimization':optimize_ms,'ring_candidate':ring_ms},'fit_status':status,'surface_residual_gate_passed':surface_support,
            'weak_constraint_flag':weak,'objective_sensitivity':perturbations,
            'body_boundary_support':boundary_info,'ring_surface_axis_support':ring_axis_info,
            'objective_sensitivity_metric':'max(surface median, body boundary RMS, measured ring axis lever residual) when ring axis enabled; boundary median otherwise',
            'boundary_residual_rms_m':float(np.sqrt(np.mean(boundary_residual**2))) if len(boundary_residual) else None,'boundary_residual_median_m':float(np.median(boundary_residual)) if len(boundary_residual) else None,
            'joint_objective_score_m':score,'pose_hypotheses':hypotheses,'reasonable_pose_count':len(hypotheses),
            'pose':candidate,'visible_surface_center_m':np.median(points,axis=0).tolist(),
            'observed_height_p05_p50_p95_m':np.percentile(local[:,2],[5,50,95]).tolist(),
            'body_band_point_count':int(bodyband.sum()),'observed_body_band_rect_m':list(rect[1]),
            'nominal_body_dimensions_m':(np.array(list(geometry['battery']['body_dimensions_mm'].values()))/1000).tolist(),
            'cad_surface_residual_mean_m':float(residual.mean()),
            'cad_surface_residual_median_m':float(median),'cad_surface_residual_p95_m':float(np.percentile(residual,95)),
            'hypothesis_median_residuals_m':[float(np.median(f[2])) for f in fits],
            'hypothesis_objective_scores_m':[float(f[0]) for f in fits],
            'fit_objective':('SURFACE_BODY_BOUNDARY_AND_MEASURED_RING_AXIS' if ring_angle is not None else 'ROBUST_SURFACE_PLUS_VISIBLE_BODY_BOUNDARY') if boundary is not None else 'ROBUST_ONE_SIDED_OBSERVED_TO_SAMPLED_CAD_SURFACE',
            'analytic_jacobian_used':analytic_jacobian,'optimization_points':len(selected),'validation_points':min(validation_points,len(local)) if validation_points else len(selected),
            'limitations':['SAMPLING_APPROXIMATION','HIDDEN_SURFACE_NOT_OBSERVED','TABLE_PLANE_CANDIDATE',
                           'DEPTH_NEAR_MIN_RANGE','180_DEG_AMBIGUITY','NO_3D_GROUND_TRUTH'],
            'position_accuracy':'NOT_MEASURED','yaw_accuracy':'NOT_MEASURED'}


class CADAwareGeometryEstimator:
    """Reusable experimental estimator; no modification of live defaults."""
    def __init__(self,geometry,battery_triangles_m):
        self.geometry=geometry
        self.surface=sample_surface(battery_triangles_m)
    def estimate(self,points_camera_m,table_plane):
        return fit_body(points_camera_m,table_plane,self.geometry,self.surface)
