"""Experimental objective sensitivity; never a statistical confidence interval."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from cad_geometry.frames import validate_transform,periodic_error
def rotation_z(degrees):
    c,s=np.cos(np.radians(degrees)),np.sin(np.radians(degrees))
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def perturb_pose(T,delta_mm_deg):
    T=validate_transform(T);d=np.asarray(delta_mm_deg,float)
    if d.shape!=(4,) or not np.isfinite(d).all():raise ValueError('Invalid pose perturbation')
    result=T.copy();result[:3,3]+=T[:3,:3]@d[:3]/1000
    result[:3,:3]=T[:3,:3]@rotation_z(d[3])
    return validate_transform(result)
def equivalent_candidates(a,b,translation_mm=1,yaw_deg=1):
    return bool(np.linalg.norm(np.array(a['delta_mm_deg'])[:3]-np.array(b['delta_mm_deg'])[:3])<=translation_mm and
            abs(periodic_error(a['delta_mm_deg'][3],b['delta_mm_deg'][3]))<=yaw_deg)
def summarize_curve(values,losses,tolerance,weak_span):
    values=np.asarray(values,float);losses=np.asarray(losses,float)
    if len(values)<3 or not np.isfinite(losses).all():raise ValueError('Invalid loss curve')
    equal_minima=np.flatnonzero(losses==losses.min())
    ix=int(equal_minima[np.argmin(np.abs(values[equal_minima]))]);selected=values[losses<=losses[ix]+tolerance]
    span=float(np.ptp(selected)) if len(selected)>1 else 0.
    minima=[int(i) for i in range(1,len(losses)-1) if losses[i]<losses[i-1] and losses[i]<=losses[i+1]]
    curvature=None
    if 0<ix<len(values)-1 and np.allclose(np.diff(values),np.diff(values)[0]):
        curvature=float((losses[ix+1]-2*losses[ix]+losses[ix-1])/(values[ix+1]-values[ix])**2)
    return {'min_at':float(values[ix]),'near_minimum_span':span,'weak':bool(span>=weak_span),
            'local_minima_indices':minima,'local_curvature':curvature,
            'interpretation':'EMPIRICAL_OBJECTIVE_SHAPE; NOT_CI'}
class PoseObjective:
    def __init__(self,points,T0,cad_surface,body_edge_model,observed_edges,config,plane=None,body_height_m=None):
        self.points=np.asarray(points,float)
        if self.points.ndim!=2 or self.points.shape[1]!=3 or not len(self.points) or not np.isfinite(self.points).all():
            raise ValueError('EMPTY_OR_INVALID_POINT_CLOUD')
        self.T0=validate_transform(T0);self.tree=cKDTree(cad_surface)
        self.edge_tree=cKDTree(body_edge_model);self.edges=np.asarray(observed_edges,float).reshape(-1,3)
        self.config=config;self.plane=plane;self.body_height=body_height_m
        self.feature_status={'body_edge':'CANDIDATE_HEIGHT_BOUNDARY' if len(self.edges)>=20 else 'FEATURE_NOT_OBSERVABLE',
                             'ring':'FEATURE_NOT_OBSERVABLE'}
    def residuals(self,delta,ablation='A'):
        T=perturb_pose(self.T0,delta)
        local=(self.points-T[:3,3])@T[:3,:3]
        depth=self.tree.query(local)[0]/(self.config['depth_scale_mm']/1000)
        residuals=[depth/np.sqrt(len(depth))]
        if ablation in ('B','D') and self.feature_status['body_edge']!='FEATURE_NOT_OBSERVABLE':
            edge_local=(self.edges-T[:3,3])@T[:3,:3]
            edge=self.edge_tree.query(edge_local)[0]/(self.config['edge_scale_mm']/1000)
            residuals.append(edge/np.sqrt(len(edge)))
        if ablation=='D' and self.plane is not None:
            n=np.asarray(self.plane['normal_camera_xyz']);o=np.asarray(self.plane['point_camera_m'])
            height=float((o-T[:3,3])@n)
            residuals.append(np.array([(height-self.body_height/2)/(self.config['prior_scale_mm']/1000)]))
        return np.concatenate(residuals)
    def metrics(self,delta,ablation='A'):
        T=perturb_pose(self.T0,delta);local=(self.points-T[:3,3])@T[:3,:3]
        distance=self.tree.query(local)[0]*1000
        edge=None
        if len(self.edges)>=20:
            edge=float(np.mean(self.edge_tree.query((self.edges-T[:3,3])@T[:3,:3])[0])*1000)
        return {'point_cloud_loss':float(np.mean((distance/self.config['depth_scale_mm'])**2)),
                'surface_distance_mean_mm':float(distance.mean()),'surface_distance_p95_mm':float(np.percentile(distance,95)),
                'visible_edge_alignment_mean_mm':edge,'ring_feature_alignment_loss':None,
                'combined_objective_loss':float(np.sum(self.residuals(delta,ablation)**2)),
                'feature_status':self.feature_status}
    def scan(self,ablation):
        curves={}
        for axis,name in enumerate(['X','Y','Z','Yaw']):
            values=self.config['scan_yaw_deg'] if axis==3 else self.config['scan_translation_mm']
            metrics=[]
            for value in values:
                d=np.zeros(4);d[axis]=value;metrics.append(self.metrics(d,ablation))
            losses=[m['combined_objective_loss'] for m in metrics]
            curves[name]={'values':values,'unit':'deg' if axis==3 else 'mm','metrics':metrics,
                'summary':summarize_curve(values,losses,self.config['flat_loss_tolerance'],
                    self.config['weak_yaw_span_deg'] if axis==3 else self.config['weak_translation_span_mm'])}
        return curves
    def multistart(self,ablation):
        results=[]
        for initial in self.config['starts_mm_deg']:
            initial=np.array(initial,float);limit=self.config['bounds_translation_mm']
            low=np.array([-limit,-limit,-limit,initial[3]-self.config['bounds_yaw_deg']])
            high=np.array([limit,limit,limit,initial[3]+self.config['bounds_yaw_deg']])
            fit=least_squares(lambda d:self.residuals(d,ablation),initial,bounds=(low,high),
                max_nfev=self.config['max_nfev'],diff_step=1e-3,loss='linear')
            metrics=self.metrics(fit.x,ablation)
            results.append({'initial_mm_deg':initial.tolist(),'delta_mm_deg':fit.x.tolist(),
                'pose':perturb_pose(self.T0,fit.x).tolist(),'optimizer_converged':bool(fit.success),
                'optimizer_status':int(fit.status),'nfev':int(fit.nfev),
                'loss':metrics['combined_objective_loss'],'constraint_residuals':metrics})
        results.sort(key=lambda r:r['loss'])
        for i,r in enumerate(results):
            r['equivalent_to_best']=equivalent_candidates(r,results[0],self.config['equivalence_translation_mm'],self.config['equivalence_yaw_deg'])
            r['distance_to_best_mm']=float(np.linalg.norm(np.array(r['delta_mm_deg'])[:3]-np.array(results[0]['delta_mm_deg'])[:3]))
            r['yaw_to_best_180_deg']=abs(periodic_error(r['delta_mm_deg'][3],results[0]['delta_mm_deg'][3]))
            r['pairwise']=[{'candidate':j,'translation_mm':float(np.linalg.norm(np.array(r['delta_mm_deg'])[:3]-np.array(q['delta_mm_deg'])[:3])),
                'yaw_180_deg':abs(periodic_error(r['delta_mm_deg'][3],q['delta_mm_deg'][3])),
                'equivalent':equivalent_candidates(r,q,self.config['equivalence_translation_mm'],self.config['equivalence_yaw_deg'])} for j,q in enumerate(results) if j!=i]
        distinct=next((r for r in results[1:] if not r['equivalent_to_best']),None)
        separation=None if distinct is None else {'translation_mm':distinct['distance_to_best_mm'],'yaw_180_deg':distinct['yaw_to_best_180_deg'],
            'loss_gap':distinct['loss']-results[0]['loss']}
        return {'candidates':results,'candidate_separation':separation,
            'multiple_near_equal_distinct_solutions':bool(distinct is not None and distinct['loss']-results[0]['loss']<=self.config['flat_loss_tolerance']),
            'insertion_direction_ambiguity':True,'body_pose_valid':False,'ring_pose_valid':False}
def validity_gate(*,gt_independently_verified=False,calibration_verified=False,observability_verified=False,depth_valid=False):
    # A validation dataset alone does not establish readiness for motion.
    return {'body_pose_valid':False,'ring_pose_valid':False,'robot_execution_allowed':False,
            'reason':'EXPERIMENTAL_PIPELINE_NOT_VALIDATED_FOR_MOTION'}
