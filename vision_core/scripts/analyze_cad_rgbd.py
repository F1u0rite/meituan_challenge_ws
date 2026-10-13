"""Repeatable offline CAD extraction, solid insertion check and RGB-D replay."""
import sys,json,argparse,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from cad_geometry.extract import analyze_cad,save_analysis
from cad_geometry.collision import normalized_solids,insertion_path
from cad_geometry.frames import transform_points
from perception.cad_body import fit_body,sample_surface
from real_data.rgbd_recording import load_rgbd_sample,save_ply
from perception.unified_detector import UnifiedBatteryDetector
from perception.rgbd_offline import analyze_frame
def png(path,image):
    ok,buf=cv2.imencode('.png',image)
    if not ok:raise IOError(path)
    buf.tofile(str(path))
def jsonfile(path,data):path.write_text(json.dumps(data,indent=2,ensure_ascii=False),encoding='utf-8')
def meshplot(meshes,path):
    fig=plt.figure(figsize=(12,5))
    for index,(name,tri) in enumerate(meshes.items(),1):
        ax=fig.add_subplot(1,2,index,projection='3d')
        ax.add_collection3d(Poly3DCollection(tri*1000,alpha=.3,edgecolor='gray',linewidth=.15))
        xyz=tri.reshape(-1,3)*1000
        ax.auto_scale_xyz(xyz[:,0],xyz[:,1],xyz[:,2]);ax.set_box_aspect(np.maximum(np.ptp(xyz,axis=0),1))
        ax.set(xlabel='X mm',ylabel='Y mm',zlabel='Z mm',title=name)
    fig.tight_layout();fig.savefig(path,dpi=160);plt.close(fig)
def project(points,intr):
    points=np.asarray(points)
    return np.column_stack([points[:,0]/points[:,2]*intr.fx+intr.ppx,points[:,1]/points[:,2]*intr.fy+intr.ppy])
def overlay(frame,maps,fit,tri,path):
    out=frame.color_bgr.copy();mask=maps['object'];out[mask]=(out[mask]*.55+np.array([0,180,0])*.45).astype(np.uint8)
    pose=fit['pose'];T=pose.get('body_pose')
    if T is not None:
        transformed=transform_points(tri.reshape(-1,3),T).reshape(-1,3,3)
        for t in transformed[::3]:
            if (t[:,2]>.01).all():
                uv=project(t,frame.intrinsics).astype(np.int32)
                cv2.polylines(out,[uv],True,(255,255,0),1,cv2.LINE_AA)
        centers=[(np.array(T)[:3,3],(255,0,255)),(np.array(pose['ring_pose'])[:3,3],(0,255,255)),
                 (fit['visible_surface_center_m'],(255,150,30))]
        for center,color in centers:
            uv=tuple(project(np.array([center]),frame.intrinsics)[0].astype(int))
            cv2.circle(out,uv,7,color,2)
        for sign in [-1,1]:
            p=np.array(pose['ring_pose'])[:3,3];q=p+sign*.025*np.array(T)[:3,0]
            uv=project([p,q],frame.intrinsics).astype(int)
            cv2.arrowedLine(out,tuple(uv[0]),tuple(uv[1]),(0,255,255),2)
    for i,text in enumerate(['green=observed cloud mask; cyan=CAD prediction',
             'magenta=body candidate; yellow=ring candidate; blue=visible median',
             fit['fit_status'],'body_pose_valid=false | ring_pose_valid=false']):
        cv2.putText(out,text,(12,28+i*28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),4)
        cv2.putText(out,text,(12,28+i*28),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
    png(path,out);return out
def run(output,do_rgbd=True,do_collision=True):
    output.mkdir(parents=True,exist_ok=True)
    data,battery,hook=analyze_cad(ROOT/'assets/cad/电池道具.stp',ROOT/'assets/cad/舌规V1.STEP')
    save_analysis(data,battery,hook,output)
    meshes=np.load(output/'cad_meshes.npz');meshplot(dict(meshes),output/'cad_frames.png')
    if do_collision:
        B,H=normalized_solids(data,battery,hook)
        cases=[{'name':'nominal'}]
        cases += [{'name':f'lateral_{x}', 'lateral_mm':x} for x in [-4,-2,2,4]]
        cases += [{'name':f'vertical_{x}','vertical_mm':x} for x in [-6,-3,3,6]]
        cases += [{'name':f'yaw_{x}','yaw_deg':x} for x in [-10,-5,5,10]]
        cases += [{'name':f'depth_{x}','end_mm':x} for x in [10,20,40,60]]
        results=[]
        for c in cases:
            name=c.pop('name');result=insertion_path(data,B,H,**c);result['name']=name
            results.append(result);print('collision',name,result['status'],flush=True)
        jsonfile(output/'hook_ring_path_checks.json',results)
        fig,ax=plt.subplots(figsize=(11,6))
        colors={'COLLISION':'tomato','NOMINAL_CLEARANCE':'seagreen','UNKNOWN':'gray'}
        ax.barh([r['name'] for r in results],[max(r['minimum_sampled_gap_mm'],.02) for r in results],
                color=[colors[r['status']] for r in results])
        ax.set(xlabel='Minimum evaluated gap (mm); continuous intervals also certified',title='Complete solids, nominal CAD only')
        fig.tight_layout();fig.savefig(output/'hook_clearance_sweep.png',dpi=150);plt.close(fig)
    if do_rgbd:
        surface=sample_surface(meshes['battery']);detector=UnifiedBatteryDetector()
        session=ROOT/'data/rgbd/session_20261011_143230_red_top'
        rows=[];thumbs=[];hashes={}
        for folder in sorted(session.glob('sample_*')):
            print('replay',folder.name,flush=True)
            paths=[folder/'raw_color.png',folder/'aligned_depth.png',folder/'metadata.json',folder/'camera_intrinsics.json']
            for p in paths:hashes[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
            frame=load_rgbd_sample(folder);report,maps,points,rgb=analyze_frame(frame,detector.detect_batteries(frame.color_bgr))
            fit=fit_body(points,report['table_plane'],data,surface)
            row={'sample_id':folder.name,'point_count':len(points),'table_plane':report['table_plane'],'cad_fit':fit,
                 'baseline_geometry':report['body_geometry'],'source_metadata':json.loads((folder/'metadata.json').read_text(encoding='utf-8'))}
            dest=output/'rgbd'/folder.name;dest.mkdir(parents=True,exist_ok=True)
            jsonfile(dest/'result.json',row);save_ply(dest/'observed_points.ply',points,rgb)
            image=overlay(frame,maps,fit,meshes['battery'],dest/'overlay.png')
            T=fit['pose'].get('body_pose')
            if T is not None:
                cloud=transform_points(surface,T);save_ply(dest/'cad_predicted_points.ply',cloud,np.tile([0,220,220],(len(cloud),1)))
                fig=plt.figure(figsize=(7,6));ax=fig.add_subplot(111,projection='3d')
                ax.scatter(*points[::8].T,s=1,c='red',label='observed')
                ax.scatter(*cloud[::30].T,s=1,c='cyan',alpha=.25,label='predicted CAD (hidden included)')
                ax.set(xlabel='camera X m',ylabel='camera Y m',zlabel='camera Z m',title=folder.name)
                ax.legend();fig.tight_layout();fig.savefig(dest/'cloud_comparison.png',dpi=130);plt.close(fig)
            thumb=cv2.resize(image,(384,216));cv2.putText(thumb,folder.name,(8,208),0,.5,(255,255,255),1)
            thumbs.append(thumb);rows.append(row)
        jsonfile(output/'raw_input_sha256.json',hashes)
        changed=[p for p,h in hashes.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        if changed:raise RuntimeError('Raw files changed: '+str(changed))
        while len(thumbs)%4:thumbs.append(np.zeros((216,384,3),np.uint8))
        png(output/'rgbd_contact_sheet.png',np.vstack([np.hstack(thumbs[i:i+4]) for i in range(0,len(thumbs),4)]))
        good=[r for r in rows if 'cad_surface_residual_mean_m' in r['cad_fit']]
        summary={'sample_count':len(rows),'fitted_count':len(good),
            'geometric_support_count':sum(r['cad_fit']['fit_status']=='GEOMETRIC_SUPPORT_ONLY' for r in rows),
            'weak_constraint_count':sum(r['cad_fit'].get('weak_constraint_flag',False) for r in rows),
            'mismatch_count':sum(r['cad_fit']['fit_status']=='CAD_OBSERVATION_MISMATCH' for r in rows),
            'residual_mean_m_median':float(np.median([r['cad_fit']['cad_surface_residual_mean_m'] for r in good])) if good else None,
            'residual_p95_m_median':float(np.median([r['cad_fit']['cad_surface_residual_p95_m'] for r in good])) if good else None,
            'actual_camera_started':False,'new_recordings':0,'raw_files_unchanged':True,
            'body_pose_valid':False,'ring_pose_valid':False,'3D_position_accuracy':'NOT_MEASURED',
            'yaw_accuracy':'NOT_MEASURED','calibration_status':data['calibration_status'],'frames':rows}
        jsonfile(output/'rgbd_summary.json',summary)
        print(json.dumps({k:v for k,v in summary.items() if k!='frames'},indent=2),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'outputs/cad_analysis')
    p.add_argument('--skip-rgbd',action='store_true');p.add_argument('--skip-collision',action='store_true');a=p.parse_args()
    run(a.output,not a.skip_rgbd,not a.skip_collision)
