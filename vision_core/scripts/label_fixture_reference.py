"""Offline fixture reference clicks; independently measured fixture coordinates only."""
import sys,argparse,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import cv2,numpy as np
from real_data.rgbd_recording import load_rgbd_sample
from perception.rgbd_geometry import deproject_pixels
from real_data.pose_experiment import write_json
def camera_point(frame,uv,radius=2,min_valid=5):
    u,v=map(int,uv)
    if not (0<=u<frame.aligned_depth.shape[1] and 0<=v<frame.aligned_depth.shape[0]):raise ValueError('Pixel out of bounds')
    patch=frame.aligned_depth[max(v-radius,0):v+radius+1,max(u-radius,0):u+radius+1]
    valid=patch[patch>0]
    if len(valid)<min_valid:raise ValueError('INSUFFICIENT_REFERENCE_DEPTH')
    z=float(np.median(valid))*frame.depth_scale
    return deproject_pixels([uv],[z],frame.intrinsics)[0].tolist()
def main():
    p=argparse.ArgumentParser();p.add_argument('--sample',type=Path,required=True);p.add_argument('--labels',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Keep previous calibration records; choose a new output')
    labels=json.loads(a.labels.read_text(encoding='utf-8-sig'))
    if not labels.get('independently_measured') or not labels.get('source') or not labels.get('measurement_uncertainty_mm'):
        p.error('Independent physical fixture measurements must be supplied first')
    for row in labels['points']:
        xyz=np.asarray(row['fixture_xyz_mm'],float)
        if xyz.shape!=(3,) or not np.isfinite(xyz).all():p.error('Missing measured fixture XYZ')
    if sum(r['role']=='fit' for r in labels['points'])<4 or sum(r['role']=='heldout' for r in labels['points'])<2:
        p.error('Need >=4 fit and >=2 heldout fixture marks')
    frame=load_rgbd_sample(a.sample);clicks=[]
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(clicks)<len(labels['points']):
            try:clicks.append({'pixel':[x,y],'camera_point_m':camera_point(frame,[x,y])})
            except ValueError as error:print(error,flush=True)
    title='Fixture marks: click in listed order | u undo | s save | q exit'
    cv2.namedWindow(title,cv2.WINDOW_NORMAL);cv2.setMouseCallback(title,click)
    try:
        while True:
            image=frame.color_bgr.copy()
            for i,c in enumerate(clicks):cv2.circle(image,tuple(c['pixel']),5,(0,255,255),2);cv2.putText(image,str(i+1),tuple(c['pixel']),0,.7,(0,255,255),2)
            current=labels['points'][len(clicks)]['name'] if len(clicks)<len(labels['points']) else 'all points selected'
            cv2.putText(image,'Next: '+current,(10,30),0,.7,(0,255,255),2)
            cv2.imshow(title,image);key=cv2.waitKey(30)&255
            if key in (ord('q'),27):break
            if key==ord('u') and clicks:clicks.pop()
            if key==ord('s') and len(clicks)==len(labels['points']):
                out={'source':labels['source'],'measurement_uncertainty_mm':labels['measurement_uncertainty_mm'],
                    'same_camera_depth_correlation':True,'reference_pixels':clicks,'rgbd_sample':str(a.sample)}
                for role,prefix in [('fit',''),('heldout','heldout_')]:
                    ix=[i for i,r in enumerate(labels['points']) if r['role']==role]
                    out[prefix+'camera_points_m']=[clicks[i]['camera_point_m'] for i in ix]
                    out[prefix+'fixture_points_m']=[(np.array(labels['points'][i]['fixture_xyz_mm'])/1000).tolist() for i in ix]
                write_json(a.output,out);break
    finally:cv2.destroyAllWindows()
if __name__=='__main__':main()
