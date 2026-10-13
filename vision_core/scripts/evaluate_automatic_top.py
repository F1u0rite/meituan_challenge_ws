"""Offline automatic top evaluation. Never starts a camera."""
import argparse,json,sys,hashlib
from pathlib import Path
import cv2,numpy as np,yaml,torch
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2]/'src/mtc_vision_adapter')]
from input.realsense_rgbd_source import RGBDFrame
from perception.rgbd_geometry import CameraIntrinsics,project_points
from perception.automatic_segmented_top import AutomaticSegmentedTop
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


def serial(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(str(type(value)))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--input',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--geometry',required=True,type=Path)
    p.add_argument('--checkpoint',required=True,type=Path)
    p.add_argument('--max-frames',type=int,default=0)
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(2);torch.set_num_interop_threads(1)
    root=Path(__file__).resolve().parents[1]
    predictor=SAM2ImagePredictor(build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml',str(a.checkpoint),device='cpu',apply_postprocessing=False))
    pipe=AutomaticSegmentedTop(yaml.safe_load((root/'config/simplified_rgbd.yaml').read_text()),
        json.loads(a.geometry.read_text()),predictor=predictor)
    rows=[];folders=sorted(a.input.glob('frame_*'))
    if a.max_frames:folders=folders[:a.max_frames]
    for folder in folders:
        m=json.loads((folder/'metadata.json').read_text())
        frame=RGBDFrame(cv2.imread(str(folder/'color.png')),np.load(folder/'depth.npz')['depth'],
            CameraIntrinsics(**m['intrinsics']),m['depth_scale'],m['timestamps'],{},alignment_metadata=m['alignment'])
        sid='color_'+str(frame.timestamps['color_frame_number'])+'_depth_'+str(frame.timestamps['depth_frame_number'])
        result=pipe.process(frame,sid);image=frame.color_bgr.copy()
        for obj in result['objects']:
            mask=obj.pop('top_mask',None)
            if mask is not None:
                image[mask]=np.uint8(.75*image[mask]+.25*np.array([0,255,255]))
                cv2.imwrite(str(a.output/(folder.name+'_'+obj['color']+'_mask.png')),np.uint8(mask)*255)
            if obj['candidate_available']:
                poly=np.rint(obj['geometry']['top_polygon_px']).astype(np.int32)
                cv2.polylines(image,[poly],True,(0,255,255),2)
                for key,c in [('green_point_camera_m',(0,255,0)),('red_point_camera_m',(0,0,255))]:
                    pixel=np.rint(project_points([obj[key]],frame.intrinsics)[0]).astype(int)
                    cv2.drawMarker(image,tuple(pixel),c,cv2.MARKER_CROSS,12,2)
                cv2.putText(image,obj['color'],tuple(poly.min(axis=0)-[0,8]),0,.6,(255,255,255),1)
        cv2.putText(image,'AUTO PROMPTS | green=top red=CAD hole candidates',(15,30),0,.6,(255,255,255),2)
        cv2.imwrite(str(a.output/(folder.name+'_overlay.png')),image)
        rows.append(result)
        (a.output/'rows.json').write_text(json.dumps(rows,default=serial,indent=2))
        print('FRAME',folder.name,'missing',result['missing_colors'],'ms',round(result['processing_ms']),flush=True)
    summary=dict(frames=len(rows),manual_prompts=False,camera_started=False,motion_usable=False,
        all_four_candidate_frames=sum(not r['missing_colors'] for r in rows),colors={},
        mean_processing_ms=float(np.mean([r['processing_ms'] for r in rows])) if rows else None,
        geometry_sha256=hashlib.sha256(a.geometry.read_bytes()).hexdigest())
    for c in ['green','blue','yellow','red']:
        objects=[o for r in rows for o in r['objects'] if o['color']==c]
        valid=[o for o in objects if o['candidate_available']]
        summary['colors'][c]=dict(candidates=len(valid),geometry_consistent=sum(o['geometry_consistent'] for o in valid))
        if valid:
            points=np.array([o['red_point_camera_m'] for o in valid])*1000
            radius=np.linalg.norm(points-np.median(points,axis=0),axis=1)
            summary['colors'][c].update(red_median_camera_mm=np.median(points,axis=0).tolist(),
                apparent_red_radius_p95_mm=float(np.percentile(radius,95)))
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print('SUMMARY',json.dumps(summary),flush=True)


if __name__=='__main__':main()
