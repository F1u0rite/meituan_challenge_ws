"""Current RGB-D support plane and top seeds for automatic SAM prompts.

No pixel slots, fixed camera pose, cached plane or manual top boxes are used.
The support-plane hypothesis is checked against object heights; outputs are
diagnostic candidates, not robot poses.
"""
import cv2
import numpy as np
from .rgbd_geometry import deproject_pixels,fit_plane_ransac,validate_frame


def _point(mask):
    distance=cv2.distanceTransform(np.uint8(mask),cv2.DIST_L2,5)
    y,x=np.unravel_index(np.argmax(distance),distance.shape)
    return [int(x),int(y)]


class AutomaticTopPrompt:
    def __init__(self,*,top_height_m=.080,height_band_m=.015,plane_threshold_m=.0035):
        self.top_height_m=top_height_m
        self.height_band_m=height_band_m
        self.plane_threshold_m=plane_threshold_m

    def propose(self,frame,object_masks,*,qualified):
        out=dict(support_plane=None,objects={},reasons=[],manual_prompts=False)
        if not qualified:out['reasons']=['RGBD_PAIR_REJECTED'];return out
        validate_frame(frame.color_bgr,frame.aligned_depth,frame.intrinsics,frame.depth_scale)
        z=frame.aligned_depth*frame.depth_scale
        valid=(z>.15)&(z<1.2)
        union=np.zeros(z.shape,bool)
        for mask in object_masks.values():
            if np.asarray(mask).shape!=z.shape:raise ValueError('Object mask shape mismatch')
            union|=np.asarray(mask,bool)
        # Low-chroma background proposes a support plane; the plane's image
        # location is not fixed. Robot/background clutter still requires testing.
        hsv=cv2.cvtColor(frame.color_bgr,cv2.COLOR_BGR2HSV)
        outside=valid&~cv2.dilate(np.uint8(union),np.ones((15,15),np.uint8)).astype(bool)&(hsv[:,:,1]<80)
        v,u=np.where(outside)
        if len(v)<1000:out['reasons']=['INSUFFICIENT_SUPPORT_BACKGROUND'];return out
        take=np.linspace(0,len(v)-1,min(4000,len(v)),dtype=int)
        cloud=deproject_pixels(np.column_stack((u[take],v[take])),z[v[take],u[take]],frame.intrinsics)
        plane=fit_plane_ransac(cloud,threshold_m=.004,min_inliers=700)
        if plane is None:out['reasons']=['NO_DOMINANT_SUPPORT_PLANE'];return out
        normal=np.array(plane['normal_camera_xyz']);origin=np.array(plane['point_camera_m'])
        if normal@origin>0:normal=-normal
        out['support_plane']=dict(point_camera_m=origin.tolist(),normal_camera_xyz=normal.tolist(),
            inlier_ratio=plane['inlier_ratio'],source='CURRENT_LOW_CHROMA_BACKGROUND_HYPOTHESIS')
        for identity,mask in object_masks.items():
            record=dict(available=False,reasons=[],box_px=None,point_coords_px=[],point_labels=[])
            out['objects'][identity]=record
            mask=np.asarray(mask,bool)
            inside=cv2.erode(np.uint8(mask),np.ones((5,5),np.uint8)).astype(bool)
            v,u=np.where(inside&valid)
            if len(v)<100:record['reasons']=['INSUFFICIENT_OBJECT_DEPTH'];continue
            pts=deproject_pixels(np.column_stack((u,v)),z[v,u],frame.intrinsics)
            height=(pts-origin)@normal
            band=abs(height-self.top_height_m)<=self.height_band_m
            if band.sum()<80:record['reasons']=['NO_EXPECTED_HEIGHT_SUPPORT'];continue
            candidates=pts[band];remaining=np.ones(len(candidates),bool);chosen=None
            for _ in range(4):
                ix=np.flatnonzero(remaining)
                if len(ix)<60:break
                ix=ix[np.linspace(0,len(ix)-1,min(1500,len(ix)),dtype=int)]
                fit=fit_plane_ransac(candidates[ix],threshold_m=self.plane_threshold_m,min_inliers=50)
                if fit is None:break
                n=np.array(fit['normal_camera_xyz']);o=np.array(fit['point_camera_m'])
                if n@normal<0:n=-n
                match=abs((candidates-o)@n)<=self.plane_threshold_m
                remaining[match]=False
                # The declared upright-on-common-support constraint separates
                # cap planes from vertical faces; insertion sign is unrelated.
                if n@normal<.85:continue
                selected=abs((pts-o)@n)<=self.plane_threshold_m
                selected&=abs(height-self.top_height_m)<=self.height_band_m
                if selected.sum()<80:continue
                chosen=(selected,n,o);break
            if chosen is None:record['reasons']=['NO_SUPPORT_PARALLEL_CAP'];continue
            selected,n,o=chosen
            seed=np.zeros(mask.shape,np.uint8);seed[v[selected],u[selected]]=1
            seed=cv2.morphologyEx(seed,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            count,labels,stats,_=cv2.connectedComponentsWithStats(seed,8)
            if count<2:record['reasons']=['NO_CONNECTED_CAP_SEED'];continue
            label=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]));seed=labels==label
            x,y,bw,bh=cv2.boundingRect(np.uint8(seed))
            if seed.sum()<80 or min(bw,bh)<8:record['reasons']=['CAP_SEED_TOO_SMALL'];continue
            # Positive click stays in actual depth support, never in a nominal
            # box centre that could fall in an occluded area.
            points=[_point(seed)];point_labels=[1]
            lower=np.zeros(mask.shape,bool);lower[v[height<self.top_height_m-.025],u[height<self.top_height_m-.025]]=True
            lower&=~cv2.dilate(np.uint8(seed),np.ones((9,9),np.uint8)).astype(bool)
            if lower.any():points.append(_point(lower));point_labels.append(0)
            record.update(available=True,seed_mask=seed,
                box_px=[max(0,x-3),max(0,y-3),min(mask.shape[1]-1,x+bw+3),min(mask.shape[0]-1,y+bh+3)],
                point_coords_px=points,point_labels=point_labels,
                seed_pixels=int(seed.sum()),cap_normal_camera=n.tolist(),
                observed_height_median_m=float(np.median(height[selected])))
        return out
