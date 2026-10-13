"""Deterministic RGB domain-gap simulation, separate from scene geometry and perception."""
from dataclasses import dataclass
import cv2
import numpy as np

BACKGROUND_TYPES=('dark','medium','light','beige','light_wood','dark_wood','noisy')

@dataclass
class AugmentedFrame:
    image: np.ndarray
    metadata: dict

def _background(shape,kind,rng):
    h,w=shape[:2]
    colors={'dark':(55,60,65),'medium':(125,130,135),'light':(220,220,218),'beige':(190,205,220),
            'light_wood':(165,195,220),'dark_wood':(65,85,105),'noisy':(115,120,125)}
    bg=np.full((h,w,3),colors[kind],np.float32)
    if 'wood' in kind:
        x=np.arange(w)[None,:,None]; waves=10*np.sin(x/24+rng.uniform(0,6.28))+5*np.sin(x/7)
        bg+=waves
    if kind=='noisy': bg+=rng.normal(0,18,bg.shape)
    return np.clip(bg,0,255).astype(np.uint8)

class RobustnessAugmentation:
    def __init__(self,config,difficulty='clean',seed=42):
        if difficulty not in ('clean','medium','hard'): raise ValueError('difficulty must be clean, medium, or hard')
        self.config=config; self.difficulty=difficulty; self.seed=seed

    def apply(self,image,board_corners):
        if self.difficulty=='clean': return AugmentedFrame(image.copy(),{'difficulty':'clean','background_type':'dark','active':[],'parameters':{}})
        rng=np.random.default_rng(self.seed); cfg=self.config['robustness'][self.difficulty]
        out=image.copy(); h,w=out.shape[:2]; active=[]; params={}
        kind=str(rng.choice(BACKGROUND_TYPES)); bg=_background(out.shape,kind,rng)
        board_mask=np.zeros((h,w),np.uint8); cv2.fillConvexPoly(board_mask,np.rint(board_corners).astype(np.int32),255)
        out[board_mask==0]=bg[board_mask==0]; active.append('background'); params['background_type']=kind
        # Independent battery color variation plus global camera color response.
        hsv=cv2.cvtColor(out,cv2.COLOR_BGR2HSV).astype(np.float32)
        sat_mask=(hsv[:,:,1]>45)&(board_mask>0)
        local_hue=rng.uniform(-cfg['hue_deg'],cfg['hue_deg']); local_sat=rng.uniform(*cfg['saturation'])
        hsv[:,:,0][sat_mask]=(hsv[:,:,0][sat_mask]+local_hue/2)%180; hsv[:,:,1][sat_mask]*=local_sat
        out=cv2.cvtColor(np.clip(hsv,0,255).astype(np.uint8),cv2.COLOR_HSV2BGR); active.append('battery_color_variation')
        params.update(hue_shift_deg=float(local_hue),saturation=float(local_sat))
        brightness=float(rng.uniform(*cfg['brightness'])); contrast=float(rng.uniform(*cfg['contrast'])); gamma=float(rng.uniform(*cfg['gamma']))
        wb=rng.uniform(0.82 if self.difficulty=='hard' else .92,1.18 if self.difficulty=='hard' else 1.08,3)
        out=np.clip((out.astype(np.float32)-127.5)*contrast+127.5,0,255)
        out=np.clip(out*brightness*wb,0,255); out=255*np.power(out/255,gamma)
        active += ['brightness','contrast','gamma','white_balance']; params.update(brightness=brightness,contrast=contrast,gamma=gamma,white_balance=wb.tolist())
        # Local shadow and highlight affect only part of the frame.
        shadow=np.full((h,w),255,np.uint8); pts=rng.integers([0,0],[w,h],size=(4,2),endpoint=False).astype(np.int32)
        cv2.fillConvexPoly(shadow,cv2.convexHull(pts),int(255*rng.uniform(.35,.7 if self.difficulty=='hard' else .82)))
        out*=shadow[:,:,None]/255.0; active.append('shadow'); params['shadow_min']=int(shadow.min())
        highlight=np.zeros((h,w),np.uint8); center=tuple(rng.integers([0,0],[w,h]).tolist()); axes=tuple(rng.integers([80,60],[320,240]).tolist())
        cv2.ellipse(highlight,center,axes,float(rng.uniform(0,180)),0,360,int(rng.uniform(35,95 if self.difficulty=='hard' else 65)),-1)
        out=np.clip(out+highlight[:,:,None],0,255); active.append('highlight'); params['highlight_peak']=int(highlight.max())
        sigma=float(rng.uniform(*cfg['noise_sigma'])); out+=rng.normal(0,sigma,out.shape); active.append('noise'); params['noise_sigma']=sigma
        blur_k=int(rng.choice([3,5] if self.difficulty=='medium' else [3,5,7])); out=cv2.GaussianBlur(np.clip(out,0,255).astype(np.uint8),(blur_k,blur_k),rng.uniform(.3,1.8)); active.append('blur'); params['blur_kernel']=blur_k
        if self.difficulty=='hard':
            length=int(rng.integers(5,14)); kernel=np.zeros((length,length),np.float32); kernel[length//2,:]=1/length
            out=cv2.filter2D(out,-1,kernel); active.append('motion_blur'); params['motion_blur_length']=length
            # Small realistic occluders: one edge segment and one board interior patch.
            edge=int(rng.integers(0,4)); p1=board_corners[edge]; p2=board_corners[(edge+1)%4]; t=float(rng.uniform(.25,.65)); center_edge=p1*(1-t)+p2*t
            cv2.rectangle(out,(round(center_edge[0]-35),round(center_edge[1]-14)),(round(center_edge[0]+35),round(center_edge[1]+14)),(80,80,80),-1)
            active.append('board_edge_occlusion')
            centroid=np.mean(board_corners,axis=0)+rng.normal(0,[180,100]); cv2.rectangle(out,(round(centroid[0]-24),round(centroid[1]-18)),(round(centroid[0]+24),round(centroid[1]+18)),(90,90,90),-1)
            active.append('partial_object_occlusion')
        quality=int(rng.integers(cfg['jpeg_quality'][0],cfg['jpeg_quality'][1]+1)); ok,encoded=cv2.imencode('.jpg',out,[cv2.IMWRITE_JPEG_QUALITY,quality])
        if ok: out=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
        active.append('jpeg'); params['jpeg_quality']=quality
        return AugmentedFrame(out,{'difficulty':self.difficulty,'background_type':kind,'active':active,'parameters':params})
