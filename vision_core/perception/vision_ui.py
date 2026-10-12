"""1600x900 actual diagnostic UI; renders the same values saved in prediction JSON."""
import cv2,numpy as np
from time import perf_counter,time
from .rgbd_geometry import project_points
from cad_geometry.frames import transform_points
COLORS={'red':(55,55,245),'blue':(255,130,60),'yellow':(30,220,250),'green':(70,230,80),'unknown':(190,190,190)}
def mm(value):return 'N/A' if value is None else f'{value*1000:.1f} mm'
def vec(values):return 'N/A' if values is None else ', '.join(f'{v*1000:.1f}' for v in values)+' mm'
def object_label(item):
    return f"{item.get('candidate_state','DIAGNOSTIC')} {item['color_label'].upper()} | {item['object_id']} | Z {mm(item['surface_depth_z'])}"
def text(image,label,x,y,color=(220,230,235),scale=.46):
    cv2.putText(image,str(label),(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)

class VisionUI:
    def __init__(self):
        self.mode='annotated';self.show_cad=True;self.show_info=True;self.selected=0
        self.show_candidates=False;self.paused=False;self.recording=False;self.last_display_data={}
    def handle_key(self,key):
        if key==ord('c'):self.show_candidates=not self.show_candidates
        elif key==ord('d'):self.mode='annotated' if self.mode=='depth' else 'depth'
        elif key==ord('m'):self.mode='annotated' if self.mode=='mask' else 'mask'
        elif key==ord('b'):self.mode='annotated' if self.mode=='raw' else 'raw'
        elif key==ord('f'):self.mode='annotated' if self.mode=='foreground' else 'foreground'
        elif key==ord('p'):self.mode='annotated' if self.mode=='cloud' else 'cloud'
        elif key==ord('o'):self.show_cad=not self.show_cad
        elif key==ord('i'):self.show_info=not self.show_info
        elif key==ord(' '):self.paused=not self.paused
        elif ord('1')<=key<=ord('9'):self.selected=key-ord('1')
        return key
    def render(self,frame,result,views,geometry,sdk_project=None,display_fps=None):
        ui_start=perf_counter();rgb=frame.color_bgr.copy();mask=np.zeros(rgb.shape,np.uint8)
        depth_start=perf_counter()
        depth_m=frame.aligned_depth.astype(float)*frame.depth_scale
        depth=cv2.applyColorMap(np.uint8(np.clip(depth_m,0,1)*255),cv2.COLORMAP_TURBO)
        depth[frame.aligned_depth==0]=0
        depth_ms=(perf_counter()-depth_start)*1000
        def project(p):return project_points(p,frame.intrinsics,sdk_project)
        def marker(point,color,style=cv2.MARKER_CROSS):
            uv=project([point])[0]
            if np.isfinite(uv).all() and 0<=uv[0]<rgb.shape[1] and 0<=uv[1]<rgb.shape[0]:
                cv2.drawMarker(rgb,tuple(np.rint(uv).astype(int)),color,style,16,2)
        def line(a,b,color,arrow=False):
            uv=project([a,b])
            if not np.isfinite(uv).all() or np.abs(uv).max()>100000:return
            a,b=[tuple(np.rint(v).astype(int)) for v in uv]
            if arrow:cv2.arrowedLine(rgb,a,b,color,2,tipLength=.22)
            else:cv2.line(rgb,a,b,color,1)
        if self.show_candidates:
            for candidate in result.get('candidate_diagnostics',[]):
                if candidate['state'] in ('ACCEPTED_OBJECT','CONFIRMED_OBJECT'):continue
                x,y,w,h=candidate['bbox']
                for xx in range(x,x+w,14):
                    cv2.line(rgb,(xx,y),(min(xx+5,x+w),y),(100,100,100),1)
                    cv2.line(rgb,(xx,y+h),(min(xx+5,x+w),y+h),(100,100,100),1)
                text(rgb,candidate['state']+' '+candidate['color'],x,max(12,y-3),(150,150,150),.35)
        for item,payload in zip(result['objects'],views):
            ambiguous=item.get('candidate_state')=='AMBIGUOUS'
            color=(160,160,160) if ambiguous else COLORS[item['color_label']];mask[payload['mask']]=color
            x,y,w,h=item['bbox']
            if ambiguous:
                for xx in range(x,x+w,12):
                    cv2.line(rgb,(xx,y),(min(xx+6,x+w),y),color,1);cv2.line(rgb,(xx,y+h),(min(xx+6,x+w),y+h),color,1)
                for yy in range(y,y+h,12):
                    cv2.line(rgb,(x,yy),(x,min(yy+6,y+h)),color,1);cv2.line(rgb,(x+w,yy),(x+w,min(yy+6,y+h)),color,1)
            else:cv2.rectangle(rgb,(x,y),(x+w,y+h),color,2)
            contours,_=cv2.findContours(np.uint8(payload['mask']),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(rgb,contours,-1,(255,255,255),1)
            cv2.drawMarker(rgb,tuple(np.rint(item['pixel_center']).astype(int)),color,cv2.MARKER_CROSS,18,2)
            # Compact per-instance tags avoid overlapping adjacent batteries.
            # Full object_id is retained in the per-object table and selected panel.
            yy=max(36,y-8);short_id=item['object_id'].rsplit('-',1)[-1]
            tag='CONFIRMED ' if item.get('candidate_state')=='CONFIRMED_OBJECT' else ('? ' if ambiguous else 'DIAG ')
            top_label=f"{tag}{item['color_label'].upper()} {item['surface_depth_z']*1000:.1f}mm"
            id_label=f"ID ..{short_id} C{item['color_confidence']:.2f}"
            label_width=max(cv2.getTextSize(t,0,.42,1)[0][0] for t in (top_label,id_label))+8
            cv2.rectangle(rgb,(max(0,x),yy-34),(min(rgb.shape[1]-1,x+label_width),yy+4),(20,25,30),-1)
            text(rgb,top_label,max(0,x+3),yy-17,color,.42)
            text(rgb,id_label,max(0,x+3),yy,color,.42)
            marker(item['visible_surface_center_xyz'],(255,150,30))
            if self.show_cad and item['body_pose_candidate'] is not None:
                T=np.array(item['body_pose_candidate']);R=T[:3,:3];body=T[:3,3];ring=np.array(item['ring_center_candidate'])
                dims=geometry['battery']['body_dimensions_mm'];half=np.array([dims['long'],dims['width'],dims['height']])/2000
                corners=np.array([[x,y,z] for x in [-half[0],half[0]] for y in [-half[1],half[1]] for z in [-half[2],half[2]]])
                corners_camera=transform_points(corners,T)
                for i in range(8):
                    for j in range(i+1,8):
                        if np.count_nonzero(corners[i]!=corners[j])==1:line(corners_camera[i],corners_camera[j],(220,190,60))
                marker(body,(220,70,220));marker(ring,(0,255,255),cv2.MARKER_DIAMOND)
                line(body,body+R[:,0]*.04,(220,70,220),True)
                line(ring,ring+R[:,1]*.03,(40,160,255),True)
                for direction in item['insertion_direction_candidates']:
                    line(ring,ring+np.array(direction)*.035,(0,255,255),True)
        foreground=np.logical_or.reduce([p['foreground'] for p in views]).astype(np.uint8)*255 if views else None
        if foreground is None:
            # Binary cloud mask is available; not claim the entire plane foreground here.
            foreground=np.any(mask,axis=2).astype(np.uint8)*255
        foreground=cv2.cvtColor(foreground,cv2.COLOR_GRAY2BGR)
        main={'raw':frame.color_bgr,'annotated':rgb,'depth':depth,'mask':mask,'foreground':foreground}.get(self.mode,rgb)
        if self.mode=='cloud':
            main=np.zeros_like(rgb)
            for item,payload in zip(result['objects'],views):
                points=payload['points'][::8]
                # Diagnostic X/Z orthographic scatter, explicitly not a camera image.
                u=np.rint((points[:,0]+.5)*rgb.shape[1]).astype(int)
                v=np.rint(points[:,2]*rgb.shape[0]).astype(int)
                ok=(u>=0)&(u<rgb.shape[1])&(v>=0)&(v<rgb.shape[0])
                main[v[ok],u[ok]]=COLORS[item['color_label']]
            text(main,'POINT CLOUD X/Z ORTHOGRAPHIC | camera m',12,28)
        canvas=np.full((900,1600,3),(22,27,32),np.uint8)
        cv2.rectangle(canvas,(0,0),(1600,65),(32,44,52),-1)
        mode=result.get('mode','DIAGNOSTIC').upper()
        text(canvas,'MEITUAN  |  GENERIC BATTERY RGB-D / CAD',18,28,(240,245,250),.7)
        text(canvas,f"{mode} | view={self.mode} | BODY/RING INVALID | no robot output",18,53,(70,200,255))
        if 'detection_fps' in result:
            def hz(value):return 'N/A' if value is None else f'{value:.1f}'
            text(canvas,f"CAM {hz(result.get('camera_fps'))} | DET {hz(result.get('detection_fps'))} | CAD {hz(result.get('cad_fps'))} FPS",1020,30)
            age=result.get('cad_result_age_ms')
            text(canvas,f"DET {result['processing_latency_ms']:.0f}ms | CAD age {'N/A' if age is None else f'{age:.0f}ms'} | DROP {result.get('capture_dropped_frames',0)}",1020,54)
        else:
            rate=f"{display_fps:.1f}" if display_fps is not None else 'N/A (offline)'
            capture=frame.stream_profiles.get("color",{}).get("fps","N/A")
            capture_label="Capture" if result.get("mode","").startswith("LIVE") else "Recorded"
            processing=1000/max(result["timing_ms"]["total"],1e-9)
            text(canvas,f"{capture_label} {capture} FPS | Process {processing:.2f} Hz | {result['timing_ms']['total']:.0f}ms",1020,30)
            cad_rate=result.get("cad_pose_update_rate_hz");cad_rate_text="N/A offline" if cad_rate is None else f"{cad_rate:.2f}Hz"
            text(canvas,f"CAD updates {cad_rate_text} | limit {result.get('cad_update_limit_hz','N/A')}Hz | REC {self.recording}",1020,54)
        def place(im,x,y,w,h):
            scale=min(w/im.shape[1],h/im.shape[0]);s=cv2.resize(im,(round(im.shape[1]*scale),round(im.shape[0]*scale)))
            canvas[y:y+h,x:x+w]=(12,16,20)
            xx=x+(w-s.shape[1])//2;yy=y+(h-s.shape[0])//2
            canvas[yy:yy+s.shape[0],xx:xx+s.shape[1]]=s
        place(main,12,80,1040,590)
        place(depth,1070,80,245,138);place(mask,1330,80,255,138)
        text(canvas,'Aligned depth (0-1000mm)',1070,239);text(canvas,'Instance masks',1330,239)
        cv2.rectangle(canvas,(12,684),(1052,846),(32,38,43),-1)
        text(canvas,'OBJECTS | white=contour  cyan=CAD  magenta=body  yellow=ring & +/- entry',22,708)
        self.last_display_data={'sample_id':result.get('sample_id'),'objects':result['objects'],
            'labels':[object_label(o) for o in result['objects']], 'selected_index':self.selected,
            'source_frame_id':result.get('source_frame_id'),'camera_fps':result.get('camera_fps'),
            'detection_fps':result.get('detection_fps'),'cad_fps':result.get('cad_fps'),
            'cad_result_age_ms':result.get('cad_result_age_ms'),
            'capture_dropped_frames':result.get('capture_dropped_frames',0)}
        for i,item in enumerate(result['objects'][:6]):
            text(canvas,f"{i+1}. {object_label(item)} | {item['tracking_reliability']} | CAD {item['fit_status']}",22,731+i*19,COLORS[item['color_label']],.42)
        if not result['objects']:
            text(canvas,'NO ACCEPTED OBJECT | '+str(result.get('background_filter_report',{}).get('table_failure')),22,738,(80,130,255))
        if self.show_info and result['objects']:
            item=result['objects'][min(self.selected,len(result['objects'])-1)]
            old=item.get('historical_cad') if not item.get('cad_fresh') else None
            yaw=item['body_yaw_candidate']
            if old is not None:
                # Historic camera coordinates are text only; never project across frames without TF.
                yaw=old.get('body_yaw_candidate')
            pose_age=None if item['cad_result_timestamp'] is None else max(0,(time() if result.get('mode','').startswith('LIVE') else result['timestamp'])-item['cad_result_timestamp'])
            freshness='N/A' if pose_age is None else ('STALE ' if pose_age>1 else 'CURRENT_FRAME ')+f'{pose_age:.2f}s'
            lines=[('OBJECT ID',item['object_id']),('COLOR / confidence',f"{item['color_label']} / {item['color_confidence']:.3f}"),
                ('BBOX px',item['bbox']),('DEPTH VALID',f"{item['depth_valid_ratio']:.1%}"),
                ('SURFACE Z',mm(item['surface_depth_z'])),('SURFACE XYZ camera',vec(item['visible_surface_center_xyz'])),
                ('SURFACE range',mm(item['camera_to_surface_distance'])),
                ('BODY XYZ ESTIMATED' if old is None else 'STALE BODY XYZ',vec(item['model_estimated_body_center_xyz'] if old is None else old.get('body_xyz'))),
                ('BODY Z / range',mm(item['body_center_camera_z'])+' / '+mm(item['camera_to_body_center_distance'])),
                ('BODY YAW candidate' if old is None else 'STALE BODY YAW','N/A' if yaw is None else f"{np.rad2deg(yaw):.2f} deg / {yaw:.4f} rad"),
                ('RING CENTER ESTIMATED' if old is None else 'STALE RING CENTER',vec(item['ring_center_candidate'] if old is None else old.get('ring_xyz'))),
                ('RING feature',item['ring_feature_status']),
                ('HOOK +/- yaw rad' if old is None else 'STALE HOOK +/- XYZ unit vectors',item['hook_insertion_yaw_candidates'] if old is None else old.get('insertion_directions')),
                ('FIT median residual',mm(item['pose_quality'].get('cad_surface_residual_median_m'))),
                ('CAD freshness',f"{item['candidate_state'] if 'candidate_state' in item else 'DIAGNOSTIC'} current={item['cad_fresh']} age={'N/A' if item['cad_result_timestamp'] is None else freshness}" if old is None else f"STALE {old['result_age_ms']:.0f}ms src={old['source_frame_id']} motion UNKNOWN_WITHOUT_TF"),
                ('POSE VALID','body=false ring=false | UNVERIFIED'),
                ('DET / POSE uncertainty','det='+str(item.get('detection_ambiguity','legacy'))+' pose='+str(item.get('pose_ambiguity','legacy'))+' cad_stale='+str(item.get('cad_stale','legacy'))+' det_stale='+str(item.get('detection_stale','unknown'))),
                ('REJECTION',item['rejection_reason'])]
            for i,(name,value) in enumerate(lines):
                y=272+i*30;text(canvas,name,1070,y,(120,155,175),.42)
                # Long reasons wrap onto two lines, rather than cover the image.
                value=str(value)
                text(canvas,value[:70],1070,y+14,(235,235,235),.40)
        cv2.rectangle(canvas,(0,855),(1600,899),(30,39,46),-1)
        text(canvas,'s save | r record/stop | c raw candidates | d depth | m mask | f foreground | p cloud | b raw | o CAD | i info | 1-9 select',16,875,(220,230,235),.46)
        obs=result.get('observation',{})
        footer=('OBSERVATION '+obs.get('state','N/A')+' | distinct CAD '+str(obs.get('distinct_capture_observations',0))) if obs else 'n next | SPACE pause/play offline | q exit'
        text(canvas,footer+' | CAD estimates are NOT validated grasp targets',16,894,(65,190,255),.46)
        self.last_timing_ms={"depth_render":depth_ms,"ui_overlay":(perf_counter()-ui_start)*1000-depth_ms}
        return canvas
