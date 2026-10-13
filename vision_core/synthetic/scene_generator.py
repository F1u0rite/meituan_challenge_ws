"""Deterministic top-down competition scene generator. Ground truth is private to evaluation."""
from dataclasses import dataclass
from pathlib import Path
import copy
import cv2
import numpy as np
import yaml
from .geometry import rectangle_corners, overlaps
from .augmentation import augment


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'competition.yaml'


def load_config(path=DEFAULT_CONFIG):
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    assert config['renderer_colors']['simulation_only'] is True
    return config


@dataclass
class Scene:
    canonical_image: np.ndarray
    augmented_image: np.ndarray
    ground_truth: dict
    image_to_canonical: np.ndarray

    @property
    def image(self):
        return self.augmented_image


class SceneGenerator:
    def __init__(self, config=None):
        self.config = config or load_config()

    def generate(self, seed=42, augmentation=False, mode='random_stress'):
        cfg = self.config
        rng = np.random.default_rng(seed)
        ppm = cfg['render']['pixels_per_mm']
        board = cfg['board']
        width = round(board['width_mm'] * ppm)
        height = round(board['height_mm'] * ppm)
        image = np.full((height, width, 3), cfg['render']['background_bgr'], np.uint8)
        cv2.rectangle(image, (0,0), (width-1,height-1), (35,35,35), 5)
        region = cfg['battery_region']
        x, y, rw, rh = (region[k] for k in ('x_mm','y_mm','width_mm','height_mm'))
        cv2.rectangle(image, (round(x*ppm),round(y*ppm)), (round((x+rw)*ppm),round((y+rh)*ppm)),
                      cfg['render']['region_bgr'], -1)
        cv2.rectangle(image, (round(x*ppm),round(y*ppm)), (round((x+rw)*ppm),round((y+rh)*ppm)), (70,70,70), 2)
        targets = {}
        for name, center in cfg['targets']['centers_mm'].items():
            center_px = tuple(round(v*ppm) for v in center)
            outer, inner = cfg['targets']['outer_size_mm'], cfg['targets']['inner_size_mm']
            cv2.circle(image, center_px, round(outer*ppm/2), cfg['render']['target_bgr'], -1)
            cv2.circle(image, center_px, round(outer*ppm/2), (60,60,60), 2)
            cv2.circle(image, center_px, round(inner*ppm/2), (255,255,255), -1)
            cv2.circle(image, center_px, round(inner*ppm/2), (60,60,60), 2)
            cv2.putText(image, name, (center_px[0]-18,center_px[1]+7), cv2.FONT_HERSHEY_SIMPLEX, .7, (30,30,30), 2)
            targets[name] = {'x_mm': center[0], 'y_mm': center[1], 'outer_radius_mm': outer/2,
                             'inner_radius_mm': inner/2}
        base = cfg['robot_base']; base_center=tuple(round(v*ppm) for v in base['center_mm'])
        cv2.circle(image,base_center,round(base['diameter_mm']*ppm/2),(230,230,230),-1)
        cv2.circle(image,base_center,round(base['diameter_mm']*ppm/2),(60,60,60),2)
        cv2.putText(image,'ROBOT BASE',(base_center[0]-78,base_center[1]+8),cv2.FONT_HERSHEY_SIMPLEX,.65,(30,30,30),2)
        batteries = []
        polygons = []
        bw, bh = cfg['battery']['width_mm'], cfg['battery']['height_mm']
        colors = list(cfg['renderer_colors']['bgr'])
        if len(colors) != cfg['battery']['count']:
            raise ValueError('Battery count and synthetic color count differ')
        if mode not in ('official','random_stress'):
            raise ValueError("mode must be 'official' or 'random_stress'")
        assigned_colors=list(rng.permutation(colors)) if mode=='official' else colors
        for index, color in enumerate(assigned_colors):
            col, row = index % 2, index // 2
            slot_name='ABCD'[index]
            if mode=='official':
                slot=cfg['battery_slots'][slot_name]; cx,cy=map(float,slot['center_mm']); yaw=float(slot['yaw_deg'])
                polygon=rectangle_corners(cx,cy,bw,bh,yaw)
            else:
                cell_x, cell_y = x + col*rw/2, y + row*rh/2
                cell_w, cell_h = rw/2, rh/2
                for _ in range(10000):
                    yaw = float(rng.uniform(0, 180)); theta = np.deg2rad(yaw)
                    half_x = (abs(np.cos(theta))*bw + abs(np.sin(theta))*bh)/2
                    half_y = (abs(np.sin(theta))*bw + abs(np.cos(theta))*bh)/2
                    if half_x >= cell_w/2 or half_y >= cell_h/2: continue
                    cx = float(rng.uniform(cell_x+half_x, cell_x+cell_w-half_x))
                    cy = float(rng.uniform(cell_y+half_y, cell_y+cell_h-half_y))
                    polygon = rectangle_corners(cx, cy, bw, bh, yaw)
                    if all(not overlaps(polygon, other, gap=1) for other in polygons): break
                else: raise RuntimeError('Unable to place batteries; check configuration')
            polygons.append(polygon)
            points = np.rint(polygon*ppm).astype(np.int32)
            cv2.fillConvexPoly(image, points, cfg['renderer_colors']['bgr'][color])
            cv2.polylines(image, [points], True, (25,25,25), 1)
            cv2.putText(image,slot_name,(round(cx*ppm)-9,round(cy*ppm)+8),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            batteries.append({'id': slot_name, 'slot': slot_name, 'color': color, 'x_mm': cx, 'y_mm': cy, 'yaw_deg': yaw})
        canonical = image.copy()
        if augmentation:
            opts = copy.deepcopy(cfg['augmentation'])
            if augmentation is True:
                for value in opts.values(): value['enabled'] = True
            elif isinstance(augmentation, dict):
                for key, enabled in augmentation.items(): opts[key]['enabled'] = bool(enabled)
            image, H = augment(image, opts, rng)
        else:
            H = np.eye(3)
        gt = {'seed': seed, 'mode':mode, 'batteries': batteries, 'targets': targets,
              'robot_base':{'x_mm':base['center_mm'][0],'y_mm':base['center_mm'][1],'diameter_mm':base['diameter_mm']},
              'coordinate_system': 'origin upper-left; x right; y down; yaw clockwise [0,180)'}
        return Scene(canonical, image, gt, np.linalg.inv(H))


def render_cad_rgbd(geometry,battery_triangles_m,instances,*,width=1280,height=720,fx=700.,table_z=.6):
    """Synthetic z-buffer rasterization of the SAME normalized CAD mesh.
    This fixture is not a RealSense observation or real accuracy evidence.
    instances = [(color, x_camera_m, y_camera_m, yaw_rad), ...].
    """
    from input.realsense_rgbd_source import RGBDFrame
    from perception.rgbd_geometry import CameraIntrinsics,project_points
    from cad_geometry.frames import transform_points
    hsv_hues={'red':0,'blue':110,'yellow':30,'green':60,'unknown':0}
    intr=CameraIntrinsics(width,height,fx,fx,width/2,height/2)
    image=np.full((height,width,3),110,np.uint8);zbuffer=np.full((height,width),table_z,float)
    instance_map=np.zeros((height,width),np.uint16);truth=[]
    body_height=geometry['battery']['body_dimensions_mm']['height']/1000
    triangles=np.asarray(battery_triangles_m)
    for index,(color,x,y,yaw) in enumerate(instances,1):
        c,s=np.cos(yaw),np.sin(yaw)
        R=np.diag([1.,-1.,-1.])@np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=[x,y,table_z-body_height/2]
        camera_tri=transform_points(triangles.reshape(-1,3),T).reshape(-1,3,3)
        bgr=cv2.cvtColor(np.uint8([[[hsv_hues[color],0 if color=='unknown' else 220,210]]]),cv2.COLOR_HSV2BGR)[0,0]
        for tri in camera_tri:
            uv=project_points(tri,intr)
            if not np.isfinite(uv).all():continue
            lo=np.maximum(np.floor(uv.min(axis=0)).astype(int),[0,0])
            hi=np.minimum(np.ceil(uv.max(axis=0)).astype(int),[width-1,height-1])
            if (lo>hi).any():continue
            a,b,d=uv;den=(b[1]-d[1])*(a[0]-d[0])+(d[0]-b[0])*(a[1]-d[1])
            if abs(den)<1e-10:continue
            yy,xx=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
            w0=((b[1]-d[1])*(xx-d[0])+(d[0]-b[0])*(yy-d[1]))/den
            w1=((d[1]-a[1])*(xx-d[0])+(a[0]-d[0])*(yy-d[1]))/den
            w2=1-w0-w1;inside=(w0>=-1e-6)&(w1>=-1e-6)&(w2>=-1e-6)
            inverse=w0/tri[0,2]+w1/tri[1,2]+w2/tri[2,2]
            candidate=np.divide(1.,inverse,out=np.full_like(inverse,np.inf),where=inverse>0)
            region=zbuffer[lo[1]:hi[1]+1,lo[0]:hi[0]+1];front=inside&(candidate<region)
            region[front]=candidate[front]
            image[lo[1]:hi[1]+1,lo[0]:hi[0]+1][front]=bgr
            instance_map[lo[1]:hi[1]+1,lo[0]:hi[0]+1][front]=index
        truth.append({'instance_id':index,'color':color,'body_transform_model':T.tolist()})
    depth=np.rint(zbuffer/.001).astype(np.uint16)
    frame=RGBDFrame(image,depth,intr,.001,{'host_utc':'2020-01-01T00:00:00+00:00'},
        {'source':'SYNTHETIC_CAD_RASTER; NOT REALSENSE','color':{'width':width,'height':height},
         'depth':{'width':width,'height':height},'alignment':'same synthetic optical grid'})
    return frame,truth,instance_map
