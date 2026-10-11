"""Generate an explicitly synthetic RGB-D format check without camera access."""
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from input.realsense_rgbd_source import RGBDFrame
from perception.rgbd_geometry import CameraIntrinsics, analyze_rgbd
from real_data.rgbd_recording import save_rgbd_sample


def main(argv=None):
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--cad-multi',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--show',action='store_true')
    args=parser.parse_args(argv)
    if args.cad_multi:return cad_multi_demo(args.output,args.show)
    h, w = 240, 320
    color = np.full((h, w, 3), (72, 76, 80), np.uint8)
    mask = np.zeros((h, w), np.uint8)
    cv2.rectangle(mask, (105, 65), (215, 175), 1, -1)
    color[mask > 0] = (30, 35, 190)
    depth = np.full((h, w), 950, np.uint16)
    depth[mask > 0] = 830
    depth[90:96, 120:126] = 0
    intr = CameraIntrinsics(w, h, 280, 280, w/2, h/2)
    result, points, rgb = analyze_rgbd(color, depth, mask, intr, .001)
    result['data_origin'] = 'SYNTHETIC_FORMAT_CHECK'
    frame = RGBDFrame(color, depth, intr, .001,
                      {'host_utc': 'SYNTHETIC', 'color_sdk_ms': None, 'depth_sdk_ms': None},
                      {'color': {'format': 'BGR8', 'width': w, 'height': h},
                       'depth': {'format': 'Z16', 'width': w, 'height': h}})
    debug = color.copy()
    cv2.putText(debug, 'SYNTHETIC RGB-D FORMAT CHECK', (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 255, 255), 1)
    cv2.putText(debug, 'Visible XYZ m: '+', '.join(f'{x:.3f}' for x in result['visible_surface_position_m']),
                (8, 220), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    depth_view = cv2.applyColorMap(np.uint8(depth.astype(np.float32)/1200*255), cv2.COLORMAP_TURBO)
    cv2.putText(depth_view, 'ALIGNED Z16 SYNTHETIC', (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 255, 255), 1)
    folder = ROOT/'outputs'/'rgbd_offline_demo'/'sample_0001'
    save_rgbd_sample(folder, frame, result, points, rgb, np.hstack((debug, depth_view)),
                     scene_label='synthetic_red_top')
    print(folder)


def cad_multi_demo(output=None,show=False):
    import json,uuid
    from datetime import datetime,timezone,timedelta
    from synthetic.scene_generator import render_cad_rgbd
    from perception.realtime_rgbd import RGBDVisionPipeline
    from perception.vision_ui import VisionUI
    from real_data.rgbd_recording import save_vision_sample
    pipeline=RGBDVisionPipeline();ui=VisionUI()
    output=output or ROOT/'outputs/live_rgbd_integration'/('synthetic_cad_'+uuid.uuid4().hex[:8])
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True)
    triangles=np.load(ROOT/'outputs/cad_analysis/cad_meshes.npz')['battery']
    poses=[('red',-.21,-.1,0.),('blue',0.,-.1,.15),('yellow',.21,-.1,-.1),
           ('green',-.21,.1,.1),('red',0.,.1,-.15),('unknown',.21,.1,0.)]
    rows=[];window=False
    try:
        for i in range(2):
            moving=[(color,x+i*.001,y,yaw) for color,x,y,yaw in poses]
            frame,truth,instance_map=render_cad_rgbd(pipeline.geometry,triangles,moving)
            frame.timestamps['host_utc']=(datetime(2020,1,1,tzinfo=timezone.utc)+timedelta(seconds=i*.1)).isoformat()
            result,views=pipeline.process(frame,force_cad=True)
            result.update(mode='SYNTHETIC_CAD_RGBD_NOT_REAL',sample_id=f'sample_{i+1:04d}')
            screenshot=ui.render(frame,result,views,pipeline.geometry)
            folder=output/result['sample_id']
            save_vision_sample(folder,frame,result,views,screenshot)
            (folder/'synthetic_ground_truth.json').write_text(json.dumps(truth,indent=2),encoding='utf-8')
            cv2.imencode('.png',instance_map)[1].tofile(str(folder/'synthetic_instance_map.png'))
            if show:
                cv2.namedWindow('SYNTHETIC generic CAD RGB-D',cv2.WINDOW_NORMAL)
                cv2.resizeWindow('SYNTHETIC generic CAD RGB-D',1600,900)
                cv2.imshow('SYNTHETIC generic CAD RGB-D',screenshot);cv2.waitKey(700);window=True
            rows.append(result)
            print('synthetic',i,[(o['color_label'],o['object_id']) for o in result['objects']],flush=True)
        summary={'source':'SYNTHETIC_CAD_RASTER; NOT_REAL_DEPTH','frames':len(rows),
            'object_counts':[len(r['objects']) for r in rows],
            'colors':[[o['color_label'] for o in r['objects']] for r in rows],
            'same_cad_for_all':True,'pose_valid':False,'gui_window_display_called':window,
            'physical_accuracy':'NOT_MEASURED','camera_started':False}
        (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print(output,json.dumps(summary,indent=2),flush=True)
    finally:
        if show:cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
