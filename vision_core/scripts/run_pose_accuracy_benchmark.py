"""GT accuracy or explicit NOT_MEASURED; retain every failed sample."""
import sys,argparse,json,shutil,csv
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np,cv2
from evaluation.pose_accuracy import compare_pose,aggregate
from real_data.pose_experiment import verify_integrity,write_json
from real_data.rgbd_recording import load_rgbd_sample
from scripts.analyze_cad_rgbd import png
def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))
def run(session,output,diagnostics=None):
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True);records=[];thumbs=[]
    for folder in sorted(session.glob('sample_*')):
        frame=None
        try:
            frame=load_rgbd_sample(folder)
            if (folder/'experiment_metadata.json').is_file():
                meta=read(folder/'experiment_metadata.json');integrity=verify_integrity(folder)
                prediction=read(folder/'prediction.json');gt=read(folder/'fixture_gt.json');calibration=read(folder/'fixture_calibration.json')
            else:
                meta={'case_id':folder.name,'relative_static_confirmed':False}
                integrity={'valid':True,'status':'LEGACY_SESSION_NO_CAPTURE_HASH_MANIFEST'}
                diagnostic=read(diagnostics/folder.name/'diagnostic.json') if diagnostics and (diagnostics/folder.name/'diagnostic.json').is_file() else None
                fit=diagnostic.get('baseline_fit',{}) if diagnostic else {}
                prediction={'body_pose_candidate':fit.get('pose',{}).get('body_pose'),'unit':'m','coordinate_frame':'camera_color_optical',
                    'depth_valid':bool(fit.get('pose',{}).get('body_pose')),'multiple_solutions':fit.get('weak_constraint_flag',True),
                    'insertion_direction_resolved':False}
                gt=None;calibration=None
        except (OSError,ValueError,KeyError,TypeError) as error:
            meta={'case_id':folder.name,'relative_static_confirmed':False}
            integrity={'valid':False,'exception':str(error)}
            prediction=None;gt=None;calibration=None
        result=compare_pose(prediction,gt,calibration)
        if not integrity['valid']:result.update(status='NOT_MEASURED',reasons=['DATASET_INTEGRITY_FAILURE'])
        row={'sample_id':folder.name,'case_id':meta.get('case_id',folder.name),'relative_static_confirmed':meta.get('relative_static_confirmed',False),
             'evaluation':result,'integrity':integrity,'GT':gt,'prediction':prediction}
        records.append(row)
        if result['reasons'] or result['status']=='NOT_MEASURED':
            dest=output/'failures'/folder.name;dest.mkdir(parents=True)
            if (folder/'raw_color.png').exists():shutil.copyfile(folder/'raw_color.png',dest/'raw_color.png')
            image=frame.color_bgr.copy() if frame is not None else np.zeros((720,1280,3),np.uint8)
            for i,reason in enumerate(result['reasons']):
                cv2.putText(image,reason,(15,35+i*30),0,.7,(0,255,255),2)
            if (folder/'visualization.png').exists():shutil.copyfile(folder/'visualization.png',dest/'prediction_overlay.png')
            png(dest/'failure.png',image);write_json(dest/'failure.json',row)
            thumbs.append(cv2.resize(image,(384,216)))
    if not records:raise ValueError('No samples found')
    summary=aggregate(records);write_json(output/'summary.json',summary);write_json(output/'cases.json',records)
    fields=['sample','case','status','x_mm','y_mm','xy_mm','yaw_deg','reasons']
    with (output/'errors.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for r in records:
            e=r['evaluation'];w.writerow(dict(zip(fields,[r['sample_id'],r['case_id'],e['status'],e.get('x_error_mm'),
                e.get('y_error_mm'),e['xy_error_mm'],e['yaw_error_deg'],';'.join(e['reasons'])])))
    if thumbs:
        while len(thumbs)%4:thumbs.append(np.zeros((216,384,3),np.uint8))
        png(output/'failure_contact_sheet.png',np.vstack([np.hstack(thumbs[i:i+4]) for i in range(0,len(thumbs),4)]))
    print(json.dumps(summary,indent=2));return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--diagnostics',type=Path);a=p.parse_args();run(a.session,a.output,a.diagnostics)
