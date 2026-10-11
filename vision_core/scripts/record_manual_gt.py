"""Record only independently measured values. This never reads predictions."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from real_data.pose_experiment import write_json,now
from evaluation.pose_accuracy import validate_gt
def record(path,case_id,values):
    path=Path(path);data=json.loads(path.read_text(encoding='utf-8-sig'))
    if case_id not in data:raise ValueError('Unknown case')
    error=validate_gt(values)
    if error:raise ValueError(error)
    # Audit each previous record before the user explicitly replaces it.
    history=path.parent/'gt_history';history.mkdir(exist_ok=True)
    write_json(history/(case_id+'_'+now().replace(':','').replace('.','')+'.json'),data[case_id])
    data[case_id]=values;write_json(path,data)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--gt',type=Path,required=True);p.add_argument('--case',required=True)
    p.add_argument('--x-mm',type=float,required=True);p.add_argument('--y-mm',type=float,required=True);p.add_argument('--yaw-deg',type=float,required=True)
    p.add_argument('--source',required=True);p.add_argument('--resolution-mm',type=float,required=True)
    p.add_argument('--xy-uncertainty-mm',type=float,required=True);p.add_argument('--yaw-uncertainty-deg',type=float,required=True)
    p.add_argument('--confirm-independently-measured',action='store_true',required=True)
    a=p.parse_args();record(a.gt,a.case,{'coordinate_frame':'fixture','unit':'mm','reference_point':'body_center_projection',
        'xy_mm':[a.x_mm,a.y_mm],'body_yaw_deg':a.yaw_deg,'measurement_source':a.source,
        'measurement_resolution_mm':a.resolution_mm,'xy_uncertainty_mm':a.xy_uncertainty_mm,
        'yaw_uncertainty_deg':a.yaw_uncertainty_deg,'status':'human_measured_confirmed','independently_measured':True,'recorded_at':now()})
