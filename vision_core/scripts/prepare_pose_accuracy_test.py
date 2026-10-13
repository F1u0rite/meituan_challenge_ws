"""Create unfilled manual-GT pilot templates; no camera or fabricated reference."""
import sys,json,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from real_data.pose_experiment import write_json
def prepare(output,repeats=3):
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    if repeats<2:raise ValueError('Need repeated observations')
    output.mkdir(parents=True)
    cases=[];gt={}
    for pos in [(0,0),(30,0),(0,30)]:
        for yaw in [0,30,90]:
            case_id=f'pilot_{len(cases)+1:02d}'
            cases.append({'case_id':case_id,'proposed_xy_mm':pos,'proposed_body_yaw_deg':yaw,'repeat_frames':repeats})
            gt[case_id]={'coordinate_frame':'fixture','unit':'mm','reference_point':'body_center_projection',
                'xy_mm':None,'body_yaw_deg':None,'insertion_yaw_deg':None,'measurement_source':None,
                'measurement_resolution_mm':None,'xy_uncertainty_mm':None,'yaw_uncertainty_deg':None,
                'independently_measured':False,'status':'NOT_MEASURED'}
    plan={'schema_version':1,'color':'red','proposed_camera_distance_m':.4,'camera_distance_status':'TO_BE_CHECKED_WITH_ACTUAL_PROFILE',
        'camera_must_be_fixed':True,'cases':cases,'expected_frames':len(cases)*repeats,
        'fixture_frame':{'origin':'physically measured fixture origin','X':'ruler baseline, marked +X',
            'Y':'orthogonal in table plane, right-handed','Z':'table normal upward',
            'unit':'mm','reference_point':'body_center_projection','yaw_zero':'body long axis along +X','yaw_positive':'CCW viewed from +Z'},
        'gt_policy':'PROPOSED_PLACEMENT_IS_NOT_GT; MEASURE_AND_CONFIRM_EACH_CASE'}
    write_json(output/'pilot_plan.json',plan);write_json(output/'manual_gt.json',gt)
    write_json(output/'fixture_calibration.json',{'from_frame':'camera_color_optical','to_frame':'fixture','unit':'m',
        'T_fixture_from_camera_m':None,'source':None,'status':'NOT_CALIBRATED','uncertainty':None})
    write_json(output/'calibration_points_template.json',{'camera_points_m':None,'fixture_points_m':None,
        'heldout_camera_points_m':None,'heldout_fixture_points_m':None,
        'source':None,'measurement_uncertainty_mm':None,'same_camera_depth_correlation':True})
    return plan
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'outputs/pose_accuracy/pilot_red_v1')
    p.add_argument('--repeats',type=int,default=3);a=p.parse_args();print(json.dumps(prepare(a.output,a.repeats),indent=2))
