"""Canonical or complete synthetic camera perspective benchmark."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from synthetic.scene_generator import SceneGenerator,load_config
from synthetic.camera_projection import CameraProjection
from synthetic.io import write_png
from perception.board_detector import BoardDetector
from perception.board_rectifier import BoardRectifier
from perception.battery_detector import BatteryDetector
from evaluation.evaluator import Evaluator,aggregate
from evaluation.board_evaluator import evaluate_board,aggregate_boards

def main():
    p=argparse.ArgumentParser(); p.add_argument('--num-scenes',type=int,default=1000)
    p.add_argument('--seed',type=int,default=42); p.add_argument('--camera-projection',action='store_true')
    p.add_argument('--augmentation',choices=['on','off'],default='off')
    p.add_argument('--mode',choices=['official','random_stress'],default='random_stress')
    p.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parents[1]/'outputs')
    a=p.parse_args(); cfg=load_config(); gen=SceneGenerator(cfg); detector=BatteryDetector(cfg); evaluator=Evaluator(cfg)
    projector=CameraProjection(cfg); board_detector=BoardDetector(cfg); rectifier=BoardRectifier(cfg)
    battery_results=[]; board_results=[]; failure_count=0; failure_dir=a.output_dir/'perspective_failures'
    for i in range(a.num_scenes):
        seed=a.seed+i; scene=gen.generate(seed,a.augmentation=='on',a.mode); image=scene.image
        if a.camera_projection:
            camera=projector.project(scene.canonical_image,seed); board=board_detector.detect(camera.image)
            board_results.append(evaluate_board(board,camera.board_corners_gt_px))
            if not board.detected:
                failure_count+=1
                if failure_count<=20: write_png(failure_dir/f'board_failure_{seed}.png',camera.image)
                battery_results.append(evaluator.evaluate(scene.ground_truth,[])); continue
            image=rectifier.rectify(camera.image,board.corners_px).image
        result=evaluator.evaluate(scene.ground_truth,detector.detect(image)); battery_results.append(result)
        if result.failed_cases:
            failure_count+=1
            if failure_count<=20: write_png(failure_dir/f'battery_failure_{seed}.png',image)
    report=aggregate(battery_results)
    if a.camera_projection: report['board']=aggregate_boards(board_results)
    report['perspective_failure_scenes']=failure_count
    a.output_dir.mkdir(parents=True,exist_ok=True)
    name=('official_perspective_benchmark.json' if a.mode=='official' else 'perspective_benchmark.json') if a.camera_projection else 'benchmark.json'
    (a.output_dir/name).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
