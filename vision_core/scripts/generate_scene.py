"""Write a reproducible scene image and JSON ground truth."""
import argparse
import json
import sys
from pathlib import Path
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from synthetic.scene_generator import SceneGenerator
from synthetic.io import write_png


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--index',type=int,default=1)
    parser.add_argument('--augmentation',choices=['on','off'],default='off')
    parser.add_argument('--mode',choices=['official','random_stress'],default='official')
    parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parents[1]/'outputs')
    args=parser.parse_args()
    scene=SceneGenerator().generate(args.seed,args.augmentation=='on',args.mode)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    stem=f'scene_{args.index:06d}'
    write_png(args.output_dir/f'{stem}.png',scene.image)
    (args.output_dir/f'{stem}_gt.json').write_text(json.dumps(scene.ground_truth,indent=2),encoding='utf-8')
    print(args.output_dir/stem)


if __name__=='__main__': main()
