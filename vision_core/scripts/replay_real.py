"""Replay independently reviewed validation sessions through PixelBatteryDetector."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from real_data.replay import run_replay

def main():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root', type=Path, default=root/'data'/'real')
    p.add_argument('--output-dir', type=Path, default=root/'outputs'/'real_replay')
    p.add_argument('--color-config', type=Path, default=root/'config'/'realsense_color.yaml')
    p.add_argument('--iou-threshold', type=float, default=.3)
    a = p.parse_args()
    report = run_replay(a.data_root, a.output_dir, a.color_config, iou_threshold=a.iou_threshold)
    print(f'{report["status"]}; reviewed={report["reviewed_images"]}; report={a.output_dir/"replay_report.json"}')

if __name__ == '__main__':
    main()
