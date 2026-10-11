"""Generate provisional real HSV statistics without overwriting live thresholds."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from real_data.calibration import run_calibration

def main():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root', type=Path, default=root/'data'/'real')
    p.add_argument('--output-dir', type=Path, default=root/'outputs'/'real_calibration')
    p.add_argument('--base-config', type=Path, default=root/'config'/'realsense_color.yaml')
    a = p.parse_args()
    report = run_calibration(a.data_root, a.output_dir, a.base_config)
    print(f'{report["status"]}; source sessions={len(report["session_ids"])}; output={a.output_dir}')

if __name__ == '__main__':
    main()
