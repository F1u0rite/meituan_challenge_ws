"""Assign an entire real capture session to calibration or validation."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from real_data.dataset import assign_split

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    p.add_argument('split', choices=('calibration', 'validation'))
    a = p.parse_args()
    info = assign_split(a.session, a.split)
    print(f'{info["session_id"]}: {info["split"]}')

if __name__ == '__main__':
    main()
