"""GUI tool for reviewed polygon, rectangle, brush-mask and negative labels."""
import argparse
import sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from real_data.annotation import annotate, mark_negative
from real_data.dataset import load_raw


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sample', type=Path, help='Folder containing raw.png and metadata.json')
    p.add_argument('--color', choices=('red', 'yellow', 'blue', 'green', 'negative'), required=True)
    p.add_argument('--mode', choices=('polygon', 'rectangle', 'mask'), default='polygon')
    p.add_argument('--brush', type=int, default=12)
    p.add_argument('--yaw-gt-deg', type=float, help='Measured 180-degree body yaw, if available')
    p.add_argument('--yaw-gt-source', help='How physical yaw was measured; required with --yaw-gt-deg')
    a = p.parse_args()
    if a.color == 'negative':
        mark_negative(a.sample)
        print(f'Reviewed negative: {a.sample}')
        return
    image = load_raw(a.sample)
    points = []
    painting = False
    mask = np.zeros(image.shape[:2], np.uint8)
    def mouse(event, x, y, _flags, _userdata):
        nonlocal painting
        if a.mode == 'mask':
            if event == cv2.EVENT_LBUTTONDOWN:
                painting = True
            if event == cv2.EVENT_LBUTTONUP:
                painting = False
            if painting or event == cv2.EVENT_LBUTTONDOWN:
                cv2.circle(mask, (x, y), a.brush, 255, -1)
        elif event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            if a.mode == 'rectangle' and len(points) > 2:
                points.pop(0)
    cv2.namedWindow('Review ROI | Enter save | c clear | q quit', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Review ROI | Enter save | c clear | q quit', mouse)
    try:
        while True:
            view = image.copy()
            if a.mode == 'mask':
                view[mask > 0] = (view[mask > 0]*.55 + np.array([0, 255, 255])*.45).astype(np.uint8)
            else:
                for point in points:
                    cv2.circle(view, point, 4, (0, 255, 255), -1)
                if len(points) >= 2:
                    cv2.polylines(view, [np.asarray(points, np.int32)], a.mode == 'polygon', (0, 255, 255), 2)
            cv2.putText(view, f'{a.color} {a.mode} | click/paint ROI | Enter save | c clear | q quit',
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
            cv2.imshow('Review ROI | Enter save | c clear | q quit', view)
            key = cv2.waitKey(30) & 0xff
            if key == ord('q'):
                break
            if key == ord('c'):
                points.clear(); mask[:] = 0
            if key in (10, 13):
                try:
                    item = annotate(a.sample, a.color, a.mode, vertices=points if a.mode != 'mask' else None,
                                    binary_mask=mask if a.mode == 'mask' else None,
                                    yaw_gt_deg=a.yaw_gt_deg, yaw_gt_source=a.yaw_gt_source)
                except ValueError as exc:
                    print(f'ROI not saved: {exc}')
                else:
                    print(f'Saved annotation {item["annotation_id"]}; repeat for another object or q to quit.')
                    points.clear(); mask[:] = 0
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
