import cv2
import numpy as np

from evaluation.pixel_stability import PixelStabilityRecorder
from perception.battery_only import PixelBatteryDetector


def test_pixel_batteries_and_no_mm_coordinates():
    image = np.full((720, 1280, 3), 30, np.uint8)
    colors = {'red': (0, 0, 255), 'yellow': (0, 255, 255),
              'blue': (255, 0, 0), 'green': (0, 255, 0)}
    for index, bgr in enumerate(colors.values()):
        x = 100 + index*240
        cv2.rectangle(image, (x, 250), (x+90, 380), bgr, -1)
    found = PixelBatteryDetector().detect(image)
    assert {d.color for d in found} == set(colors)
    assert all(0 <= d.u < 1280 and 0 <= d.v < 720 for d in found)
    assert all(not hasattr(d, 'x_mm') and not hasattr(d, 'y_mm') for d in found)
    assert all(d.box_px.shape == (4, 2) and 0 <= d.yaw_deg < 180 for d in found)


def test_pixel_recording_summary(tmp_path):
    image = np.full((720, 1280, 3), 30, np.uint8)
    cv2.rectangle(image, (100, 250), (190, 380), (0, 0, 255), -1)
    found = PixelBatteryDetector().detect(image)
    recorder = PixelStabilityRecorder(tmp_path/'battery.csv')
    recorder.record('t1', 1, found)
    recorder.record('t2', 2, found)
    recorder.record('t3', 3, [])
    path, summary = recorder.stop()
    assert summary['coordinate_mode'] == 'IMAGE_PIXEL'
    assert summary['colors']['red']['detection_rate'] == 2/3
    assert summary['colors']['red']['color_consistency'] == 1
    assert summary['colors']['red']['u_px']['std'] == 0
    assert path.exists()
