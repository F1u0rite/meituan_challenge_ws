import json
from pathlib import Path
import cv2
import numpy as np
import pytest
import yaml

from input.realsense_source import RealSenseSource
from evaluation.temporal_stability import orientation_stats
from perception.battery_only import PixelBatteryDetector
from real_data.annotation import annotate, mark_negative, read_annotation, roi_mask
from real_data.calibration import describe, hue_arc, run_calibration
from real_data.dataset import assign_split, create_session, save_sample, iter_samples
from real_data.quality import diagnose
from real_data.replay import match_one_to_one, run_replay, yaw_error_180


def color_image(color=(0, 0, 255)):
    image = np.full((480, 640, 3), 135, np.uint8)
    cv2.rectangle(image, (170, 140), (310, 360), color, -1)
    return image


def test_quality_dark_empty_and_camera_missing_options():
    with pytest.raises(ValueError):
        diagnose(np.empty((0, 0, 3), np.uint8))
    assert diagnose(np.zeros((480, 640, 3), np.uint8))['status'] == 'REJECTED'
    source = RealSenseSource()
    with pytest.raises(RuntimeError):
        source.camera_metadata()

    class Device:
        def get_info(self, _key):
            return 'D435i'
        def query_sensors(self):
            return []
    class Profile:
        def get_device(self):
            return Device()
        def get_stream(self, _stream):
            raise RuntimeError('no profile')
    class Keys:
        name = 1
        serial_number = 2
    class Stream:
        color = 1
    class SDK:
        camera_info = Keys()
        stream = Stream()
    source._started = True
    source._profile = Profile()
    source._rs = SDK()
    camera = source.camera_metadata()
    assert camera['exposure'] == 'UNAVAILABLE'
    assert camera['white_balance'] == 'UNAVAILABLE'
    assert camera['fps'] == 'UNAVAILABLE'


def test_roi_polygon_rectangle_mask_and_empty():
    shape = (100, 100, 3)
    polygon = roi_mask(shape, 'polygon', vertices=[(10,10), (80,10), (40,80)])
    rectangle = roi_mask(shape, 'rectangle', vertices=[(10,10), (80,80)])
    painted = roi_mask(shape, 'mask', binary_mask=polygon)
    assert polygon.sum() > 0 and rectangle.sum() > polygon.sum()
    assert np.array_equal(polygon, painted)
    with pytest.raises(ValueError):
        roi_mask(shape, 'polygon', vertices=[])
    with pytest.raises(ValueError):
        roi_mask(shape, 'mask', binary_mask=np.zeros((100,100), np.uint8))


def test_hsv_percentiles_and_red_wrap():
    arc = hue_arc(np.array([178, 179, 0, 1, 2]*20), 5)
    assert len(arc) == 2
    assert arc[0][0] == 0 and arc[1][1] == 179
    stats = describe([1, 2, 3, 4, 5])
    assert stats['median'] == 3 and stats['p95'] > 4
    assert orientation_stats([179, 1])['std_deg'] < 2


def test_session_split_and_metadata_not_ground_truth(tmp_path):
    session = create_session(tmp_path, 'red', 'static')
    sample, metadata = save_sample(session, color_image(), camera_metadata={'exposure': 'UNAVAILABLE'})
    assert metadata['camera']['exposure'] == 'UNAVAILABLE'
    assert metadata['ground_truth_annotation'] == 'NOT ANNOTATED'
    assert read_annotation(sample) is None
    annotate(sample, 'red', 'rectangle', vertices=[(170,140), (310,360)])
    assert len(read_annotation(sample)['objects']) == 1
    assign_split(session, 'calibration')
    with pytest.raises(ValueError):
        assign_split(session, 'validation')
    assert len(list(iter_samples(tmp_path, 'calibration'))) == 1
    assert len(list(iter_samples(tmp_path, 'validation'))) == 0


def test_four_class_config_and_empty_image(tmp_path):
    detector = PixelBatteryDetector()
    colors = ((0,0,255), (0,255,255), (255,0,0), (0,255,0))
    image = np.full((480, 1000, 3), 75, np.uint8)
    for index, color in enumerate(colors):
        cv2.rectangle(image, (50+index*230, 140), (150+index*230, 350), color, -1)
    assert {d.color for d in detector.detect(image)} == {'red','yellow','blue','green'}
    assert detector.detect(np.full((480,640,3), 75, np.uint8)) == []
    with pytest.raises(ValueError):
        detector.detect(np.empty((0,0,3), np.uint8))
    config = yaml.safe_load(Path(detector.config_path if hasattr(detector, 'config_path') else
                                 Path(__file__).resolve().parents[1]/'config'/'realsense_color.yaml').read_text())
    path = tmp_path/'copy.yaml'
    path.write_text(yaml.safe_dump(config))
    assert PixelBatteryDetector(path).config['hsv_ranges'] == config['hsv_ranges']


def test_calibration_replay_confusion_and_failures(tmp_path):
    root = tmp_path/'data'
    calibration = create_session(root, 'red', 'static')
    cal_sample, _ = save_sample(calibration, color_image())
    annotate(cal_sample, 'red', 'polygon', vertices=[(170,140), (310,140), (310,360), (170,360)])
    assign_split(calibration, 'calibration')
    validation = create_session(root, 'red', 'static')
    val_sample, _ = save_sample(validation, color_image())
    annotate(val_sample, 'red', 'rectangle', vertices=[(170,140), (310,360)],
             yaw_gt_deg=179, yaw_gt_source='fixture_protractor')
    assign_split(validation, 'validation')
    negative = create_session(root, 'negative', 'static')
    neg_sample, _ = save_sample(negative, color_image())
    mark_negative(neg_sample)
    assign_split(negative, 'validation')
    report = run_calibration(root, tmp_path/'calibration',
                             Path(__file__).resolve().parents[1]/'config'/'realsense_color.yaml')
    assert report['colors']['red']['sample_count'] == 1
    assert report['colors']['blue']['status'] == 'NOT MEASURED'
    config = tmp_path/'calibration'/'suggested_realsense_color.yaml'
    assert yaml.safe_load(config.read_text())['provenance']['status'] == 'provisional'
    preliminary = run_replay(root, tmp_path/'preliminary', config)
    assert preliminary['status'] == 'INSUFFICIENT DATA'
    assert preliminary['metrics']['red']['precision'] is None
    result = run_replay(root, tmp_path/'replay', config,
                        min_validation_sessions=1, min_annotated_objects=1)
    assert result['status'] == 'VALIDATION RUN'
    assert result['metrics']['red']['tp'] == 1
    assert result['metrics']['red']['fp'] == 1
    assert result['metrics']['red']['precision'] == .5
    assert result['metrics']['red']['center_error_px'] is not None
    assert result['metrics']['red']['iou'] is not None
    assert result['metrics']['red']['yaw_error_deg'] is not None
    assert yaw_error_180(1,179) == 2
    assert result['confusion_matrix']['background']['red'] == 1
    assert (tmp_path/'replay'/'failure_contact_sheet.png').exists()
    assert list((tmp_path/'replay'/'failures').rglob('raw.png'))
    assert json.loads((tmp_path/'replay'/'replay_report.json').read_text())['status'] == 'VALIDATION RUN'


def test_one_to_one_matching_and_insufficient_data(tmp_path):
    mask = np.zeros((20,20), bool); mask[2:10,2:10] = True
    from perception.battery_only import PixelBatteryDetection
    contour = np.array([[[2,2]], [[9,2]], [[9,9]], [[2,9]]], np.int32)
    pred = [PixelBatteryDetection('red', 5,5,0,.9,contour,np.zeros((4,2))) for _ in range(2)]
    matches, missing_gt, extra_pred = match_one_to_one([mask], pred, mask.shape)
    assert len(matches) == 1 and len(extra_pred) == 1 and not missing_gt
    config = Path(__file__).resolve().parents[1]/'config'/'realsense_color.yaml'
    report = run_replay(tmp_path/'empty', tmp_path/'out', config)
    assert report['status'] == 'INSUFFICIENT DATA'
    assert report['metrics']['red']['precision'] is None
    assert report['metrics']['red']['center_error_px'] is None


def test_rejected_capture_excluded_from_hsv(tmp_path):
    root = tmp_path/'data'
    session = create_session(root, 'red', 'static')
    sample, metadata = save_sample(session, np.zeros((480,640,3), np.uint8))
    annotate(sample, 'red', 'rectangle', vertices=[(30,30), (200,200)])
    assign_split(session, 'calibration')
    assert metadata['image_quality']['status'] == 'REJECTED'
    report = run_calibration(root, tmp_path/'out',
                             Path(__file__).resolve().parents[1]/'config'/'realsense_color.yaml')
    assert report['colors']['red']['status'] == 'NOT MEASURED'
