"""Nominal mounting geometry: optical origin, axes and estimation status."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts/calibration'))
AVAILABLE = importlib.util.find_spec('derive_nominal_mount') is not None
if AVAILABLE:
    from derive_nominal_mount import derive
    from geometry import Transform, CalibrationError


def design():
    return {'data_kind': 'MOUNT_DESIGN', 'length_unit': 'm',
            'front_glass_center_in_F': [0.100, 0, 0.040],
            'optical_axes_in_F_xyzw': [0, 0, 0, 1]}


class MountFeatureGate(unittest.TestCase):
    def test_nominal_mount_feature_exists(self):
        self.assertTrue(AVAILABLE, 'nominal mounting derivation not implemented')


@unittest.skipUnless(AVAILABLE, 'feature not implemented')
class MountDesignTests(unittest.TestCase):
    def test_optical_origin_is_left_imager_behind_front_glass(self):
        result = derive(design())
        for actual, expected in zip(result['T_F_depth_optical']['translation'], [0.0825, 0, 0.0358]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(result['status'], 'ESTIMATED')
        self.assertFalse(result['deployment_ready'])
        self.assertFalse(result['motion_authorized'])

    def test_sensor_offset_rotates_with_camera_housing(self):
        d = design()
        z = math.sqrt(0.5)
        d['optical_axes_in_F_xyzw'] = [0, 0, z, z]
        actual = derive(d)['T_F_depth_optical']['translation']
        for a, e in zip(actual, [0.100, -0.0175, 0.0358]):
            self.assertAlmostEqual(a, e)

    def test_ros_camera_axes_recover_designed_optical_axes(self):
        result = derive(design())
        d = result['T_F_camera_link_nominal']
        t = Transform(d['translation'], d['rotation_xyzw'])
        optical = t.compose(Transform([0, 0, 0], [-0.5, 0.5, -0.5, 0.5]))
        self.assertAlmostEqual(abs(optical.rotation[3]), 1)
        self.assertEqual(tuple(optical.translation), tuple(result['T_F_depth_optical']['translation']))

    def test_real_data_and_mm_are_not_accepted_as_mount_design(self):
        for key, value in [('data_kind', 'REAL'), ('length_unit', 'mm')]:
            d = design()
            d[key] = value
            with self.subTest(key=key), self.assertRaises(CalibrationError):
                derive(d)
