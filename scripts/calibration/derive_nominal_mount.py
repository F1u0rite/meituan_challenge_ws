#!/usr/bin/env python3
"""D435i layout estimate in physical design frame F; does not publish TF."""
import argparse
import json
from pathlib import Path

from geometry import CalibrationError, Transform


def as_dict(t):
    return {'translation': list(t.translation), 'rotation_xyzw': list(t.rotation)}


def derive(document):
    if not isinstance(document, dict) or document.get('data_kind') != 'MOUNT_DESIGN':
        raise CalibrationError('INVALID_DESIGN', 'require MOUNT_DESIGN; no measured calibration is inferred')
    if document.get('length_unit') != 'm':
        raise CalibrationError('UNIT_MISMATCH', 'design positions must be m')
    # H: front-glass housing center, axes parallel to optical axes.
    # Factory drawing dimensions are nominal/reference dimensions, not per-device calibration.
    t_f_h = Transform(document.get('front_glass_center_in_F'), document.get('optical_axes_in_F_xyzw'))
    t_h_d = Transform([-0.0175, 0, -0.0042], [0, 0, 0, 1])
    t_f_d = t_f_h.compose(t_h_d)
    # ROS camera_link and depth optical origins nominally coincide; verify actual driver TF.
    t_l_d = Transform([0, 0, 0], [-0.5, 0.5, -0.5, 0.5])
    return {'status': 'ESTIMATED', 'data_kind': 'MOUNT_DESIGN',
            'reference_frame': 'flange_design_F', 'length_unit': 'm',
            'quaternion_order': 'xyzw', 'T_F_front_glass_center': as_dict(t_f_h),
            'T_H_depth_optical_nominal': as_dict(t_h_d),
            'T_F_depth_optical': as_dict(t_f_d),
            'T_F_camera_link_nominal': as_dict(t_f_d.compose(t_l_d.inverse())),
            'deployment_ready': False, 'motion_authorized': False,
            'pending': ['verify flange_design_F relative to deployed robot frame',
                        'check V1 tool and bracket geometry, holes, occlusion and cable space',
                        'read actual depth/color/internal TF from this D435i',
                        'verify installation and calibration errors against task tolerance']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('design', type=Path)
    args = parser.parse_args()
    try:
        result = derive(json.loads(args.design.read_text(encoding='utf-8-sig')))
    except (CalibrationError, OSError, ValueError, TypeError) as error:
        print(json.dumps({'status': 'REJECTED', 'code': getattr(error, 'code', 'INVALID_INPUT'),
                          'message': str(error), 'motion_authorized': False}, ensure_ascii=True))
        return 2
    print(json.dumps(result, indent=2, ensure_ascii=True, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
