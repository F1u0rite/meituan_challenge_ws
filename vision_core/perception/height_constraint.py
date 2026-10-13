"""Known physical heights constrain camera points along the current table normal."""
import numpy as np
from .rgbd_geometry import frame_capture_time


class HeightConstrainedPoints:
    def __init__(self, *, top_height_m=.080, hole_height_m=.060, max_correction_m=.010):
        values = np.asarray([top_height_m, hole_height_m, max_correction_m], float)
        if not np.isfinite(values).all() or not 0 < hole_height_m < top_height_m < .2 or not 0 < max_correction_m <= .03:
            raise ValueError('invalid physical heights/correction limit')
        self.top_height = float(top_height_m)
        self.hole_height = float(hole_height_m)
        self.limit = float(max_correction_m)

    def apply(self, frame, result):
        stamp, _ = frame_capture_time(frame)
        sid = f"color_{frame.timestamps['color_frame_number']}_depth_{frame.timestamps['depth_frame_number']}"
        if result.get('source_frame_id') != sid or result.get('timestamps') != frame.timestamps or result.get('capture_timestamp') != stamp:
            raise ValueError('height constraint/source frame mismatch')
        output = dict(result)
        output['objects'] = [dict(obj) for obj in result['objects']]
        plane = result.get('support_plane')
        normal = point = None
        if result.get('pair_qualified') is True and plane:
            normal = np.asarray(plane['normal_camera_xyz'], float)
            point = np.asarray(plane['point_camera_m'], float)
            if normal.shape != (3,) or point.shape != (3,) or not np.isfinite([normal,point]).all() or abs(np.linalg.norm(normal)-1) > 1e-3 or normal[2] >= 0:
                raise ValueError('invalid upward current support plane')
        output['height_constraint'] = dict(enabled=True,
            physical_top_height_m=self.top_height, physical_hole_height_m=self.hole_height,
            max_correction_m=self.limit, height_source='USER_CONFIRMED_PHYSICAL_DIMENSIONS',
            direction_source='CURRENT_SUPPORT_PLANE_UPWARD_NORMAL',
            accuracy_is_not_measured_by_height_constraint=True)
        for obj in output['objects']:
            available = obj.get('candidate_available') is True and result.get('pair_qualified') is True
            obj['raw_candidate_available'] = available
            obj['coordinate_variant'] = 'RAW_UNCORRECTED'
            obj['height_constraint'] = dict(enabled=True, applied=False,
                status='NO_CURRENT_CANDIDATE', top_correction_m=None, hole_correction_m=None)
            for colour in ('green','red'):
                obj[f'raw_{colour}_point_camera_m'] = None
                obj[f'raw_{colour}_height_m'] = None
            if not available:
                self._clear(obj)
                continue
            top = np.asarray(obj['green_point_camera_m'],float)
            hole = np.asarray(obj['red_point_camera_m'],float)
            if top.shape != (3,) or hole.shape != (3,) or not np.isfinite([top,hole]).all():
                raise ValueError('invalid raw height-constraint coordinates')
            obj['raw_green_point_camera_m'] = top.tolist()
            obj['raw_red_point_camera_m'] = hole.tolist()
            if normal is None:
                self._reject(obj, 'NO_CURRENT_SUPPORT_PLANE')
                continue
            raw_top_height = float((top-point)@normal)
            raw_hole_height = float((hole-point)@normal)
            obj['raw_green_height_m'],obj['raw_red_height_m'] = raw_top_height,raw_hole_height
            d_top = self.top_height-raw_top_height
            d_hole = self.hole_height-raw_hole_height
            obj['height_constraint'].update(top_correction_m=d_top, hole_correction_m=d_hole,
                upward_normal_camera_xyz=normal.tolist(), geometry_diagnostics_reference='RAW_POINTS')
            # The input hole must be the axial CAD offset from the same top.
            if np.linalg.norm((top-hole)-(self.top_height-self.hole_height)*normal) > 1e-5:
                self._reject(obj, 'CAD_TOP_TO_HOLE_OFFSET_MISMATCH')
                continue
            if max(abs(d_top),abs(d_hole)) > self.limit:
                self._reject(obj, 'HEIGHT_CORRECTION_EXCEEDS_CONFIGURED_LIMIT')
                continue
            obj.update(green_point_camera_m=(top+d_top*normal).tolist(),
                red_point_camera_m=(hole+d_hole*normal).tolist(),
                green_height_m=self.top_height, red_height_m=self.hole_height,
                coordinate_variant='HEIGHT_CONSTRAINED',
                body_pose_valid=False,ring_pose_valid=False,motion_usable=False)
            obj['height_constraint'].update(applied=True,status='APPLIED')
        output['missing_colors'] = [o['color'] for o in output['objects'] if not o['candidate_available']]
        return output

    @staticmethod
    def _clear(obj):
        obj.update(candidate_available=False,green_point_camera_m=None,
            red_point_camera_m=None,green_height_m=None,red_height_m=None)

    @classmethod
    def _reject(cls,obj,reason):
        cls._clear(obj)
        obj['height_constraint']['status'] = reason
        obj['reasons'] = list(obj.get('reasons',[]))+[reason]
