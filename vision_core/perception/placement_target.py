"""T0 target-hole position from a current support plane and a 200x100 mm border."""
import cv2
import numpy as np
from .rgbd_geometry import deproject_pixels, frame_capture_time


class T0PlacementTarget:
    def __init__(self, geometry, *, physical_hole_height_m=None):
        battery = geometry['battery']
        offset = np.asarray(battery['T_battery_from_ring_m'], float)[:3, 3]
        if not np.isfinite(offset).all() or np.linalg.norm(offset[:2]) > 1e-6:
            raise ValueError('T0 requires the CAD hole horizontally centred above the body base')
        self.height = float(offset[2] + battery['body_dimensions_mm']['height'] / 2000.)
        self.height_source = 'CAD_BODY_HALF_HEIGHT_PLUS_BODY_TO_RING_OFFSET'
        if physical_hole_height_m is not None:
            self.height = float(physical_hole_height_m)
            self.height_source = 'USER_CONFIRMED_PHYSICAL_HOLE_HEIGHT'
        if not np.isfinite(self.height) or not 0 < self.height < .2:
            raise ValueError('invalid CAD hole height above base')

    def estimate(self, frame, result):
        target = dict(target_id='T0', candidate_available=False,
            coordinate_frame='camera_color_optical', unit_position='m',
            table_center_camera_m=None, target_hole_center_camera_m=None,
            nominal_hole_height_above_table_m=self.height,
            position_semantics='DESIRED_HOLE_CENTER_AFTER_UPRIGHT_PLACEMENT',
            height_source=self.height_source,
            model_assumptions=['UPRIGHT_BATTERY', 'HORIZONTAL_HOLE_OFFSET_ZERO',
                'PLACE_BODY_BASE_CENTRE_AT_T0_CENTRE'],
            ring_pose_valid=False, motion_usable=False,
            absolute_accuracy='NOT_MEASURED', reasons=[])
        stamp, _ = frame_capture_time(frame)
        sid = f"color_{frame.timestamps['color_frame_number']}_depth_{frame.timestamps['depth_frame_number']}"
        if result.get('source_frame_id') != sid or result.get('timestamps') != frame.timestamps or result.get('capture_timestamp') != stamp:
            raise ValueError('T0 result/source frame mismatch')
        if result.get('pair_qualified') is not True or not result.get('support_plane'):
            target['reasons'] = ['NO_QUALIFIED_CURRENT_SUPPORT_PLANE']
            return target
        plane = result['support_plane']
        normal = np.asarray(plane['normal_camera_xyz'], float)
        point = np.asarray(plane['point_camera_m'], float)
        if normal.shape != (3,) or point.shape != (3,) or not np.isfinite([normal, point]).all() or abs(np.linalg.norm(normal)-1) > 1e-3 or normal[2] >= 0:
            raise ValueError('invalid current upward support normal')
        gray = cv2.cvtColor(frame.color_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        candidates = []
        # Multiple edge thresholds recover faint borders; size/shape and uniqueness
        # checks remain mandatory. No fixed image ROI, colour slot or history.
        for low, high in ((35, 90), (25, 60), (50, 120)):
            contours, _ = cv2.findContours(cv2.Canny(gray, low, high), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            for contour in contours:
                if cv2.contourArea(contour) < 4000:
                    continue
                approx = cv2.approxPolyDP(contour, .02*cv2.arcLength(contour, True), True)
                if len(approx) != 4 or not cv2.isContourConvex(approx):
                    continue
                corners = self._refine(contour, approx[:, 0, :])
                if corners is None:
                    continue
                rays = deproject_pixels(corners, np.ones(4), frame.intrinsics)
                denom = rays @ normal
                if np.any(np.abs(denom) < 1e-6):
                    continue
                depths = (point @ normal) / denom
                if np.any(depths < .15) or np.any(depths > 1.5):
                    continue
                xyz = rays * depths[:, None]
                edges = np.roll(xyz, -1, axis=0)-xyz
                lengths = np.linalg.norm(edges, axis=1)
                span = np.sort([lengths[::2].mean(), lengths[1::2].mean()])
                if np.any(np.abs(span-[.1, .2]) > [.012, .024]):
                    continue
                if any(abs(edges[j] @ edges[(j+1)%4])/(lengths[j]*lengths[(j+1)%4]) > .15 for j in range(4)):
                    continue
                if np.max(np.abs(lengths-np.roll(lengths, 2))) > .01:
                    continue
                candidates.append((xyz.mean(axis=0), corners, span))
        if not candidates:
            target['reasons'] = ['NO_CURRENT_200X100MM_T0_BORDER']
            return target
        centres = np.array([c[0] for c in candidates])
        centre = np.median(centres, axis=0)
        if np.max(np.linalg.norm(centres-centre, axis=1)) > .008:
            target['reasons'] = ['AMBIGUOUS_MULTIPLE_T0_SIZED_BORDERS']
            return target
        target.update(candidate_available=True, table_center_camera_m=centre.tolist(),
            target_hole_center_camera_m=(centre+self.height*normal).tolist(),
            upward_normal_camera_xyz=normal.tolist(),
            observed_border_span_m=np.median([c[2] for c in candidates],axis=0).tolist(),
            border_corners_px=np.median([c[1] for c in candidates],axis=0).tolist(),
            reasons=['T0_IDENTITY_FROM_BORDER_SIZE_AND_UNIQUENESS',
                'PHYSICAL_HEIGHT_CONFIGURED' if self.height_source.startswith('USER_') else 'NOMINAL_CAD_HEIGHT_NOT_PHYSICALLY_MEASURED',
                'ABSOLUTE_POSITION_UNVERIFIED'])
        return target

    @staticmethod
    def _refine(contour, corners):
        corners = np.asarray(corners, float)
        centre = corners.mean(axis=0)
        corners = corners[np.argsort(np.arctan2(corners[:,1]-centre[1],corners[:,0]-centre[0]))]
        pixels = contour[:,0,:].astype(float)
        lines = []
        for j in range(4):
            start, end = corners[j], corners[(j+1)%4]
            direction = end-start
            length = np.linalg.norm(direction)
            if length < 8:
                return None
            fraction = ((pixels-start)@direction)/(length*length)
            distance = np.abs((pixels-start)@np.array([direction[1],-direction[0]]))/length
            points = pixels[(fraction>.15)&(fraction<.85)&(distance<5)]
            if len(points) < 8:
                return None
            vx, vy, x, y = cv2.fitLine(points.astype(np.float32), cv2.DIST_HUBER, 0, .01, .01).ravel()
            lines.append(np.array([vy,-vx,vx*y-vy*x]))
        refined = []
        for j in range(4):
            homogeneous = np.cross(lines[j-1],lines[j])
            if abs(homogeneous[2]) < 1e-6:
                return None
            refined.append(homogeneous[:2]/homogeneous[2])
        return np.array(refined)
