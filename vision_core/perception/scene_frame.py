"""Current-frame T0 scene reference; never reuse a saved camera transform."""
import numpy as np
import cv2
from .rgbd_geometry import deproject_pixels, frame_capture_time, validate_frame


def _vector(value, size, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError('invalid ' + name)
    return value


class SceneFrameInitializer:
    """T0 centre origin, signed long-edge X, Y = Z cross X, table-up Z.

    A fixed, coplanar ArUco marker's corner 0->1 must point along scene +X.
    Alternatively a current-image point indicates T0's positive long end.
    Pixel references are valid only in this initialization invocation.
    """
    def __init__(self, *, positive_x_pixel=None, aruco_id=None,
                 aruco_dictionary='DICT_4X4_50', min_stable_frames=5,
                 max_translation_m=.003, max_rotation_deg=1.):
        if positive_x_pixel is not None and aruco_id is not None:
            raise ValueError('choose one direction reference')
        if isinstance(min_stable_frames, bool) or not isinstance(min_stable_frames, int) or min_stable_frames < 2:
            raise ValueError('at least two stable frames required')
        limits = _vector([max_translation_m, max_rotation_deg], 2, 'stability limits')
        if np.any(limits <= 0):
            raise ValueError('stability limits must be positive')
        self.pixel = None if positive_x_pixel is None else _vector(positive_x_pixel, 2, 'positive X pixel')
        self.marker_id = aruco_id
        self.marker_detector = None
        if aruco_id is not None:
            if not isinstance(aruco_id, int) or isinstance(aruco_id, bool) or aruco_id < 0:
                raise ValueError('invalid ArUco ID')
            allowed = {'DICT_4X4_50', 'DICT_4X4_100', 'DICT_5X5_50', 'DICT_6X6_50'}
            if aruco_dictionary not in allowed:
                raise ValueError('unsupported ArUco dictionary')
            dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, aruco_dictionary))
            if aruco_id >= len(dictionary.bytesList):
                raise ValueError('ArUco ID exceeds dictionary')
            self.marker_detector = cv2.aruco.ArucoDetector(dictionary)
        self.dictionary = aruco_dictionary
        self.required = min_stable_frames
        self.translation_limit, self.rotation_limit = map(float, limits)
        self.anchor = None
        self.count = 0
        self.last_stamp = None
        self.pixel_expired = False

    def _reset(self):
        self.anchor = None
        self.count = 0

    @staticmethod
    def _intersections(pixels, frame, normal, origin):
        pixels = np.asarray(pixels, float).reshape(-1, 2)
        if not np.isfinite(pixels).all() or np.any(pixels < 0) or np.any(pixels[:, 0] >= frame.intrinsics.width) or np.any(pixels[:, 1] >= frame.intrinsics.height):
            raise ValueError('reference pixels outside current image')
        rays = deproject_pixels(pixels, np.ones(len(pixels)), frame.intrinsics)
        denom = rays @ normal
        if np.any(abs(denom) < 1e-6):
            raise ValueError('reference rays parallel to table')
        depth = (origin @ normal) / denom
        if np.any(depth < .15) or np.any(depth > 1.5):
            raise ValueError('reference intersection out of range')
        return rays * depth[:, None]

    def _direction(self, frame, normal, origin):
        if self.pixel is not None:
            point = self._intersections([self.pixel], frame, normal, origin)[0]
            return point - origin, 'CURRENT_IMAGE_POSITIVE_LONG_END', None
        if self.marker_detector is None:
            return None, 'DIRECTION_REFERENCE_REQUIRED', None
        corners, ids, _ = self.marker_detector.detectMarkers(frame.color_bgr)
        matches = [] if ids is None else np.flatnonzero(ids.ravel() == self.marker_id)
        if len(matches) != 1:
            return None, 'ARUCO_MISSING_OR_DUPLICATE', None
        pixels = corners[int(matches[0])].reshape(4, 2)
        xyz = self._intersections(pixels, frame, normal, origin)
        # Reject a marker placed above the table using observed interior depth.
        mask = np.zeros(frame.aligned_depth.shape, np.uint8)
        cv2.fillConvexPoly(mask, np.rint(pixels).astype(np.int32), 1)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8)).astype(bool)
        v, u = np.where(mask & (frame.aligned_depth > 0))
        if len(v) < 30 or len(v) < .6 * max(1, mask.sum()):
            return None, 'ARUCO_INSUFFICIENT_TABLE_DEPTH', pixels.tolist()
        pick = np.linspace(0, len(v) - 1, min(500, len(v)), dtype=int)
        observed = deproject_pixels(np.column_stack((u[pick], v[pick])),
            frame.aligned_depth[v[pick], u[pick]] * frame.depth_scale, frame.intrinsics)
        if np.percentile(abs((observed - origin) @ normal), 90) > .005:
            return None, 'ARUCO_NOT_COPLANAR_WITH_TABLE', pixels.tolist()
        return xyz[1] - xyz[0], 'FIXED_TABLE_ARUCO_0_TO_1', pixels.tolist()

    def process(self, frame, result):
        validate_frame(frame.color_bgr, frame.aligned_depth, frame.intrinsics, frame.depth_scale)
        stamp, _ = frame_capture_time(frame)
        sid = f"color_{frame.timestamps['color_frame_number']}_depth_{frame.timestamps['depth_frame_number']}"
        if result.get('source_frame_id') != sid or result.get('timestamps') != frame.timestamps or result.get('capture_timestamp') != stamp:
            self._reset()
            raise ValueError('scene reference/source frame mismatch')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self._reset()
            raise ValueError('scene initialization needs distinct increasing frames')
        self.last_stamp = stamp
        depth = frame.aligned_depth * frame.depth_scale
        depth_ratio = float(((depth > .15) & (depth < 1.2)).mean())
        out = dict(schema='mtc.scene_reference.v1', source_frame_id=sid,
            capture_timestamp_s=stamp, timestamps=dict(frame.timestamps),
            coordinate_frame='t0_scene', input_coordinate_frame='camera_color_optical',
            unit_position='m', ready=False, status='NOT_READY',
            stable_frames=0, required_stable_frames=self.required,
            depth_valid_ratio=depth_ratio, T_scene_from_camera=None,
            origin_camera_m=None, axes_camera_xyz=None,
            direction_reference=None, aruco_id=self.marker_id,
            aruco_dictionary=self.dictionary if self.marker_id is not None else None,
            reference_validity='EXACT_SOURCE_FRAME_ONLY',
            absolute_accuracy='NOT_MEASURED', robot_transform_available=False,
            motion_usable=False, reasons=[],
            assumptions=['T0_FIXED_TO_FIELD', 'T0_BORDER_IS_200X100MM',
                'SCENE_X_ALONG_T0_LONG_EDGE', 'SCENE_Z_TABLE_UP'])
        def reject(reason):
            self._reset()
            out.update(status=reason, reasons=[reason])
            return out
        if self.pixel_expired:
            return reject('POSITIVE_END_PIXEL_EXPIRED_AFTER_CAMERA_MOVEMENT')
        if result.get('pair_qualified') is not True:
            return reject('RGBD_PAIR_REJECTED')
        if depth_ratio < .5:
            return reject('INSUFFICIENT_CURRENT_DEPTH')
        plane = result.get('support_plane')
        if plane is None or not np.isfinite(float(plane.get('inlier_ratio', 0))) or float(plane.get('inlier_ratio', 0)) < .6:
            return reject('NO_SUPPORTED_CURRENT_TABLE')
        targets = [t for t in result.get('placement_targets', []) if t.get('target_id') == 'T0']
        if len(targets) != 1 or targets[0].get('candidate_available') is not True:
            return reject('NO_UNIQUE_CURRENT_T0')
        target = targets[0]
        if target.get('coordinate_frame') != 'camera_color_optical' or target.get('unit_position') != 'm':
            return reject('T0_FRAME_OR_UNIT_MISMATCH')
        z = _vector(plane['normal_camera_xyz'], 3, 'table normal')
        if abs(np.linalg.norm(z) - 1) > 1e-3 or z[2] >= 0:
            return reject('INVALID_UPWARD_TABLE_NORMAL')
        origin = _vector(target['table_center_camera_m'], 3, 'T0 origin')
        plane_point = _vector(plane['point_camera_m'], 3, 'table plane point')
        if abs((origin - plane_point) @ z) > .002:
            return reject('T0_ORIGIN_NOT_ON_CURRENT_TABLE')
        corners = self._intersections(target['border_corners_px'], frame, z, origin)
        if len(corners) != 4:
            return reject('INVALID_T0_CORNERS')
        centred = corners - corners.mean(axis=0)
        _, _, axes = np.linalg.svd(centred, full_matrices=False)
        x = axes[0] - (axes[0] @ z) * z
        x /= np.linalg.norm(x)
        span = np.sort(np.ptp(centred @ axes[:2].T, axis=0))
        if np.any(abs(span - [.1, .2]) > [.015, .03]):
            return reject('T0_SIZE_INCONSISTENT')
        out.update(origin_camera_m=origin.tolist(), observed_span_m=span.tolist(),
            t0_border_corners_px=np.asarray(target['border_corners_px'], float).tolist())
        direction, reason, marker_pixels = self._direction(frame, z, origin)
        out['direction_reference'] = reason
        out['marker_corners_px'] = marker_pixels
        if direction is None:
            return reject(reason)
        direction -= (direction @ z) * z
        length = np.linalg.norm(direction)
        if length < .02 or abs(x @ direction) / length < np.cos(np.deg2rad(15)):
            return reject('DIRECTION_NOT_ALIGNED_WITH_T0_LONG_EDGE')
        if x @ direction < 0:
            x = -x
        y = np.cross(z, x)
        rotation = np.vstack((x, y, z))
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = -rotation @ origin
        translation_delta = rotation_delta = 0.
        moved = False
        if self.anchor is not None:
            old_origin, old_rotation = self.anchor
            translation_delta = float(np.linalg.norm(origin - old_origin))
            rotation_delta = float(np.rad2deg(np.arccos(np.clip(
                (np.trace(rotation @ old_rotation.T) - 1) / 2, -1., 1.))))
            moved = bool(translation_delta > self.translation_limit or rotation_delta > self.rotation_limit)
        if moved and self.pixel is not None:
            self.pixel_expired = True
            out.update(camera_moved=True, translation_from_stability_anchor_m=translation_delta,
                rotation_from_stability_anchor_deg=rotation_delta)
            return reject('POSITIVE_END_PIXEL_EXPIRED_AFTER_CAMERA_MOVEMENT')
        if self.anchor is None or moved:
            self.anchor = (origin.copy(), rotation.copy())
            self.count = 1
        else:
            self.count += 1
        ready = self.count >= self.required
        out.update(ready=ready, status='READY' if ready else 'CAMERA_MOVED_REINITIALIZING' if moved else 'WAITING_FOR_STABLE_REFERENCE',
            stable_frames=self.count, camera_moved=moved,
            translation_from_stability_anchor_m=translation_delta,
            rotation_from_stability_anchor_deg=rotation_delta,
            axes_camera_xyz=dict(x=x.tolist(), y=y.tolist(), z=z.tolist()),
            T_scene_from_camera=matrix.tolist() if ready else None)
        if self.pixel is not None:
            out['assumptions'].append('POSITIVE_END_PIXEL_VALID_ONLY_FOR_THIS_CURRENT_VIEW')
        else:
            out['assumptions'].append('MARKER_FIXED_COPLANAR_AND_0_TO_1_ALIGNED_WITH_SCENE_X')
        return out


def scene_coordinates(coordinates, reference):
    """Convert only source-matched current candidates; incomplete output stays null."""
    stamp = coordinates.get('capture_timestamp_s', coordinates.get('capture_timestamp'))
    if coordinates.get('source_frame_id') != reference['source_frame_id'] or coordinates.get('timestamps') != reference['timestamps'] or stamp != reference['capture_timestamp_s']:
        raise ValueError('coordinates/scene reference source mismatch')
    if coordinates.get('coordinate_frame') != 'camera_color_optical' or coordinates.get('unit_position') != 'm':
        raise ValueError('coordinates must be camera optical metres')
    matrix = None
    if reference['ready']:
        matrix = np.asarray(reference['T_scene_from_camera'], float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]) or not np.allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3), atol=1e-6) or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-6:
            raise ValueError('invalid current scene transform')
    def convert(point):
        if matrix is None or point is None:
            return None
        return (matrix[:3, :3] @ _vector(point, 3, 'camera point') + matrix[:3, 3]).tolist()
    objects = [dict(color=obj['color'], candidate_available=bool(matrix is not None and obj.get('candidate_available') is True),
        green_point_scene_m=convert(obj.get('green_point_camera_m')) if obj.get('candidate_available') is True else None,
        red_point_scene_m=convert(obj.get('red_point_camera_m')) if obj.get('candidate_available') is True else None,
        coordinate_variant=obj.get('coordinate_variant', 'RAW_UNCORRECTED'))
        for obj in coordinates.get('objects', [])]
    targets = [dict(target_id=t['target_id'], candidate_available=bool(matrix is not None and t.get('candidate_available') is True),
        target_hole_center_scene_m=convert(t.get('target_hole_center_camera_m')) if t.get('candidate_available') is True else None)
        for t in coordinates.get('placement_targets', [])]
    return dict(coordinate_frame='t0_scene', unit_position='m', source_frame_id=reference['source_frame_id'],
        ready=reference['ready'], status=reference['status'], objects=objects,
        placement_targets=targets, absolute_accuracy='NOT_MEASURED', motion_usable=False,
        robot_transform_available=False)
