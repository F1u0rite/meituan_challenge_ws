"""Optional RealSense RGB-D source. Constructing this class never starts a camera."""
from dataclasses import dataclass
from datetime import datetime, timezone
import numpy as np
from perception.rgbd_geometry import CameraIntrinsics, validate_frame


@dataclass
class RGBDFrame:
    color_bgr: np.ndarray
    aligned_depth: np.ndarray
    intrinsics: CameraIntrinsics
    depth_scale: float
    timestamps: dict
    stream_profiles: dict
    raw_depth: object = None
    raw_depth_intrinsics: object = None
    alignment_metadata: object = None
    camera_configuration: object = None


class RealSenseRGBDSource:
    def __init__(self, *, preferred_color=(1280, 720), preferred_depth=(640, 480), fps=15):
        self.preferred_color = preferred_color
        self.preferred_depth = preferred_depth
        self.fps = fps
        self.pipeline = None
        self.align = None
        self.profile = None
        self.depth_scale = None
        self.stream_profiles = None

    @staticmethod
    def _options(device, rs, stream_type, fmt):
        found = set()
        for sensor in device.query_sensors():
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() == stream_type and profile.format() == fmt:
                    try:
                        video = profile.as_video_stream_profile()
                        found.add((video.width(), video.height(), profile.fps()))
                    except RuntimeError:
                        pass
        return sorted(found)

    @staticmethod
    def _choose(options, preferred, fps):
        if not options:
            raise RuntimeError('Required RealSense stream format is unavailable')
        return min(options, key=lambda o: (abs(o[2]-fps),
                                             abs(o[0]-preferred[0])+abs(o[1]-preferred[1]),
                                             -o[0]*o[1]))

    def start(self):
        import pyrealsense2 as rs
        self.rs = rs
        context = rs.context()
        devices = context.query_devices()
        if len(devices) == 0:
            raise RuntimeError('No RealSense device connected')
        device = devices[0]
        color_options = self._options(device, rs, rs.stream.color, rs.format.bgr8)
        depth_options = self._options(device, rs, rs.stream.depth, rs.format.z16)
        color = self._choose(color_options, self.preferred_color, self.fps)
        # Match frame rates where possible, while allowing distinct resolutions.
        matched_depth = [o for o in depth_options if o[2] == color[2]]
        depth = self._choose(matched_depth or depth_options, self.preferred_depth, color[2])
        if depth[2] != color[2]:
            raise RuntimeError(f'No matching Color/Depth FPS. Color {color}; Depth options {depth_options}')
        config = rs.config()
        config.enable_stream(rs.stream.color, *color[:2], rs.format.bgr8, color[2])
        config.enable_stream(rs.stream.depth, *depth[:2], rs.format.z16, depth[2])
        self.pipeline = rs.pipeline()
        try:
            self.profile = self.pipeline.start(config)
        except Exception:
            self.pipeline = None
            raise RuntimeError(f'RealSense rejected queried stream pair Color={color} Depth={depth}')
        self.align = rs.align(rs.stream.color)
        self.depth_scale = float(self.profile.get_device().first_depth_sensor().get_depth_scale())
        self.stream_profiles = {'color': {'width': color[0], 'height': color[1], 'fps': color[2], 'format': 'BGR8'},
                                'depth': {'width': depth[0], 'height': depth[1], 'fps': depth[2], 'format': 'Z16'},
                                'color_supported': color_options, 'depth_supported': depth_options}
        return self.stream_profiles

    def read(self, *, latest=False):
        if self.pipeline is None:
            raise RuntimeError('RGB-D source is not started')
        raw_frames = self.pipeline.wait_for_frames()
        dropped=0
        if latest:
            # Bounded drain prevents accumulating old frames when CAD is slower
            # than capture. Only actual complete SDK frame sets are processed.
            for _ in range(12):
                newer=self.pipeline.poll_for_frames()
                if not newer:break
                raw_frames=newer;dropped+=1
        raw_depth_frame = raw_frames.get_depth_frame()
        raw_color_frame = raw_frames.get_color_frame()
        raw_depth = None
        raw_intrinsics = None
        alignment_metadata = {'method': 'SDK_DEPTH_TO_COLOR', 'depth_to_color_extrinsics': 'UNAVAILABLE'}
        if raw_depth_frame and raw_color_frame:
            raw_depth = np.asanyarray(raw_depth_frame.get_data()).copy()
            ri = raw_depth_frame.profile.as_video_stream_profile().get_intrinsics()
            raw_intrinsics = CameraIntrinsics(ri.width, ri.height, ri.fx, ri.fy, ri.ppx, ri.ppy,
                str(ri.model).split('.')[-1], tuple(ri.coeffs))
            try:
                extr = raw_depth_frame.profile.get_extrinsics_to(raw_color_frame.profile)
                alignment_metadata['depth_to_color_extrinsics'] = {
                    'rotation_sdk_column_major': list(extr.rotation), 'translation_m': list(extr.translation)}
            except (RuntimeError, AttributeError):
                pass
        frames = self.align.process(raw_frames)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError('Missing aligned Color or Depth frame')
        color_bgr = np.asanyarray(color.get_data()).copy()
        aligned_depth = np.asanyarray(depth.get_data()).copy()
        intr = depth.profile.as_video_stream_profile().get_intrinsics()
        self._last_intrinsics = intr
        intrinsics = CameraIntrinsics(intr.width, intr.height, intr.fx, intr.fy, intr.ppx,
                                      intr.ppy, str(intr.model).split('.')[-1], tuple(intr.coeffs))
        validate_frame(color_bgr, aligned_depth, intrinsics, self.depth_scale)
        timestamps = {'host_utc': datetime.now(timezone.utc).isoformat(),
                      'queued_frames_discarded': dropped,
                      'color_sdk_ms': float(color.get_timestamp()),
                      'depth_sdk_ms': float(depth.get_timestamp()),
                      'color_frame_number': int(color.get_frame_number()),
                      'depth_frame_number': int(depth.get_frame_number()),
                      'color_timestamp_domain': str(color.get_frame_timestamp_domain()),
                      'depth_timestamp_domain': str(depth.get_frame_timestamp_domain())}
        return RGBDFrame(color_bgr, aligned_depth, intrinsics, self.depth_scale,
                         timestamps, self.stream_profiles, raw_depth, raw_intrinsics,
                         alignment_metadata, self.camera_configuration())

    def camera_configuration(self):
        result = {'actual_stream_profiles': self.stream_profiles, 'depth_scale_m_per_unit': self.depth_scale,
                  'additional_postprocessing_filters_applied_by_program': [],
                  'sensor_firmware_postprocessing': 'UNAVAILABLE'}
        device = self.profile.get_device()
        try:
            result['device_model'] = device.get_info(self.rs.camera_info.name)
            result['device_serial_local_only'] = device.get_info(self.rs.camera_info.serial_number)
        except (RuntimeError, AttributeError):
            result['device_model'] = 'UNAVAILABLE'
        for name in ['exposure', 'gain', 'enable_auto_exposure', 'white_balance', 'enable_auto_white_balance', 'visual_preset']:
            options = []
            try:
                option = getattr(self.rs.option, name)
                for sensor in device.query_sensors():
                    if sensor.supports(option):
                        options.append({'sensor_name': sensor.get_info(self.rs.camera_info.name),
                                        'value': float(sensor.get_option(option))})
            except (RuntimeError, AttributeError):
                pass
            result[name] = options or 'UNAVAILABLE'
        try:
            advanced = self.rs.rs400_advanced_mode(device)
            result['disparity_shift'] = int(advanced.get_depth_table().disparityShift)
        except (RuntimeError, AttributeError):
            result['disparity_shift'] = 'UNAVAILABLE'
        return result

    def deproject(self, uv, depth_m):
        if self.rs is None:
            raise RuntimeError('RGB-D source is not started')
        intr = self._last_intrinsics
        return self.rs.rs2_deproject_pixel_to_point(intr, list(uv), depth_m)

    def project(self, point_m):
        if self.pipeline is None:
            raise RuntimeError("RGB-D source is not started")
        intr=self._last_intrinsics
        if not any(abs(c)>1e-10 for c in intr.coeffs):
            x,y,z=point_m
            if z<=0:raise ValueError('Point is behind camera')
            return [x/z*intr.fx+intr.ppx,y/z*intr.fy+intr.ppy]
        return self.rs.rs2_project_point_to_pixel(intr, list(point_m))

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
