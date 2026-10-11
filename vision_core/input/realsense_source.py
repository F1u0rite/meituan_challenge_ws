"""Intel RealSense D435i color-only BGR8 input adapter."""
from dataclasses import dataclass
from datetime import datetime, timezone
import numpy as np


@dataclass
class ColorFrame:
    image: np.ndarray
    timestamp: str
    frame_number: int


class RealSenseSource:
    """Owns the SDK pipeline; perception sees only BGR uint8 arrays."""

    def __init__(self, width=1280, height=720, fps=30, serial=None):
        self.width, self.height, self.fps, self.serial = width, height, fps, serial
        self._rs = None
        self._pipeline = None
        self._started = False
        self._profile = None

    def start(self):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError('pyrealsense2 is not installed. Install it in the live Python environment.') from exc
        self._rs = rs
        try:
            devices = list(rs.context().query_devices())
            if not devices:
                raise RuntimeError('No Intel RealSense camera detected. Connect the D435i and check its USB connection.')
            if self.serial:
                serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
                if self.serial not in serials:
                    raise RuntimeError(f'RealSense serial {self.serial} not found; available serials: {serials}')
            config = rs.config()
            if self.serial:
                config.enable_device(self.serial)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            self._pipeline = rs.pipeline()
            self._profile = self._pipeline.start(config)
            self._started = True
        except RuntimeError as exc:
            self._pipeline = None
            raise RuntimeError(f'Cannot start RealSense color stream ({self.width}x{self.height} BGR8 @ {self.fps} FPS): {exc}') from exc
        return self

    def read(self, timeout_ms=5000):
        if not self._started or self._pipeline is None:
            raise RuntimeError('RealSenseSource.start() must succeed before read()')
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms)
            color = frames.get_color_frame()
        except RuntimeError as exc:
            raise RuntimeError(f'RealSense color frame unavailable: {exc}') from exc
        if not color:
            raise RuntimeError('RealSense returned no color frame')
        image = np.asanyarray(color.get_data()).copy()
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise RuntimeError(f'Unexpected RealSense color frame format: {image.shape}, {image.dtype}')
        return ColorFrame(image, datetime.now(timezone.utc).isoformat(), int(color.get_frame_number()))

    def camera_metadata(self):
        """Read current SDK-reported color controls; never substitute requested values."""
        if not self._started or self._profile is None:
            raise RuntimeError('RealSenseSource.start() must succeed before metadata')
        rs = self._rs
        device = self._profile.get_device()
        def info(key):
            try:
                return device.get_info(getattr(rs.camera_info, key))
            except (AttributeError, RuntimeError):
                return 'UNAVAILABLE'
        sensor = None
        for candidate in device.query_sensors():
            try:
                if 'RGB' in candidate.get_info(rs.camera_info.name).upper():
                    sensor = candidate
                    break
            except RuntimeError:
                continue
        def option(name):
            if sensor is None:
                return 'UNAVAILABLE'
            try:
                key = getattr(rs.option, name)
                return float(sensor.get_option(key)) if sensor.supports(key) else 'UNAVAILABLE'
            except (AttributeError, RuntimeError):
                return 'UNAVAILABLE'
        try:
            fps = int(self._profile.get_stream(rs.stream.color).fps())
        except RuntimeError:
            fps = 'UNAVAILABLE'
        return {'device_model': info('name'), 'device_serial': info('serial_number'),
                'fps': fps, 'exposure': option('exposure'), 'gain': option('gain'),
                'auto_exposure_enabled': option('enable_auto_exposure'),
                'white_balance': option('white_balance'),
                'auto_white_balance_enabled': option('enable_auto_white_balance')}

    def stop(self):
        if self._started and self._pipeline is not None:
            self._pipeline.stop()
        self._pipeline = None
        self._profile = None
        self._started = False

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()
