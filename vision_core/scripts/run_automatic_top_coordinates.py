"""Opt-in automatic top/CAD coordinates to ROS2; camera opens only with --live."""
import argparse
import json
import sys
import os
from contextlib import ExitStack
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PYTHON_VERSION = f'{sys.version_info.major}.{sys.version_info.minor}'
INSTALL_BASE = Path(os.environ.get('MTC_VISION_INSTALL_BASE', ROOT / 'install/yuze'
    if (ROOT / 'install/yuze').is_dir() else ROOT / 'install')).expanduser().resolve()
sys.path[:0] = [str(ROOT / 'vision_core'), str(ROOT / 'src/mtc_vision_adapter'),
    str(INSTALL_BASE / f'mtc_interfaces/lib/python{PYTHON_VERSION}/site-packages'),
    str(INSTALL_BASE / f'lib/python{PYTHON_VERSION}/site-packages'),
    f'/opt/ros/jazzy/lib/python{PYTHON_VERSION}/site-packages']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--input', type=Path, help='Recorded frame_* RGB-D folder; never opens camera')
    source.add_argument('--live', action='store_true', help='Explicitly open D435i and release it on exit')
    parser.add_argument('--geometry', required=True, type=Path)
    parser.add_argument('--checkpoint', required=True, type=Path)
    parser.add_argument('--frames', type=int, default=6, help='Bounded run; 0 means all/until Ctrl+C')
    parser.add_argument('--wait-subscriber-s', type=float, default=0.)
    parser.add_argument('--send-images', action='store_true', help='Send JPEG RGB stream plus result-source overlays')
    parser.add_argument('--image-fps', type=float, default=10.)
    parser.add_argument('--jpeg-quality', type=int, default=80)
    parser.add_argument('--disable-height-constraint', action='store_true', help='Export uncorrected raw heights')
    parser.add_argument('--top-height-mm', type=float, default=80.)
    parser.add_argument('--hole-height-mm', type=float, default=60.)
    parser.add_argument('--max-height-correction-mm', type=float, default=10.)
    parser.add_argument('--scene-aruco-id', type=int, help='Opt-in current T0 scene coordinates from a fixed table marker')
    parser.add_argument('--scene-aruco-dictionary', default='DICT_4X4_50')
    parser.add_argument('--scene-stable-frames', type=int, default=5)
    args = parser.parse_args()
    if args.frames < 0:
        parser.error('frames must be nonnegative')
    import math
    if not math.isfinite(args.image_fps) or not 0 < args.image_fps <= 30 or not 1 <= args.jpeg_quality <= 100:
        parser.error('image fps must be 0..30; JPEG quality must be 1..100')
    # Validate data before creating publishers or opening hardware.
    geometry = json.loads(args.geometry.read_text())
    if not args.checkpoint.is_file():
        parser.error('checkpoint does not exist')
    folders = sorted(args.input.glob('frame_*')) if args.input else None
    if folders is not None and not folders:
        parser.error('no recorded frame_* samples')
    import cv2
    import numpy as np
    import torch
    import yaml
    from input.realsense_rgbd_source import RGBDFrame, RealSenseRGBDSource
    from perception.rgbd_geometry import CameraIntrinsics
    from perception.automatic_segmented_top import AutomaticSegmentedTop
    from perception.placement_target import T0PlacementTarget
    from perception.height_constraint import HeightConstrainedPoints
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    from mtc_vision_adapter.automatic_top_result_publisher import AutomaticTopResultPublisher

    heights = None if args.disable_height_constraint else HeightConstrainedPoints(
        top_height_m=args.top_height_mm/1000., hole_height_m=args.hole_height_mm/1000.,
        max_correction_m=args.max_height_correction_mm/1000.)
    scene = None
    if args.scene_aruco_id is not None:
        from perception.scene_frame import SceneFrameInitializer
        scene = SceneFrameInitializer(aruco_id=args.scene_aruco_id,
            aruco_dictionary=args.scene_aruco_dictionary,
            min_stable_frames=args.scene_stable_frames)
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    predictor = SAM2ImagePredictor(build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml',
        str(args.checkpoint), device='cpu', apply_postprocessing=False))
    pipeline = AutomaticSegmentedTop(
        yaml.safe_load((ROOT / 'vision_core/config/simplified_rgbd.yaml').read_text()),
        geometry, predictor=predictor)
    placement = T0PlacementTarget(geometry,
        physical_hole_height_m=None if heights is None else heights.hole_height)
    with ExitStack() as cleanup:
        sink = cleanup.enter_context(AutomaticTopResultPublisher(args.wait_subscriber_s))
        images = None
        if args.send_images:
            from mtc_vision_adapter.rgb_image_publisher import RGBImagePublisher
            images = cleanup.enter_context(RGBImagePublisher(fps=args.image_fps,
                quality=args.jpeg_quality, wait_for_subscriber_s=args.wait_subscriber_s))
        camera = None
        capture = None
        if args.live:
            camera = RealSenseRGBDSource()
            cleanup.callback(camera.stop)
            camera.start()
            for _ in range(20):
                camera.read(latest=True)
            if images is not None:
                from perception.dual_rate_rgbd import CaptureWorker
                def preview(identity, frame, pair_report):
                    from mtc_vision_adapter.rgb_image_publisher import source_frame_id
                    images.publish_frame(frame, source_frame_id(frame), mode='live',
                        pair_qualified=pair_report['qualified'])
                capture = CaptureWorker(camera, on_preview=preview)
                cleanup.callback(capture.close)
                capture.start()
        count = 0
        while not args.frames or count < args.frames:
            if camera is not None:
                if capture is not None:
                    latest = capture.slot.get(timeout=1.)
                    if capture.error:
                        raise RuntimeError(capture.error)
                    if latest is None:
                        continue
                    _, frame = latest
                else:
                    frame = camera.read(latest=True)
            else:
                if count >= len(folders):
                    break
                folder = folders[count]
                metadata = json.loads((folder / 'metadata.json').read_text())
                with np.load(folder / 'depth.npz') as archive:
                    depth = archive['depth'].copy()
                frame = RGBDFrame(cv2.imread(str(folder / 'color.png')), depth,
                    CameraIntrinsics(**metadata['intrinsics']), metadata['depth_scale'],
                    metadata['timestamps'], {}, alignment_metadata=metadata['alignment'])
            sid = f"color_{frame.timestamps['color_frame_number']}_depth_{frame.timestamps['depth_frame_number']}"
            if images is not None and camera is None:
                images.publish_frame(frame, sid, mode='offline')
            result = pipeline.process(frame, sid)
            result['placement_targets'] = [placement.estimate(frame, result)]
            if heights is not None:
                result = heights.apply(frame, result)
            if scene is not None:
                result['scene_reference'] = scene.process(frame, result)
            packet = sink.publish_result(result, mode='live' if camera is not None else 'offline')
            if images is not None:
                images.publish_overlay(frame, result, mode='live' if camera is not None else 'offline')
            count += 1
            print(json.dumps(dict(sequence=packet['sequence'], source_frame_id=sid,
                source_mode=packet['source_mode'], missing_colors=packet['missing_colors'],
                coordinates=[dict(color=o['color'], green=o['green_point_camera_m'],
                    red=o['red_point_camera_m']) for o in packet['objects']])), flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
