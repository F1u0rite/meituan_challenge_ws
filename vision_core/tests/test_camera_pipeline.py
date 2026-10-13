import cv2
import numpy as np
from synthetic.scene_generator import SceneGenerator
from synthetic.camera_projection import CameraProjection
from perception.board_detector import BoardDetector,order_corners
from perception.board_rectifier import BoardRectifier
from perception.battery_detector import BatteryDetector
from evaluation.evaluator import Evaluator

def test_projection_reproducible_and_in_frame():
    gen=SceneGenerator(); scene=gen.generate(42); projector=CameraProjection(gen.config)
    a,b=projector.project(scene.canonical_image,123),projector.project(scene.canonical_image,123)
    assert np.array_equal(a.image,b.image); assert np.allclose(a.board_corners_gt_px,b.board_corners_gt_px)
    h,w=a.image.shape[:2]; c=a.board_corners_gt_px
    assert ((c[:,0]>=0)&(c[:,0]<w)&(c[:,1]>=0)&(c[:,1]<h)).all()

def test_corner_order():
    shuffled=np.float32([[90,120],[10,10],[100,20],[15,110]])
    assert np.allclose(order_corners(shuffled),np.float32([[10,10],[100,20],[90,120],[15,110]]))

def test_gt_rectification_math_and_size():
    gen=SceneGenerator(); scene=gen.generate(4); camera=CameraProjection(gen.config).project(scene.canonical_image,4)
    result=BoardRectifier(gen.config).rectify(camera.image,camera.board_corners_gt_px)
    assert result.image.shape[:2]==scene.canonical_image.shape[:2]
    source=np.float32([[[0,0],[scene.image.shape[1]-1,0],[scene.image.shape[1]-1,scene.image.shape[0]-1],[0,scene.image.shape[0]-1]]])
    projected=cv2.perspectiveTransform(cv2.perspectiveTransform(source,camera.canonical_to_camera),result.camera_to_board)
    assert np.max(np.abs(projected-source))<1e-3

def test_mild_perspective_full_pipeline():
    gen=SceneGenerator(); scene=gen.generate(42,mode='official'); camera=CameraProjection(gen.config).project(scene.canonical_image,42)
    board=BoardDetector(gen.config).detect(camera.image); assert board.detected
    rectified=BoardRectifier(gen.config).rectify(camera.image,board.corners_px).image
    detections=BatteryDetector(gen.config).detect(rectified); assert len(detections)==4
    assert not Evaluator(gen.config).evaluate(scene.ground_truth,detections).failed_cases
