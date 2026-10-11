"""Rectify a detected board into its configured canonical pixel grid."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Rectification:
    image: np.ndarray
    camera_to_board: np.ndarray


class BoardRectifier:
    def __init__(self,config): self.config=config

    @property
    def output_size(self):
        ppm=self.config['render']['pixels_per_mm']; board=self.config['board']
        return round(board['width_mm']*ppm),round(board['height_mm']*ppm)

    def rectify(self,image,corners_px):
        width,height=self.output_size
        dst=np.float32([[0,0],[width-1,0],[width-1,height-1],[0,height-1]])
        H=cv2.getPerspectiveTransform(np.asarray(corners_px,np.float32),dst)
        return Rectification(cv2.warpPerspective(image,H,(width,height)),H)
