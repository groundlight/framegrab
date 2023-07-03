import logging

import numpy as np
from typing import Union
from PIL import Image
import cv2
import requests

logger = logging.getLogger(__name__)


class MotionDetector:
    # Simple motion detector using the three frame differencing
    # commonly attributed to Collins, et. al A system for video surveillance and monitoring. Technical report, 2000
    # Defaults to 1% pixel difference threshold (good for many applications)

    def __init__(self, pct_threshold: float = 1, val_threshold: int = 50) -> bool:
        """
        :param val_threshold: The minimum brightness change for a pixel for it to be considered changed
        :param pct_threshold: Percent of pixels needed to change before motion is detected
        """
        self.unused = True
        self.threshold = 50
        self.pixel_val_threshold = 50
        self.pixel_pct_threshold = pct_threshold
        self.log_pixel_percent = True

    @staticmethod
    def get_numpy_array(filename: str) -> np.ndarray:
        if filename.startswith("http"):
            image = Image.open(requests.get(filename, stream=True).raw)
            return np.array(image)
        elif filename.endswith("jpeg"):
            # Return a numpy array in BGR format since the SDK
            # assumes this ordering for now.
            return cv2.imread(filename=filename)

    def pixel_threshold(self, img: np.ndarray, threshold_val: float = None) -> bool:
        """Returns true if more then pixel_pct_threshold% of pixels have value greater than pixel_val_threshold"""
        if threshold_val is None:
            threshold_val = self.pixel_val_threshold
        total_pixels = np.prod(img.shape)
        hi_pixels = np.sum(img > threshold_val)
        pct_hi = float(hi_pixels) / float(total_pixels) * 100
        if pct_hi > self.pixel_pct_threshold:
            logger.debug(f"Motion detected with {pct_hi:.3f}% pixels changed")
            return True
        else:
            if self.log_pixel_percent:
                logger.debug(
                    f"No motion detected: {pct_hi:.3f}% < {self.pixel_pct_threshold}%"
                )
            return False

    def motion_detected(self, new_img: Union[str, np.ndarray]) -> bool:
        """
        Returns true if motion is detected by comparing the new image and a previous baseline.
        :param new_img: New image as either a URL, local filepath or a Numpy array
        """
        if isinstance(new_img, str):
            new_img = self.get_numpy_array(filename=new_img)
        if self.unused:
            self.base_img = new_img
            self.base2 = self.base_img
            self.unused = False
            return True

        new_img16 = new_img.astype(np.int16)
        diff1 = np.abs(new_img16 - self.base_img) > self.threshold
        diff2 = np.abs(new_img16 - self.base2) > self.threshold

        self.base2 = self.base_img
        self.base_img = new_img
        binarized = (
            diff1 & diff2
        ) * 255  # The 255 here guarantees that every pixel which is detected to have changed is > pixel_val_threshold.

        motion_detected = self.pixel_threshold(binarized)
        motion_detected = not not motion_detected  # normalize a numpy.bool_ if needed
        assert isinstance(motion_detected, bool)
        return motion_detected
