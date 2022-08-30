
# Simple Collins motion detection to decimate stream.  Defaults to 1% pixel difference threshold

import logging
import numpy as np


logger = logging.getLogger('groundlight.stream')

class MotionDetector():

    def __init__(self):
        self.unused = True
        self.threshold = 50
        self.pixel_val_threshold = 50
        self.pixel_pct_threshold = 1
        self.log_pixel_percent = True

    def pixel_threshold(self, img:np.ndarray, threshold_val:float=None):
        """Returns true if more then pixel_pct_threshold% of pixels have value greater than pixel_val_threshold
        """
        if threshold_val is None:
            threshold_val = self.pixel_val_threshold
        total_pixels = np.prod(img.shape)
        hi_pixels = np.sum(img > threshold_val)
        pct_hi = float(hi_pixels) / float(total_pixels) * 100
        if pct_hi > self.pixel_pct_threshold:
            logger.debug(f"Motion detected with {pct_hi:.2f}% pixels changed")
            return True
        else:
            if self.log_pixel_percent:
                logger.debug(f"No motion detected: {pct_hi:.2f}% < {self.pixel_pct_threshold}%")
            return False

    def motion_detected(self, new_img:np.ndarray):

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
        binarized = (diff1 & diff2) * 255

        motion_detected = self.pixel_threshold(binarized)
        motion_detected = not not motion_detected  # normalize a numpy.bool_ if needed
        assert isinstance(motion_detected, bool)
        return motion_detected

