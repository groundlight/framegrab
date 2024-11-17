import logging
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


class MotionDetector:
    """Motion detector using three-frame differencing algorithm.

    This implements the motion detection algorithm described in:
        Collins et al., "A System for Video Surveillance and Monitoring",
        Carnegie Mellon University, Pittsburgh, PA, Technical Report CMU-RI-TR-00-12, May 2000.

    The algorithm compares each new frame against the previous two frames to detect motion.
    Motion is detected when a sufficient percentage of pixels show significant brightness changes
    across consecutive frames.

    Args:
        pct_threshold (float, optional): Percentage of pixels that must change for motion to be detected.
            Defaults to 1.0 (1% of pixels).
        val_threshold (int, optional): Minimum brightness change required for a pixel to be considered changed.
            Defaults to 50.
    """

    def __init__(self, pct_threshold: float = 1, val_threshold: int = 50) -> None:
        self.unused = True
        self.pixel_val_threshold = val_threshold
        self.pixel_pct_threshold = pct_threshold
        self.log_pixel_percent = True

    def pixel_threshold(self, img: np.ndarray, threshold_val: Optional[float] = None) -> bool:
        """Check if enough pixels exceed the brightness threshold.

        Args:
            img: Input image array
            threshold_val: Optional override for the brightness threshold value.
                If None, uses self.pixel_val_threshold.

        Returns:
            bool: True if percentage of changed pixels exceeds pct_threshold
        """
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
                logger.debug(f"No motion detected: {pct_hi:.3f}% < {self.pixel_pct_threshold}%")
            return False

    def motion_detected(self, new_img: np.ndarray) -> bool:
        """Process a new frame and detect if motion occurred.

        Uses three-frame differencing - compares the new frame against the previous
        two frames to detect consistent motion.

        Args:
            new_img: New frame to analyze for motion

        Returns:
            bool: True if motion was detected
        """
        if self.unused:
            self.base_img = new_img
            self.base2 = self.base_img
            self.unused = False
            return True

        new_img16 = new_img.astype(np.int16)

        if new_img16.shape != self.base_img.shape:
            return True

        diff1 = np.abs(new_img16 - self.base_img) > self.pixel_val_threshold
        diff2 = np.abs(new_img16 - self.base2) > self.pixel_val_threshold

        self.base2 = self.base_img
        self.base_img = new_img
        binarized = (
            diff1 & diff2
        ) * 255  # The 255 here guarantees that every pixel which is detected to have changed is > pixel_val_threshold.

        motion_detected = self.pixel_threshold(binarized)
        motion_detected = not not motion_detected  # normalize a numpy.bool_ if needed
        assert isinstance(motion_detected, bool)
        return motion_detected
