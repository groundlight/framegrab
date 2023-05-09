import unittest
import numpy as np
from framegrab.motion import MotionDetector

class TestMotionDetector(unittest.TestCase):

    def setUp(self):
        self.motion_detector = MotionDetector()

    def create_image(self, width, height, value):
        return np.full((height, width), value, dtype=np.uint8)

    def test_pixel_threshold(self):
        img = self.create_image(100, 100, 60)
        self.assertTrue(self.motion_detector.pixel_threshold(img))

    def test_pixel_threshold_no_motion(self):
        img = self.create_image(100, 100, 40)
        self.assertFalse(self.motion_detector.pixel_threshold(img))

    def test_motion_detected_first_call(self):
        img = self.create_image(100, 100, 60)
        self.assertTrue(self.motion_detector.motion_detected(img))

    def test_motion_detected_no_motion(self):
        img1 = self.create_image(100, 100, 60)
        img2 = self.create_image(100, 100, 60)
        self.motion_detector.motion_detected(img1)  # Initialize base images
        self.assertFalse(self.motion_detector.motion_detected(img2))

    def test_motion_detected_motion(self):
        img1 = self.create_image(100, 100, 60)
        img2 = self.create_image(100, 100, 120)
        self.motion_detector.motion_detected(img1)  # Initialize base images
        self.assertTrue(self.motion_detector.motion_detected(img2))

    def test_motion_detected_motion_below_threshold(self):
        img1 = self.create_image(100, 100, 60)
        img2 = self.create_image(100, 100, 90)
        self.motion_detector.motion_detected(img1)  # Initialize base images
        self.assertFalse(self.motion_detector.motion_detected(img2))

    def test_ignore_small_pixel_value_changes(self):
        img1 = self.create_image(100, 100, 60)
        img2 = self.create_image(100, 100, 64)  # Small change, below the default val_threshold
        self.motion_detector.motion_detected(img1)  # Initialize base images
        self.assertFalse(self.motion_detector.motion_detected(img2))

    def test_ignore_small_number_of_large_changes(self):
        img1 = self.create_image(100, 100, 60)
        img2 = np.copy(img1)
        img2[0:10, 0:10] = 130  # Change a small number of pixels by a large amount
        self.motion_detector.motion_detected(img1)  # Initialize base image
        self.assertFalse(self.motion_detector.motion_detected(img2))

    def test_detect_motion_with_configured_threshold(self):
        self.motion_detector = MotionDetector(pct_threshold=5)  # Configure detector with 5% pixel change threshold
        img1 = self.create_image(100, 100, 60)
        
        img2 = np.copy(img1)
        img2[0:45, 0:10] = 130  # Change slightly less than 5% of pixels by a large amount
        self.motion_detector.motion_detected(img1)  # Initialize base image
        self.assertFalse(self.motion_detector.motion_detected(img2))

        img3 = np.copy(img1)
        img3[0:51, 0:10] = 130  # Change slightly more than 5% of pixels by a large amount
        self.motion_detector.motion_detected(img1)  # Initialize base image
        self.motion_detector.motion_detected(img1)  # again to really reset
        self.assertTrue(self.motion_detector.motion_detected(img3))
