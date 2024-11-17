import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from framegrab.grabber import HttpLiveStreamingFrameGrabber


class TestHttpLiveStreamingFrameGrabber(unittest.TestCase):
    def setUp(self):
        """Common setup for all HLS tests"""
        self.mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.base_config = {
            "input_type": "hls",
            "id": {"hls_url": "http://example.com/stream.m3u8"},
            "name": "test_stream",
        }

    @patch("cv2.VideoCapture")
    def test_grab_frame_success(self, mock_cv2):
        """Test that the grabber initializes correctly with a valid config"""
        # Setup mock
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.read.return_value = (True, self.mock_frame)
        mock_cv2.return_value = mock_capture

        grabber = HttpLiveStreamingFrameGrabber(self.base_config)
        self.assertEqual(grabber.hls_url, "http://example.com/stream.m3u8")

        frame = grabber.grab()
        mock_cv2.assert_called_once_with("http://example.com/stream.m3u8")
        mock_capture.read.assert_called_once()
        self.assertEqual(frame.shape, (480, 640, 3))
        mock_capture.release.assert_called_once()

    def test_init_without_hls_url(self):
        """Test that initialization fails without an HLS URL"""
        config = {"input_type": "hls", "name": "test_stream"}

        with self.assertRaises(ValueError):
            HttpLiveStreamingFrameGrabber(config)

    @patch("cv2.VideoCapture")
    def test_grab_with_failed_connection(self, mock_cv2):
        """Test that initialization fails when the stream cannot be opened"""
        # Setup mock
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = False
        mock_cv2.return_value = mock_capture

        with self.assertRaises(ValueError) as cm:
            grabber = HttpLiveStreamingFrameGrabber(self.base_config)
            grabber.grab()

        self.assertIn("Could not open HLS stream", str(cm.exception))

    @patch("cv2.VideoCapture")
    def test_grab_frame_failure(self, mock_cv2):
        """Test frame grab failure"""
        # Setup mock
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.read.return_value = (False, None)
        mock_capture.retrieve.return_value = (False, None)
        mock_cv2.return_value = mock_capture

        grabber = HttpLiveStreamingFrameGrabber(self.base_config)

        with self.assertRaises(Exception):
            grabber.grab()

        mock_capture.release.assert_called_once()

    @patch("cv2.VideoCapture")
    def test_invalid_resolution_option(self, mock_cv2):
        """Test that setting resolution raises an error"""
        # Setup mock
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_cv2.return_value = mock_capture

        config = self.base_config.copy()
        config["options"] = {"resolution": {"width": 1920, "height": 1080}}

        grabber = HttpLiveStreamingFrameGrabber(config)

        with self.assertRaises(ValueError):
            grabber.apply_options(config["options"])
