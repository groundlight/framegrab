"""Unit tests for RTSP backends (GStreamer and FFmpeg).

These tests use mocks to avoid requiring actual RTSP streams,
making them suitable for CI/CD automation.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import numpy as np
import cv2

from framegrab.grabber import RTSPFrameGrabber
from framegrab.config import RTSPFrameGrabberConfig
from framegrab.exceptions import GrabError


class TestRTSPBackends(unittest.TestCase):
    """Test RTSP frame grabber with both GStreamer and FFmpeg backends."""

    def setUp(self):
        """Set up test fixtures."""
        self.test_rtsp_url = "rtsp://test.example.com/stream"
        self.test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    def tearDown(self):
        """Clean up after tests."""
        pass

    # =========================================================================
    # Configuration Tests
    # =========================================================================

    def test_config_default_backend_is_ffmpeg(self):
        """Test that default backend is FFmpeg."""
        config = RTSPFrameGrabberConfig(rtsp_url=self.test_rtsp_url)
        self.assertEqual(config.backend, "ffmpeg")

    def test_config_accepts_gstreamer_backend(self):
        """Test that GStreamer backend can be configured."""
        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer"
        )
        self.assertEqual(config.backend, "gstreamer")

    def test_config_accepts_ffmpeg_backend(self):
        """Test that FFmpeg backend can be configured."""
        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="ffmpeg"
        )
        self.assertEqual(config.backend, "ffmpeg")

    def test_config_rejects_invalid_backend(self):
        """Test that invalid backend raises validation error."""
        with self.assertRaises(Exception):  # Pydantic ValidationError
            RTSPFrameGrabberConfig(
                rtsp_url=self.test_rtsp_url,
                backend="invalid"
            )

    # =========================================================================
    # GStreamer Backend Tests
    # =========================================================================

    @patch('cv2.getBuildInformation')
    def test_gstreamer_backend_checks_support(self, mock_build_info):
        """Test that GStreamer backend checks for GStreamer support."""
        # Mock OpenCV without GStreamer support
        mock_build_info.return_value = "Video I/O:\n  GStreamer: NO\n"

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer"
        )

        with self.assertRaises(RuntimeError) as context:
            RTSPFrameGrabber(config)

        self.assertIn("GStreamer", str(context.exception))
        self.assertIn("not built with GStreamer support", str(context.exception))

    @patch('cv2.VideoCapture')
    @patch('cv2.getBuildInformation')
    def test_gstreamer_backend_initialization_with_support(self, mock_build_info, mock_videocapture):
        """Test that GStreamer backend initializes when support is available."""
        # Mock OpenCV with GStreamer support
        mock_build_info.return_value = "Video I/O:\n  GStreamer: YES\n"

        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.getBackendName.return_value = "GSTREAMER"
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer"
        )

        grabber = RTSPFrameGrabber(config)

        # Verify GStreamer pipeline was used
        self.assertEqual(grabber.config.backend, "gstreamer")
        mock_videocapture.assert_called_once()
        
        # Verify it used CAP_GSTREAMER backend
        call_args = mock_videocapture.call_args
        # Check if apiPreference was passed (may be positional or keyword arg)
        if len(call_args[0]) > 1:
            self.assertEqual(call_args[0][1], cv2.CAP_GSTREAMER)
        elif 'apiPreference' in call_args[1]:
            self.assertEqual(call_args[1]['apiPreference'], cv2.CAP_GSTREAMER)

        grabber.release()

    @patch('cv2.VideoCapture')
    @patch('cv2.getBuildInformation')
    def test_gstreamer_backend_builds_correct_pipeline(self, mock_build_info, mock_videocapture):
        """Test that GStreamer backend builds correct pipeline string."""
        # Mock OpenCV with GStreamer support
        mock_build_info.return_value = "Video I/O:\n  GStreamer: YES\n"

        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer"
        )

        grabber = RTSPFrameGrabber(config)

        # Get the pipeline string from the VideoCapture call
        call_args = mock_videocapture.call_args
        pipeline = call_args[0][0]

        # Verify key elements are in the pipeline
        self.assertIn("rtspsrc", pipeline)
        self.assertIn(self.test_rtsp_url, pipeline)
        # Note: protocol is optional and not set by default (uses GStreamer auto-negotiation)
        self.assertIn("buffer-mode=none", pipeline)
        self.assertIn("decodebin", pipeline)
        self.assertIn("queue", pipeline)
        self.assertIn("leaky=downstream", pipeline)
        self.assertIn("appsink", pipeline)
        self.assertIn("drop=true", pipeline)

        grabber.release()

    @patch('cv2.VideoCapture')
    @patch('cv2.getBuildInformation')
    def test_gstreamer_backend_with_max_fps(self, mock_build_info, mock_videocapture):
        """Test that GStreamer backend includes videorate when sample_rate is set."""
        # Mock OpenCV with GStreamer support
        mock_build_info.return_value = "Video I/O:\n  GStreamer: YES\n"

        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer",
            sample_rate=15
        )

        grabber = RTSPFrameGrabber(config)

        # Get the pipeline string
        call_args = mock_videocapture.call_args
        pipeline = call_args[0][0]

        # Verify videorate is in the pipeline
        self.assertIn("videorate", pipeline)
        self.assertIn("max-rate=15", pipeline)

        grabber.release()

    @patch('cv2.VideoCapture')
    @patch('cv2.getBuildInformation')
    def test_gstreamer_backend_grab_frame(self, mock_build_info, mock_videocapture):
        """Test that GStreamer backend can grab frames."""
        # Mock OpenCV with GStreamer support
        mock_build_info.return_value = "Video I/O:\n  GStreamer: YES\n"

        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.grab.return_value = True
        mock_capture.retrieve.return_value = (True, self.test_frame)
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer"
        )

        grabber = RTSPFrameGrabber(config)
        frame = grabber.grab()

        # Verify frame was grabbed correctly
        self.assertIsInstance(frame, np.ndarray)
        self.assertEqual(frame.shape, self.test_frame.shape)
        mock_capture.grab.assert_called()
        mock_capture.retrieve.assert_called()

        grabber.release()

    @patch('cv2.VideoCapture')
    @patch('cv2.getBuildInformation')
    def test_gstreamer_backend_grab_failure_raises_error(self, mock_build_info, mock_videocapture):
        """Test that GStreamer backend raises GrabError on grab failure."""
        # Mock OpenCV with GStreamer support
        mock_build_info.return_value = "Video I/O:\n  GStreamer: YES\n"

        # Mock VideoCapture with grab failure
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.grab.return_value = False
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer"
        )

        grabber = RTSPFrameGrabber(config)

        with self.assertRaises(GrabError):
            grabber.grab()

        grabber.release()

    # =========================================================================
    # FFmpeg Backend Tests
    # =========================================================================

    @patch('cv2.VideoCapture')
    def test_ffmpeg_backend_initialization(self, mock_videocapture):
        """Test that FFmpeg backend initializes correctly."""
        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.getBackendName.return_value = "FFMPEG"
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="ffmpeg",
            keep_connection_open=True
        )

        grabber = RTSPFrameGrabber(config)

        # Verify FFmpeg backend was used
        self.assertEqual(grabber.config.backend, "ffmpeg")
        mock_videocapture.assert_called()

        grabber.release()

    @patch('cv2.VideoCapture')
    def test_ffmpeg_backend_grab_frame(self, mock_videocapture):
        """Test that FFmpeg backend can grab frames."""
        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.retrieve.return_value = (True, self.test_frame)
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="ffmpeg",
            keep_connection_open=True
        )

        grabber = RTSPFrameGrabber(config)
        frame = grabber.grab()

        # Verify frame was grabbed correctly
        self.assertIsInstance(frame, np.ndarray)
        self.assertEqual(frame.shape, self.test_frame.shape)

        grabber.release()

    @patch('cv2.VideoCapture')
    def test_ffmpeg_backend_with_keep_connection_open_false(self, mock_videocapture):
        """Test FFmpeg backend with keep_connection_open=False."""
        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.read.return_value = (True, self.test_frame)
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="ffmpeg",
            keep_connection_open=False
        )

        grabber = RTSPFrameGrabber(config)
        frame = grabber.grab()

        # Verify frame was grabbed
        self.assertIsInstance(frame, np.ndarray)
        self.assertEqual(frame.shape, self.test_frame.shape)

        # Verify connection was opened for the grab
        self.assertTrue(mock_videocapture.called)

        grabber.release()

    @patch('cv2.VideoCapture')
    def test_ffmpeg_backend_grab_failure_raises_error(self, mock_videocapture):
        """Test that FFmpeg backend raises GrabError on grab failure."""
        # Mock VideoCapture with read failure
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_capture.retrieve.return_value = (False, None)
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="ffmpeg",
            keep_connection_open=True
        )

        grabber = RTSPFrameGrabber(config)

        with self.assertRaises(GrabError):
            grabber.grab()

        grabber.release()

    # =========================================================================
    # Timeout Tests
    # =========================================================================

    @patch('cv2.VideoCapture')
    @patch('cv2.getBuildInformation')
    def test_gstreamer_backend_includes_timeout_in_pipeline(self, mock_build_info, mock_videocapture):
        """Test that GStreamer backend includes timeout in pipeline."""
        # Mock OpenCV with GStreamer support
        mock_build_info.return_value = "Video I/O:\n  GStreamer: YES\n"

        # Mock VideoCapture
        mock_capture = MagicMock()
        mock_capture.isOpened.return_value = True
        mock_videocapture.return_value = mock_capture

        config = RTSPFrameGrabberConfig(
            rtsp_url=self.test_rtsp_url,
            backend="gstreamer",
            timeout=10.0
        )

        grabber = RTSPFrameGrabber(config)

        # Get the pipeline string
        call_args = mock_videocapture.call_args
        pipeline = call_args[0][0]

        # Verify timeout is in the pipeline (10 seconds = 10000000 microseconds)
        self.assertIn("tcp-timeout=10000000", pipeline)

        grabber.release()

    def test_rtsp_config_default_timeout_is_5_seconds(self):
        """Test that default timeout is 5 seconds."""
        config = RTSPFrameGrabberConfig(rtsp_url=self.test_rtsp_url)
        self.assertEqual(config.timeout, 5.0)


if __name__ == "__main__":
    unittest.main()

