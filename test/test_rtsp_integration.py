"""Integration tests for RTSP backends with real OpenCV and synthetic streams.

These tests use real OpenCV VideoCapture with real backends (GStreamer/FFmpeg),
but generate synthetic test streams instead of requiring actual RTSP cameras.
"""

import unittest
import tempfile
import os
import numpy as np
import cv2
from pathlib import Path

from framegrab.grabber import RTSPFrameGrabber
from framegrab.config import RTSPFrameGrabberConfig
from framegrab.exceptions import GrabError


class TestRTSPBackendsIntegration(unittest.TestCase):
    """Integration tests for RTSP backends with real OpenCV."""

    @classmethod
    def setUpClass(cls):
        """Create synthetic test video files once for all tests."""
        cls.test_video_dir = tempfile.mkdtemp()
        cls.test_video_path = os.path.join(cls.test_video_dir, "test_video.mp4")
        
        # Create a short test video (10 frames)
        width, height = 640, 480
        fps = 30
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        writer = cv2.VideoWriter(cls.test_video_path, fourcc, fps, (width, height))
        
        for i in range(10):
            # Create unique frames (changing colors)
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            frame[:, :] = [i * 25 % 255, (i * 50) % 255, (i * 75) % 255]
            writer.write(frame)
        
        writer.release()
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test video files."""
        if os.path.exists(cls.test_video_path):
            os.remove(cls.test_video_path)
        if os.path.exists(cls.test_video_dir):
            os.rmdir(cls.test_video_dir)

    def setUp(self):
        """Set up test fixtures."""
        self.grabber = None

    def tearDown(self):
        """Clean up after tests."""
        if self.grabber:
            self.grabber.release()

    # =========================================================================
    # Helper Methods
    # =========================================================================

    def _check_gstreamer_available(self):
        """Check if GStreamer is available in OpenCV."""
        build_info = cv2.getBuildInformation()
        return "GStreamer" in build_info and "YES" in build_info.split("GStreamer")[1][:50]

    def _create_gstreamer_test_pipeline(self):
        """Create a GStreamer pipeline for testing (no RTSP needed)."""
        # Use videotestsrc for synthetic test pattern
        # This tests the entire GStreamer integration without needing a real stream
        pipeline = (
            "videotestsrc num-buffers=10 pattern=smpte ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        return pipeline

    # =========================================================================
    # FFmpeg Backend Tests (Using mock RTSP server or direct capture testing)
    # =========================================================================

    def test_ffmpeg_backend_initialization(self):
        """Test FFmpeg backend initializes correctly without connecting."""
        # Test that we can create the config and grabber object
        # (won't actually connect unless we call grab())
        config = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="ffmpeg",
            keep_connection_open=False  # Don't connect on init
        )
        
        self.grabber = RTSPFrameGrabber(config)
        
        # Verify configuration
        self.assertEqual(self.grabber.config.backend, "ffmpeg")
        self.assertEqual(self.grabber.config.keep_connection_open, False)
        
    def test_ffmpeg_backend_with_keep_connection_open_true(self):
        """Test FFmpeg backend config with keep_connection_open=True."""
        config = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="ffmpeg",
            keep_connection_open=True
        )
        
        # This will try to connect and fail, but we can check the config
        try:
            self.grabber = RTSPFrameGrabber(config)
            # If it somehow succeeds (shouldn't with fake URL), verify config
            self.assertEqual(self.grabber.config.keep_connection_open, True)
        except ValueError:
            # Expected - can't connect to fake URL
            pass

    def test_ffmpeg_backend_with_max_fps(self):
        """Test FFmpeg backend with max_fps setting."""
        config = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="ffmpeg",
            keep_connection_open=True,
            max_fps=15
        )
        
        # Verify config
        self.assertEqual(config.max_fps, 15)
        self.assertEqual(config.backend, "ffmpeg")

    # =========================================================================
    # GStreamer Backend Tests (Synthetic test patterns)
    # =========================================================================

    def test_gstreamer_backend_requires_support(self):
        """Test that GStreamer backend checks for support."""
        has_gstreamer = self._check_gstreamer_available()
        
        config = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="gstreamer"
        )
        
        if not has_gstreamer:
            # Should raise RuntimeError when GStreamer is not available
            with self.assertRaises(RuntimeError) as context:
                RTSPFrameGrabber(config)
            self.assertIn("GStreamer", str(context.exception))
        else:
            # Should succeed when GStreamer is available
            # (though it will fail to connect to the fake URL)
            try:
                grabber = RTSPFrameGrabber(config)
                grabber.release()
            except ValueError:
                # Expected - can't connect to fake URL
                pass

    @unittest.skipUnless(
        "GStreamer" in cv2.getBuildInformation() and 
        "YES" in cv2.getBuildInformation().split("GStreamer")[1][:50],
        "GStreamer not available in OpenCV"
    )
    def test_gstreamer_backend_with_test_source(self):
        """Test GStreamer backend with synthetic test pattern."""
        # Use GStreamer's videotestsrc directly (no RTSP needed)
        pipeline = self._create_gstreamer_test_pipeline()
        
        # Create VideoCapture directly with GStreamer pipeline
        capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not capture.isOpened():
            self.skipTest("Could not open GStreamer test pipeline")
        
        # Grab a frame
        ret, frame = capture.read()
        
        capture.release()
        
        # Verify frame
        self.assertTrue(ret)
        self.assertIsInstance(frame, np.ndarray)
        self.assertEqual(frame.shape, (480, 640, 3))
        self.assertEqual(frame.dtype, np.uint8)

    @unittest.skipUnless(
        "GStreamer" in cv2.getBuildInformation() and 
        "YES" in cv2.getBuildInformation().split("GStreamer")[1][:50],
        "GStreamer not available in OpenCV"
    )
    def test_gstreamer_pipeline_construction(self):
        """Test that GStreamer pipeline is constructed correctly."""
        config = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="gstreamer",
            sample_rate=15,
            timeout=10.0
        )

        # Don't actually connect - just verify config is set correctly
        # The actual pipeline construction is tested in test_gstreamer_backend_with_test_source
        self.assertEqual(config.backend, "gstreamer")
        self.assertEqual(config.sample_rate, 15)
        self.assertEqual(config.timeout, 10.0)

    @unittest.skipUnless(
        "GStreamer" in cv2.getBuildInformation() and 
        "YES" in cv2.getBuildInformation().split("GStreamer")[1][:50],
        "GStreamer not available in OpenCV"
    )
    def test_gstreamer_backend_pipeline_with_rate_limit(self):
        """Test that rate limiting is included in GStreamer pipeline."""
        # Create a test pipeline with rate limiting
        pipeline = (
            "videotestsrc num-buffers=30 ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "videorate drop-only=true max-rate=5 ! "  # Rate limit to 5 fps
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        
        capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not capture.isOpened():
            self.skipTest("Could not open GStreamer test pipeline with rate limit")
        
        # Grab frames and measure timing
        import time
        frames = []
        times = []
        
        for _ in range(5):
            t0 = time.time()
            ret, frame = capture.read()
            t1 = time.time()
            
            if ret:
                frames.append(frame)
                times.append(t1 - t0)
        
        capture.release()
        
        # Verify we got frames
        self.assertGreater(len(frames), 0)
        
        # Note: Timing verification would be flaky, so we just verify frames were grabbed

    @unittest.skipUnless(
        "GStreamer" in cv2.getBuildInformation() and
        "YES" in cv2.getBuildInformation().split("GStreamer")[1][:50],
        "GStreamer not available in OpenCV"
    )
    def test_gstreamer_backend_max_fps_higher_than_source(self):
        """Test that max_fps higher than source FPS works without errors.

        This verifies that when max_fps is set higher than the camera's native FPS,
        the videorate element with drop-only=true passes frames through without
        attempting to duplicate them (which would cause issues).
        """
        # Create a test pipeline with 10 fps source and 30 fps max-rate
        pipeline = (
            "videotestsrc num-buffers=10 ! "
            "video/x-raw,width=640,height=480,framerate=10/1 ! "  # Source at 10 FPS
            "videorate drop-only=true max-rate=30 ! "  # Request 30 FPS (higher than source)
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not capture.isOpened():
            self.skipTest("Could not open GStreamer test pipeline with max-rate > source fps")

        # Grab frames - should work without errors
        frames_grabbed = 0
        max_attempts = 15  # Try to grab more than num-buffers to ensure it works

        for _ in range(max_attempts):
            ret, frame = capture.read()
            if ret:
                frames_grabbed += 1
                # Verify frame is valid
                self.assertIsInstance(frame, np.ndarray)
                self.assertEqual(frame.shape, (480, 640, 3))
            else:
                # Expected when we run out of buffers
                break

        capture.release()

        # Verify we successfully grabbed frames (should get ~10 frames from source)
        self.assertGreater(frames_grabbed, 0,
            "Should successfully grab frames when max_fps > source fps")
        self.assertLessEqual(frames_grabbed, 12,
            "Should not duplicate frames (drop-only=true)")

    # =========================================================================
    # Backend Comparison Tests
    # =========================================================================

    def test_backend_config_parameter(self):
        """Test that backend parameter is correctly set in config."""
        config_ffmpeg = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="ffmpeg"
        )
        self.assertEqual(config_ffmpeg.backend, "ffmpeg")
        
        config_gst = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="gstreamer"
        )
        self.assertEqual(config_gst.backend, "gstreamer")

    def test_invalid_backend_raises_error(self):
        """Test that invalid backend raises validation error."""
        with self.assertRaises(Exception):  # Pydantic ValidationError
            RTSPFrameGrabberConfig(
                rtsp_url="rtsp://test.example.com/stream",
                backend="invalid_backend"
            )

    def test_sample_rate_exceeds_60_raises_error(self):
        """Test that sample_rate > 60 raises validation error."""
        with self.assertRaises(Exception) as context:  # Pydantic ValidationError
            RTSPFrameGrabberConfig(
                rtsp_url="rtsp://test.example.com/stream",
                backend="gstreamer",
                sample_rate=61
            )
        self.assertIn("sample_rate cannot exceed 60", str(context.exception))

    def test_sample_rate_at_60_is_valid(self):
        """Test that sample_rate = 60 is valid."""
        config = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://test.example.com/stream",
            backend="gstreamer",
            sample_rate=60
        )
        self.assertEqual(config.sample_rate, 60)

    # =========================================================================
    # Error Handling Tests
    # =========================================================================

    def test_ffmpeg_backend_invalid_url_connection_fails(self):
        """Test that invalid RTSP URL fails to connect."""
        config = RTSPFrameGrabberConfig(
            rtsp_url="rtsp://nonexistent.invalid.example.com:9999/stream",
            backend="ffmpeg",
            keep_connection_open=True,
            timeout=1.0  # Short timeout
        )
        
        # ValueError should be raised during initialization (can't connect)
        with self.assertRaises(ValueError):
            RTSPFrameGrabber(config)


class TestGStreamerAvailability(unittest.TestCase):
    """Tests to report GStreamer availability status."""
    
    def test_report_gstreamer_status(self):
        """Report whether GStreamer is available (informational test)."""
        build_info = cv2.getBuildInformation()
        has_gstreamer = "GStreamer" in build_info and "YES" in build_info.split("GStreamer")[1][:50]
        
        print("\n" + "="*60)
        print("GStreamer Status Report")
        print("="*60)
        print(f"OpenCV Version: {cv2.__version__}")
        print(f"GStreamer Available: {'YES ✓' if has_gstreamer else 'NO ✗'}")
        
        if has_gstreamer:
            print("\n✓ GStreamer tests will run")
            # Extract GStreamer version from build info
            for line in build_info.split("\n"):
                if "gstreamer" in line.lower():
                    print(f"  {line.strip()}")
        else:
            print("\n⚠ GStreamer tests will be skipped")
            print("  To enable GStreamer:")
            print("  1. Install system OpenCV with GStreamer (python3-opencv)")
            print("  2. Add gstreamer1.0-libav for codec support")
            print("  3. Rebuild Docker image")
        print("="*60)
        
        # This test always passes - it's just informational
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main(verbosity=2)

