# import os
# import shutil
# import tempfile
# import unittest

# import cv2
# import numpy as np

# from framegrab.grabber import FileStreamFrameGrabber


# class TestFileStreamFrameGrabber(unittest.TestCase):
#     def setUp(self):
#         """Create temporary video files for testing."""

#         self.temp_dir = tempfile.mkdtemp()
#         self.mp4_path = os.path.join(self.temp_dir, "test_video.mp4")
#         self.mjpeg_path = os.path.join(self.temp_dir, "test_video.mjpeg")

#         # Create test videos
#         width, height = 640, 480
#         fps = 30.0

#         # Create MP4 test video
#         fourcc_mp4 = cv2.VideoWriter.fourcc(*"mp4v")
#         writer_mp4 = cv2.VideoWriter(self.mp4_path, fourcc_mp4, fps, (width, height))

#         # Create MJPEG test video
#         fourcc_mjpeg = cv2.VideoWriter.fourcc(*"MJPG")
#         writer_mjpeg = cv2.VideoWriter(
#             self.mjpeg_path, fourcc_mjpeg, fps, (width, height)
#         )

#         # Generate 1 second of frames for both videos
#         for _ in range(int(fps)):
#             frame = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
#             writer_mp4.write(frame)
#             writer_mjpeg.write(frame)

#         writer_mp4.release()
#         writer_mjpeg.release()

#         self.base_config_mp4 = {
#             "input_type": "file",
#             "name": "test_video_mp4",
#             "id": {"filename": self.mp4_path},
#         }

#         self.base_config_mjpeg = {
#             "input_type": "file",
#             "name": "test_video_mjpeg",
#             "id": {"filename": self.mjpeg_path},
#         }

#     def tearDown(self):
#         """Clean up temporary files."""
#         shutil.rmtree(self.temp_dir)

#     def test_init_success_mp4(self):
#         """Test successful initialization with MP4."""
#         grabber = FileStreamFrameGrabber(self.base_config_mp4)
#         self.assertEqual(grabber.fps_source, 30.0)
#         self.assertFalse(grabber.should_drop_frames)
#         grabber.release()

#     def test_init_success_mjpeg(self):
#         """Test successful initialization with MJPEG."""
#         with FileStreamFrameGrabber(self.base_config_mjpeg) as grabber:
#             # MJPEG files have variable frame rates and also the FPS that cv2 returns is not accurate
#             self.assertGreater(grabber.fps_source, 20.0)
#             self.assertLess(grabber.fps_source, 40.0)
#             self.assertFalse(grabber.should_drop_frames)

#     def test_init_with_fps_target_mp4(self):
#         """Test initialization with FPS target for MP4."""
#         target_fps = 15.0
#         config = self.base_config_mp4.copy()
#         config["options"] = {"max_fps": target_fps}

#         grabber = FileStreamFrameGrabber(config)
#         self.assertTrue(grabber.should_drop_frames)
#         self.assertEqual(grabber.fps_target, target_fps)
#         grabber.release()

#     def test_init_with_fps_target_mjpeg(self):
#         """Test initialization with FPS target for MJPEG."""
#         target_fps = 15.0
#         config = self.base_config_mjpeg.copy()
#         config["options"] = {"max_fps": target_fps}

#         grabber = FileStreamFrameGrabber(config)
#         self.assertTrue(grabber.should_drop_frames)
#         self.assertEqual(grabber.fps_target, target_fps)
#         grabber.release()

#     def test_init_without_filename(self):
#         """Test initialization fails without filename."""
#         config = {"input_type": "file", "name": "test_video"}

#         with self.assertRaises(ValueError):
#             FileStreamFrameGrabber(config)

#     def test_grab_frame_mp4(self):
#         """Test frame grabbing from MP4."""
#         grabber = FileStreamFrameGrabber(self.base_config_mp4)
#         frame = grabber.grab()

#         self.assertIsInstance(frame, np.ndarray)
#         self.assertEqual(frame.shape, (480, 640, 3))
#         grabber.release()

#     def test_grab_frame_mjpeg(self):
#         """Test frame grabbing from MJPEG."""
#         grabber = FileStreamFrameGrabber(self.base_config_mjpeg)
#         frame = grabber.grab()

#         self.assertIsInstance(frame, np.ndarray)
#         self.assertEqual(frame.shape, (480, 640, 3))
#         grabber.release()

#     def test_invalid_file(self):
#         """Test initialization with invalid file."""
#         config = self.base_config_mp4.copy()
#         config["id"]["filename"] = "nonexistent.mp4"

#         with self.assertRaises(ValueError):
#             FileStreamFrameGrabber(config)

#     def test_invalid_resolution_option(self):
#         """Test that setting resolution raises an error for both formats."""
#         for config in [self.base_config_mp4, self.base_config_mjpeg]:
#             config_with_res = config.copy()
#             config_with_res["options"] = {"resolution": {"width": 1920, "height": 1080}}

#             grabber = FileStreamFrameGrabber(config_with_res)
#             with self.assertRaises(ValueError):
#                 grabber.apply_options(config_with_res["options"])
#             grabber.release()
