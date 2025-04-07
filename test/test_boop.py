import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from framegrab.config import (
    BaslerFrameGrabberConfig,
    FrameGrabberConfig,
    FileStreamFrameGrabberConfig,
    GenericUSBFrameGrabberConfig,
    HttpLiveStreamingFrameGrabberConfig,
    MockFrameGrabberConfig,
    RaspberryPiCSI2FrameGrabberConfig,
    RealSenseFrameGrabberConfig,
    RTSPFrameGrabberConfig,
    YouTubeLiveFrameGrabberConfig
)
from framegrab.grabber import (
    BaslerFrameGrabber,
    FrameGrabber,
    FileStreamFrameGrabber,
    GenericUSBFrameGrabber,
    HttpLiveStreamingFrameGrabber,
    MockFrameGrabber,
    RaspberryPiCSI2FrameGrabber,
    RTSPFrameGrabber,
    RealSenseFrameGrabber,
    YouTubeLiveFrameGrabber
)
import cv2
import pdb
class TestAllGrabberTypes(unittest.TestCase):

    def _get_mock_image(self):
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def _test_grabber_helper(self, grabber, resolution_width = None, resolution_height = None):
        original_grabber_config_as_dict = grabber.config.to_framegrab_config_dict()
        expected_input_type = next(key for key, value in FrameGrabberConfig.get_input_type_to_class_dict().items() if value == type(grabber.config))
        self.assertEqual(original_grabber_config_as_dict["input_type"], expected_input_type)

        expected_id_key = FrameGrabberConfig.get_input_type_to_id_dict()[expected_input_type]
        # make sure the id field moves from a direct attribute to nested under the id key
        self.assertEqual(original_grabber_config_as_dict["id"][expected_id_key], getattr(grabber.config, expected_id_key))
        self.assertNotIn(expected_id_key, original_grabber_config_as_dict)

        grabber.release()
        new_grabber_config = FrameGrabberConfig.from_framegrab_config_dict(original_grabber_config_as_dict)
        new_grabber = FrameGrabber.create_grabber(new_grabber_config)
        self.assertEqual(new_grabber.config.to_framegrab_config_dict(), original_grabber_config_as_dict)

        frame = new_grabber.grab()
        expected_frame = self._get_mock_image()
        expected_frame = new_grabber._crop(expected_frame)

        if resolution_width and resolution_height:
            expected_frame = cv2.resize(expected_frame, (resolution_width, resolution_height))

        expected_frame = new_grabber._digital_zoom(expected_frame)
        np.testing.assert_array_equal(frame, expected_frame)

        new_options = {"zoom": {"digital": 4}}
        new_grabber.apply_options(new_options)
        self.assertEqual(new_grabber.config.digital_zoom, 4)
        new_grabber.release()

    @patch('framegrab.grabber.GenericUSBFrameGrabber._find_cameras')
    @patch('cv2.VideoCapture')
    def test_generic_usb_grabber(self, mock_video_capture, mock_find_cameras):
        mock_capture_instance = MagicMock()
        mock_video_capture.return_value = mock_capture_instance
        mock_capture_instance.isOpened.return_value = True
        mock_capture_instance.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
        mock_video_capture.return_value.isOpened.return_value = True

        serial_number = "1234567890"

        # Mock the _find_cameras method to return a specific camera configuration
        mock_find_cameras.return_value = [{
            'serial_number': serial_number,
            'device_path': '/dev/video5',
            'idx': 5,
            'camera_name': 'lalala'
        }]

        resolution_width = 640
        resolution_height = 480
        digital_zoom = 2
        config = GenericUSBFrameGrabberConfig(
            name="usb_framegrabber",
            serial_number=serial_number,
            resolution_width=resolution_width,
            resolution_height=resolution_height,
            digital_zoom=digital_zoom
        )
        usb_framegrabber = GenericUSBFrameGrabber(config)
        self._test_grabber_helper(usb_framegrabber, resolution_width, resolution_height)

    @patch('cv2.VideoCapture')
    def test_rtsp_grabber(self, mock_video_capture):
        mock_capture_instance = MagicMock()
        mock_video_capture.return_value = mock_capture_instance
        mock_capture_instance.isOpened.return_value = True
        mock_capture_instance.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
        mock_video_capture.return_value.isOpened.return_value = True

        rtsp_url = "rtsp://localhost:8000/test"
        config = RTSPFrameGrabberConfig(name="rtsp_framegrabber", rtsp_url=rtsp_url, keep_connection_open=False, max_fps=10)
        rtsp_framegrabber = RTSPFrameGrabber(config)
        self._test_grabber_helper(rtsp_framegrabber)

    @patch('pypylon.pylon.TlFactory')
    @patch('pypylon.pylon.InstantCamera')
    @patch('pypylon.pylon.ImageFormatConverter')
    def test_basler_grabber(self, mock_image_format_converter, mock_instant_camera, mock_tl_factory):
        mock_tl_factory_instance = MagicMock()
        mock_tl_factory.GetInstance.return_value = mock_tl_factory_instance
        mock_device = MagicMock()
        mock_device.GetSerialNumber.return_value = "1234567890"
        mock_tl_factory_instance.EnumerateDevices.return_value = [mock_device]
        
        mock_camera_instance = MagicMock()
        mock_camera_instance.GetNodeMap.return_value = MagicMock()
        mock_instant_camera.return_value = mock_camera_instance
        mock_grab_result = MagicMock()
        mock_grab_result.GrabSucceeded.return_value = True
        mock_camera_instance.GrabOne.return_value.__enter__.return_value = mock_grab_result
        
        mock_image = MagicMock()
        mock_image.GetArray.return_value = self._get_mock_image()
        mock_image_format_converter.return_value.Convert.return_value = mock_image

        basler_framegrabber_config = BaslerFrameGrabberConfig(name="basler_framegrabber", serial_number="1234567890", basler_options={"ExposureTime": 10000})
        basler_framegrabber = BaslerFrameGrabber(basler_framegrabber_config)
        self._test_grabber_helper(basler_framegrabber)
   
    @unittest.skip("This test needs to be run on a realsesne compatible device")
    @patch('pyrealsense2.pyrealsense2.context')
    @patch('pyrealsense2.pyrealsense2.pipeline')
    @patch('pyrealsense2.pyrealsense2.config')
    def test_realsense_grabber(self, mock_rs_config, mock_pipeline, mock_context):
        mock_ctx_instance = MagicMock()

        mock_device = MagicMock()
        serial_number = "1234567890"
        mock_device.get_info.return_value = serial_number
        mock_ctx_instance.devices = [mock_device]

        mock_context.return_value = mock_ctx_instance
        
        mock_pipeline_instance = MagicMock()
        mock_pipeline.return_value = mock_pipeline_instance
        mock_frames = MagicMock()
        mock_color_frame = MagicMock()
        mock_color_frame.get_data.return_value = self._get_mock_image()
        mock_frames.get_color_frame.return_value = mock_color_frame
        mock_pipeline_instance.wait_for_frames.return_value = mock_frames

        mock_rs_config_instance = MagicMock()
        mock_rs_config.return_value = mock_rs_config_instance

        realsense_framegrabber_config = RealSenseFrameGrabberConfig(name="realsense_framegrabber", resolution_width=640, resolution_height=480, side_by_side_depth=True)
        realsense_framegrabber = RealSenseFrameGrabber(realsense_framegrabber_config)
        self._test_grabber_helper(realsense_framegrabber)
    

    @unittest.skip("This test needs to be run on a Raspberry Pi due to imports")
    @patch('picamera2.Picamera2.global_camera_info')
    def test_raspberry_pi_grabber(self, mock_global_camera_info):
        mock_camera_instance = MagicMock()
        mock_global_camera_info.return_value = mock_camera_instance
        mock_camera_instance.capture_array.return_value = self._get_mock_image()

        raspberry_pi_framegrabber_config = RaspberryPiCSI2FrameGrabberConfig(name="raspberry_pi_framegrabber")
        raspberry_pi_framegrabber = RaspberryPiCSI2FrameGrabber(raspberry_pi_framegrabber_config)
        self._test_grabber_helper(raspberry_pi_framegrabber)

    @patch('framegrab.config.YouTubeLiveFrameGrabberConfig.hls_url', return_value="https://fakeurl.com")
    @patch('cv2.VideoCapture')
    def test_http_grabber(self, mock_video_capture, mock_extract_hls_url):
        mock_capture_instance = MagicMock()
        mock_capture_instance.isOpened.return_value = True
        mock_capture_instance.read.return_value = (True, self._get_mock_image())
        mock_video_capture.return_value = mock_capture_instance

        http_framegrabber_config = HttpLiveStreamingFrameGrabberConfig(name="http_framegrabber", hls_url="http://randomurl.com/test")
        http_framegrabber = HttpLiveStreamingFrameGrabber(http_framegrabber_config)
        self._test_grabber_helper(http_framegrabber)

    @patch('framegrab.config.YouTubeLiveFrameGrabberConfig.hls_url', return_value="https://fakeurl.com")
    @patch('cv2.VideoCapture')
    def test_youtube_grabber(self, mock_video_capture, mock_extract_hls_url):
        mock_capture_instance = MagicMock()
        mock_capture_instance.isOpened.return_value = True
        mock_capture_instance.read.return_value = (True, self._get_mock_image())
        mock_video_capture.return_value = mock_capture_instance

        crop = {
            "relative": {
                "top": 0.05,
                "bottom": 0.95,
                "left": 0.03,
                "right": 0.97
            }
        }
        config = YouTubeLiveFrameGrabberConfig(name="youtube_framegrabber", youtube_url="https://www.youtube.com/watch?v=7_srED6k0bE", keep_connection_open=False, crop=crop)
        youtube_framegrabber = YouTubeLiveFrameGrabber(config)
        self._test_grabber_helper(youtube_framegrabber)

    @patch('cv2.VideoCapture')
    def test_filesystem_grabber(self, mock_video_capture):
        mock_capture_instance = MagicMock()
        mock_video_capture.return_value = mock_capture_instance
        mock_capture_instance.isOpened.return_value = True
        mock_capture_instance.read.return_value = (True, self._get_mock_image())
        mock_capture_instance.get.return_value = 30.0
        mock_video_capture.return_value = mock_capture_instance

        filesystem_framegrabber_config = FileStreamFrameGrabberConfig(name="filesystem_framegrabber", filename="test.mp4")
        filesystem_framegrabber = FileStreamFrameGrabber(filesystem_framegrabber_config)
        self._test_grabber_helper(filesystem_framegrabber)
    
    @patch('framegrab.grabber.MockFrameGrabber._grab_implementation')
    def test_mock_framegrabber(self, mock_grab_implementation):
        mock_grab_implementation.return_value = self._get_mock_image()
        mock_grabber_config = MockFrameGrabberConfig(serial_number="123")
        mock_grabber = MockFrameGrabber(mock_grabber_config)
        self._test_grabber_helper(mock_grabber)
