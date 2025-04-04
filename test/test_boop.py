import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from framegrab.grabber import BaslerFrameGrabber, GenericUSBFrameGrabber, RTSPFrameGrabber, RealSenseFrameGrabber, YouTubeLiveFrameGrabber, RaspberryPiCSI2FrameGrabber
import pdb
import copy
import cv2
class TestAllGrabberTypes(unittest.TestCase):

    def _get_mock_image(self):
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def _test_grabber_helper(self, grabber, resolution_width = 640, resolution_height = 480, digital_zoom = 1):
        grabber_as_dict = grabber.to_dict()
        grabber.release()
        new_grabber = grabber.from_dict(grabber_as_dict)
        pdb.set_trace()
        self.assertEqual(new_grabber.to_dict(), grabber_as_dict)
        frame = grabber.grab()
        expected_frame = cv2.resize(self._get_mock_image(), (resolution_width // digital_zoom, resolution_height // digital_zoom))
        np.testing.assert_array_equal(frame, expected_frame)

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
        usb_framegrabber = GenericUSBFrameGrabber(
            camera_name="usb_framegrabber",
            serial_number=serial_number,
            resolution_width=resolution_width,
            resolution_height=resolution_height,
            digital_zoom=digital_zoom
        )
        self._test_grabber_helper(usb_framegrabber, resolution_width, resolution_height, digital_zoom)

    @patch('cv2.VideoCapture')
    def test_rtsp_grabber(self, mock_video_capture):
        mock_capture_instance = MagicMock()
        mock_video_capture.return_value = mock_capture_instance
        mock_capture_instance.isOpened.return_value = True
        mock_capture_instance.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
        mock_video_capture.return_value.isOpened.return_value = True

        rtsp_url = "rtsp://localhost:8000/test"
        rtsp_framegrabber = RTSPFrameGrabber(camera_name="rtsp_framegrabber", rtsp_url=rtsp_url, keep_connection_open=False, max_fps=10)
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
        mock_instant_camera.return_value = mock_camera_instance
        mock_grab_result = MagicMock()
        mock_grab_result.GrabSucceeded.return_value = True
        mock_camera_instance.GrabOne.return_value.__enter__.return_value = mock_grab_result
        
        mock_image = MagicMock()
        mock_image.GetArray.return_value = self._get_mock_image()
        mock_image_format_converter.return_value.Convert.return_value = mock_image

        basler_framegrabber = BaslerFrameGrabber(camera_name="basler_framegrabber", serial_number="1234567890", basler_options={"ExposureTime": 10000})
        self._test_grabber_helper(basler_framegrabber)
   
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

        realsense_framegrabber = RealSenseFrameGrabber(camera_name="realsense_framegrabber")
        self._test_grabber_helper(realsense_framegrabber)
    

    @unittest.skip("This test needs to be run on a Raspberry Pi due to imports")
    @patch('picamera2.Picamera2.global_camera_info')
    def test_raspberry_pi_grabber(self, mock_global_camera_info):
        mock_camera_instance = MagicMock()
        mock_global_camera_info.return_value = mock_camera_instance
        mock_camera_instance.capture_array.return_value = self._get_mock_image()

        raspberry_pi_framegrabber = RaspberryPiCSI2FrameGrabber(camera_name="raspberry_pi_framegrabber")
        self._test_grabber_helper(raspberry_pi_framegrabber)

    @patch('framegrab.grabber.YouTubeLiveFrameGrabber._extract_hls_url', return_value="https://fakeurl.com")
    @patch('cv2.VideoCapture')
    def test_youtube_grabber(self, mock_video_capture, mock_extract_hls_url):
        mock_capture_instance = MagicMock()
        mock_capture_instance.isOpened.return_value = True
        mock_capture_instance.read.return_value = (True, self._get_mock_image())
        mock_video_capture.return_value = mock_capture_instance

        youtube_framegrabber = YouTubeLiveFrameGrabber(camera_name="youtube_framegrabber", youtube_url="https://www.youtube.com/watch?v=7_srED6k0bE", keep_connection_open=False)
        self._test_grabber_helper(youtube_framegrabber)
