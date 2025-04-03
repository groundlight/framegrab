import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from framegrab.grabber import BaslerFrameGrabber, RTSPFrameGrabber, GenericUSBFrameGrabber
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
