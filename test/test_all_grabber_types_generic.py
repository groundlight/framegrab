import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from framegrab.config import (
    BaslerFrameGrabberConfig,
    FrameGrabberConfig,
    FileStreamFrameGrabberConfig,
    GenericUSBFrameGrabberConfig,
    HttpLiveStreamingFrameGrabberConfig,
    InputTypes,
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
class TestAllGrabberTypes(unittest.TestCase):
    """ Basic tests that show we can instantiate each grabber type's config and create a grabber from it """

    
    def _get_mock_image(self):
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def setup_mock_generic_usb_grabber(self, mock_video_capture, initial_width=640, initial_height=480):
        mock_capture_instance = MagicMock()
        mock_video_capture.return_value = mock_capture_instance

        # Initialize resolution storage
        resolution = {'width': initial_width, 'height': initial_height}
        
        # Mock the get method to return the current resolution
        def mock_get(prop_id):
            if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
                return resolution['width']
            elif prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
                return resolution['height']
            return 0

        mock_capture_instance.get.side_effect = mock_get

        # Mock the set method to update the resolution
        def mock_set(prop_id, value):
            if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
                resolution['width'] = value
            elif prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
                resolution['height'] = value

        mock_capture_instance.set.side_effect = mock_set

        # Mock the read method to return a frame with the current resolution
        def mock_read():
            frame = np.zeros((int(resolution['height']), int(resolution['width']), 3), dtype=np.uint8)
            return True, frame

        mock_capture_instance.read.side_effect = mock_read

        return mock_capture_instance
    
    def _test_grabber_helper(self, grabber, resolution_width = None, resolution_height = None):
        original_grabber_config_as_dict = grabber.config.to_framegrab_config_dict()
        expected_input_type = next(key for key, value in FrameGrabberConfig.get_input_type_to_class_dict().items() if value == type(grabber.config))
        self.assertEqual(original_grabber_config_as_dict["input_type"], expected_input_type.value)

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
        self.setup_mock_generic_usb_grabber(mock_video_capture)
        serial_number = "1234567890"

        # Mock the _find_cameras method to return a specific camera configuration
        mock_find_cameras.return_value = [{
            'serial_number': serial_number,
            'device_path': '/dev/video5',
            'idx': 5,
            'camera_name': 'lalala'
        }]

        resolution_width = 1920
        resolution_height = 1080
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

    @unittest.skip("This test needs to be run on a basler compatible device")
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

        basler_framegrabber.apply_options({"basler_options": {"ExposureTime": 10000}})
        self.assertEqual(basler_framegrabber.config.basler_options["ExposureTime"], 10000)
   
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

        mock_depth_frame = MagicMock()
        mock_depth_frame.get_data.return_value = np.zeros((480, 640), dtype=np.uint16)
        mock_frames.get_depth_frame.return_value = mock_depth_frame

        mock_frames.get_color_frame.return_value = mock_color_frame
        mock_pipeline_instance.wait_for_frames.return_value = mock_frames

        mock_rs_config_instance = MagicMock()
        mock_rs_config.return_value = mock_rs_config_instance

        realsense_framegrabber_config = RealSenseFrameGrabberConfig(name="realsense_framegrabber", resolution_width=640, resolution_height=480, side_by_side_depth=False)
        realsense_framegrabber = RealSenseFrameGrabber(realsense_framegrabber_config)
        self._test_grabber_helper(realsense_framegrabber)

        realsense_framegrabber.apply_options({"side_by_side_depth": True})
        self.assertEqual(realsense_framegrabber.config.side_by_side_depth, True)
    

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

    @patch('streamlink.streams', return_value={"best": MagicMock(url="https://fakeurl.com/stream.m3u8")})
    @patch('cv2.VideoCapture')
    def test_youtube_grabber(self, mock_video_capture, mock_streams):
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
    

    @patch('streamlink.streams', return_value={"best": MagicMock(url="https://fakeurl.com/stream.m3u8")})
    def test_create_config(self, mock_streams):
        """ Test the create method of FrameGrabberConfig. We want to make sure that all possible parameters are supported """
        for input_type in list(InputTypes):
            model = FrameGrabberConfig.get_class_for_input_type(input_type)

            # we want to populate every possible parameter for testing
            constructor_params = model.model_fields
            # Create a dictionary to hold parameter names and mock values
            params = {}
            for param in constructor_params:
                if param == 'self':
                    continue
                # Assign mock values based on parameter name or type
                if param == 'name':
                    params[param] = f"{input_type}_framegrabber"
                elif param == 'serial_number':
                    params[param] = "1234567890"
                elif param == 'resolution_width' or param == 'resolution_height':
                    params[param] = 640 if 'width' in param else 480
                elif param == "num_90_deg_rotations":
                    params[param] = 1
                elif "keep_connection_open" in param:
                    params[param] = False
                elif param == "rtsp_url":
                    params[param] = "rtsp://example.com"
                elif param == "hls_url":
                    params[param] = "http://example.com"
                elif param == "youtube_url":
                    params[param] = "https://www.youtube.com/watch?v=7_srED6k0bE"
                elif param == "side_by_side_depth":
                    params[param] = True
                elif param == "filename":
                    params[param] = "test.mp4"
                elif param == "fps":
                    params[param] = 30
                elif param == "digital_zoom":
                    params[param] = 2
                elif param == "crop":
                    params[param] = {
                        "relative": {
                            "top": 0.05,
                            "bottom": 0.95,
                            "left": 0.03,
                            "right": 0.97
                        }
                    }
                elif param == "basler_options":
                    params[param] = {"ExposureTime": 10000}
                elif param == "max_fps":
                    params[param] = 30
                else:
                    raise ValueError(f"Unknown parameter: {param}")

            config_with_create = FrameGrabberConfig.create(input_type=input_type, **params)
            config_with_init = model(**params)

            self.assertEqual(config_with_create.to_framegrab_config_dict(), config_with_init.to_framegrab_config_dict())
    
    def test_determine_input_type_in_create(self):
        config = FrameGrabberConfig.create(rtsp_url="rtsp://example.com", digital_zoom=2)
        self.assertEqual(type(config), RTSPFrameGrabberConfig)

        # this won't work because there are multiple input types that could be mapped to the serial_number field
        with self.assertRaises(ValueError):
            FrameGrabberConfig.create(serial_number="1234567890", resolution_width=640, resolution_height=480, digital_zoom=2)
