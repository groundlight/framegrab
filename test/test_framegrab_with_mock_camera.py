"""A suite of tests that can be run without any physical cameras.
Intended to check basic functionality like cropping, zooming, config validation, etc.
"""

import os
import unittest
from unittest.mock import patch
from framegrab.grabber import FrameGrabber, RTSPFrameGrabber
from framegrab.config import FrameGrabberConfig, InputTypes

class TestFrameGrabWithMockCamera(unittest.TestCase):
    def test_crop_pixels(self):
        """Grab a frame, crop a frame by pixels, and make sure the shape is correct."""
        config = {
            "name": "mock_camera",
            "input_type": "mock",
            "options": {
                "resolution": {
                    "width": 640,
                    "height": 480,
                },
                "crop": {
                    "pixels": {
                        "top": 40,
                        "bottom": 440,
                        "left": 120,
                        "right": 520,
                    }
                },
            },
        }
        grabber = FrameGrabber.create_grabber(config)

        frame = grabber.grab()

        grabber.release()

        assert frame.shape == (400, 400, 3)

    def test_crop_relative(self):
        """Grab a frame, crop a frame in an relative manner (0 to 1), and make sure the shape is correct."""
        config = {
            "name": "mock_camera",
            "input_type": "mock",
            "options": {
                "resolution": {
                    "width": 640,
                    "height": 480,
                },
                "crop": {
                    "relative": {
                        "top": 0.1,
                        "bottom": 0.9,
                        "left": 0.1,
                        "right": 0.9,
                    }
                },
            },
        }
        grabber = FrameGrabber.create_grabber(config)

        frame = grabber.grab()

        grabber.release()

        assert frame.shape == (384, 512, 3)

    def test_zoom(self):
        """Grab a frame, zoom a frame, and make sure the shape is correct."""
        config = {
            "name": "mock_camera",
            "input_type": "mock",
            "options": {
                "resolution": {
                    "width": 640,
                    "height": 480,
                },
                "zoom": {
                    "digital": 2,
                },
            },
        }
        grabber = FrameGrabber.create_grabber(config)

        frame = grabber.grab()

        grabber.release()

        assert frame.shape == (240, 320, 3)

    def test_attempt_create_grabber_with_invalid_input_type(self):
        config = {"input_type": "some_invalid_camera_type"}

        with self.assertRaises(ValueError):
            FrameGrabber.create_grabber(config)

    def test_create_grabber_without_name(self):
        config = {"input_type": "mock"}

        grabber = FrameGrabber.create_grabber(config)

        # Check that some camera name was added
        assert len(grabber.config.name) > 2

        grabber.release()

    def test_create_grabber_with_name(self):
        user_provided_name = "my_camera"

        config = {"name": user_provided_name, "input_type": "mock"}

        grabber = FrameGrabber.create_grabber(config)

        assert grabber.config.name == user_provided_name

        grabber.release()

    def test_create_grabbers_without_names(self):
        configs = [
            {"input_type": "mock"},
            {"input_type": "mock"},
            {"input_type": "mock"},
        ]

        grabbers = FrameGrabber.create_grabbers(configs)

        grabber_names = set([grabber.config.name for grabber in grabbers.values()])

        # Make sure all the grabbers have unique names
        assert len(configs) == len(grabber_names)

        for grabber in grabbers.values():
            grabber.release()

    def test_create_grabber_with_context_manager(self):
        user_provided_name = "my_camera"

        config = {"name": user_provided_name, "input_type": "mock"}

        with FrameGrabber.create_grabber(config) as grabber:
            assert grabber.config.name == user_provided_name

    def test_attempt_create_more_grabbers_than_exist(self):
        """Try to provide a config with more cameras than are actually plugged in.
        The MockFrameGrabber class implicitly only has three cameras that can be discovered.
        """

        # Connect to 3 grabbers, this should be fine
        configs = [
            {"input_type": "mock"},
            {"input_type": "mock"},
            {"input_type": "mock"},
        ]

        grabbers = FrameGrabber.create_grabbers(configs)

        # Try to connect to another grabber, this should raise an exception because there are only 3 mock cameras available
        try:
            FrameGrabber.create_grabber({"input_type": "mock"})
            self.fail()
        except ValueError:
            pass
        finally:
            # release all the grabbers
            for grabber in grabbers.values():
                grabber.release()

    def test_attempt_create_grabbers_with_duplicate_names(self):
        configs = [
            {"name": "camera1", "input_type": "mock"},
            {"name": "camera2", "input_type": "mock"},
            {"name": "camera1", "input_type": "mock"},
        ]

        # Should raise an exception because camera1 is duplicated
        with self.assertRaises(ValueError):
            FrameGrabber.create_grabbers(configs)


    def test_substitute_rtsp_url(self):
        """Test that the RTSP password is substituted correctly."""
        os.environ["RTSP_PASSWORD_1"] = "password1"

        # rtsp_url will be subsitituted here with the pydantic validator
        config = FrameGrabberConfig.create(
            input_type=InputTypes.RTSP,
            rtsp_url="rtsp://admin:{{RTSP_PASSWORD_1}}@10.0.0.1",
        )
        
        assert config.rtsp_url == "rtsp://admin:password1@10.0.0.1"

    def test_substitute_rtsp_url_password_not_set(self):
        """Test that an exception is raised if the user adds a placeholder but neglects to set the environment variable."""
        with self.assertRaises(ValueError):
            FrameGrabberConfig.create(
                input_type=InputTypes.RTSP,
                rtsp_url="rtsp://admin:{{SOME_NONEXISTENT_ENV_VARIABLE}}@10.0.0.1",
            )

    def test_substitute_rtsp_url_without_placeholder(self):
        """Users should be able to use RTSP urls without a password placeholder. In this case, the config should be returned unchanged."""
        rtsp_url = "rtsp://admin:password@10.0.0.1"
        config = FrameGrabberConfig.create(
            input_type=InputTypes.RTSP,
            rtsp_url=rtsp_url,
        )

        assert config.rtsp_url == rtsp_url

    def test_create_grabbers_with_one_invalid_config(self):
        """
        Defines a list of camera configurations, where one of the configurations is invalid.
        
        `create_grabbers` should return all valid grabbers that it was able to create.
        """
        configs = [
            {
                "id": {
                    "serial_number": "123" # valid 'mock' camera
                },
                "name": "Mock Camera 1",
                "input_type": "mock",
            },
            {
                "id": {
                    "rtsp_url": "rtsp://INVALID:INVALID@0.0.0.0:0" # invalid rtsp url
                    },
                "name": "Invalid RTSP Camera",
                "input_type": "rtsp",
            },
                        {
                "id": {
                    "serial_number": "456" # valid 'mock' camera
                },
                "name": "Mock Camera 2",
                "input_type": "mock",
            },
        ]
        
        grabbers = FrameGrabber.create_grabbers(configs)
        for grabber in grabbers.values():
            grabber.release()
        
        assert len(grabbers) == 2
        
        
        
           