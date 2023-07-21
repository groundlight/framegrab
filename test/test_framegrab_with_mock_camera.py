"""A suite of tests that can be run without any physical cameras.
Intended to check basic functionality like cropping, zooming, config validation, etc.
"""

import unittest
from framegrab.grabber import FrameGrabber

class TestFrameGrabWithMockCamera(unittest.TestCase):
    def test_crop_pixels(self):
        """Grab a frame, crop a frame by pixels, and make sure the shape is correct.
        """
        config = {
            'name': 'mock_camera',
            'input_type': 'mock',
            'options': {
                'resolution': {
                    'width': 640,
                    'height': 480,
                },
                'crop': {
                    'pixels': {
                        'top': 40,
                        'bottom': 440,
                        'left': 120,
                        'right': 520,
                    }
                }
            }
        }
        grabber = FrameGrabber.create_grabber(config)

        frame = grabber.grab()

        grabber.release()

        assert frame.shape == (400, 400, 3)

    def test_crop_relative(self):
        """Grab a frame, crop a frame in an relative manner (0 to 1), and make sure the shape is correct.
        """
        config = {
            'name': 'mock_camera',
            'input_type': 'mock',
            'options': {
                'resolution': {
                    'width': 640,
                    'height': 480,
                },
                'crop': {
                    'relative': {
                        'top': .1,
                        'bottom': .9,
                        'left': .1,
                        'right': .9,
                    }
                }
            }
        }
        grabber = FrameGrabber.create_grabber(config)

        frame = grabber.grab()

        grabber.release()

        assert frame.shape == (384, 512, 3)

    def test_zoom(self):
        """Grab a frame, zoom a frame, and make sure the shape is correct.
        """
        config = {
            'name': 'mock_camera',
            'input_type': 'mock',
            'options': {
                'resolution': {
                    'width': 640,
                    'height': 480,
                },
                'zoom': {
                    'digital': 2,
                }
            }
        }
        grabber = FrameGrabber.create_grabber(config)

        frame = grabber.grab()

        grabber.release()

        assert frame.shape == (240, 320, 3)

    def test_attempt_create_grabber_with_invalid_input_type(self):
        config = {
            'input_type': 'some_invalid_camera_type'
        }

        with self.assertRaises(ValueError):
            FrameGrabber.create_grabber(config)

    def test_create_grabber_without_name(self):
        config = {
            'input_type': 'mock'
        }

        grabber = FrameGrabber.create_grabber(config)

        # Check that some camera name was added
        assert len(grabber.config['name']) > 2

        grabber.release()

    def test_create_grabber_with_name(self):
        user_provided_name = 'my_camera'

        config = {
            'name': user_provided_name,
            'input_type': 'mock',
        }

        grabber = FrameGrabber.create_grabber(config)

        assert grabber.config['name'] == user_provided_name

        grabber.release()

    def test_create_grabbers_without_names(self):
        configs = [
            {'input_type': 'mock'},
            {'input_type': 'mock'},
            {'input_type': 'mock'},
        ]

        grabbers = FrameGrabber.create_grabbers(configs)

        grabber_names = set([grabber.config['name'] for grabber in grabbers.values()])

        # Make sure all the grabbers have unique names
        assert len(configs) == len(grabber_names)

        for grabber in grabbers.values():
            grabber.release()

    def test_attempt_create_more_grabbers_than_exist(self):
        """Try to provide a config with more cameras than are actually plugged in.
        The MockFrameGrabber class implicitly only has three cameras that can be discovered.
        """

        # Connect to 3 grabbers, this should be fine
        configs = [
            {'input_type': 'mock'},
            {'input_type': 'mock'},
            {'input_type': 'mock'},
        ]

        grabbers = FrameGrabber.create_grabbers(configs)

        # Try to connect to another grabber, this should raise an exception because there are only 3 mock cameras available
        try:
            FrameGrabber.create_grabber({'input_type': 'mock'})
            self.fail()
        except ValueError:
            pass
        finally:
            # release all the grabbers
            for grabber in grabbers.values():
                grabber.release()

    def test_attempt_create_grabbers_with_duplicate_names(self):
        configs = [
            {'name': 'camera1', 'input_type': 'mock'},
            {'name': 'camera2', 'input_type': 'mock'},
            {'name': 'camera1', 'input_type': 'mock'},
        ]

        # Should raise an exception because camera1 is duplicated
        with self.assertRaises(ValueError):
            FrameGrabber.create_grabbers(configs)
