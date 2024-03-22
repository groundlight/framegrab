#!/usr/bin/env python3
"""Finds a single USB camera (or built-in webcam), grabs an image and displays the image in a window.
"""

from framegrab import FrameGrabber

config = {
    'name': 'My Camera',
    'input_type': 'generic_usb',
}

grabber = FrameGrabber.create_grabber(config)

frame = grabber.grabimg()

frame.show()

grabber.release()
