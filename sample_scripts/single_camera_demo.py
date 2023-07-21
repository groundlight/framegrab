"""Finds a single USB camera (or built-in webcam) and displays its feed in a window. 
Press 'q' to quit.
"""

import cv2
from framegrab import FrameGrabber

config = {
    'name': 'My Camera',
    'input_type': 'generic_usb',
}

grabber = FrameGrabber.create_grabber(config)

while True:
    frame = grabber.grab()

    cv2.imshow('FrameGrab Single-Camera Demo', frame)

    key = cv2.waitKey(30)
    if key == ord('q'):
        break

cv2.destroyAllWindows()

grabber.release()