#!/usr/bin/env python3
"""Accesses a live YouTuBe stream and displays its feed in a window.
Press 'q' to quit.
"""

import cv2
from framegrab import FrameGrabber

config = {
    'name': 'Shibuya Crossing',
    'input_type': 'youtube',
    'id': {
        'youtube_url': "https://www.youtube.com/watch?v=3kPH7kTphnE"
    },
}

grabber = FrameGrabber.create_grabber(config)

while True:
    frame = grabber.grab()

    cv2.imshow('FrameGrab YouTube Stream Demo', frame)

    key = cv2.waitKey(30)
    if key == ord('q'):
        break

cv2.destroyAllWindows()

grabber.release()
