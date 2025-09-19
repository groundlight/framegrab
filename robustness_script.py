import framegrab
from framegrab import FrameGrabber
from framegrab.config import RTSPFrameGrabberConfig
from framegrab.exceptions import GrabError

import cv2
import time
import os

print(framegrab.__version__)

rtsp_url = os.environ.get('RTSP_URL')
if rtsp_url is None:
    raise ValueError('Please set an RTSP URL in your environment variables')

config = RTSPFrameGrabberConfig(rtsp_url=rtsp_url)
grabber = FrameGrabber.create_grabber(config)

while True:
    try:
        frame = grabber.grab()
    except GrabError:
        cv2.destroyAllWindows()
        frame = None
        print(f'Failed to grab frame!')

    # # STRATEGY 1: Just wait, see if we can reconnect automatically (this doesn't seem to work ever)
    # if frame is None:
    #     print('waiting...')
    #     time.sleep(1)
    #     continue

    # STRATEGY 2: Try to reconnect explicitly (this works well)
    if frame is None:
        print('Reconnecting...')
        grabber.release()
        try:
            grabber = FrameGrabber.create_grabber(config)
        except ValueError:
            print('Failed to reconnect.')
            time.sleep(1)
        finally:
            continue

    cv2.imshow('Feed', frame)
    now = time.time()
    print(f'{now}: {frame.shape}')

    key = cv2.waitKey(30)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
grabber.release()