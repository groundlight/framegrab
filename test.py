import os
import framegrab
from framegrab.config import RTSPFrameGrabberConfig, MockFrameGrabberConfig, GenericUSBFrameGrabberConfig
import cv2
import time

print(framegrab.__version__)

# config = RTSPFrameGrabberConfig(
#     rtsp_url=os.environ.get('RTSP_URL'),
#     keep_connection_open=True,
#     num_90_deg_rotations=1,
# )
config = GenericUSBFrameGrabberConfig(
    resolution_height=720,
    resolution_width=1280,
)

grabbers = framegrab.FrameGrabber.create_grabbers([config])

for grabber_name, grabber in grabbers.items():
    print('-' * 10)
    print(grabber_name)
    print(grabber.config["name"])


# while True:
#     frame = grabber.grab()
#     cv2.imshow('frame', frame)
#     print(frame.shape, time.time())
#     key = cv2.waitKey(33)
#     if key == ord('q'):
#         break
#     if key == ord('k'):
#         # grabber.config.resolution_height = 480
#         # grabber.config.resolution_width = 640
#         # print(grabber.config.resolution_height)
        
#         options = {'resolution': {'width': 640, 'height': 480}}
#         grabber.apply_options(options)
    
# grabber.release()
# cv2.destroyAllWindows()
# print('done')

# options = grabber.get_options()
# assert options == {'zoom': {'digital': 2.0}}
# options["zoom"] = {'digital': 2.1}
# grabber.apply_options(options)
# options = grabber.get_options()
# assert options == {'zoom': {'digital': 2.1}}

# frame = grabber.grab()

# cv2.imshow('frame', frame)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


