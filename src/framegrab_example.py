from framegrab import FrameGrabber
import cv2
import time
import random

STREAM = 0
FPS = 5

webcam_serial_numbers = FrameGrabber.find_webcam_serial_numbers()
print(webcam_serial_numbers)

for serial_number, devname in webcam_serial_numbers.items():
    pass

# grabber1 = FrameGrabber.create_grabber(stream=STREAM, fps_target=FPS)
# grabber2 = FrameGrabber.create_grabber()
# time.sleep(1)

# while True:
#     print(time.time(), 'grabbing frame...')
#     frame1 = grabber1.grab()
#     cv2.imshow('frame1', frame1)
#     cv2.waitKey(random.randint(1000,10000))
#     cv2.destroyAllWindows()