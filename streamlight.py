import io
import logging
from logging.config  import dictConfig
import os
import time

import cv2
import yaml

from groundlight import Groundlight


fname = os.path.join(os.path.dirname(__file__), 'logging.yaml')
dictConfig(yaml.safe_load(open(fname, 'r')))

logger = logging.getLogger(name='groundlight.stream')

RTSP = "rtsp://admin:password@10.77.0.29:554/cam/realmonitor?channel=1&subtype=0"
DETECTOR = "772d549499394726b06fd6e36ec41153"
ENDPOINT = "https://device.integ.positronix.ai/device-api"
TOKEN = 'api_29imQxusKndanuiigGzLqAoL3Zj_AD2VFYi191ghbUJeLHJ11GDfVCjfa55JCS'

logger.info(f"configured to read camera at {RTSP}")
logger.info(f"using detector ID {DETECTOR}")

gl = Groundlight(endpoint=ENDPOINT, api_token=TOKEN)

logger.info(f"available detectors:")
logger.info(gl.list_detectors())

cap = cv2.VideoCapture(RTSP, cv2.CAP_FFMPEG)

while True:
   start = time.time()
   cap = cv2.VideoCapture(RTSP, cv2.CAP_FFMPEG)
   if not cap.isOpened():
       logger.error(f'Cannot open stream {RTSP=}')
       # logger.debug(cv2.getBuildInformation())
       exit(-1)

   ret, frame = cap.read()

   logger.debug(f"Original {frame.shape=}")
   frame = cv2.resize(frame, (480,270))
   logger.debug(f"Resized {frame.shape=}")
   is_success, buffer = cv2.imencode(".jpg", frame)
   logger.debug(f"buffer size is {len(buffer)}")
   io_buf = io.BytesIO(buffer)
   #cv2.imwrite('temp.jpg', frame)
   end = time.time()
   logger.info(f"Time to prep image {1000*(end-start):.1f}ms")
   image_query = gl.submit_image_query(detector_id=det_id, image=io_buf)
   start = end
   end = time.time()
   logger.info(f"API time for image {1000*(start-end):.1f}ms")

cap.release()
