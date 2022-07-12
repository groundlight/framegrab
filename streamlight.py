'''Captures frames from a real-time video stream and sends frames as
image queries to a configured detector using the Groundlight API

usage: streamlight [options] -t TOKEN -d DETECTOR

options:
  -d, --detector=ID      detector id to which the image queries are sent
  -e, --endpoint=URL     api endpoint [default: https://device.positronix.ai/device-api]
  -f, --fps=FPS          number of frames to capture per second. 0 to use maximum rate possible. [default: 5]
  -h, --help             show this message.
  -s, --stream=STREAM    id, filename or URL of a video stream (e.g. rtsp://host:port/script?params) [default: 0]
  -t, --token=TOKEN      api token to authenticate with the groundlight api
  -v, --verbose
  --noresize             upload images in full original resolution instead of 480x272
'''
import io
import logging
from logging.config  import dictConfig
import math
import os
from queue import Queue
import time
from threading import Thread
from xmlrpc.client import Boolean

import cv2
import docopt
import yaml

from grabber import FrameGrabber
from groundlight import Groundlight


fname = os.path.join(os.path.dirname(__file__), 'logging.yaml')
dictConfig(yaml.safe_load(open(fname, 'r')))
logger = logging.getLogger(name='groundlight.stream')\

INTEG = "https://device.integ.positronix.ai/device-api"


def frame_processor(q:Queue, client:Groundlight, detector:str, resize:bool):
    logger.debug(f'frame_processor({q=}, {client=}, {detector=})')
    while True:
       frame = q.get() # locks
       # prepare image
       start = time.time()
       logger.debug(f"Original {frame.shape=}")
       if resize:
         frame = cv2.resize(frame, (480,270))
       logger.debug(f"Resized {frame.shape=}")
       is_success, buffer = cv2.imencode(".jpg", frame)
       io_buf = io.BytesIO(buffer)
       end = time.time()
       logger.info(f"Prepared the image in {1000*(end-start):.1f}ms")
       # send image query
       image_query = client.submit_image_query(detector_id=detector, image=io_buf)
       logger.debug(f'{image_query=}')
       start = end
       end = time.time()
       logger.info(f"API time for image {1000*(end-start):.1f}ms")


def main():
    args = docopt.docopt(__doc__)
    if args.get('--verbose'):
        logger.level = logging.DEBUG
        logger.debug(f'{args=}')
    
    if args.get('--noresize'):
        resize_images = False
    else:
        resize_images = True 

    ENDPOINT = args['--endpoint']
    if ENDPOINT == 'integ':
        ENDPOINT = INTEG
    TOKEN = args['--token']
    DETECTOR = args['--detector']
    STREAM = args['--stream']
    try:
        STREAM = int(STREAM)
    except ValueError as e:
        logger.debug(f'{STREAM=} is not an int, so it must be a filename or url.')
    FPS = args['--fps']
    try:
       FPS = float(FPS)
       logger.debug(f'{FPS=}')
    except ValueError as e:
       logger.error(f'Invalid argument {FPS=}. Must be a number.')
       exit(-1)

    logger.debug(f'creating groundlight client with {ENDPOINT=} and {TOKEN=}')
    gl = Groundlight(endpoint=ENDPOINT, api_token=TOKEN)
    grabber = FrameGrabber.create_grabber(stream=STREAM, fps_target=FPS)
    q = Queue()
    workers = []
    '''create worker threads one per requested FPS.  
    use max of 10 if FPS is zero (max rate). there may be a better number to use'''
    if FPS == 0:
       worker_thread_count = 10
    else:
       worker_thread_count = math.ceil(FPS)
    for i in range(worker_thread_count):
       thread = Thread(target=frame_processor, kwargs=dict(q=q, client=gl, detector=DETECTOR, resize=resize_images))
       workers.append(thread)
       thread.start()

    try:
       desired_delay = 1/FPS
    except ZeroDivisionError:
       desired_delay = 1
       logger.debug(f'FPS set to 0.  Using maximum stream rate')
    start = time.time()
    
    try:
      while True:
         frame = grabber.grab()
         now = time.time()
         logger.info(f'captured a new frame after {now-start}.')
         start = now
         if frame is None:
            logger.warning(f'continuing because {frame=}')
            continue
         q.put(frame)
         now = time.time()
         if desired_delay > 0:
            actual_delay = desired_delay - (now-start)
            logger.debug(f'waiting for {actual_delay=} to capture the next frame.')
            if actual_delay < 0:
               logger.warning(f'Falling behind the desired {FPS=}! looks like putting frames into the worker queue is taking too long: {now-start}s. The queue contains {len(q)} frames.')
               actual_delay = 0
            time.sleep(actual_delay)

    except KeyboardInterrupt:
      logger.info("exiting with KeyboardInterrupt.  you will may have to hit ctrl-c several times to kill the worker threads")
      for thread in workers:
         thread.join(timeout=1)
      quit()


if __name__ == '__main__':
    main()
