'''Captures frames from a video device, file or stream and sends frames as
image queries to a configured detector using the Groundlight API

usage: streamlight [options] -t TOKEN -d DETECTOR

options:
  -d, --detector=ID      detector id to which the image queries are sent
  -e, --endpoint=URL     api endpoint [default: https://api.groundlight.ai/device-api]
  -f, --fps=FPS          number of frames to capture per second. 0 to use maximum rate possible. [default: 5]
  -h, --help             show this message.
  -s, --stream=STREAM    id, filename or URL of a video stream (e.g. rtsp://host:port/script?params) [default: 0]
  -t, --token=TOKEN      api token to authenticate with the groundlight api
  -v, --verbose          enable debug logs
  -w, --width=WIDTH      resize images to w pixels wide (and scale height proportionately if not set explicitly)
  -y, --height=HEIGHT    resize images to y pixels high (and scale width proportionately if not set explicitly)
  -m, --motion                 enable motion detection with pixel change threshold percentage (disabled by default)
  -r, --threshold=THRESHOLD    set detection threshold for motion detection [default: 1]
  -p, --postmotion=POSTMOTION  minimum number of seconds to capture for every motion detection [default: 1]
  -i, --maxinterval=MAXINT     maximum number of seconds before sending frames even without motion [default: 1000]
'''
import io
import logging
from logging.config  import dictConfig
import math
from operator import truediv
import os
from queue import Queue
import time
from threading import Thread
from xmlrpc.client import Boolean

import cv2
import docopt
import yaml

from grabber import FrameGrabber
from motion import MotionDetector
from groundlight import Groundlight


fname = os.path.join(os.path.dirname(__file__), 'logging.yaml')
dictConfig(yaml.safe_load(open(fname, 'r')))
logger = logging.getLogger(name='groundlight.stream')


def frame_processor(q:Queue, client:Groundlight, detector:str):
    logger.debug(f'frame_processor({q=}, {client=}, {detector=})')
    while True:
       frame = q.get() # locks
       # prepare image
       start = time.time()
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

def resize_if_needed(frame, width:int, height:int):
   #scales cv2 image frame to widthxheight pixels
   #values of 0 for width or height will keep proportional.

   if ((width==0) & (height==0)):
      return
   
   image_height, image_width, _ =frame.shape
   if width > 0 :
      target_width = width
   else:
      target_width = int(image_width * (height/image_height))
   if height > 0:
      target_height = height
   else:
      target_height = int(image_height * (width/image_width))

   logger.debug(f"resizing from {frame.shape=} to {target_width=}x{target_height=}")
   frame = cv2.resize(frame, (target_width,target_height))


def main():
    args = docopt.docopt(__doc__)
    if args.get('--verbose'):
        logger.level = logging.DEBUG
        logger.debug(f'{args=}')

    resize_width = 0
    if args.get('--width'):
      try:
         resize_width = int(args['--width'])
      except ValueError as e:
         logger.warning(f'invalid width parameter: {args["--width"]} ignoring --width argument.')
   
    resize_height = 0
    if args.get('--height'):
      try:
         resize_height = int(args['--height'])
      except ValueError as e:
         logger.warning(f'invalid height parameter: {args["--height"]} ignoring --height argument.')

    ENDPOINT = args['--endpoint']
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

    if args.get('--motion'):
      motion_detect = True
      MOTION_THRESHOLD = args['--threshold']
      POST_MOTION = args['--postmotion']
      MAX_INTERVAL = args['--maxinterval']
      try:
         motion_threshold = int(MOTION_THRESHOLD)
      except ValueError as e:
         logger.error(f'Invalid arguement {MOTION_THRESHOLD=} must be an integer')
         exit(-1)
      try:
         post_motion_time = float(POST_MOTION)
      except ValueError as e:
         logger.error(f'Invalid arguement {POST_MOTION=} must be a number')
         exit(-1)
      try:
         max_frame_interval = float(MAX_INTERVAL)
      except ValueError as e:
         logger.error(f'Invalid arguement {MAX_INTERVAL=} must be a number')
         exit(-1)
      logger.info(f'Motion detection enabled with {MOTION_THRESHOLD=} and post-motion capture of {POST_MOTION=} and max interval of {MAX_INTERVAL=}')
    else:
      motion_detect = False
      logger.info(f'Motion detection disabled.')

    logger.debug(f'creating groundlight client with {ENDPOINT=} and {TOKEN=}')
    gl = Groundlight(endpoint=ENDPOINT, api_token=TOKEN)
    grabber = FrameGrabber.create_grabber(stream=STREAM, fps_target=FPS)
    q = Queue()
    m = MotionDetector(pct_threshold = motion_threshold)
    workers = []
    if FPS == 0:
       worker_thread_count = 10
    else:
       worker_thread_count = math.ceil(FPS)
    for i in range(worker_thread_count):
       thread = Thread(target=frame_processor, kwargs=dict(q=q, client=gl, detector=DETECTOR))
       workers.append(thread)
       thread.start()

    try:
       desired_delay = 1/FPS
    except ZeroDivisionError:
       desired_delay = 1
       logger.debug(f'FPS set to 0.  Using maximum stream rate')
    start = time.time()

    last_frame_time = time.time()
    try:
      while True:
         frame = grabber.grab()
         if frame is None:
            logger.warning(f'No frame captured! {frame=}')
            continue

         now = time.time()
         logger.info(f'captured a new frame after {now-start:.3}. of size {frame.shape=} ')
         start = now

         if motion_detect:
            if m.motion_detected(frame):
               motion_start = time.time()
               add_frame_to_queue = True
            elif time.time() - motion_start < post_motion_time:
               logger.debug(f'adding post motion frame after {(time.time() - motion_start):.3} with {post_motion_time=}')
               add_frame_to_queue = True
            elif time.time() - last_frame_time > max_frame_interval:
               logger.debug(f'adding frame after {(time.time()-last_frame_time):.3}s for {max_frame_interval=}s')
               add_frame_to_queue = True
            else:
               logger.debug(f'skipping frame per motion detection settings')
               add_frame_to_queue = False
         else:
            add_frame_to_queue = True
         
         if add_frame_to_queue:
             resize_if_needed(frame, resize_width, resize_height)
             q.put(frame)
             last_frame_time = time.time()
             logger.debug('added frame to queue!')

         now = time.time()
         if desired_delay > 0:
            actual_delay = desired_delay - (now-start)
            logger.debug(f'waiting for {actual_delay=} to capture the next frame.')
            if actual_delay < 0:
               logger.warning(f'Falling behind the desired {FPS=}! looks like putting frames into the worker queue is taking too long: {now-start}s. The queue contains {q.qsize()} frames.')
               actual_delay = 0
            time.sleep(actual_delay)

    except KeyboardInterrupt:
      logger.info("exiting with KeyboardInterrupt.  you will may have to hit ctrl-c several times to kill the worker threads")
  
      for thread in workers:
         thread.join(timeout=1)
      quit()


if __name__ == '__main__':
    main()
