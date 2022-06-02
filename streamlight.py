'''Captures frames from a real-time video stream and sends frames as
image queries to a configured detector using the Groundlight API

usage: streamlight [options] -t TOKEN -d DETECTOR

options:
  -d, --detector=ID      detector id to which the image queries are sent
  -e, --endpoint=URL     api endpoint [default: https://device.positronix.ai/device-api]
  -f, --framerate=FPS    number of frames to capture per second.
  -h, --help             show this message.
  -s, --stream=STREAM    id, filename or URL of a video stream (e.g. rtsp://host:port/script?params) [default: 0]
  -t, --token=TOKEN      api token to authenticate with the groundlight api
  -v, --verbose
'''
import io
import logging
from logging.config  import dictConfig
import os
import time

import cv2
import docopt
import yaml

from groundlight import Groundlight


fname = os.path.join(os.path.dirname(__file__), 'logging.yaml')
dictConfig(yaml.safe_load(open(fname, 'r')))
logger = logging.getLogger(name='groundlight.stream')

INTEG = "https://device.integ.positronix.ai/device-api"


def main():
    args = docopt.docopt(__doc__)
    if args.get('--verbose'):
        logger.level = logging.DEBUG
        logger.debug(f'{args=}')

    ENDPOINT = args['--endpoint']
    if ENDPOINT == 'integ':
        ENDPOINT = INTEG
    TOKEN = args['--token']
    DETECTOR = args['--detector']
    STREAM = args['--stream']
    try:
        STREAM=int(STREAM)
    except ValueError as e:
        logger.debug(f'{STREAM=} is not an int, so it must be a filename or url.')

    logger.debug(f'creating groundlight client with {ENDPOINT=} and {TOKEN=}')
    gl = Groundlight(endpoint=ENDPOINT, api_token=TOKEN)

    logger.debug(f'initializing video capture: {STREAM=}')
    cap = cv2.VideoCapture(STREAM, cv2.CAP_FFMPEG)

    while True:
       start = time.time()
       if not cap.isOpened():
           logger.error(f'Cannot open stream {STREAM=}')
           exit(-1)

       ret, frame = cap.read()
       logger.debug(f"Original {frame.shape=}")
       frame = cv2.resize(frame, (480,270))
       logger.debug(f"Resized {frame.shape=}")

       is_success, buffer = cv2.imencode(".jpg", frame)
       logger.debug(f"buffer size is {len(buffer)}")
       io_buf = io.BytesIO(buffer)
       end = time.time()
       logger.info(f"Time to prep image {1000*(end-start):.1f}ms")
       image_query = gl.submit_image_query(detector_id=DETECTOR, image=io_buf)
       start = end
       end = time.time()
       logger.info(f"API time for image {1000*(start-end):.1f}ms")

    cap.release()


if __name__ == '__main__':
    main()
