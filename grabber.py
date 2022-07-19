from abc import ABCMeta, abstractmethod
import logging
import re
from threading import Thread, Lock
import time
from pathlib import Path
import urllib

import cv2
import pafy
import numpy as np

logger = logging.getLogger('groundlight.stream')


class FrameGrabber(metaclass=ABCMeta):

    @staticmethod
    def create_grabber(stream=None, **kwargs):
        logger.debug(f'Input {stream=}')
        if type(stream) == int:
            return DeviceFrameGrabber(stream=stream)
        elif type(stream) == str and stream[:4] == 'rtsp':
            logger.debug(f'found rtsp stream {stream=}')
            return RTSPFrameGrabber(stream=stream)
        elif type(stream) == str and stream.find("youtube.com") > 0:
            logger.debug(f'found youtube stream {stream=}')
            return YouTubeFrameGrabber(stream=stream)
        elif type(stream) == str and Path(stream).is_file():
            logger.debug(f'found filename stream {stream=}')
            return FileStreamFrameGrabber(stream=stream, **kwargs)
        elif type(stream) == str and stream[:4] == 'http':
            logger.debug(f'found image url {stream=}')
            return SingleRefreshedImageUrlGrabber(url=stream, **kwargs)
        else:
            raise ValueError(f'cannot create a frame grabber from {stream=}')


    @abstractmethod
    def grab():
        pass

class FileStreamFrameGrabber(FrameGrabber):

    def __init__(self, stream=None, fps_target = 0):
        '''stream must be an filename'''
        try:
            self.capture = cv2.VideoCapture(stream)
            logger.debug(f'initialized video capture with backend={self.capture.getBackendName()}')
            ret, frame = self.capture.read()
            self.fps_source = round(self.capture.get(cv2.CAP_PROP_FPS), 2)
            self.fps_target = fps_target
            logger.debug(f'source FPS : {self.fps_source=}  / target FPS : {self.fps_target}')
            self.remainder = 0.0
        except Exception as e:
            logger.error(f'could not initialize DeviceFrameGrabber: {stream=} filename is invalid or read error')
            raise e

    def grab(self):
        '''decimates stream to self.fps_target, 0 fps to use full original stream.
        consistent with existing behavior based on VideoCapture.read()
        which may return None when it cannot read a frame.
        '''
        start = time.time()

        if self.fps_target > 0 and self.fps_target < self.fps_source :
            drop_frames = (self.fps_source / self.fps_target) - 1 + self.remainder
            for i in range(round(drop_frames)):
                ret, frame = self.capture.read()
            self.remainder = round(drop_frames - round(drop_frames), 2)
            logger.debug(f'dropped {round(drop_frames)} frames to meet {self.fps_target} FPS target from {self.fps_source} FPS source (off by {self.remainder} frames)')
        else:
            logger.debug(f'frame dropping disabled for {self.fps_target} FPS target from {self.fps_source} FPS source')

        ret, frame = self.capture.read()
        if not ret:
            raise RuntimeWarning('could not read frame from {self.capture=}.  possible end of file.')
        now = time.time()
        logger.info(f'read the frame in {now-start}s.')
        return frame

class DeviceFrameGrabber(FrameGrabber):
    '''Grabs frames directly from a device via a VideoCapture object that
    is kept open for the lifetime of this instance.

    importantly, this grabber does not buffer frames on behalf of the
    caller, so each call to grab will directly read a frame from the
    device
    '''


    def __init__(self, stream=None):
        '''stream must be an int representing a device id'''
        try:
            self.capture = cv2.VideoCapture(int(stream))
            logger.debug(f'initialized video capture with backend={self.capture.getBackendName()}')
        except Exception as e:
            logger.error(f'could not initialize DeviceFrameGrabber: {stream=} must be an int corresponding to a valid device id.')
            raise e


    def grab(self):
        '''consistent with existing behavior based on VideoCapture.read()
        which may return None when it cannot read a frame.
        '''
        start = time.time()
        ret, frame = self.capture.read()
        if not ret:
            raise RuntimeWarning('could not read frame from {self.capture=}')
        now = time.time()
        logger.info(f'read the frame in {now-start}s.')
        return frame

class RTSPFrameGrabber(FrameGrabber):
    '''grabs the most recent frame from an rtsp stream. The RTSP capture
    object has a non-configurable built-in buffer, so just calling
    grab would return the oldest frame in the buffer rather than the
    latest frame. This class uses a thread to continously drain the
    buffer by grabbing and discarding frames and only returning the
    latest frame when explicitly requested.
    '''

    def __init__(self, stream=None):
        self.lock = Lock()
        self.stream = stream
        self.capture = cv2.VideoCapture(self.stream)
        logger.debug(f'initialized video capture with backend={self.capture.getBackendName()}')
        if not self.capture.isOpened():
            raise ValueError(f'could not open {self.stream=}')
        self.thread = Thread(target=self._drain, name='drain_thread')
        self.thread.start()


    def grab(self):
        start = time.time()
        with self.lock:
            logger.debug(f'grabbed lock to read frame from buffer')
            ret, frame = self.capture.read() # grab and decode since we want this frame
            if not ret:
                logger.error(f'could not read frame from {self.capture=}')
            now = time.time()
            logger.info(f'read the frame in {now-start}s.')
            return frame


    def _drain(self):
        logger.debug(f'starting thread to drain the video capture buffer')
        while True:
            with self.lock:
                ret = self.capture.grab() # just grab and don't decode


class YouTubeFrameGrabber(FrameGrabber):
    '''grabs the most recent frame from an YouTube stream. To avoid extraneous bandwidth
    this class tears down the stream between each frame grab.  maximum framerate
    is likely around 0.5fps in most cases.
    '''

    def __init__(self, stream=None):
        self.stream = stream
        self.video = pafy.new(self.stream)
        self.best_video = self.video.getbest(preftype="mp4")
        self.capture = cv2.VideoCapture(self.best_video.url)
        logger.debug(f'initialized video capture with backend={self.capture.getBackendName()}')
        if not self.capture.isOpened():
            raise ValueError(f'could not initially open {self.stream=}')
        self.capture.release()


    def grab(self):
        start = time.time()
        self.capture = cv2.VideoCapture(self.best_video.url)
        ret, frame = self.capture.read() # grab and decode since we want this frame
        if not ret:
            logger.error(f'could not read frame from {self.capture=}')
        now = time.time()
        logger.info(f'read the frame in {now-start}s.')
        self.capture.release()
        return frame


class SingleRefreshedImageUrlGrabber(FrameGrabber):
    ''' grabs the current image at a single URL. 
    max frame rate should be set by the user as a function of the URL refresh, 
    default = 0.01 (1 frame every 100 seconds)
    '''

    def __init__(self, url=None, fps_target=0.01, **kwargs):
        self.url = url
        self.fps_target = fps_target
        self.wait_time = 1.0/self.fps_target
        logger.info(f'{self.wait_time=}')
        self.last_grab_time = time.time()

    def grab(self):
        start = time.time()
        elapsed = start - self.last_grab_time
        if elapsed < self.wait_time:
            wait_time = self.wait_time - elapsed
            logger.info(f'sleeping for {wait_time}s targeting {self.fps_target}fps')
            time.sleep(wait_time)
        try:
            req = urllib.request.urlopen(self.url)
            response = req.read()
            arr = np.asarray(bytearray(response), dtype=np.uint8)
            frame = cv2.imdecode(arr, -1) # 'Load it as it is'
        except Exception as e:
            logger.error(f'could not grab frame from {self.url}: {str(e)}')
            frame = None
        now = time.time()
        elapsed = now - start
        logger.info(f'read the url image into frame in {elapsed}s')
        self.last_grab_time = now

        return frame
