from abc import ABCMeta, abstractmethod
import logging
from threading import Thread, Lock
import time

import cv2

import pafy

logger = logging.getLogger('groundlight.stream')


class FrameGrabber(metaclass=ABCMeta):

    @staticmethod
    def create_grabber(stream=None):
        if type(stream) == int:
            return DeviceFrameGrabber(stream=stream)
        elif type(stream) == str and stream[:4] == 'rtsp':
            logger.debug(f'found rtsp stream {stream=}')
            return RTSPFrameGrabber(stream=stream)
        elif type(stream) == str and stream.find("youtube.com") > 0:
            logger.debug(f'found youtube stream {stream=}')
            return YouTubeFrameGrabber(stream=stream)
        else:
            raise ValueError(f'cannot create a frame grabber from {stream=}')


    @abstractmethod
    def grab():
        pass


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
                logger.error(f'could not read frame from {capture=}')
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
        self.lock = Lock()
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
        with self.lock:
            logger.debug(f'grabbed lock to read frame from buffer')
            self.capture = cv2.VideoCapture(self.best_video.url)
            ret, frame = self.capture.read() # grab and decode since we want this frame
            if not ret:
                logger.error(f'could not read frame from {capture=}')
            now = time.time()
            logger.info(f'read the frame in {now-start}s.')
            self.capture.release()
            return frame

