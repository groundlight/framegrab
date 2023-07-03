import fnmatch
import logging
import os
import random
import re
import time
import urllib
from abc import ABC, abstractmethod
from pathlib import Path
from threading import Lock, Thread

import cv2
import numpy as np
import pafy
import subprocess

from enum import Enum
from typing import List

logger = logging.getLogger(__name__)

class InputTypes(Enum):
    WEBCAM = 'webcam'
    RTSP = 'rtsp'
    REALSENSE = 'realsense'
    YOUTUBE = 'youtube'

class FrameGrabber(ABC):
    @staticmethod
    def create_grabbers(configs: List[dict]) -> dict:
        grabbers = {}
        for config in configs:
            grabber = FrameGrabber.create_grabber(config)
            grabbers[config['name']] = grabber
        return grabbers

    @staticmethod
    def create_grabber(config: dict):

        input_type = config.get('input_type', None)
        if input_type is None:
            raise ValueError(
                f'No input_type provided. Valid types are {[i.value for i in InputTypes]}'
                )

        if input_type == InputTypes.WEBCAM.value:
            return WebcamFrameGrabber(config)
        elif input_type == InputTypes.RTSP.value:
            return RTSPFrameGrabber(config)
        elif input_type == InputTypes.REALSENSE.value:
            return None
        elif input_type == InputTypes.YOUTUBE.value:
            return None
        else:
            raise ValueError(
                f'Unable to determine input_type. Valid types are {[i.value for i in InputTypes]}'
                )

    @staticmethod
    def autodiscover() -> dict:
        """just a stub
        autodiscovers cameras and returns a dictionary of grabbers
        """
        return {}

    def _postprocess(self, frame: np.ndarray) -> np.ndarray:
        # just return in the input if there are no postprocessing options provided.
        options = self.config.get('options', None)
        if not options:
            return frame

        # crop the frame
        top = options.get('crop_top', 0)
        left = options.get('crop_left', 0)
        bottom = options.get('crop_bottom', frame.shape[0])
        right = options.get('crop_right', frame.shape[1])
        frame = frame[top:bottom,left:right]

        return frame

    @abstractmethod
    def read() -> np.ndarray:
        pass

    @abstractmethod
    def release() -> None:
        pass

    @abstractmethod
    def _apply_options() -> None:
        pass

class WebcamFrameGrabber(FrameGrabber):
    indices_in_use = set()

    def __init__(self, config: dict):
        self.config = config
        self.idx = None

        # assign cameras based on serial number
        if self.config.get('serial_number', None):
            found_webcam_devnames = WebcamFrameGrabber._find_webcam_devnames()
            desired_serial_number = self.config.get('serial_number', None)
            for devname, serial_number in found_webcam_devnames.items():
                if serial_number == desired_serial_number:
                    self.capture = cv2.VideoCapture(devname)
                    break
            else:
                raise Exception(
                    f'Unable to find webcam with the specified serial_number: {desired_serial_number}. '
                    'Please ensure that the serial number is correct and that it is plugged in.'
                    )
        # since no serial number is provided, just assign the next available camera
        else:
            for idx in range(20):
                if idx in WebcamFrameGrabber.indices_in_use:
                    logger.debug(f'Webcam index {idx} already in use.')
                    continue

                capture = cv2.VideoCapture(idx)
                ret, _ = capture.read()
                if ret and capture.isOpened():
                    logger.info(f'Connected to webcam index {idx}.')
                    self.capture = capture
                    self.idx = idx
                    WebcamFrameGrabber.indices_in_use.add(idx)
                    break
            else:
                raise Exception(
                    'Unable to connect to webcam by index. Is your webcam plugged in?'
                )

        # set the buffer size to 1 to always get the most recent frame
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
    def read(self) -> np.ndarray:
        ret, frame = self.capture.read()
        frame = self._postprocess(frame)
        frame = self._apply_options(frame)
        return frame

    def release(self) -> None:
        self.capture.release()
        if self.idx:
            WebcamFrameGrabber.indices_in_use.remove(self.idx)

    def _apply_options(self, frame: np.ndarray) -> np.ndarray:
        """just a stub
        should apply can camera specific options to a frame and return the frame
        """
        return frame

    @staticmethod
    def _find_webcam_devnames() -> dict: 
        """Finds all plugged in webcams and returns a dictionary mapping device names to
        to serial numbers. This is useful for connecting the dots between user provided configurations
        and actual plugged in devices.

        This function only works on Linux, and was specifically tested on an Nvidia Jetson.
        """
        start = time.time()
        # ls /dev/video* returns device paths for all plugged in webcams
        command = 'ls /dev/video*'
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout, _ = process.communicate()
        output = stdout.decode('utf-8')
        devices = output.strip().split('\n')

        plugged_in_devices = {}
        for devpath in devices:
            # ls -l /sys/class/video4linux/video0/device returns a path that points back into the /sys/bus/usb/devices/
            # directory where can determine the serial number.
            # e.g. /sys/bus/usb/devices/2-3.2:1.0 -> /sys/bus/usb/devices/<bus>-<port>.<subport>:<config>.<interface>
            devname = devpath.split('/')[-1]
            command = f'ls -l /sys/class/video4linux/{devname}/device'
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            stdout, _ = process.communicate()
            output = stdout.decode('utf-8')
            bus_port_subport = output.split('/')[-1].split(':')[0]

            # find the serial number
            command = f'cat /sys/bus/usb/devices/{bus_port_subport}/serial'
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            stdout, _ = process.communicate()
            serial_number = stdout.decode('utf-8').strip()

            if serial_number:
                plugged_in_devices[devpath] = serial_number

        stop = time.time()
        time_elapsed = stop - start
        print(f'_find_webcam_devnames ran in {time_elapsed} seconds')

        return plugged_in_devices

class RTSPFrameGrabber(FrameGrabber):
    """grabs the most recent frame from an rtsp stream. The RTSP capture
    object has a non-configurable built-in buffer, so just calling
    grab would return the oldest frame in the buffer rather than the
    latest frame. This class uses a thread to continously drain the
    buffer by grabbing and discarding frames and only returning the
    latest frame when explicitly requested.
    """

    def __init__(self, config: dict):
        self.config = config

        stream = config['address']
        self.capture = cv2.VideoCapture(stream)

        logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
        if not self.capture.isOpened():
            raise ValueError(f"could not open RTSP stream: {stream}")
        
        self.run = True
        self.lock = Lock()
        Thread(target=self._drain).start()

    def read(self) -> np.ndarray:
        start = time.time()
        with self.lock:
            logger.debug(f"grabbed lock to read frame from buffer")
            ret, frame = self.capture.read()  # grab and decode since we want this frame
            if not ret:
                logger.error(f"could not read frame from {self.capture}")
            now = time.time()
            logger.debug(f"read the frame in {1000*(now-start):.1f}ms")

            frame = self._postprocess(frame)

            return frame

    def release(self) -> None:
        self.run = False # to stop the buffer drain thread

    def _apply_options() -> None:
        pass

    def _drain(self) -> None:
        logger.debug(f"starting thread to drain the video capture buffer")
        while self.run:
            with self.lock:
                ret = self.capture.grab()  # just grab and don't decode

        # release the stream on the way out
        self.capture.release()

class DirectoryFrameGrabber(FrameGrabber):
    def __init__(self, stream=None, fps_target=0):
        """stream must be an file mask"""
        try:
            self.filename_list = []
            for filename in os.listdir():
                if fnmatch.fnmatch(filename, stream):
                    self.filename_list.append(filename)
            logger.debug(f"found {len(self.filename_list)} files matching stream: {stream}")
            random.shuffle(self.filename_list)
        except Exception as e:
            logger.error(f"could not initialize DirectoryFrameGrabber: stream: {stream} filename is invalid or read error")
            raise e
        if len(self.filename_list) == 0:
            logger.warning(f"no files found matching stream: {stream}")

    def grab(self):
        if len(self.filename_list) == 0:
            raise RuntimeWarning(f"could not read frame from {self.capture}.  possible end of file.")

        start = time.time()
        frame = cv2.imread(self.filename_list[0], cv2.IMREAD_GRAYSCALE)
        self.filename_list.pop(0)
        logger.debug(f"read the frame in {1000*(time.time()-start):.1f}ms")

        return frame

class FileStreamFrameGrabber(FrameGrabber):
    def __init__(self, stream=None, fps_target=0):
        """stream must be an filename"""
        try:
            self.capture = cv2.VideoCapture(stream)
            logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
            ret, frame = self.capture.read()
            self.fps_source = round(self.capture.get(cv2.CAP_PROP_FPS), 2)
            self.fps_target = fps_target
            logger.debug(f"source FPS : {self.fps_source}  / target FPS : {self.fps_target}")
            self.remainder = 0.0
        except Exception as e:
            logger.error(f"could not initialize DeviceFrameGrabber: stream: {stream} filename is invalid or read error")
            raise e

    def read(self) -> np.ndarray:
        """decimates stream to self.fps_target, 0 fps to use full original stream.
        consistent with existing behavior based on VideoCapture.read()
        which may return None when it cannot read a frame.
        """
        start = time.time()

        if self.fps_target > 0 and self.fps_target < self.fps_source:
            drop_frames = (self.fps_source / self.fps_target) - 1 + self.remainder
            for i in range(round(drop_frames)):
                ret, frame = self.capture.read()
            self.remainder = round(drop_frames - round(drop_frames), 2)
            logger.info(
                f"dropped {round(drop_frames)} frames to meet {self.fps_target} FPS target from {self.fps_source} FPS source (off by {self.remainder} frames)"
            )
        else:
            logger.debug(f"frame dropping disabled for {self.fps_target} FPS target from {self.fps_source} FPS source")

        ret, frame = self.capture.read()
        if not ret:
            raise RuntimeWarning(f"could not read frame from {self.capture}.  possible end of file.")
        now = time.time()
        logger.debug(f"read the frame in {1000*(now-start):.1f}ms")
        return frame

class DeviceFrameGrabber(FrameGrabber):
    """Grabs frames directly from a device via a VideoCapture object that
    is kept open for the lifetime of this instance.

    importantly, this grabber does not buffer frames on behalf of the
    caller, so each call to grab will directly read a frame from the
    device
    """

    def __init__(self, stream=None):
        """stream must be an int representing a device id"""
        try:
            self.capture = cv2.VideoCapture(int(stream))
            logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
        except Exception as e:
            logger.error(
                f"could not initialize DeviceFrameGrabber: stream: {stream} must be an int corresponding to a valid device id."
            )
            raise e

    def grab(self) -> np.ndarray:
        """consistent with existing behavior based on VideoCapture.read()
        which may return None when it cannot read a frame.
        """
        start = time.time()
        ret, frame = self.capture.read()
        if not ret:
            raise RuntimeWarning("could not read frame from {self.capture}")
        now = time.time()
        logger.debug(f"read the frame in {1000*(now-start):.1f}ms")
        return frame

class YouTubeFrameGrabber(FrameGrabber):
    """grabs the most recent frame from an YouTube stream. To avoid extraneous bandwidth
    this class tears down the stream between each frame grab.  maximum framerate
    is likely around 0.5fps in most cases.
    """

    def __init__(self, stream=None):
        self.stream = stream
        self.video = pafy.new(self.stream)
        self.best_video = self.video.getbest(preftype="mp4")
        self.capture = cv2.VideoCapture(self.best_video.url)
        logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
        if not self.capture.isOpened():
            raise ValueError(f"could not initially open {self.stream}")
        self.capture.release()

    def reset_stream(self):
        self.video = pafy.new(self.stream)
        self.best_video = self.video.getbest(preftype="mp4")
        self.capture = cv2.VideoCapture(self.best_video.url)
        logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
        if not self.capture.isOpened():
            raise ValueError(f"could not initially open {self.stream}")
        self.capture.release()

    def grab(self):
        start = time.time()
        self.capture = cv2.VideoCapture(self.best_video.url)
        ret, frame = self.capture.read()  # grab and decode since we want this frame
        if not ret:
            logger.error(f"could not read frame from {self.capture}. attempting to reset stream")
            self.reset_stream()
            self.capture = cv2.VideoCapture(self.best_video.url)
            ret, frame = self.capture.read()
            if not ret:
                logger.error(f"failed to effectively reset stream {self.stream} / {self.best_video.url}")
        now = time.time()
        logger.debug(f"read the frame in {1000*(now-start):.1f}ms")
        self.capture.release()
        return frame


class ImageURLFrameGrabber(FrameGrabber):
    """grabs the current image at a single URL.
    NOTE: if image is expected to be refreshed or change with a particular frequency,
    it is up to the user of the class to call the `grab` method with that frequency
    """

    def __init__(self, url=None, **kwargs):
        self.url = url

    def grab(self):
        start = time.time()
        try:
            req = urllib.request.urlopen(self.url)
            response = req.read()
            arr = np.asarray(bytearray(response), dtype=np.uint8)
            frame = cv2.imdecode(arr, -1)  # 'Load it as it is'
        except Exception as e:
            logger.error(f"could not grab frame from {self.url}: {str(e)}")
            frame = None
        now = time.time()
        elapsed = now - start
        logger.info(f"read image from URL {self.url} into frame in {elapsed}s")

        return frame
