import fnmatch
import logging
import os
import platform
import random
import re
import time
import urllib
from abc import ABC, abstractmethod
from pathlib import Path
from threading import Lock, Thread
import subprocess
from typing import List

import cv2
import numpy as np
# import pafy

# Optional imports
try:
    from pypylon import pylon
except ImportError:
    pylon = None

logger = logging.getLogger(__name__)

OPERATING_SYSTEM = platform.system()
DIGITAL_ZOOM_MAX = 4

class InputTypes:
    """Defines the available input types from FrameGrabber objects
    """
    WEBCAM = 'webcam'
    RTSP = 'rtsp'
    REALSENSE = 'realsense'
    YOUTUBE = 'youtube'
    BASLER_USB = 'basler_usb'

    def get_options() -> list:
        """Get a list of the available input type options
        """
        output = []
        for attr in vars(InputTypes):
            if '_' not in attr:
                output.append(getattr(InputTypes, attr))
        return output

class FrameGrabber(ABC):
    # used for naming FrameGrabber objects that have no user-defined name
    unnamed_grabber_count = 0

    @staticmethod
    def validate_config(config: dict) -> dict:
        """Check the config to ensure it conforms to the required format and data types
        Returns a corrected version of the config.
        """
        output_config = config.copy()
        if output_config.get('serial_number', False):
            output_config['serial_number'] = str(output_config['serial_number'])
        return output_config
        
    @staticmethod
    def create_grabbers(configs: List[dict]) -> dict:
        
        # Sort the configs such that configs with serial_number attributes appear first
        # This will ensure that they are able to connect to the camera with the specified 
        # serial number, and that no other FrameGrabbers claim that camera first.
        configs.sort(key=lambda config: 'serial_number' not in config)

        grabbers = {}
        for config in configs:
            grabber = FrameGrabber.create_grabber(config)
            grabbers[config['name']] = grabber
        return grabbers

    @staticmethod
    def create_grabber(config: dict):
        # ensure the config is properly constructed
        config = FrameGrabber.validate_config(config)

        input_type = config.get('input_type', None)
        if input_type is None:
            raise ValueError(
                f'No input_type provided. Valid types are {InputTypes.get_options()}'
                )

        # If a name wasn't supplied, create one
        if not config.get('name', False):
            FrameGrabber.unnamed_grabber_count += 1
            count = FrameGrabber.unnamed_grabber_count
            config['name'] = f'Unnamed Camera {count} ({input_type})'

        # Based on input_type, create correct type of FrameGrabber
        if input_type == InputTypes.WEBCAM:
            return WebcamFrameGrabber(config)
        elif input_type == InputTypes.RTSP:
            return RTSPFrameGrabber(config)
        elif input_type == InputTypes.BASLER_USB:
            return BaslerUSBFrameGrabber(config)
        elif input_type == InputTypes.REALSENSE:
            raise NotImplementedError
        elif input_type == InputTypes.YOUTUBE:
            raise NotImplementedError
        else:
            raise ValueError(
                f'The provided input_type ({input_type}) is not valid. Valid types are {InputTypes.get_options()}'
                )

    @staticmethod
    def autodiscover() -> dict:
        """just a stub
        autodiscovers cameras and returns a dictionary of grabbers
        """
        return {}

    def grab(self) -> np.ndarray:
        """
        """
        frame = self._read()

        # crop
        options = self.config.get('options')
        if options:
            frame = self._crop(frame)

        # zoom
        if options and options.get('digital_zoom'):
            frame = self._digital_zoom(frame)

        return frame

    def _crop(self, frame: np.ndarray) -> np.ndarray:
        options = self.config['options']
        top = options.get('crop_top', 0)
        left = options.get('crop_left', 0)
        bottom = options.get('crop_bottom', frame.shape[0])
        right = options.get('crop_right', frame.shape[1])
        frame = frame[top:bottom,left:right]

        return frame
    
    def _digital_zoom(self, frame: np.ndarray) -> np.ndarray:
        digital_zoom = self.config['options']['digital_zoom']
        if digital_zoom < 1 or digital_zoom > DIGITAL_ZOOM_MAX:
            raise ValueError(
                f'Invalid value for digital_zoom ({digital_zoom}). '
                f'Digital zoom cannot be greater than {DIGITAL_ZOOM_MAX}.'
            )
        else:
            top = (frame.shape[0] - frame.shape[0] / digital_zoom) / 2 
            bottom = frame.shape[0] - top
            left = (frame.shape[1] - frame.shape[1] / digital_zoom) / 2 
            right = frame.shape[1] - left
            frame = frame[int(top):int(bottom),int(left):int(right)]

        return frame

    def _set_cv2_resolution(self) -> None:
        """Set the resolution of the cv2.VideoCapture object based on the FrameGrabber's config
        If the FrameGrabber lacks either of these properties (height and width), this method
        will do nothing.
        """
        options = self.config.get('options', False)
        if options:
            width = options.get('width', False)
            if width:
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            height = options.get('height', False)
            if width:
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    @abstractmethod
    def _read() -> np.ndarray:
        pass

    @abstractmethod
    def apply_options(config: dict) -> None:
        pass

    @abstractmethod
    def release() -> None:
        pass

    @staticmethod
    def _find_webcam_devnames() -> dict: 
        """Finds all plugged in webcams and returns a dictionary mapping device names to
        to serial numbers. This is useful for connecting the dots between user provided configurations
        and actual plugged in devices.

        This function only works on Linux, and was specifically tested on an Nvidia Jetson.
        """

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

        return plugged_in_devices

class WebcamFrameGrabber(FrameGrabber):
    indices_in_use = set()

    def __init__(self, config: dict):
        self.config = config

        name = self.config['name']
        serial_number = self.config.get('serial_number', False)

        if serial_number and OPERATING_SYSTEM != 'Linux':
            logger.warning(
                f'Matching webcams with serial_number is not supported on your operating system, {OPERATING_SYSTEM}. '
                'Webcams will be sequentially assigned instead.'
            )

        # assign camera based on serial number if serial was provided
        if serial_number and OPERATING_SYSTEM == 'Linux':
            found_webcam_devnames = WebcamFrameGrabber._find_webcam_devnames()
            for devname, curr_serial_number in found_webcam_devnames.items():
                if curr_serial_number == serial_number:

                    # Extract the index from the device name, e.g. /dev/video0 -> 0
                    # This might only work on Linux, and should be re-evaluated if we add serial number
                    # recognition for other operating systems 
                    idx = int(re.findall(r'\d+', devname)[-1])

                    if idx in WebcamFrameGrabber.indices_in_use:
                        raise ValueError(
                            f'Webcam index {idx} already in use. '
                            f'Did you use the same serial number ({serial_number}) for two different cameras?'
                            )

                    capture = cv2.VideoCapture(idx)
                    if capture.isOpened():
                        break
                        
            else:
                raise Exception(
                    f'Unable to find webcam with the specified serial_number: {serial_number}. '
                    'Please ensure that the serial number is correct and that the webcam is plugged in.'
                    )
        # If no serial number was provided, just assign the next available camera by index
        else:
            for idx in range(20):
                if idx in WebcamFrameGrabber.indices_in_use:
                    logger.debug(f'Webcam index {idx} already in use.')
                    continue

                capture = cv2.VideoCapture(idx)
                if capture.isOpened():
                    break
            else:
                raise Exception(
                    'Unable to connect to webcam by index. Is your webcam plugged in?'
                )

        # A valid capture has been found, saving it for later
        self.capture = capture
        self.apply_options(config)

        # Log the current webcam index as 'in use' to prevent other WebcamFrameGrabbers from stepping on it
        self.idx = idx
        WebcamFrameGrabber.indices_in_use.add(idx)

        logger.info(
            f'Webcam (name={name}, serial_number={serial_number}) connected with index {idx}.'
            )

        # set the buffer size to 1 to always get the most recent frame
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._set_cv2_resolution()
        
    def _read(self) -> np.ndarray:
        ret, frame = self.capture.read()
        return frame

    def release(self) -> None:
        self.capture.release()
        WebcamFrameGrabber.indices_in_use.remove(self.idx)

    def apply_options(self, config: dict) -> None:
        config = FrameGrabber.validate_config(config)
        self.config = config
        self._set_cv2_resolution()

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

        stream = self.config.get('address')
        if not stream:
            name = self.config['name']
            raise ValueError(
                f'No RTSP URL provided for {name}. '
                'Please add an address attribute to the configuration.'
            )
        self.capture = cv2.VideoCapture(stream)

        if not self.capture.isOpened():
            raise ValueError(
                f"Could not open RTSP stream: {stream}. "
                "Is RSTP URL correct? Is the camera connected to the network?"
                )

        logger.debug(f"Initialized video capture with backend={self.capture.getBackendName()}")
        
        self.run = True
        self.lock = Lock()
        Thread(target=self._drain).start()

    def _read(self) -> np.ndarray:
        start = time.time()
        with self.lock:
            logger.debug(f"Grabbed lock to read frame from buffer")
            ret, frame = self.capture.read()  # grab and decode since we want this frame
            if not ret:
                logger.error(f"Could not read frame from {self.capture}")
            now = time.time()
            logger.debug(f"Read the frame in {1000*(now-start):.1f}ms")

            return frame

    def release(self) -> None:
        self.run = False # to stop the buffer drain thread

    def apply_options(self) -> None:
        pass

    def _drain(self) -> None:
        while self.run:
            with self.lock:
                ret = self.capture.grab()  # just grab and don't decode

        # release the stream on the way out
        self.capture.release()

class BaslerUSBFrameGrabber(FrameGrabber):
    """Basler USB Camera
    """

    def __init__(self, config: dict):
        if pylon is None:
            raise ImportError(
                'Using Basler USB cameras requires the Pypylon packages, which is not installed on this system. '
                'Please install Pypylon and try again.'
            )

        self.config = config

        tlf = pylon.TlFactory.GetInstance()
        devices = tlf.EnumerateDevices()

        if not devices:
            raise ValueError(
                'No Basler USB cameras were found. Is your camera plugged in?'
            )

        # Attempt to match the provided serial number with a plugged in device. If no serial number was provided, just
        # pick the first found device that is not currently in use. 
        serial_number = config.get('serial_number', None)
        for device in devices:
            curr_serial_number = device.GetSerialNumber()
            if serial_number is None or serial_number == curr_serial_number:
                try:
                    self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(device))
                    self.camera.Open()
                    logger.info(f'Connected to Basler USB camera with serial number {curr_serial_number}.')
                    break
                except pylon.RuntimeException as e:
                    logger.debug(f'Unable to connect to Basler USB camera with serial number {curr_serial_number}: {e}')
        else:
            raise ValueError(
                f'Unable to connect to Basler USB camera with serial number {serial_number}. '
                'Please verify that the camera is plugged in and that the serial number is correct.'
            )

        # Apply the options from the config
        options = self.config.get('options')
        if options:
            self.apply_options(options)

    def _read(self) -> np.ndarray:
        with self.camera.GrabOne(2000) as result:
            if result.GrabSucceeded():
                frame = result.GetArray()
            else:
                frame = None

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return frame
            
    def release(self) -> None:
        self.camera.Close()

    def apply_options(self, options: dict) -> None:
        self.config['options'] = options

        self.camera.PixelFormat.SetValue('RGB8')
        
        exposure = options.get('exposure')
        if exposure:
            self.camera.ExposureTime.SetValue(exposure)

class RealSenseFrameGrabber(FrameGrabber):
    """Intel RealSense Depth Camera
    """

    def __init__(self, config: dict):
        self.config = config

    def _read(self) -> np.ndarray:
        pass

    def release(self) -> None:
        pass

class GigEFrameGrabber(FrameGrabber):
    """GigE Camera
    """

    def __init__(self, config: dict):
        self.config = config

    def _read(self) -> np.ndarray:
        pass

    def release(self) -> None:
        pass

# # TODO update this class to work with the latest updates
# class DirectoryFrameGrabber(FrameGrabber):
#     def __init__(self, stream=None, fps_target=0):
#         """stream must be an file mask"""
#         try:
#             self.filename_list = []
#             for filename in os.listdir():
#                 if fnmatch.fnmatch(filename, stream):
#                     self.filename_list.append(filename)
#             logger.debug(f"found {len(self.filename_list)} files matching stream: {stream}")
#             random.shuffle(self.filename_list)
#         except Exception as e:
#             logger.error(f"could not initialize DirectoryFrameGrabber: stream: {stream} filename is invalid or read error")
#             raise e
#         if len(self.filename_list) == 0:
#             logger.warning(f"no files found matching stream: {stream}")

#     def grab(self):
#         if len(self.filename_list) == 0:
#             raise RuntimeWarning(f"could not read frame from {self.capture}.  possible end of file.")

#         start = time.time()
#         frame = cv2.imread(self.filename_list[0], cv2.IMREAD_GRAYSCALE)
#         self.filename_list.pop(0)
#         logger.debug(f"read the frame in {1000*(time.time()-start):.1f}ms")

#         return frame

# # TODO update this class to work with the latest updates
# class FileStreamFrameGrabber(FrameGrabber):
#     def __init__(self, stream=None, fps_target=0):
#         """stream must be an filename"""
#         try:
#             self.capture = cv2.VideoCapture(stream)
#             logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
#             ret, frame = self.capture.read()
#             self.fps_source = round(self.capture.get(cv2.CAP_PROP_FPS), 2)
#             self.fps_target = fps_target
#             logger.debug(f"source FPS : {self.fps_source}  / target FPS : {self.fps_target}")
#             self.remainder = 0.0
#         except Exception as e:
#             logger.error(f"could not initialize DeviceFrameGrabber: stream: {stream} filename is invalid or read error")
#             raise e

#     def _read(self) -> np.ndarray:
#         """decimates stream to self.fps_target, 0 fps to use full original stream.
#         consistent with existing behavior based on VideoCapture.read()
#         which may return None when it cannot read a frame.
#         """
#         start = time.time()

#         if self.fps_target > 0 and self.fps_target < self.fps_source:
#             drop_frames = (self.fps_source / self.fps_target) - 1 + self.remainder
#             for i in range(round(drop_frames)):
#                 ret, frame = self.capture.read()
#             self.remainder = round(drop_frames - round(drop_frames), 2)
#             logger.info(
#                 f"dropped {round(drop_frames)} frames to meet {self.fps_target} FPS target from {self.fps_source} FPS source (off by {self.remainder} frames)"
#             )
#         else:
#             logger.debug(f"frame dropping disabled for {self.fps_target} FPS target from {self.fps_source} FPS source")

#         ret, frame = self.capture.read()
#         if not ret:
#             raise RuntimeWarning(f"could not read frame from {self.capture}.  possible end of file.")
#         now = time.time()
#         logger.debug(f"read the frame in {1000*(now-start):.1f}ms")
#         return frame

# # TODO update this class to work with the latest updates
# class YouTubeFrameGrabber(FrameGrabber):
#     """grabs the most recent frame from an YouTube stream. To avoid extraneous bandwidth
#     this class tears down the stream between each frame grab.  maximum framerate
#     is likely around 0.5fps in most cases.
#     """

#     def __init__(self, stream=None):
#         self.stream = stream
#         self.video = pafy.new(self.stream)
#         self.best_video = self.video.getbest(preftype="mp4")
#         self.capture = cv2.VideoCapture(self.best_video.url)
#         logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
#         if not self.capture.isOpened():
#             raise ValueError(f"could not initially open {self.stream}")
#         self.capture.release()

#     def reset_stream(self):
#         self.video = pafy.new(self.stream)
#         self.best_video = self.video.getbest(preftype="mp4")
#         self.capture = cv2.VideoCapture(self.best_video.url)
#         logger.debug(f"initialized video capture with backend={self.capture.getBackendName()}")
#         if not self.capture.isOpened():
#             raise ValueError(f"could not initially open {self.stream}")
#         self.capture.release()

#     def grab(self):
#         start = time.time()
#         self.capture = cv2.VideoCapture(self.best_video.url)
#         ret, frame = self.capture.read()  # grab and decode since we want this frame
#         if not ret:
#             logger.error(f"could not read frame from {self.capture}. attempting to reset stream")
#             self.reset_stream()
#             self.capture = cv2.VideoCapture(self.best_video.url)
#             ret, frame = self.capture.read()
#             if not ret:
#                 logger.error(f"failed to effectively reset stream {self.stream} / {self.best_video.url}")
#         now = time.time()
#         logger.debug(f"read the frame in {1000*(now-start):.1f}ms")
#         self.capture.release()
#         return frame

# # TODO update this class to work with the latest updates
# class ImageURLFrameGrabber(FrameGrabber):
#     """grabs the current image at a single URL.
#     NOTE: if image is expected to be refreshed or change with a particular frequency,
#     it is up to the user of the class to call the `grab` method with that frequency
#     """

#     def __init__(self, url=None, **kwargs):
#         self.url = url

#     def grab(self):
#         start = time.time()
#         try:
#             req = urllib.request.urlopen(self.url)
#             response = req.read()
#             arr = np.asarray(bytearray(response), dtype=np.uint8)
#             frame = cv2.imdecode(arr, -1)  # 'Load it as it is'
#         except Exception as e:
#             logger.error(f"could not grab frame from {self.url}: {str(e)}")
#             frame = None
#         now = time.time()
#         elapsed = now - start
#         logger.info(f"read image from URL {self.url} into frame in {elapsed}s")

#         return frame