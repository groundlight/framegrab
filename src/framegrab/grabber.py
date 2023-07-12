import logging
import platform
import re
import subprocess
import time
from abc import ABC, abstractmethod
from threading import Lock, Thread
from typing import Dict, List

import cv2
import numpy as np

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO) # not sure if this is the right long term solution

# Optional imports
try:
    from pypylon import pylon
except ImportError:
    logger.warning(
        'Dependency for Basler cameras (pypylon) not found. '
        'If you are not trying to use a Basler camera, you can ignore this message. '
        'If you are trying to use a Basler camera, you will need to install pypylon.'
    )
    pylon = None

try:
    from pyrealsense2 import pyrealsense2 as rs
except ImportError:
    logger.warning(
        'Dependency for Intel RealSense cameras (pyrealsense2) not found. '
        'If you are not trying to use a RealSense camera, you can ignore this message. '
        'If you are trying to use a RealSense camera, you will need to install pyrealsense2.'
    )
    rs = None

OPERATING_SYSTEM = platform.system()
DIGITAL_ZOOM_MAX = 4

class InputTypes:
    """Defines the available input types from FrameGrabber objects"""

    WEBCAM = "webcam"
    RTSP = "rtsp"
    REALSENSE = "realsense"
    BASLER = "basler"

    def get_options() -> list:
        """Get a list of the available InputType options"""
        output = []
        for attr_name in vars(InputTypes):
            attr_value = getattr(InputTypes, attr_name)
            if "__" not in attr_name and isinstance(attr_value, str):
                output.append(attr_value)
        return output

class FrameGrabber(ABC):
    # for naming FrameGrabber objects that have no user-defined name
    unnamed_grabber_count = 0

    @staticmethod
    def _validate_config(config: dict) -> dict:
        """Check the config to ensure it conforms to the required format and data types
        Returns a corrected version of the config.
        """
        output_config = config.copy()

        # Ensure that serial numbers are strings
        try:
            output_config["id"]["serial_number"] = str(output_config["id"]["serial_number"])
        except KeyError:
            pass

        # Ensure that there are some options, even if they are empty
        if not output_config.get("options"):
            output_config["options"] = {}

        return output_config

    @staticmethod
    def create_grabbers(configs: List[dict]) -> dict:
        """
        Creates multiple FrameGrab objects based on user-provided configurations

        Parameters:
        configs (List[dict]): A list of dictionaries, where each dictionary contains the configuration
                              for a FrameGrabber.

        Returns:
        dict: A dictionary where the keys are the camera names, and the values are FrameGrab
        objects.
        """

        # Sort the configs such that configs with serial numbers appear first
        # This will ensure that they are able to connect to the camera with the specified
        # serial number, and that no other FrameGrabbers claim that camera first.
        configs.sort(key=lambda config: "serial_number" not in config.get("id", {}))

        grabbers = {}
        for config in configs:
            grabber = FrameGrabber.create_grabber(config)
            name = grabber.config['name']
            grabbers[name] = grabber

        return grabbers

    @staticmethod
    def create_grabber(config: dict):
        """Returns a single FrameGrabber object given a configuration dictionary."""

        # Ensure the config is properly constructed and typed
        config = FrameGrabber._validate_config(config)

        # At a minimum, input_type must be provided
        input_type = config.get("input_type", None)
        if input_type is None:
            raise ValueError(f"No input_type provided. Valid types are {InputTypes.get_options()}")

        # Based on input_type, create correct type of FrameGrabber
        if input_type == InputTypes.WEBCAM:
            grabber = WebcamFrameGrabber(config)
        elif input_type == InputTypes.RTSP:
            grabber = RTSPFrameGrabber(config)
        elif input_type == InputTypes.BASLER:
            grabber = BaslerFrameGrabber(config)
        elif input_type == InputTypes.REALSENSE:
            grabber = RealSenseFrameGrabber(config)
        else:
            raise ValueError(
                f"The provided input_type ({input_type}) is not valid. Valid types are {InputTypes.get_options()}"
            )

        # If a name wasn't supplied, create one
        if not config.get("name", False):
            FrameGrabber.unnamed_grabber_count += 1
            count = FrameGrabber.unnamed_grabber_count
            config["name"] = f"Camera {count} ({input_type})"

        # Apply the options so that resolution, exposure, etc. is correct
        grabber.apply_options(config["options"])

        return grabber

    @staticmethod
    def autodiscover() -> dict:
        """Autodiscovers cameras and returns a dictionary of FrameGrabber objects"""
        autodiscoverable_input_types = (
            InputTypes.REALSENSE,
            InputTypes.WEBCAM,
            InputTypes.BASLER,
        )

        grabbers = {}
        for input_type in autodiscoverable_input_types:
            while True:
                try:
                    config = {"input_type": input_type}
                    grabber = FrameGrabber.create_grabber(config)
                    name = grabber.config["name"]
                    grabbers[name] = grabber
                except (ValueError, ImportError):
                    # ValueError is taken to mean that we have reached the end of enumeration for the current input_type.
                    # ImportError means the requisite packages aren't installed for the current input_type.
                    # In both cases, it's time to move on to the next input_type.
                    break

        return grabbers

    @abstractmethod
    def grab(self) -> np.ndarray:
        """Read a frame from the camera, zoom and crop if called for, and then perform any camera-specific
        postprocessing operations.
        Returns a frame.
        """
        pass

    def _crop(self, frame: np.ndarray) -> np.ndarray:
        """Looks at FrameGrabber's options and decides to either crop in an absolute manner (by pixels) or
        in a relative manner (normalized).

        Returns a cropped frame.
        """
        absolute_crop_params = self.config.get("options", {}).get("crop", {}).get("absolute")
        if absolute_crop_params:
            return self._crop_absolute(frame, absolute_crop_params)

        relative_crop_params = self.config.get("options", {}).get("crop", {}).get("relative")
        if relative_crop_params:
            return self._crop_relative(frame, relative_crop_params)

        return frame

    def _crop_absolute(self, frame: np.ndarray, crop_params: Dict[str, int]) -> np.ndarray:
        """Crops the provided frame according to the FrameGrabbers cropping configuration.
        Crops according to pixels positions.
        """
        top = crop_params.get("top", 0)
        bottom = crop_params.get("bottom", frame.shape[0])
        left = crop_params.get("left", 0)
        right = crop_params.get("right", frame.shape[1])
        frame = frame[top:bottom, left:right]

        return frame

    def _crop_relative(self, frame: np.ndarray, crop_params: Dict[str, float]) -> np.ndarray:
        """Crops the provided frame according to the FrameGrabbers cropping configuration.
        Crops according to relative positions (0-1).
        """
        top = crop_params.get("top", 0) * frame.shape[0]
        bottom = crop_params.get("bottom", 1) * frame.shape[0]
        left = crop_params.get("left", 0) * frame.shape[1]
        right = crop_params.get("right", 1) * frame.shape[1]
        frame = frame[int(top) : int(bottom), int(left) : int(right)]

        return frame

    def _digital_zoom(self, frame: np.ndarray) -> np.ndarray:
        digital_zoom = self.config.get("options", {}).get("zoom", {}).get("digital")

        if digital_zoom is None:
            pass
        elif digital_zoom < 1 or digital_zoom > DIGITAL_ZOOM_MAX:
            raise ValueError(
                f"Invalid value for digital_zoom ({digital_zoom}). "
                f"Digital zoom cannot be greater than {DIGITAL_ZOOM_MAX}."
            )
        else:
            top = (frame.shape[0] - frame.shape[0] / digital_zoom) / 2
            bottom = frame.shape[0] - top
            left = (frame.shape[1] - frame.shape[1] / digital_zoom) / 2
            right = frame.shape[1] - left
            frame = frame[int(top) : int(bottom), int(left) : int(right)]

        return frame

    def _set_cv2_resolution(self) -> None:
        """Set the resolution of the cv2.VideoCapture object based on the FrameGrabber's config.
        If the FrameGrabber lacks both of these properties (height and width), this method
        will do nothing.
        """
        resolution = self.config.get("options", {}).get("resolution", {})
        height = resolution.get("height")
        width = resolution.get("width")

        if width:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    @abstractmethod
    def apply_options(options: dict) -> None:
        """Update any camera-specific options, such as resolution or exposure"""
        pass

    @abstractmethod
    def release() -> None:
        """A cleanup method. Releases/closes the video capture, terminates any related threads, etc."""
        pass


class WebcamFrameGrabber(FrameGrabber):
    """For any generic webcam"""

    # keep track of the webcams that are already in use so that we don't try to connect to them twice
    indices_in_use = set()

    def __init__(self, config: dict):
        self.config = config

        serial_number = self.config.get("id", {}).get("serial_number")

        if serial_number and OPERATING_SYSTEM != "Linux":
            logger.warning(
                f"Matching webcams with serial_number is not supported on your operating system, {OPERATING_SYSTEM}. "
                "Webcams will be sequentially assigned instead."
            )

        # Find the serial number of connected webcams. Currently only works on Linux.
        if OPERATING_SYSTEM == "Linux":
            found_webcams = WebcamFrameGrabber._find_webcam_serial_numbers()
        else:
            found_webcams = {}

        # Assign camera based on serial number if 1) serial was provided and 2) we know the 
        # serial numbers of plugged in devices
        if found_webcams.get('serial_number', False):
            idx = found_webcams.get('idx')

            if idx in WebcamFrameGrabber.indices_in_use:
                raise ValueError(
                    f"Webcam index {idx} already in use. "
                    f"Did you use the same serial number ({serial_number}) for two different cameras?"
                )

            capture = cv2.VideoCapture(idx)
            if not capture.isOpened():
                raise ValueError(
                    f"Unable to find webcam with the specified serial_number: {serial_number}. "
                    "Please ensure that the serial number is correct and that the webcam is plugged in."
                )
        # If no serial number was provided, just assign the next available camera by index
        else:
            for idx in range(20):
                if idx in WebcamFrameGrabber.indices_in_use:
                    continue  # Webcam is already in use, moving on

                capture = cv2.VideoCapture(idx)
                if capture.isOpened():
                    break  # Found a valid capture, no need to look any further
            else:
                raise ValueError(f"Unable to connect to webcam by index. Is your webcam plugged in?")
            
        # If a serial_number wasn't provided, attempt to find one and add it to the config
        if not serial_number:
            for serial_number, values in found_webcams.items():
                curr_idx = values['idx']
                if idx == curr_idx:
                    self.config['id']['serial_number'] = serial_number
                    break

        # A valid capture has been found, saving it for later
        self.capture = capture

        # Log the current webcam index as 'in use' to prevent other WebcamFrameGrabbers from stepping on it
        self.idx = idx
        WebcamFrameGrabber.indices_in_use.add(idx)

    def grab(self) -> np.ndarray:
        _, frame = self.capture.read()
        frame = self._crop(frame)
        frame = self._digital_zoom(frame)
        return frame

    def release(self) -> None:
        self.capture.release()
        WebcamFrameGrabber.indices_in_use.remove(self.idx)

    def apply_options(self, options: dict) -> None:
        self.config["options"] = options
        self._set_cv2_resolution()

        # set the buffer size to 1 to always get the most recent frame
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    @staticmethod
    def _find_webcam_serial_numbers() -> dict:
        """Finds all plugged in webcams and returns a dictionary mapping serial_numbers to details about the device.
        This is useful for connecting the dots between user provided configurations
        and actual plugged in devices.

        This function only works on Linux, and was specifically tested on an Nvidia Jetson.
        """

        # ls /dev/video* returns device paths for all plugged in webcams
        command = "ls /dev/video*"
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout, _ = process.communicate()
        output = stdout.decode("utf-8")
        devices = output.strip().split("\n")

        found_webcams = {}
        for devpath in devices:
            # ls -l /sys/class/video4linux/video0/device returns a path that points back into the /sys/bus/usb/devices/
            # directory where can determine the serial number.
            # e.g. /sys/bus/usb/devices/2-3.2:1.0 -> /sys/bus/usb/devices/<bus>-<port>.<subport>:<config>.<interface>
            devname = devpath.split("/")[-1]
            command = f"ls -l /sys/class/video4linux/{devname}/device"
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            stdout, _ = process.communicate()
            output = stdout.decode("utf-8")
            bus_port_subport = output.split("/")[-1].split(":")[0]

            # find the serial number
            command = f"cat /sys/bus/usb/devices/{bus_port_subport}/serial"
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            stdout, _ = process.communicate()
            serial_number = stdout.decode("utf-8").strip()

            # find the index
            idx = int(re.findall(r"\d+", devname)[-1])

            if serial_number:
                found_webcams[serial_number] = {
                    'devname': devname,
                    'idx': idx,
                }

        return found_webcams


class RTSPFrameGrabber(FrameGrabber):
    """Grabs the most recent frame from an rtsp stream. The RTSP capture
    object has a non-configurable built-in buffer, so just calling
    grab would return the oldest frame in the buffer rather than the
    latest frame. This class uses a thread to continously drain the
    buffer by grabbing and discarding frames and only returning the
    latest frame when explicitly requested.
    """

    def __init__(self, config: dict):
        self.config = config

        stream = self.config.get("address", {}).get("rtsp_url")
        if not stream:
            raise ValueError(f"No RTSP URL provided. " "Please add an address attribute to the configuration.")
        self.capture = cv2.VideoCapture(stream)

        if not self.capture.isOpened():
            raise ValueError(
                f"Could not open RTSP stream: {stream}. "
                "Is the RSTP URL correct? Is the camera connected to the network?"
            )

        logger.debug(f"Initialized video capture with backend={self.capture.getBackendName()}")

        self.run = True
        self.lock = Lock()

        # The _drain thread needs to periodically wait to avoid overloading the CPU. Ideally this would be done 
        # at the rate of the RTSP feed's FPS. Unfortunately, OpenCV cannot consistently read the FPS of an RTSP 
        # feed, so we will assume a high FPS of 60.
        self.drain_rate = 1 / 60

        Thread(target=self._drain).start()

    def grab(self) -> np.ndarray:
        with self.lock:
            ret, frame = self.capture.retrieve() # grab and decode since we want this frame
            if not ret:
                logger.error(f"Could not read frame from {self.capture}")

        frame = self._crop(frame)
        frame = self._digital_zoom(frame)

        return frame

    def release(self) -> None:
        self.run = False  # to stop the buffer drain thread

        with self.lock:
            self.capture.release()

    def apply_options(self, options: dict) -> None:
        self.config["options"] = options

    def _drain(self) -> None:
        """Repeatedly grabs frames without decoding them.
        This keeps the buffer empty so that when we actually want to read a frame,
        we can get the most current one.
        """
        while self.run:
            with self.lock:
                _ = self.capture.grab()
            time.sleep(self.drain_rate)

class BaslerFrameGrabber(FrameGrabber):
    """Basler USB and GigE Cameras"""

    serial_numbers_in_use = set()

    def __init__(self, config: dict):
        if pylon is None:
            raise ImportError(
                "Using Basler cameras requires the pypylon package, which is not installed on this system. "
                "Please install pypylon and try again."
            )

        self.config = config

        tlf = pylon.TlFactory.GetInstance()
        devices = tlf.EnumerateDevices()

        if not devices:
            raise ValueError("No Basler cameras were found. Is your camera connected?")

        # Attempt to match the provided serial number with a plugged in device. If no serial number was provided, just
        # pick the first found device that is not currently in use.
        serial_number = config.get("id", {}).get("serial_number")
        for device in devices:
            curr_serial_number = device.GetSerialNumber()
            if curr_serial_number in BaslerFrameGrabber.serial_numbers_in_use:
                continue
            if serial_number is None or serial_number == curr_serial_number:
                camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(device))
                camera.Open()
                logger.info(f"Connected to Basler camera with serial number {curr_serial_number}.")
                break
        else:
            raise ValueError(
                f"Unable to connect to Basler camera with serial number: {serial_number}. "
                "Please verify that the camera is connected and that the serial number is correct."
            )

        # A valid camera has been found, remember the serial_number to prevent
        # other FrameGrabbers from using it
        self.camera = camera
        self.serial_number = curr_serial_number
        BaslerFrameGrabber.serial_numbers_in_use.add(self.serial_number)

    def grab(self) -> np.ndarray:
        with self.camera.GrabOne(2000) as result:
            if result.GrabSucceeded():
                frame = result.GetArray()
            else:
                frame = None

        frame = self._crop(frame)
        frame = self._digital_zoom(frame)

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return frame

    def release(self) -> None:
        self.camera.Close()
        BaslerFrameGrabber.serial_numbers_in_use.remove(self.serial_number)

    def apply_options(self, options: dict) -> None:
        self.config["options"] = options

        if self.pixel_format:
            self.camera.PixelFormat.SetValue(self.pixel_format)

        if self.exposure:
            self.camera.ExposureTime.SetValue(self.exposure)

    @property
    def pixel_format(self):
        return self.config.get("options", {}).get("pixel_format")

    @property
    def exposure(self):
        return self.config.get("options", {}).get("exposure")


class RealSenseFrameGrabber(FrameGrabber):
    """Intel RealSense Depth Camera"""

    def __init__(self, config: dict):
        if rs is None:
            raise ImportError(
                "Using IntelRealSense cameras requires the pyrealsense2 package, which is not installed on this system. "
                "Please install pyrealsense2 and try again."
            )

        self.config = config

        ctx = rs.context()
        if len(ctx.devices) == 0:
            raise ValueError(f"No Intel RealSense cameras detected. Is your camera plugged in?")

        provided_serial_number = self.config.get("id", {}).get("serial_number")

        # Iterate through each detected camera and attempt to match it with the provided camera config
        for device in ctx.devices:
            pipeline = rs.pipeline()
            rs_config = rs.config()

            curr_serial_number = device.get_info(rs.camera_info.serial_number)

            if provided_serial_number is None or curr_serial_number == provided_serial_number:
                rs_config.enable_device(curr_serial_number)

                # Try to connect to the camera
                try:
                    pipeline_profile = pipeline.start(rs_config)
                    break  # succesfully connected, breaking out of loop
                except RuntimeError as e:
                    # the current camera is not available, moving on to the next
                    continue

        else:
            raise ValueError(
                f"Unable to connect to Intel RealSense camera with serial_number: {provided_serial_number}. "
                "Is the serial number correct? Is the camera plugged in?"
            )

        # A valid pipeline was found, saving for later
        self.pipeline = pipeline

    def grab(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays and convet from RGB to BGR
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)

        depth_image = self._crop(depth_image)
        depth_image = self._digital_zoom(depth_image)

        color_image = self._crop(color_image)
        color_image = self._digital_zoom(color_image)

        # side by side
        display_side_by_side = self.config.get("options", {}).get("depth", {}).get("side_by_side")
        if display_side_by_side:
            return self._side_by_side(depth_image, color_image)
        else:
            return color_image

    def _side_by_side(self, depth_image: np.ndarray, color_image: np.ndarray) -> np.ndarray:
        """Merges color image and depth image into a wider image, all in RGB"""

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_BONE)
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(
                color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA
            )
            sidebyside = np.hstack((resized_color_image, depth_colormap))
        else:
            sidebyside = np.hstack((color_image, depth_colormap))
        return sidebyside

    def release(self) -> None:
        self.pipeline.stop()

    def apply_options(self, options: dict) -> None:
        self.config["options"] = options

# # TODO update this class to work with the latest updates
# import os
# import fnmatch
# import random
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

# # TODO update this class to work with the latest updates'
# import pafy
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
# import urllib
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
