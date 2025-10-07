import logging
import platform
import re
import subprocess
import time
from abc import ABC, abstractmethod
from threading import Lock, Thread
from typing import Dict, List, Optional, Union

import cv2
import numpy as np
import yaml

from .config import (
    DEFAULT_FOURCC,
    DEFAULT_FPS,
    BaslerFrameGrabberConfig,
    FileStreamFrameGrabberConfig,
    FrameGrabberConfig,
    GenericUSBFrameGrabberConfig,
    HttpLiveStreamingFrameGrabberConfig,
    InputTypes,
    MockFrameGrabberConfig,
    RaspberryPiCSI2FrameGrabberConfig,
    RealSenseFrameGrabberConfig,
    ROS2GrabberConfig,
    RTSPFrameGrabberConfig,
    YouTubeLiveFrameGrabberConfig,
)
from .exceptions import GrabError
from .rtsp_discovery import AutodiscoverMode, RTSPDiscovery
from .unavailable_module import UnavailableModuleOrObject

# The wsdiscovery packages calls logging.basicConfig, which will wipe out any logging config
# that framegrab users have set, which is not good. To clear the config that wsdiscovery sets, we
# will run the following
# I made a PR to fix this aspect of python-ws-discovery: https://github.com/andreikop/python-ws-discovery/pull/89
root_logger = logging.getLogger()
if root_logger.hasHandlers():
    root_logger.handlers.clear()

logger = logging.getLogger(__name__)

# Create a logger for this module
# -- Optional imports --
# Only used for Basler cameras, not required otherwise
try:
    from pypylon import pylon
except ImportError as e:
    pylon = UnavailableModuleOrObject(e)

# Only used for RealSense cameras, not required otherwise
try:
    from pyrealsense2 import pyrealsense2 as rs
except ImportError as e:
    rs = UnavailableModuleOrObject(e)

# Only used for CSI2 cameras with Raspberry Pi, not required otherwise
try:
    from picamera2 import Picamera2
except ImportError as e:
    Picamera2 = UnavailableModuleOrObject(e)

# Only used for Youtube Live streams, not required otherwise
try:
    import streamlink
except ImportError as e:
    streamlink = UnavailableModuleOrObject(e)

try:
    from .ros2_client import ROS2Client
except ImportError as e:
    ROS2Client = UnavailableModuleOrObject(e)

logger = logging.getLogger(__name__)

OPERATING_SYSTEM = platform.system()
NOISE = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)  # in case a camera can't get a frame


class FrameGrabber(ABC):
    # for naming FrameGrabber objects that have no user-defined name
    unnamed_grabber_count = 0
    config: FrameGrabberConfig

    def __init__(self, config: Union[FrameGrabberConfig, dict]):
        """To create a FrameGrabber object with the generic FrameGrabber class or with a config dict, use create_grabber()"""
        if isinstance(config, dict):
            config = self.config_class.from_framegrab_config_dict(config)
        if not isinstance(config, self.config_class):
            raise TypeError(
                f"Expected config to be of type {self.config_class.__name__}, but got {type(config).__name__}"
            )

        self.config = config
        self._initialize_grabber_implementation()
        # Apply the options so that resolution, exposure, etc. are correct
        # a little hacky to convert back to a dictionary temporarily but it works
        options = config.to_framegrab_config_dict()["options"]
        self.apply_options(options)

    @abstractmethod
    def _initialize_grabber_implementation(self):
        """Each FrameGrabber must implement its own method of initializing the grabber"""
        pass

    @staticmethod
    def _validate_dict_config(config: dict) -> FrameGrabberConfig:
        """Check the config to ensure it conforms to the required format and data types
        Returns a corrected version of the config.
        """
        output_config = config.copy()
        model_config = FrameGrabberConfig.from_framegrab_config_dict(output_config)
        return model_config

    @staticmethod
    def create_grabbers(
        configs: List[Union[dict, FrameGrabberConfig]], warmup_delay: float = 1.0
    ) -> Dict[str, "FrameGrabber"]:
        """
        Creates multiple FrameGrab objects based on user-provided configurations

        Parameters:
        configs (List[dict]): A list of dictionaries, where each dictionary contains the configuration
                              for a FrameGrabber.

        warmup_delay (float, optional): The number of seconds to wait after creating the grabbers. USB
            cameras often need a moment to warm up before they can be used; grabbing frames too early
            might result in dark or blurry images.
            Defaults to 1.0. Only happens if there are any generic_usb cameras in the config list.

        Returns:
        dict: A dictionary where the keys are the camera names, and the values are FrameGrab
        objects.
        """

        configs_as_model_config = [
            FrameGrabberConfig.from_framegrab_config_dict(config) if isinstance(config, dict) else config
            for config in configs
        ]

        # Sort the configs such that configs with serial numbers appear first
        # This will ensure that they are able to connect to the camera with the specified
        # serial number, and that no other FrameGrabbers claim that camera first.
        configs_as_model_config.sort(
            key=lambda config: not hasattr(config, "serial_number") or config.serial_number is None
        )

        # Do not allow duplicate camera names
        names = [config.name for config in configs_as_model_config if config.name is not None]
        if len(names) != len(set(names)):
            raise ValueError(
                f"Duplicate camera names were provided in configurations. Please ensure that each camera name is unique. "
                f"Provided camera names: {names}"
            )

        # Create the grabbers
        grabber_list = []
        for config in configs_as_model_config:
            try:
                grabber = FrameGrabber.create_grabber(config, warmup_delay=0)
                grabber_list.append(grabber)
            except ValueError as e:
                camera_name = config.name
                logger.error(
                    f"Failed to connect to {camera_name}. Please check its connection and provided configuration: {config}. Error: {e}",
                    exc_info=True,
                )

        grabbers = FrameGrabber.grabbers_to_dict(grabber_list)

        # Do the warmup delay if necessary
        grabber_types = set([grabber.config.get_input_type() for grabber in grabbers.values()])
        if InputTypes.GENERIC_USB in grabber_types and warmup_delay > 0:
            logger.info(
                f"Waiting {warmup_delay} seconds for camera(s) to warm up. "
                "Pass in warmup_delay = 0 to suppress this behavior."
            )
            time.sleep(warmup_delay)

        return grabbers

    @staticmethod
    def from_yaml(filename: Optional[str] = None, yaml_str: Optional[str] = None) -> List["FrameGrabber"]:
        """Creates multiple FrameGrabber objects based on a YAML file or YAML string.

        Args:
            filename (str, optional): The filename of the YAML file to load.
                Either filename or yaml_str must be provided, but not both.
            yaml_str (str, optional): A YAML string to parse.
                Either filename or yaml_str must be provided, but not both.

        Returns:
            List[FrameGrabber]: A list of FrameGrabber objects created from the YAML configuration.
        """
        if filename is None and yaml_str is None:
            raise ValueError("Either filename or yaml_str must be provided.")
        if filename is not None and yaml_str is not None:
            raise ValueError("Only one of filename or yaml_str can be provided.")
        if filename:
            with open(filename, "r") as f:
                yaml_str = f.read()
        full_config = yaml.safe_load(yaml_str)
        if "image_sources" not in full_config:
            raise ValueError("Invalid config file. Camera configs must be under the 'image_sources' key.")
        image_sources = full_config["image_sources"]
        # Check that it's a list.
        if image_sources is None or not isinstance(image_sources, list):
            raise ValueError("Invalid config file. 'image_sources' must be a list.")
        grabber_dict = FrameGrabber.create_grabbers(image_sources)
        grabber_list = []
        for _, grabber in grabber_dict.items():
            grabber_list.append(grabber)
        return grabber_list

    @staticmethod
    def grabbers_to_dict(grabber_list: list) -> dict:
        """Converts a list of FrameGrabber objects into a dictionary where the keys are the camera names.
        Sorts the grabbers by serial_number to make sure they always come up in the same order.
        Autogenerates names for any unnamed grabbers.
        """

        # Sort the grabbers by serial_number to make sure they always come up in the same order
        # Sort the grabbers by serial_number if available, otherwise use a default value
        def get_sort_key(grabber):
            if hasattr(grabber.config, "serial_number") and grabber.config.serial_number is not None:
                return str(grabber.config.serial_number)
            return "~"  # Tilde character is after alphanumeric in ASCII/Unicode sorting

        grabber_list = sorted(grabber_list, key=get_sort_key)

        # Create the grabbers dictionary, autogenerating names for any unnamed grabbers
        grabbers = {}
        for grabber in grabber_list:
            # Add the grabber to the dictionary
            grabber_name = grabber.config.name
            grabbers[grabber_name] = grabber

        return grabbers

    @staticmethod
    def create_grabber_yaml(yaml_config: str, warmup_delay: float = 1.0):
        """Create a FrameGrabber object based on the provided configuration.

        Parameters:
            config (str): A yaml string containing configuration settings for the FrameGrabber.

            warmup_delay (float, optional): The number of seconds to wait after creating the grabbers. USB
                cameras often need a moment to warm up before they can be used; grabbing frames too early
                might result in dark or blurry images.
                Defaults to 1.0. Only applicable to generic_usb cameras.

        Returns:
                An instance of a FrameGrabber subclass based on the provided
                configuration. The specific subclass will be determined by the content of the
                configuration dictionary.
        """
        config = yaml.safe_load(yaml_config)
        grabber = FrameGrabber.create_grabber(config, warmup_delay)
        return grabber

    @staticmethod
    def create_grabber(config: Union[dict, FrameGrabberConfig], warmup_delay: float = 1.0) -> "FrameGrabber":
        """Create a FrameGrabber object based on the provided configuration.

        Parameters:
            config (dict or FrameGrabberConfig): A dictionary or FrameGrabberConfig object containing configuration settings for the FrameGrabber.

            warmup_delay (float, optional): The number of seconds to wait after creating the grabbers. USB
                cameras often need a moment to warm up before they can be used; grabbing frames too early
                might result in dark or blurry images.
                Defaults to 1.0. Only applicable to generic_usb cameras.

        Returns:
                An instance of a FrameGrabber subclass based on the provided
                configuration. The specific subclass will be determined by the content of the
                configuration dictionary.

        """
        if isinstance(config, dict):
            model_config = FrameGrabber._validate_dict_config(config)
        else:
            model_config = config

        input_type = model_config.get_input_type()

        # Based on input_type, create correct type of FrameGrabber
        if input_type == InputTypes.GENERIC_USB:
            grabber = GenericUSBFrameGrabber(model_config)
        elif input_type == InputTypes.RTSP:
            grabber = RTSPFrameGrabber(model_config)
        elif input_type == InputTypes.BASLER:
            grabber = BaslerFrameGrabber(model_config)
        elif input_type == InputTypes.REALSENSE:
            grabber = RealSenseFrameGrabber(model_config)
        elif input_type == InputTypes.RPI_CSI2:
            grabber = RaspberryPiCSI2FrameGrabber(model_config)
        elif input_type == InputTypes.HLS:
            grabber = HttpLiveStreamingFrameGrabber(model_config)
        elif input_type == InputTypes.YOUTUBE_LIVE:
            grabber = YouTubeLiveFrameGrabber(config)
        elif input_type == InputTypes.FILE_STREAM:
            grabber = FileStreamFrameGrabber(model_config)
        elif input_type == InputTypes.MOCK:
            grabber = MockFrameGrabber(model_config)
        elif input_type == InputTypes.ROS2:
            grabber = ROS2FrameGrabber(model_config)
        else:
            raise ValueError(
                f"The provided input_type ({input_type}) is not valid. Valid types are {InputTypes.get_options()}"
            )

        # Do the warmup delay if necessary
        if input_type == InputTypes.GENERIC_USB and warmup_delay > 0:
            logger.info(
                f"Waiting {warmup_delay} seconds for camera to warm up. "
                "Pass in warmup_delay = 0 to suppress this behavior."
            )
            time.sleep(warmup_delay)

        return grabber

    @staticmethod
    def autodiscover(
        warmup_delay: float = 1.0,
        rtsp_discover_mode: AutodiscoverMode = AutodiscoverMode.off,
    ) -> dict:
        """Autodiscovers cameras and returns a dictionary of FrameGrabber objects

        warmup_delay (float, optional): The number of seconds to wait after creating the grabbers. USB
            cameras often need a moment to warm up before they can be used; grabbing frames too early
            might result in dark or blurry images.
            Defaults to 1.0. Only happens if there are any generic_usb cameras in the config list.

        rtsp_discover_mode (AutodiscoverMode, optional): Options to try different default credentials
            stored in DEFAULT_CREDENTIALS for RTSP cameras.
            Consists of five options:
                off: No discovery.
                ip_only: Only discover the IP address of the camera.
                light: Only try first two usernames and passwords ("admin:admin" and no username/password).
                complete_fast: Try the entire DEFAULT_CREDENTIALS without delays in between.
                complete_slow: Try the entire DEFAULT_CREDENTIALS with a delay of 1 seconds in between.
            Defaults to off.
        """
        autodiscoverable_input_types = (
            InputTypes.REALSENSE,
            InputTypes.GENERIC_USB,
            InputTypes.BASLER,
            InputTypes.RTSP,
        )

        # Autodiscover the grabbers
        grabber_list = []
        for input_type in autodiscoverable_input_types:
            logger.info(f"Autodiscovering {input_type} cameras...")

            # If the input type is RTSP and rtsp_discover_modes is provided, use RTSPDiscovery to find the cameras
            if input_type == InputTypes.RTSP:
                if rtsp_discover_mode is not AutodiscoverMode.off:
                    onvif_devices = RTSPDiscovery.discover_onvif_devices(auto_discover_mode=rtsp_discover_mode)
                    for device in onvif_devices:
                        for index, rtsp_url in enumerate(device.rtsp_urls):
                            grabber = FrameGrabber.create_grabber(
                                {
                                    "input_type": input_type,
                                    "id": {"rtsp_url": rtsp_url},
                                    "name": f"RTSP Camera - {device.ip} - {index}",
                                },
                                warmup_delay=0,
                            )
                            grabber_list.append(grabber)
                continue

            for _ in range(
                100
            ):  # an arbitrarily high value so that we look for enough cameras, but this never becomes an infinite loop
                try:
                    config = {"input_type": input_type}
                    grabber = FrameGrabber.create_grabber(config, warmup_delay=0)
                    grabber_list.append(grabber)
                except (ValueError, ImportError):
                    # ValueError is taken to mean that we have reached the end of enumeration for the current input_type.
                    # ImportError means the requisite packages aren't installed for the current input_type.
                    # In both cases, it's time to move on to the next input_type.
                    break

        grabbers = FrameGrabber.grabbers_to_dict(grabber_list)

        # Do the warmup delay if necessary
        grabber_types = set([grabber.config.get_input_type() for grabber in grabbers.values()])
        if InputTypes.GENERIC_USB in grabber_types and warmup_delay > 0:
            logger.info(
                f"Waiting {warmup_delay} seconds for camera(s) to warm up. "
                "Pass in warmup_delay = 0 to suppress this behavior."
            )
            time.sleep(warmup_delay)

        return grabbers

    @abstractmethod
    def _grab_implementation(self) -> np.ndarray:
        """Each FrameGrabber must implement its own method of grabbing"""
        pass

    def grab(self) -> np.ndarray:
        """Read a frame from the camera and perform post processing operations such as zoom, crop and rotation if necessary.
        Returns a frame.
        """
        frame = self._grab_implementation()

        if frame is None:
            name = self.config.name  # all grabbers should have a name, either user-provided or generated
            error_msg = f"Failed to grab frame from {name}"
            raise GrabError(error_msg)

        # apply post processing operations
        frame = self._rotate(frame)
        frame = self._crop(frame)
        frame = self._digital_zoom(frame)
        return frame

    @abstractmethod
    def _default_name(self) -> str:
        raise NotImplementedError

    def _crop(self, frame: np.ndarray) -> np.ndarray:
        """Looks at FrameGrabber's options and decides to either crop by pixels or
        in a relative manner (normalized).

        Returns a cropped frame.
        """
        if self.config.crop:
            relative_crop_params = self.config.crop.get("relative")
            if relative_crop_params:
                return self._crop_relative(frame, relative_crop_params)

            pixel_crop_params = self.config.crop.get("pixels")
            if pixel_crop_params:
                return self._crop_pixels(frame, pixel_crop_params)

        return frame

    def _crop_pixels(self, frame: np.ndarray, crop_params: Dict[str, int]) -> np.ndarray:
        """Crops the provided frame according to the FrameGrabbers cropping configuration.
        Crops according to pixels positions.
        """
        top = int(crop_params.get("top", 0))
        bottom = int(crop_params.get("bottom", frame.shape[0]))
        left = int(crop_params.get("left", 0))
        right = int(crop_params.get("right", frame.shape[1]))
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
        digital_zoom = self.config.digital_zoom

        if digital_zoom is None:
            pass
        else:
            top = (frame.shape[0] - frame.shape[0] / digital_zoom) / 2
            bottom = frame.shape[0] - top
            left = (frame.shape[1] - frame.shape[1] / digital_zoom) / 2
            right = frame.shape[1] - left
            frame = frame[int(top) : int(bottom), int(left) : int(right)]

        return frame

    def _rotate(self, frame: np.ndarray) -> np.ndarray:
        """Rotates the provided frame a specified number of 90 degree rotations counterclockwise"""

        num_90_deg_rotations = self.config.num_90_deg_rotations

        if num_90_deg_rotations == 0:  # no rotation
            return frame

        # Calculate rotation
        if num_90_deg_rotations == 1:
            rotate_code = cv2.ROTATE_90_COUNTERCLOCKWISE
        elif num_90_deg_rotations == 2:
            rotate_code = cv2.ROTATE_180
        elif num_90_deg_rotations == 3:
            rotate_code = cv2.ROTATE_90_CLOCKWISE
        else:
            raise ValueError(
                f"Invalid value for num_90_deg_rotations. Got: {num_90_deg_rotations}. Expected 0, 1, 2, or 3."
            )

        # Apply the rotation
        return cv2.rotate(frame, rotate_code)

    def apply_options(self, options: dict) -> None:
        """Update generic options such as crop and zoom as well as
        camera-specific options.
        """
        framegrab_config_dict = type(self.config).to_framegrab_config_dict(self.config)
        framegrab_config_dict["options"] = options
        # this will validate the new options
        new_config = FrameGrabberConfig.from_framegrab_config_dict(framegrab_config_dict)
        self.config = new_config

    @abstractmethod
    def release() -> None:
        """A cleanup method. Releases/closes the video capture, terminates any related threads, etc."""
        pass

    def __enter__(self):
        """Context manager entry point."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit point that ensures proper resource cleanup."""
        self.release()
        return False  # re-raise any exceptions that occurred


class ROS2FrameGrabber(FrameGrabber):
    """
    Grabs frames from ROS 2 image topics
    """

    config_class = ROS2GrabberConfig

    def _initialize_grabber_implementation(self):
        topic = self.config.topic
        self._ros2_client = ROS2Client(topic)

    def _grab_implementation(self) -> np.ndarray:
        return self._ros2_client.grab()

    def _default_name(self) -> str:
        return f"ROS2 Topic {self.config.topic}"

    def release(self) -> None:
        self._ros2_client.release()


class FrameGrabberWithSerialNumber(FrameGrabber, ABC):
    def _default_name(self) -> str:
        if self.config.serial_number:
            unnamed_grabber_id = self.config.serial_number
        else:
            FrameGrabber.unnamed_grabber_count += 1
            unnamed_grabber_id = FrameGrabber.unnamed_grabber_count

        autogenerated_name = f"{unnamed_grabber_id} Camera - {unnamed_grabber_id}"
        return autogenerated_name


class GenericUSBFrameGrabber(FrameGrabberWithSerialNumber):
    """For any generic USB camera, such as a webcam"""

    # Buffer size constants for OpenCV capture
    BUFFER_SIZE_STREAMING = 2  # Larger buffer for better FPS when streaming
    BUFFER_SIZE_STANDARD = 1  # Small buffer to always get most recent frame

    # Frame read strategies for different modes
    FRAME_READS_STREAMING = 1  # Single read for minimal latency
    FRAME_READS_SNAPSHOT = 2  # Double read to ensure fresh frame

    # keep track of the cameras that are already in use so that we don't try to connect to them twice
    indices_in_use = set()

    config_class = GenericUSBFrameGrabberConfig

    def _initialize_grabber_implementation(self):
        serial_number = self.config.serial_number

        if serial_number and OPERATING_SYSTEM != "Linux":
            logger.warning(
                f"Matching USB cameras with serial_number is not supported on your operating system, {OPERATING_SYSTEM}. "
                "Cameras will be sequentially assigned instead."
            )

        # Find the serial number of connected cameras. Currently only works on Linux.
        if OPERATING_SYSTEM == "Linux":
            found_cams = GenericUSBFrameGrabber._find_cameras()
        else:
            found_cams = {}

        # Assign camera based on serial number if 1) serial_number was provided and 2) we know the
        # serial numbers of plugged in devices
        if found_cams:
            logger.debug(f"Found {len(found_cams)} USB cameras with Linux commands. Assigning camera by serial number.")
            for found_cam in found_cams:
                if serial_number and serial_number != found_cam["serial_number"]:
                    continue

                idx = found_cam["idx"]
                if idx in GenericUSBFrameGrabber.indices_in_use:
                    continue

                capture = self._connect_and_validate_capture(found_cam)
                if capture is not None:
                    break  # Found a valid capture

            else:
                raise ValueError(
                    f"Unable to find USB camera with the specified serial_number: {serial_number}. "
                    "Please ensure that the serial number is correct, that the camera is plugged in, and that the camera is not already in use."
                )
        # If we don't know the serial numbers of the cameras, just assign the next available camera by index
        else:
            logger.debug("No USB cameras found with Linux commands. Assigning camera by index.")
            for idx in range(20):  # an arbitrarily high number to make sure we check for enough cams
                if idx in GenericUSBFrameGrabber.indices_in_use:
                    continue  # Camera is already in use, moving on

                capture = cv2.VideoCapture(idx)
                if capture.isOpened():
                    break  # Found a valid capture

            else:
                raise ValueError("Unable to connect to USB camera by index. Is your camera plugged in?")

        # If a serial_number wasn't provided by the user, attempt to find it and add it to the config
        if not serial_number:
            for found_cam in found_cams:
                if idx == found_cam["idx"]:
                    self.config.serial_number = found_cam["serial_number"]
                    break

        # A valid capture has been found, saving it for later
        self.capture = capture

        # Log the current camera index as 'in use' to prevent other GenericUSBFrameGrabbers from stepping on it
        self.idx = idx
        GenericUSBFrameGrabber.indices_in_use.add(idx)

    def _has_ir_camera(self, camera_name: str) -> bool:
        """Check if the device contains an IR camera.

        Cameras such as the Logitech Brio contain an infrared camera for unlocking the computer with features
        such as Windows Hello. These cameras are not suitable for use in the context of most applications, so we will
        exclude them.
        """
        cameras_with_ir = ["logitech brio"]  # we can add to this list as we discover more cameras with IR
        for i in cameras_with_ir:
            if i in camera_name.lower():
                return True
        return False

    def _connect_and_validate_capture(self, camera_details: Dict[str, str]) -> Union[cv2.VideoCapture, None]:
        """Connect to the camera, check that it is open and not an IR camera.

        Return the camera if it is valid, otherwise return None.
        """
        idx = camera_details["idx"]
        camera_name = camera_details["camera_name"]
        device_path = camera_details["device_path"]

        capture = cv2.VideoCapture(idx)
        if not capture.isOpened():
            logger.warning(f"Could not open camera with index {idx}")
            return None

        has_ir_camera = self._has_ir_camera(camera_name)
        if has_ir_camera:
            ret, frame = capture.read()
            if not ret:
                logger.warning(f"Could not read frame from {device_path}")
                return None
            elif self._is_grayscale(frame):
                logger.info(
                    f"This device ({device_path}) is grayscale and therefore likely an IR camera. Skipping this camera."
                )
                capture.release()
                return None
            else:
                return capture
        else:
            return capture

    def _is_grayscale(self, frame: np.ndarray) -> bool:
        """Check if the provided frame is grayscale."""
        b, g, r = cv2.split(frame)

        return np.array_equal(b, g) and np.array_equal(g, r)

    def _grab_implementation(self) -> np.ndarray:
        if not self.capture.isOpened():
            self.capture.open(self.idx)

        # Video streaming mode: to minimize latency and maximize frame rate, we read only once to get the latest frame.
        # As long as the client repeatedly grabs frames, this will work fine.

        # Non streaming (snapshot) mode: OpenCV's internal buffer may return a stale frame on the first read,
        # so we read twice to ensure we get the most up-to-date image from the camera.
        iterations = self.FRAME_READS_STREAMING if self.config.video_stream else self.FRAME_READS_SNAPSHOT
        for _ in range(iterations):
            _, frame = self.capture.read()

        return frame

    def release(self) -> None:
        GenericUSBFrameGrabber.indices_in_use.remove(self.idx)
        self.capture.release()

    def apply_options(self, options: dict) -> None:
        super().apply_options(options)
        self._set_cv2_resolution()
        self._set_cv2_fourcc()
        self._set_cv2_buffersize()
        self._set_cv2_fps()

    @staticmethod
    def _run_system_command(command: str) -> str:
        """Runs a Linux system command and returns the stdout as a string."""
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout, _ = process.communicate()
        return stdout.decode("utf-8").strip()

    @staticmethod
    def _find_cameras() -> list:
        """Attempts to finds all USB cameras and returns a list dictionaries, each dictionary containing
        information about a camera, including: serial_number, device name, index, etc.
        This is useful for connecting the dots between user provided configurations
        and actual plugged in devices.

        This function only works on Linux, and was specifically tested on an Nvidia Jetson.
        """

        # ls /dev/video* returns device paths for all detected cameras
        command = "ls /dev/video*"
        output = GenericUSBFrameGrabber._run_system_command(command)

        if len(output) == 0:  # len is zero when no cameras are plugged in
            device_paths = []
        else:
            device_paths = output.split("\n")

        found_cams = []
        for device_path in device_paths:
            # ls -l /sys/class/video4linux/video0/device returns a path that points back into the /sys/bus/usb/devices/
            # directory where we can determine the serial number.
            # e.g. /sys/bus/usb/devices/2-3.2:1.0 -> /sys/bus/usb/devices/<bus>-<port>.<subport>:<config>.<interface>
            devname = device_path.split("/")[-1]
            command = f"ls -l /sys/class/video4linux/{devname}/device"
            output = GenericUSBFrameGrabber._run_system_command(command)
            bus_port_subport = output.split("/")[-1].split(":")[0]

            # find the serial number
            command = f"cat /sys/bus/usb/devices/{bus_port_subport}/serial"
            serial_number = GenericUSBFrameGrabber._run_system_command(command)

            # find the camera name (e.g. Logitech Brio)
            command = f"cat /sys/class/video4linux/{devname}/name"
            camera_name = GenericUSBFrameGrabber._run_system_command(command)

            # find the index
            idx = int(re.findall(r"\d+", devname)[-1])

            if serial_number:
                found_cams.append(
                    {
                        "serial_number": serial_number,
                        "device_path": device_path,
                        "idx": idx,
                        "camera_name": camera_name,
                    }
                )

        return found_cams

    def _set_cv2_resolution(self) -> None:
        """Set resolution from the config.

        No-op if width or height is None or unchanged. Resolution updates on
        cv2.VideoCapture can be slow, so only apply when different.
        """

        new_height = self.config.resolution_height
        new_width = self.config.resolution_width

        if new_width is None or new_height is None:
            return

        if new_width:
            current_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            if new_width != current_width:
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, new_width)
        if new_height:
            current_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if new_height != current_height:
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, new_height)

    def _set_cv2_fourcc(self) -> None:
        """Set FOURCC from the config.

        Do nothing if FOURCC is None or unchanged.
        """
        new_fourcc = self.config.fourcc

        # Check if the caller set a FOURCC
        new_fourcc_is_user_provided = new_fourcc is not None

        # Set to a default FOURCC if the user provided no FOURCC
        new_fourcc = new_fourcc if new_fourcc is not None else DEFAULT_FOURCC

        current_fourcc_str = self.capture.get(cv2.CAP_PROP_FOURCC)
        if new_fourcc != current_fourcc_str:
            fourcc_int = cv2.VideoWriter_fourcc(*new_fourcc)
            success = self.capture.set(cv2.CAP_PROP_FOURCC, fourcc_int)
            if not success:
                # Treat the failure to set FOURCC has an exception if the user explicitly requested a fourcc, otherwise just log it
                message = f"Failed to set FOURCC to '{new_fourcc}' for camera '{self.config.name}'. The camera might not support this setting."
                if new_fourcc_is_user_provided:
                    raise RuntimeError(message)
                else:
                    logger.warning(message)

    def _set_cv2_buffersize(self) -> None:
        """Set buffer size from the config (2 when streaming, else 1).

        Do nothing if unchanged. Raise RuntimeError if setting fails.
        """
        video_stream = self.config.video_stream

        # Determine desired buffer size based on video_stream setting
        if video_stream:
            desired_buffer_size = self.BUFFER_SIZE_STREAMING
        else:
            desired_buffer_size = self.BUFFER_SIZE_STANDARD

        # Check current buffer size
        current_buffer_size = int(self.capture.get(cv2.CAP_PROP_BUFFERSIZE))

        if desired_buffer_size != current_buffer_size:
            success = self.capture.set(cv2.CAP_PROP_BUFFERSIZE, desired_buffer_size)
            if not success:
                logger.warning(
                    f"Failed to set buffer size to {desired_buffer_size} for camera '{self.config.name}. The camera might not support this setting.'"
                )

    def _set_cv2_fps(self) -> None:
        """Set FPS from the config.

        Do nothing if FPS is None or unchanged. Raise RuntimeError if setting fails.
        """
        new_fps = self.config.fps

        # Check if the caller set an FPS
        new_fps_is_user_provided = new_fps is not None

        # Set to a default FPS if the user provided no FPS
        new_fps = new_fps if new_fps is not None else DEFAULT_FPS

        # Check current FPS
        current_fps = int(self.capture.get(cv2.CAP_PROP_FPS))

        if new_fps != current_fps:
            success = self.capture.set(cv2.CAP_PROP_FPS, new_fps)
            if not success:
                message = f"Failed to set FPS to {new_fps} for camera '{self.config.name}'. The camera might not support this setting."
                if new_fps_is_user_provided:
                    raise RuntimeError(message)
                else:
                    logger.warning(message)


class RTSPFrameGrabber(FrameGrabber):
    """Handles RTSP streams.

    Can operate in two modes based on the `keep_connection_open` configuration:
        1. If `true`, keeps the connection open for low-latency frame grabbing, but consumes more CPU.  (default)
        2. If `false`, opens the connection only when needed, which is slower but conserves resources.
    """

    config_class = RTSPFrameGrabberConfig

    def _initialize_grabber_implementation(self):
        self.lock = Lock()
        self.run = True
        self.config.keep_connection_open = self.config.keep_connection_open

        if self.config.keep_connection_open:
            self._open_connection()
            self._init_drain_thread()

    def _default_name(self) -> str:
        return self.config.rtsp_url

    def _open_connection(self):
        self.capture = cv2.VideoCapture(self.config.rtsp_url)
        if not self.capture.isOpened():
            raise ValueError(
                f"Could not open RTSP stream: {self.config.rtsp_url}. Is the RTSP URL correct? Is the camera connected to the network?"
            )
        logger.debug(f"Initialized video capture with backend={self.capture.getBackendName()}")

    def _close_connection(self):
        with self.lock:
            if self.capture is not None:
                self.capture.release()

    def _grab_implementation(self) -> np.ndarray:
        if not self.config.keep_connection_open:
            self._open_connection()
            try:
                return self._grab_open()
            finally:
                self._close_connection()
        else:
            return self._grab_open()

    def _grab_open(self) -> np.ndarray:
        with self.lock:
            ret, frame = self.capture.retrieve() if self.config.keep_connection_open else self.capture.read()
        if not ret:
            logger.error(f"Could not read frame from {self.capture}")
        return frame

    def release(self) -> None:
        if self.config.keep_connection_open:
            self.run = False  # to stop the buffer drain thread
            self._close_connection()

    def _init_drain_thread(self):
        if not self.config.keep_connection_open:
            return  # No need to drain if we're not keeping the connection open

        max_fps = self.config.max_fps
        self.drain_rate = 1 / max_fps
        thread = Thread(target=self._drain)
        thread.daemon = True
        thread.start()

    def _drain(self):
        while self.run:
            with self.lock:
                _ = self.capture.grab()
            time.sleep(self.drain_rate)


class BaslerFrameGrabber(FrameGrabberWithSerialNumber):
    """Basler USB and Basler GigE Cameras"""

    config_class = BaslerFrameGrabberConfig

    serial_numbers_in_use = set()

    def _initialize_grabber_implementation(self):
        # Basler cameras grab frames in different pixel formats, most of which cannot be displayed directly
        # by OpenCV. self.convert will convert them to BGR which can be used by OpenCV
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        tlf = pylon.TlFactory.GetInstance()
        devices = tlf.EnumerateDevices()

        if not devices:
            raise ValueError("No Basler cameras were found. Is your camera connected?")

        # Attempt to match the provided serial number with a plugged in device. If no serial number was provided, just
        # pick the first found device that is not currently in use.
        serial_number = self.config.serial_number
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

        # In case the serial_number wasn't provided by the user, add it to the config
        self.config.serial_number = curr_serial_number

        # A valid camera has been found, remember the serial_number to prevent
        # other FrameGrabbers from using it
        self.camera = camera
        BaslerFrameGrabber.serial_numbers_in_use.add(self.config.serial_number)

    def _grab_implementation(self) -> np.ndarray:
        with self.camera.GrabOne(2000) as result:
            if result.GrabSucceeded():
                # Convert the image to BGR for OpenCV
                image = self.converter.Convert(result)
                frame = image.GetArray()
            else:
                error_info = {
                    "ErrorCode": result.GetErrorCode(),
                    "PayloadSize": result.GetPayloadSize(),
                    "ID": result.GetID(),
                    "BlockID": result.GetBlockID(),
                    "Width": result.GetWidth(),
                    "Height": result.GetHeight(),
                    "PixelType": result.GetPixelType(),
                    "ErrorDescription": result.GetErrorDescription(),
                }

                error_message = "\n".join(f"{k}: {v}" for k, v in error_info.items())

                camera_name = self.config.name
                logger.warning(
                    f"Could not grab a frame from {camera_name}\n"
                    f"{error_message}\n"
                    f"---------------------------------------------------\n"
                )

                frame = NOISE

        return frame

    def release(self) -> None:
        BaslerFrameGrabber.serial_numbers_in_use.remove(self.config.serial_number)
        self.camera.Close()

    def apply_options(self, options: dict) -> None:
        super().apply_options(options)
        basler_options = self.config.basler_options or {}
        node_map = self.camera.GetNodeMap()
        for property_name, value in basler_options.items():
            node = node_map.GetNode(property_name)
            node.SetValue(value)


class RealSenseFrameGrabber(FrameGrabberWithSerialNumber):
    """Intel RealSense Depth Camera"""

    config_class = RealSenseFrameGrabberConfig

    def _initialize_grabber_implementation(self):
        ctx = rs.context()
        if len(ctx.devices) == 0:
            raise ValueError("No Intel RealSense cameras detected. Is your camera plugged in?")

        provided_serial_number = self.config.serial_number

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
                except RuntimeError:
                    # The current camera is not available, moving on to the next
                    continue
        else:
            raise ValueError(
                f"Unable to connect to Intel RealSense camera with serial_number: {provided_serial_number}. "
                "Is the serial number correct? Is the camera plugged in?"
            )

        # A valid pipeline was found, save the pipeline and RealSense config for later
        self.pipeline = pipeline
        self.rs_config = rs_config

        # In case the serial_number wasn't provided by the user, add it to the config
        self.config.serial_number = curr_serial_number

    def _grab_implementation(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()

        # Convert color images to numpy arrays and convert from RGB to BGR
        color_frame = frames.get_color_frame()
        color_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)

        # If side_by_side is enabled, get a depth frame and horizontally stack it with color frame
        display_side_by_side = self.config.side_by_side_depth
        if display_side_by_side:
            depth_frame = frames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            return self._horizontally_stack(depth_image, color_image)
        else:
            return color_image

    def _horizontally_stack(self, depth_image: np.ndarray, color_image: np.ndarray) -> np.ndarray:
        """Merges color image and depth image into a wider image, all in RGB"""

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_BONE)
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(
                color_image,
                dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                interpolation=cv2.INTER_AREA,
            )
            sidebyside = np.hstack((resized_color_image, depth_colormap))
        else:
            sidebyside = np.hstack((color_image, depth_colormap))
        return sidebyside

    def release(self) -> None:
        self.pipeline.stop()

    def apply_options(self, options: dict) -> None:
        """Some special handling for changing the resolution of Intel RealSense cameras"""
        old_width = self.config.resolution_width
        old_height = self.config.resolution_height

        super().apply_options(options)

        new_width = self.config.resolution_width
        new_height = self.config.resolution_height

        if (new_width and new_height) and (new_width != old_width or new_height != old_height):
            self.pipeline.stop()  # pipeline needs to be temporarily stopped in order to change the resolution
            self.rs_config.enable_stream(rs.stream.color, new_width, new_height)
            self.rs_config.enable_stream(rs.stream.depth, new_width, new_height)
            self.pipeline.start(self.rs_config)  # Restart the pipeline with the new configuration


class RaspberryPiCSI2FrameGrabber(FrameGrabberWithSerialNumber):
    "For CSI2 cameras connected to Raspberry Pis through their dedicated camera port"

    config_class = RaspberryPiCSI2FrameGrabberConfig

    def _initialize_grabber_implementation(self):
        # This will also detect USB cameras, but according to the documentation CSI2
        # cameras attached to the dedicated camera port will always come before USB
        # cameras in the resulting list of camera dictionaries
        # https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf#page=66.21
        cameras = Picamera2.global_camera_info()

        if not cameras:
            raise ValueError("No CSI2 cameras were found. Is your camera connected?")

        # Since global_camera_info() also finds USB cameras, we will only use the first
        # entry. USB cameras must be found through their specific FrameGrabber. Note
        # that only a single CSI2 camera is supported at this time.
        picam2 = Picamera2()
        picam2.configure(picam2.create_still_configuration())
        picam2.start()
        logger.info(f"Connected to Raspberry Pi CSI2 camera with id {(cameras[0])['Id']}")

        self.camera = picam2

    def _grab_implementation(self) -> np.ndarray:
        frame = self.camera.capture_array()

        # Convert to BGR for opencv
        frame = cv2.cvtColor(np.asanyarray(frame), cv2.COLOR_BGR2RGB)

        return frame

    def release(self) -> None:
        self.camera.close()


class HttpLiveStreamingFrameGrabber(FrameGrabber):
    """Handles Http Live Streaming (HLS)

    Supports two modes:
    1. Keep connection open (default): Opens the connection once and keeps it open for high-fps frame grabbing.
    2. Open connection on every frame: Opens and closes the connection on every captured frame, which conserves
        both CPU and network bandwidth but has higher latency. In practice, roughly 1FPS is achievable with this strategy.
    """

    config_class = HttpLiveStreamingFrameGrabberConfig

    def _initialize_grabber_implementation(self):
        self.type = "HLS"
        self.lock = Lock()

        if self.config.keep_connection_open:
            self._open_connection()

    def _default_name(self) -> str:
        return self.config.hls_url

    def _open_connection(self):
        self.capture = cv2.VideoCapture(self.config.hls_url)
        if not self.capture.isOpened():
            raise ValueError(f"Could not open {self.type} stream: {self.config.hls_url}. Is the HLS URL correct?")
        logger.warning(f"Initialized video capture with backend={self.capture.getBackendName()}")

    def _close_connection(self):
        logger.warning(f"Closing connection to {self.type} stream")
        with self.lock:
            if self.capture is not None:
                self.capture.release()

    def _grab_implementation(self) -> np.ndarray:
        if not self.config.keep_connection_open:
            self._open_connection()
            try:
                return self._grab_open()
            finally:
                self._close_connection()
        else:
            return self._grab_open()

    def _grab_open(self) -> np.ndarray:
        with self.lock:
            ret, frame = self.capture.read()
        if not ret:
            logger.error(f"Could not read frame from {self.capture}")
        return frame

    def release(self) -> None:
        if self.config.keep_connection_open:
            self._close_connection()


class YouTubeLiveFrameGrabber(HttpLiveStreamingFrameGrabber):
    """Grabs the most recent frame from a YouTube Live stream (which are HLS streams under the hood)

    Supports two modes:
    1. Keep connection open (default): Opens the connection once and keeps it open for high-fps frame grabbing.
    2. Open connection on every frame: Opens and closes the connection on every captured frame, which conserves
        both CPU and network bandwidth but has higher latency. In practice, roughly 1FPS is achievable with this strategy.
    """

    config_class = YouTubeLiveFrameGrabberConfig

    def _initialize_grabber_implementation(self):
        self.type = "YouTube Live"

        self.lock = Lock()

        if self.config.keep_connection_open:
            self._open_connection()

    def _default_name(self) -> str:
        return self.config.youtube_url


class FileStreamFrameGrabber(FrameGrabber):
    """Grabs frames from a video file stream, such as an MP4 or MOV file.

    Supports dropping frames to achieve a target FPS.

    Some of the supported formats: mp4, avi, mov, mjpeg, mkv, webm

    Configuration:
        id:
            filename: str  # Path to the video file
        options:
            max_fps: float  # Target frame rate (optional, defaults to source fps)

    Example:
        config = {
            "id": {"filename": "video.mp4"},
            "options": {"max_fps": 15.0}
        }
        grabber = FileStreamFrameGrabber(config)
    """

    config_class = FileStreamFrameGrabberConfig

    def _initialize_grabber_implementation(self):
        self.remainder = 0.0

        self.capture = cv2.VideoCapture(self.config.filename)
        if not self.capture.isOpened():
            raise ValueError(f"Could not open file {self.config.filename}. Is it a valid video file?")
        backend = self.capture.getBackendName()
        logger.debug(f"Initialized video capture with {backend=}")

        ret, _ = self.capture.read()
        if not ret:
            self.capture.release()
            raise ValueError(f"Could not read first frame of file {self.config.filename}. Is it a valid video file?")

        # Reset frame position back to the first frame after validation
        self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

        self.fps_source = round(self.capture.get(cv2.CAP_PROP_FPS), 2)
        if self.fps_source <= 0.1:
            logger.warning(f"Captured framerate is very low or zero: {self.fps_source} FPS")

        self.should_drop_frames = self.config.max_fps > 0 and self.config.max_fps < self.fps_source
        logger.debug(
            f"Source FPS: {self.fps_source}, Target FPS: {self.config.max_fps}, Drop Frames: {self.should_drop_frames}"
        )

    def _default_name(self) -> str:
        return self.config.filename

    def _grab_implementation(self) -> np.ndarray:
        """Grab a frame from the video file, decimating if needed to match target FPS.

        Returns:
            np.ndarray: The captured frame

        Raises:
            RuntimeWarning: If frame cannot be read, likely end of file
        """
        if self.should_drop_frames:
            self._drop_frames()

        ret, frame = self.capture.read()
        if not ret:
            raise RuntimeWarning("Could not read frame from video file. Possible end of file.")
        return frame

    def _drop_frames(self) -> None:
        """Drop frames to achieve target frame rate using frame position seeking."""
        drop_frames = (self.fps_source / self.config.max_fps) - 1 + self.remainder
        frames_to_drop = round(drop_frames)

        if frames_to_drop > 0:
            current_pos = self.capture.get(cv2.CAP_PROP_POS_FRAMES)
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, current_pos + frames_to_drop)

        self.remainder = round(drop_frames - frames_to_drop, 2)
        logger.debug(f"Dropped {frames_to_drop} frames to meet {self.config.max_fps} FPS target")

    def get_next_frame_number(self) -> int:
        """Return the next frame number that will be read (current position in file)"""
        return int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))

    def get_last_frame_read_number(self) -> int:
        """Return the frame number of the last frame read"""
        current_pos = self.get_next_frame_number()
        if current_pos:
            return self.get_next_frame_number() - 1
        else:
            return None

    def release(self) -> None:
        """Release the video capture resources."""
        if hasattr(self, "capture"):
            self.capture.release()


class MockFrameGrabber(FrameGrabberWithSerialNumber):
    """A mock camera class for testing purposes"""

    # Represents the serial numbers of the mock cameras that are discoverable
    available_serial_numbers = ("123", "456", "789")

    # Keeps track of the available serial numbers so that we don't try to connect to them twice
    serial_numbers_in_use = set()

    config_class = MockFrameGrabberConfig

    def _initialize_grabber_implementation(self):
        provided_serial_number = self.config.serial_number

        # Iterate through each detected camera and attempt to match it with the provided camera config
        for curr_serial_number in MockFrameGrabber.available_serial_numbers:
            if curr_serial_number in MockFrameGrabber.serial_numbers_in_use:
                continue  # this camera is already in use, moving on to the next
            if provided_serial_number is None or curr_serial_number == provided_serial_number:
                break  # succesfully connected, breaking out of loop
        else:
            raise ValueError(
                f"Unable to connect to MockFrameGrabber with serial_number: {provided_serial_number}. "
                f"Is the serial number correct? Available serial numbers are {MockFrameGrabber.available_serial_numbers}"
            )

        MockFrameGrabber.serial_numbers_in_use.add(curr_serial_number)

        # In case the serial_number wasn't provided by the user, add it to the config
        self.config.serial_number = curr_serial_number

    def _grab_implementation(self) -> np.ndarray:
        width = self.config.resolution_width or 640
        height = self.config.resolution_height or 480

        return np.zeros((height, width, 3), dtype=np.uint8)

    def release(self) -> None:
        MockFrameGrabber.serial_numbers_in_use.remove(self.config.serial_number)
