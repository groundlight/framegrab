import logging
import os
import platform
import re
import subprocess
import time
from abc import ABC, abstractmethod
from threading import Lock, Thread
from typing import Dict, List, Optional, Union
from urllib.parse import urlparse

import cv2
import numpy as np
import yaml

from .unavailable_module import UnavailableModule

logger = logging.getLogger(__name__)

# Optional imports
try:
    from pypylon import pylon
except ImportError as e:
    pylon = UnavailableModule(e)

try:
    from pyrealsense2 import pyrealsense2 as rs
except ImportError as e:
    rs = UnavailableModule(e)

OPERATING_SYSTEM = platform.system()
DIGITAL_ZOOM_MAX = 4
NOISE = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)  # in case a camera can't get a frame


class InputTypes:
    """Defines the available input types from FrameGrabber objects"""

    GENERIC_USB = "generic_usb"
    RTSP = "rtsp"
    REALSENSE = "realsense"
    BASLER = "basler"
    MOCK = "mock"

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
    def create_grabbers(configs: List[dict], warmup_delay: float = 1.0) -> Dict[str, "FrameGrabber"]:
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

        # Sort the configs such that configs with serial numbers appear first
        # This will ensure that they are able to connect to the camera with the specified
        # serial number, and that no other FrameGrabbers claim that camera first.
        configs.sort(key=lambda config: "serial_number" not in config.get("id", {}))

        # Do not allow duplicate camera names
        names = [config.get("name", None) for config in configs if config.get("name", None) is not None]
        if len(names) != len(set(names)):
            raise ValueError(
                f"Duplicate camera names were provided in configurations. Please ensure that each camera name is unique. "
                f"Provided camera names: {names}"
            )

        # Create the grabbers
        grabber_list = []
        for config in configs:
            grabber = FrameGrabber.create_grabber(config, autogenerate_name=False, warmup_delay=0)
            grabber_list.append(grabber)

        grabbers = FrameGrabber.grabbers_to_dict(grabber_list)

        # Do the warmup delay if necessary
        grabber_types = set([grabber.config["input_type"] for grabber in grabbers.values()])
        if InputTypes.GENERIC_USB in grabber_types and warmup_delay > 0:
            logger.info(
                f"Waiting {warmup_delay} seconds for camera(s) to warm up. "
                "Pass in warmup_delay = 0 to suppress this behavior."
            )
            time.sleep(warmup_delay)

        return grabbers

    @staticmethod
    def from_yaml(filename: Optional[str] = None, yaml_str: Optional[str] = None) -> List["FrameGrabber"]:
        """Creates multiple FrameGrab objects based on a YAML file or YAML string.
        Either filename or yaml_str must be provided, but not both.

        :param filename: The filename of the YAML file to load.
        :param yaml_str: A YAML string to parse.
        :return: A dictionary where the keys are the camera names, and the values are FrameGrabber objects.
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
        grabber_list = sorted(
            grabber_list,
            key=lambda grabber: grabber.config.get("id", {}).get("serial_number", ""),
        )

        # Create the grabbers dictionary, autogenerating names for any unnamed grabbers
        grabbers = {}
        for grabber in grabber_list:
            # If a name wasn't provided, autogenerate one
            if not grabber.config.get("name"):
                grabber._autogenerate_name()

            # Add the grabber to the dictionary
            grabber_name = grabber.config["name"]
            grabbers[grabber_name] = grabber

        return grabbers

    @staticmethod
    def create_grabber_yaml(yaml_config: str, autogenerate_name: bool = True, warmup_delay: float = 1.0):
        """Create a FrameGrabber object based on the provided configuration.

        Parameters:
            config (str): A yaml string containing configuration settings for the FrameGrabber.

            autogenerate_name (bool, optional): A flag to indicate whether to automatically
                generate a name for the FrameGrabber object if not explicitly provided. Defaults
                to True.

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
        grabber = FrameGrabber.create_grabber(config, autogenerate_name, warmup_delay)
        return grabber

    @staticmethod
    def create_grabber(config: dict, autogenerate_name: bool = True, warmup_delay: float = 1.0):
        """Create a FrameGrabber object based on the provided configuration.

        Parameters:
            config (dict): A dictionary containing configuration settings for the FrameGrabber.

            autogenerate_name (bool, optional): A flag to indicate whether to automatically
                generate a name for the FrameGrabber object if not explicitly provided. Defaults
                to True.

            warmup_delay (float, optional): The number of seconds to wait after creating the grabbers. USB
                cameras often need a moment to warm up before they can be used; grabbing frames too early
                might result in dark or blurry images.
                Defaults to 1.0. Only applicable to generic_usb cameras.

        Returns:
                An instance of a FrameGrabber subclass based on the provided
                configuration. The specific subclass will be determined by the content of the
                configuration dictionary.

        """

        # Ensure the config is properly constructed and typed
        config = FrameGrabber._validate_config(config)

        # At a minimum, input_type must be provided
        input_type = config.get("input_type", None)
        if input_type is None:
            raise ValueError(f"No input_type provided. Valid types are {InputTypes.get_options()}")

        # Based on input_type, create correct type of FrameGrabber
        if input_type == InputTypes.GENERIC_USB:
            grabber = GenericUSBFrameGrabber(config)
        elif input_type == InputTypes.RTSP:
            grabber = RTSPFrameGrabber(config)
        elif input_type == InputTypes.BASLER:
            grabber = BaslerFrameGrabber(config)
        elif input_type == InputTypes.REALSENSE:
            grabber = RealSenseFrameGrabber(config)
        elif input_type == InputTypes.MOCK:
            grabber = MockFrameGrabber(config)
        else:
            raise ValueError(
                f"The provided input_type ({input_type}) is not valid. Valid types are {InputTypes.get_options()}"
            )

        # If a name wasn't supplied and autogenerate_name is True, autogenerate a name
        if not config.get("name", False) and autogenerate_name:
            grabber._autogenerate_name()

        # Apply the options so that resolution, exposure, etc. are correct
        grabber.apply_options(config["options"])

        # Do the warmup delay if necessary
        if config["input_type"] == InputTypes.GENERIC_USB and warmup_delay > 0:
            logger.info(
                f"Waiting {warmup_delay} seconds for camera to warm up. "
                "Pass in warmup_delay = 0 to suppress this behavior."
            )
            time.sleep(warmup_delay)

        return grabber

    @staticmethod
    def autodiscover(warmup_delay: float = 1.0) -> dict:
        """Autodiscovers cameras and returns a dictionary of FrameGrabber objects

        warmup_delay (float, optional): The number of seconds to wait after creating the grabbers. USB
            cameras often need a moment to warm up before they can be used; grabbing frames too early
            might result in dark or blurry images.
            Defaults to 1.0. Only happens if there are any generic_usb cameras in the config list.
        """
        autodiscoverable_input_types = (
            InputTypes.REALSENSE,
            InputTypes.GENERIC_USB,
            InputTypes.BASLER,
        )

        # Autodiscover the grabbers
        grabber_list = []
        for input_type in autodiscoverable_input_types:
            logger.info(f"Autodiscovering {input_type} cameras...")
            for _ in range(
                100
            ):  # an arbitrarily high value so that we look for enough cameras, but this never becomes an infinite loop
                try:
                    config = {"input_type": input_type}
                    grabber = FrameGrabber.create_grabber(config, autogenerate_name=False, warmup_delay=0)
                    grabber_list.append(grabber)
                except (ValueError, ImportError):
                    # ValueError is taken to mean that we have reached the end of enumeration for the current input_type.
                    # ImportError means the requisite packages aren't installed for the current input_type.
                    # In both cases, it's time to move on to the next input_type.
                    break

        grabbers = FrameGrabber.grabbers_to_dict(grabber_list)

        # Do the warmup delay if necessary
        grabber_types = set([grabber.config["input_type"] for grabber in grabbers.values()])
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

        # apply post processing operations
        frame = self._crop(frame)
        frame = self._digital_zoom(frame)
        frame = self._rotate(frame)
        return frame

    def _autogenerate_name(self) -> None:
        """For generating and assigning unique names for unnamed FrameGrabber objects.

        Attempts to incorporate a unique identifier (serial number, url, etc.) into each
        camera name. If no unique identifier is available, a counter is used instead.
        """

        if self.config.get("id", {}).get("serial_number", None):
            unnamed_grabber_id = self.config["id"]["serial_number"]
        elif self.config.get("id", {}).get("rtsp_url", None):
            rtsp_url = self.config["id"]["rtsp_url"]
            parsed_url = urlparse(rtsp_url)
            if parsed_url.scheme == "rtsp" and parsed_url.hostname:
                unnamed_grabber_id = parsed_url.hostname
            else:
                raise ValueError("Invalid RTSP URL format")
        else:
            FrameGrabber.unnamed_grabber_count += 1
            unnamed_grabber_id = FrameGrabber.unnamed_grabber_count

        input_type = self.config["input_type"]
        autogenerated_name = f"{input_type.upper()} Camera - {unnamed_grabber_id}"
        self.config["name"] = autogenerated_name

    def _crop(self, frame: np.ndarray) -> np.ndarray:
        """Looks at FrameGrabber's options and decides to either crop by pixels or
        in a relative manner (normalized).

        Returns a cropped frame.
        """
        options = self.config.get("options", {})

        relative_crop_params = options.get("crop", {}).get("relative")
        if relative_crop_params:
            return self._crop_relative(frame, relative_crop_params)

        pixel_crop_params = options.get("crop", {}).get("pixels")
        if pixel_crop_params:
            return self._crop_pixels(frame, pixel_crop_params)

        return frame

    def _crop_pixels(self, frame: np.ndarray, crop_params: Dict[str, int]) -> np.ndarray:
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
        else:
            top = (frame.shape[0] - frame.shape[0] / digital_zoom) / 2
            bottom = frame.shape[0] - top
            left = (frame.shape[1] - frame.shape[1] / digital_zoom) / 2
            right = frame.shape[1] - left
            frame = frame[int(top) : int(bottom), int(left) : int(right)]

        return frame

    def _rotate(self, frame: np.ndarray) -> np.ndarray:
        """Rotates the provided frame a specified number of 90 degree rotations clockwise"""

        num_90_deg_rotations = self.config.get("options", {}).get("num_90_deg_rotations", 0)

        for n in range(num_90_deg_rotations):
            frame = np.rot90(frame)

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

    def apply_options(self, options: dict) -> None:
        """Update generic options such as crop and zoom as well as
        camera-specific options.
        """

        # Ensure that the user hasn't provided pixel cropping parameters and relative cropping parameters
        pixel_crop_params = options.get("crop", {}).get("pixels", {})
        relative_crop_params = options.get("crop", {}).get("relative", {})
        if pixel_crop_params and relative_crop_params:
            camera_name = self.config.get("name", "Unnamed Camera")
            raise ValueError(
                f"Pixel cropping parameters and relative cropping parameters were set for "
                f"{camera_name}. Pixel cropping and relative cropping cannot be "
                f"used together. Please adjust your configurations to use one or the other."
            )

        # Ensure valid relative cropping parameters (between 0 and 1)
        for param_name, param_value in relative_crop_params.items():
            if param_value < 0 or param_value > 1:
                camera_name = self.config.get("name", "Unnamed Camera")
                raise ValueError(
                    f"Relative cropping parameter ({param_name}) on {camera_name} is {param_value}, which is invalid. "
                    f"Relative cropping parameters must be between 0 and 1, where 1 represents the full "
                    f"width or length of the image. "
                )

        # Validate digital zoom level
        digital_zoom = options.get("zoom", {}).get("digital")
        if digital_zoom and (digital_zoom < 1 or digital_zoom > DIGITAL_ZOOM_MAX):
            raise ValueError(
                f"Invalid value for digital_zoom ({digital_zoom}). "
                f"Digital zoom must >= 1 and <= {DIGITAL_ZOOM_MAX}."
            )

        # Apply camera specific options
        self._apply_camera_specific_options(options)

        # Save the options to the config
        self.config["options"] = options

    @abstractmethod
    def _apply_camera_specific_options(options: dict) -> None:
        """Update any camera-specific options, such as resolution, exposure_us, pixel_format, etc."""
        pass

    @abstractmethod
    def release() -> None:
        """A cleanup method. Releases/closes the video capture, terminates any related threads, etc."""
        pass


class GenericUSBFrameGrabber(FrameGrabber):
    """For any generic USB camera, such as a webcam"""

    # keep track of the cameras that are already in use so that we don't try to connect to them twice
    indices_in_use = set()

    def __init__(self, config: dict):
        self.config = config

        serial_number = self.config.get("id", {}).get("serial_number")

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
                    self.config["id"] = {"serial_number": found_cam["serial_number"]}
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

        # OpenCV VideoCapture buffers frames by default. It's usually not possible to turn buffering off.
        # Buffer can be set as low as 1, but even still, if we simply read once, we will get the buffered (stale) frame.
        # Assuming buffer size of 1, we need to read twice to get the current frame.
        for _ in range(2):
            _, frame = self.capture.read()

        return frame

    def release(self) -> None:
        GenericUSBFrameGrabber.indices_in_use.remove(self.idx)
        self.capture.release()

    def _apply_camera_specific_options(self, options: dict) -> None:
        self._set_cv2_resolution()

        # set the buffer size to 1 to always get the most recent frame
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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


class RTSPFrameGrabber(FrameGrabber):
    """Handles RTSP streams. Can operate in two modes based on the `keep_connection_open` configuration:
    1.  If `true`, keeps the connection open for low-latency frame grabbing, but consumes more CPU.  (default)
    2. If `false`, opens the connection only when needed, which is slower but conserves resources.
    """

    def __init__(self, config: dict):
        rtsp_url = config.get("id", {}).get("rtsp_url")
        if not rtsp_url:
            camera_name = config.get("name", "Unnamed RTSP Stream")
            raise ValueError(
                f"No RTSP URL provided for {camera_name}. Please add an rtsp_url attribute to the config under id."
            )

        self.config = RTSPFrameGrabber._substitute_rtsp_password(config)
        self.rtsp_url = self.config["id"]["rtsp_url"]

        self.lock = Lock()
        self.run = True
        self.keep_connection_open = config.get("options", {}).get("keep_connection_open", True)

        if self.keep_connection_open:
            self._open_connection()
            self._init_drain_thread()

    @staticmethod
    def _substitute_rtsp_password(config: dict) -> dict:
        """
        Substitutes the password placeholder in the rtsp_url with the actual password
        from an environment variable.
        The URL should take this format
            Ex: rtsp://admin:{{MY_PASSWORD}}@10.0.0.0/cam/realmonitor?channel=1&subtype=0
        This function looks for an all-uppercase name between {{ and }} to find an environment
        variable with that name. If the environment variable is found, its value will be
        substituted in the rtsp_url.
        NOTE: This can also work for multiple RTSP URLs in the same config file as long
            as each one has a unique password placeholder.
        """
        pattern = r"\{\{([A-Z_][A-Z0-9_]*?)\}\}"
        rtsp_url = config.get("id", {}).get("rtsp_url", "")
        matches = re.findall(pattern, rtsp_url)

        if len(matches) == 0:
            return config  # make no change to config if no password placeholder is found
        elif len(matches) > 1:
            raise ValueError("RTSP URL should contain no more than one placeholder for the password.")

        match = matches[0]
        password_env_var = os.environ.get(match)
        if not password_env_var:
            raise ValueError(f"RTSP URL {rtsp_url} references environment variable {match} which is not set")

        placeholder = "{{" + match + "}}"
        rtsp_url = rtsp_url.replace(placeholder, password_env_var)
        config["id"]["rtsp_url"] = rtsp_url

        return config

    def _apply_camera_specific_options(self, options: dict) -> None:
        if options.get("resolution"):
            camera_name = self.config.get("name", "Unnamed RTSP Stream")
            raise ValueError(f"Resolution was set for {camera_name}, but resolution cannot be set for RTSP streams.")

    def _open_connection(self):
        self.capture = cv2.VideoCapture(self.rtsp_url)
        if not self.capture.isOpened():
            raise ValueError(
                f"Could not open RTSP stream: {self.rtsp_url}. Is the RTSP URL correct? Is the camera connected to the network?"
            )
        logger.debug(f"Initialized video capture with backend={self.capture.getBackendName()}")

    def _close_connection(self):
        with self.lock:
            if self.capture is not None:
                self.capture.release()

    def _grab_implementation(self) -> np.ndarray:
        if not self.keep_connection_open:
            self._open_connection()
            try:
                return self._grab_open()
            finally:
                self._close_connection()
        else:
            return self._grab_open()

    def _grab_open(self) -> np.ndarray:
        with self.lock:
            ret, frame = self.capture.retrieve() if self.keep_connection_open else self.capture.read()
        if not ret:
            logger.error(f"Could not read frame from {self.capture}")
        return frame

    def release(self) -> None:
        if self.keep_connection_open:
            self.run = False  # to stop the buffer drain thread
            self._close_connection()

    def _init_drain_thread(self):
        if not self.keep_connection_open:
            return  # No need to drain if we're not keeping the connection open

        max_fps = self.config.get("options", {}).get("max_fps", 30)
        self.drain_rate = 1 / max_fps
        thread = Thread(target=self._drain)
        thread.daemon = True
        thread.start()

    def _drain(self):
        while self.run:
            with self.lock:
                _ = self.capture.grab()
            time.sleep(self.drain_rate)


class BaslerFrameGrabber(FrameGrabber):
    """Basler USB and Basler GigE Cameras"""

    serial_numbers_in_use = set()

    def __init__(self, config: dict):
        self.config = config

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

        # In case the serial_number wasn't provided by the user, add it to the config
        self.config["id"] = {"serial_number": curr_serial_number}

        # A valid camera has been found, remember the serial_number to prevent
        # other FrameGrabbers from using it
        self.camera = camera
        BaslerFrameGrabber.serial_numbers_in_use.add(self.config["id"]["serial_number"])

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

                camera_name = self.config.get("name", "Unnamed Basler Camera")
                logger.warning(
                    f"Could not grab a frame from {camera_name}\n"
                    f"{error_message}\n"
                    f"---------------------------------------------------\n"
                )

                frame = NOISE

        return frame

    def release(self) -> None:
        BaslerFrameGrabber.serial_numbers_in_use.remove(self.config["id"]["serial_number"])
        self.camera.Close()

    def _apply_camera_specific_options(self, options: dict) -> None:
        if options.get("resolution"):
            raise ValueError("FrameGrab does not support setting resolution on Basler cameras.")

        basler_options = options.get("basler", {})
        node_map = self.camera.GetNodeMap()
        for property_name, value in basler_options.items():
            node = node_map.GetNode(property_name)
            node.SetValue(value)


class RealSenseFrameGrabber(FrameGrabber):
    """Intel RealSense Depth Camera"""

    def __init__(self, config: dict):
        self.config = config

        ctx = rs.context()
        if len(ctx.devices) == 0:
            raise ValueError("No Intel RealSense cameras detected. Is your camera plugged in?")

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
        self.config["id"] = {"serial_number": curr_serial_number}

    def _grab_implementation(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()

        # Convert color images to numpy arrays and convert from RGB to BGR
        color_frame = frames.get_color_frame()
        color_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)

        # If side_by_side is enabled, get a depth frame and horizontally stack it with color frame
        display_side_by_side = self.config.get("options", {}).get("depth", {}).get("side_by_side")
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

    def _apply_camera_specific_options(self, options: dict) -> None:
        # Some special handling for changing the resolution of Intel RealSense cameras
        new_width = options.get("resolution", {}).get("width")
        new_height = options.get("resolution", {}).get("height")
        if (new_width and not new_height) or (not new_width and new_height):
            camera_name = self.config.get("name", "Unnamed RealSense Camera")
            raise ValueError(
                f"Invalid resolution settings for {camera_name}. Please provide both a width and a height."
            )
        elif new_width and new_height:
            self.pipeline.stop()  # pipeline needs to be temporarily stopped in order to change the resolution
            self.rs_config.enable_stream(rs.stream.color, new_width, new_height)
            self.rs_config.enable_stream(rs.stream.depth, new_width, new_height)
            self.pipeline.start(self.rs_config)  # Restart the pipeline with the new configuration
        else:
            # If the user didn't provide a resolution, do nothing
            pass


class MockFrameGrabber(FrameGrabber):
    """A mock camera class for testing purposes"""

    # Represents the serial numbers of the mock cameras that are discoverable
    available_serial_numbers = ("123", "456", "789")

    # Keeps track of the available serial numbers so that we don't try to connect to them twice
    serial_numbers_in_use = set()

    def __init__(self, config: dict):
        self.config = config

        provided_serial_number = self.config.get("id", {}).get("serial_number")

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
        self.config["id"] = {"serial_number": curr_serial_number}

    def _grab_implementation(self) -> np.ndarray:
        width = self.config.get("options", {}).get("resolution", {}).get("width", 640)
        height = self.config.get("options", {}).get("resolution", {}).get("height", 480)

        return np.zeros((height, width, 3), dtype=np.uint8)

    def release(self) -> None:
        MockFrameGrabber.serial_numbers_in_use.remove(self.config["id"]["serial_number"])

    def _apply_camera_specific_options(self, options: dict) -> None:
        pass  # no action necessary for mock cameras
