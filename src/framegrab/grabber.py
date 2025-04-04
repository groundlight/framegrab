# TODO:
# - make other tests pass
# - Reevalute _apply_camera_specific_options, apply options stuff
# - Figure out serializtion
# - check duplicate serial number logic
# - check about serial numbers in use and available serial numbers
# - camera_name -> name, review logic for  (add tests?)
# - does to_dict need to be dict?
# - sort imports
# - add actual test for youtube grabber
# - does rtsp have the fps thing?
# - double check parameters, make sure the options in the test are serialized correctly
# - ensure all coremethods are run in tests
# - make pydantic errors pretty?
# - test rasperry pi

# Things updated:
# - _apply_camera_specific_options I think breaks some python best practices. We should use inheritence
#   to handle camera specific options instead. So I updated that.

import copy
import logging
import os
import pdb
import platform
import re
import subprocess
import time
from abc import ABC, abstractmethod
from threading import Lock, Thread
from typing import ClassVar, Dict, List, Optional, Union
from urllib.parse import urlparse

import cv2
import numpy as np
import yaml
from pydantic import BaseModel, Field, PrivateAttr, confloat, validator

from .exceptions import GrabError
from .rtsp_discovery import AutodiscoverMode, RTSPDiscovery
from .unavailable_module import UnavailableModule

# -- Optional imports --
# Only used for Basler cameras, not required otherwise
try:
    from pypylon import pylon
except (ImportError, ModuleNotFoundError) as e:
    pylon = UnavailableModule(e)

# Only used for RealSense cameras, not required otherwise
try:
    from pyrealsense2 import pyrealsense2 as rs
except (ImportError, ModuleNotFoundError) as e:
    rs = UnavailableModule(e)

# Only used for CSI2 cameras with Raspberry Pi, not required otherwise
try:
    from picamera2 import Picamera2
except (ImportError, ModuleNotFoundError) as e:
    Picamera2 = UnavailableModule(e)

# Only used for Youtube Live streams, not required otherwise
try:
    import streamlink
except ImportError as e:
    streamlink = UnavailableModule(e)

logger = logging.getLogger(__name__)

OPERATING_SYSTEM = platform.system()
DIGITAL_ZOOM_MAX = 4
NOISE = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)  # in case a camera can't get a frame


class InputTypes:
    """Defines the available input types from FrameGrabber objects"""

    GENERIC_USB = "generic_usb"
    RTSP = "rtsp"
    REALSENSE = "realsense"
    BASLER = "basler"
    RPI_CSI2 = "rpi_csi2"
    HLS = "hls"
    YOUTUBE_LIVE = "youtube_live"
    FILE_STREAM = "file_stream"
    MOCK = "mock"


class FrameGrabber(ABC, BaseModel):
    # for naming FrameGrabber objects that have no user-defined name
    unnamed_grabber_count: ClassVar[int] = 0

    crop: Optional[Dict[str, Dict[str, float]]] = None
    digital_zoom: Optional[confloat(ge=1, le=DIGITAL_ZOOM_MAX)] = None
    num_90_deg_rotations: Optional[int] = 0

    camera_name: Optional[str] = None
    autogenerate_name: bool = False

    class Config:
        extra = "forbid"

    @classmethod
    def get_input_type_to_class_dict(cls):
        input_type_to_class = {
            InputTypes.GENERIC_USB: GenericUSBFrameGrabber,
            InputTypes.RTSP: RTSPFrameGrabber,
            InputTypes.BASLER: BaslerFrameGrabber,
            InputTypes.REALSENSE: RealSenseFrameGrabber,
            InputTypes.RPI_CSI2: RaspberryPiCSI2FrameGrabber,
            InputTypes.HLS: HttpLiveStreamingFrameGrabber,
            InputTypes.YOUTUBE_LIVE: YouTubeLiveFrameGrabber,
            InputTypes.FILE_STREAM: FileStreamFrameGrabber,
            InputTypes.MOCK: MockFrameGrabber,
        }
        return input_type_to_class

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if not self.camera_name and self.autogenerate_name:
            self._autogenerate_name()

        # # TODO: figure out what to do here
        # self.apply_options(config["options"])

    def to_dict(self):
        dictionary_config = super().dict()
        del dictionary_config["crop"]
        del dictionary_config["digital_zoom"]
        del dictionary_config["num_90_deg_rotations"]
        del dictionary_config["autogenerate_name"]

        input_type = next(key for key, value in self.get_input_type_to_class_dict().items() if value == self.__class__)
        dictionary_config["input_type"] = input_type

        options = {}
        if self.crop:
            options["crop"] = self.crop
        if self.digital_zoom:
            options["zoom"] = {"digital": self.digital_zoom}
        if self.num_90_deg_rotations:
            options["rotation"] = {"num_90_deg_rotations": self.num_90_deg_rotations}

        dictionary_config["options"] = options
        return dictionary_config

    @classmethod
    def from_dict(cls, dictionary_config: dict) -> "FrameGrabber":
        dictionary_config = copy.deepcopy(dictionary_config)
        input_type = dictionary_config.pop("input_type")

        subclass = cls.get_input_type_to_class_dict()[input_type]

        options = dictionary_config.pop("options", {})

        crop = options.get("crop")
        digital_zoom = options.get("zoom", {}).get("digital")
        num_90_deg_rotations = options.get("rotation", {}).get("num_90_deg_rotations", 0)
        instance = subclass(
            crop=crop, digital_zoom=digital_zoom, num_90_deg_rotations=num_90_deg_rotations, **dictionary_config
        )

        return instance

    @validator("crop", pre=True, always=True)
    def validate_crop(cls, v):
        """Ensure that crop options are correctly specified."""
        if v:
            if "relative" in v and "pixels" in v:
                raise ValueError("Cannot specify both 'relative' and 'pixels' in crop options.")
            if "relative" in v:
                cls._validate_at_least_one_side(v["relative"], "relative")
            if "pixels" in v:
                cls._validate_at_least_one_side(v["pixels"], "pixels")
        return v

    @staticmethod
    def _validate_at_least_one_side(crop_dict, crop_type):
        """Ensure that at least one crop side is specified."""
        if not any(side in crop_dict for side in ["top", "bottom", "left", "right"]):
            raise ValueError(f"At least one side must be specified in {crop_type} crop options.")

        if crop_type == "relative":
            for param_name, param_value in crop_dict.items():
                if param_value < 0 or param_value > 1:
                    raise ValueError(
                        f"Relative cropping parameter ({param_name}) is {param_value}, which is invalid. "
                        f"Relative cropping parameters must be between 0 and 1, where 1 represents the full "
                        f"width or length of the image."
                    )

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
            try:
                grabber = FrameGrabber.create_grabber(config, autogenerate_name=False, warmup_delay=0)
                grabber_list.append(grabber)
            except ValueError as e:
                camera_name = config.get("name", "Unnamed Camera")
                logger.error(
                    f"Failed to connect to {camera_name}. Please check its connection and provided configuration: {config}",
                    exc_info=True,
                )

        # TODO: rethink this
        grabbers = FrameGrabber.grabbers_to_dict(grabber_list)

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

        input_type = config.pop("input_type")

        if input_type is None:
            raise ValueError(f"No input_type provided. Valid types are {InputTypes.get_options()}")

        grabber_class = FrameGrabber.get_input_type_to_class_dict().get(input_type)
        if grabber_class is None:
            raise ValueError(
                f"The provided input_type ({input_type}) is not valid. Valid types are {InputTypes.get_options()}"
            )

        # these are provided on the model, but shouldn't be provided in a
        # declaritive yaml or dictionary version of the camera. so we add them here.
        config["autogenerate_name"] = autogenerate_name
        config["warmup_delay"] = warmup_delay

        grabber = grabber_class.from_dict(**config)
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
            InputTypes.RPI_CSI2,
        )

        # Autodiscover the grabbers
        grabber_list = []
        for input_type in autodiscoverable_input_types:
            logger.info(f"Autodiscovering {input_type} cameras...")

            # If the input type is RTSP and rtsp_discover_modes is provided, use RTSPDiscovery to find the cameras
            if input_type == InputTypes.RTSP:
                if rtsp_discover_mode is not None:
                    onvif_devices = RTSPDiscovery.discover_onvif_devices(auto_discover_mode=rtsp_discover_mode)
                    for device in onvif_devices:
                        for index, rtsp_url in enumerate(device.rtsp_urls):
                            grabber = FrameGrabber.create_grabber(
                                {
                                    "input_type": input_type,
                                    "id": {"rtsp_url": rtsp_url},
                                    "name": f"RTSP Camera - {device.ip} - {index}",
                                },
                                autogenerate_name=False,
                                warmup_delay=0,
                            )
                            grabber_list.append(grabber)
                continue

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
            name = self.camera_name  # all grabbers should have a name, either user-provided or generated
            error_msg = f"Failed to grab frame from {name}"
            raise GrabError(error_msg)

        # apply post processing operations
        frame = self._rotate(frame)
        frame = self._crop(frame)
        frame = self._digital_zoom(frame)
        return frame

    @abstractmethod
    def _autogenerate_name(self) -> None:
        """For generating and assigning unique names for unnamed FrameGrabber objects.

        Attempts to incorporate a unique identifier (serial number, url, etc.) into each
        camera name. If no unique identifier is available, a counter is used instead.
        """
        pass

    def _crop(self, frame: np.ndarray) -> np.ndarray:
        """Looks at FrameGrabber's crop configuration and decides to either crop by pixels or
        in a relative manner (normalized).

        Returns a cropped frame.
        """
        if self.crop:
            relative_crop_params = self.crop.get("relative")
            if relative_crop_params:
                return self._crop_relative(frame, relative_crop_params)

            pixel_crop_params = self.crop.get("pixels")
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
        digital_zoom = self.digital_zoom

        if digital_zoom:
            top = (frame.shape[0] - frame.shape[0] / digital_zoom) / 2
            bottom = frame.shape[0] - top
            left = (frame.shape[1] - frame.shape[1] / digital_zoom) / 2
            right = frame.shape[1] - left
            frame = frame[int(top) : int(bottom), int(left) : int(right)]

        return frame

    def _rotate(self, frame: np.ndarray) -> np.ndarray:
        """Rotates the provided frame a specified number of 90 degree rotations clockwise"""

        if self.num_90_deg_rotations:
            for n in range(self.num_90_deg_rotations):
                frame = np.rot90(frame)

        return frame

    def apply_options(self, options: dict) -> None:
        """Update generic options such as crop and zoom as well as
        camera-specific options.
        """
        for key, value in kwargs.items():
            # TODO: we need to figure the dictionary stuff out here
            setattr(self, key, value)

        # Re-validate the model
        self.__init__(**self.dict())

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


class WithSerialNumberMixin(FrameGrabber, ABC):
    serial_number: Optional[str] = None

    def _autogenerate_name(self) -> None:
        unnamed_grabber_id = self.serial_number
        if not unnamed_grabber_id:
            FrameGrabber.unnamed_grabber_count += 1
            unnamed_grabber_id = FrameGrabber.unnamed_grabber_count
        autogenerated_name = f"{InputTypes.GENERIC_USB.upper()} Camera - {unnamed_grabber_id}"
        self.camera_name = autogenerated_name

    def to_dict(self) -> dict:
        base_dict = super().to_dict()
        del base_dict["serial_number"]
        if self.serial_number:
            base_dict["id"] = {"serial_number": self.serial_number}
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        data = copy.deepcopy(data)
        if "id" in data and "serial_number" in data["id"]:
            data["serial_number"] = data.pop("id").pop("serial_number")
        return super().from_dict(data)


class WithSerialNumberAndResolutionMixin(WithSerialNumberMixin, ABC):
    resolution_width: Optional[int] = None
    resolution_height: Optional[int] = None

    _capture: Optional[cv2.VideoCapture] = PrivateAttr(default=None)

    @validator("resolution_height", always=True)
    def validate_resolution(cls, v, values):
        if values.get("resolution_width") is not None and v is None:
            raise ValueError("resolution_height must be provided if resolution_width is provided")
        return v

    def to_dict(self) -> dict:
        base_dict = super().to_dict()
        del base_dict["resolution_width"]
        del base_dict["resolution_height"]

        if self.resolution_width is not None and self.resolution_height is not None:
            base_dict["options"]["resolution"] = {"width": self.resolution_width, "height": self.resolution_height}

        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        data = copy.deepcopy(data)

        options = data["options"]
        if "resolution" in options:
            resolution = options.pop("resolution")
            data["resolution_width"] = resolution.get("width")
            data["resolution_height"] = resolution.get("height")

        return super().from_dict(data)

    def _set_cv2_resolution(self) -> None:
        """Set the resolution of the cv2.VideoCapture object based on the FrameGrabber's config.
        If the FrameGrabber lacks both of these properties (height and width), this method
        will do nothing.

        Similarly, if the specified resolution equals the existing resolution, this function will
        do nothing. This is because setting the resolution of a cv2.VideoCapture object is non-trivial and
        can take multiple seconds, so we should only do it when something has changed.
        """

        new_height = self.resolution_height
        new_width = self.resolution_width

        if new_width:
            current_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            if new_width != current_width:
                self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, new_width)
        if new_height:
            current_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if new_height != current_height:
                self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, new_height)


class GenericUSBFrameGrabber(WithSerialNumberAndResolutionMixin):
    """For any generic USB camera, such as a webcam"""

    warmup_delay: float = 1.0

    # keep track of the cameras that are already in use so that we don't try to connect to them twice
    _indices_in_use: ClassVar[set] = PrivateAttr(default=set())
    _idx: Optional[int] = PrivateAttr(default=None)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # TODO: think about what happens with multiple cameras
        logger.info(
            f"Waiting {self.warmup_delay} seconds for camera(s) to warm up. "
            "Pass in warmup_delay = 0 to suppress this behavior."
        )
        time.sleep(self.warmup_delay)

        if self.serial_number and OPERATING_SYSTEM != "Linux":
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
                if self.serial_number and self.serial_number != found_cam["serial_number"]:
                    continue

                idx = found_cam["idx"]
                if idx in GenericUSBFrameGrabber._indices_in_use:
                    continue

                capture = self._connect_and_validate_capture(found_cam)
                if capture is not None:
                    break  # Found a valid capture

            else:
                raise ValueError(
                    f"Unable to find USB camera with the specified serial_number: {self.serial_number}. "
                    "Please ensure that the serial number is correct, that the camera is plugged in, and that the camera is not already in use."
                )
        # If we don't know the serial numbers of the cameras, just assign the next available camera by index
        else:
            logger.debug("No USB cameras found with Linux commands. Assigning camera by index.")
            for idx in range(20):  # an arbitrarily high number to make sure we check for enough cams
                if idx in GenericUSBFrameGrabber._indices_in_use:
                    continue  # Camera is already in use, moving on

                capture = cv2.VideoCapture(idx)
                if capture.isOpened():
                    break  # Found a valid capture

            else:
                raise ValueError("Unable to connect to USB camera by index. Is your camera plugged in?")

        # If a serial_number wasn't provided by the user, attempt to find it and add it to the config
        if not self.serial_number:
            for found_cam in found_cams:
                if idx == found_cam["idx"]:
                    self.serial_number = found_cam["serial_number"]
                    break

        # A valid capture has been found, saving it for later
        self._capture = capture

        # Log the current camera index as 'in use' to prevent other GenericUSBFrameGrabbers from stepping on it
        self._idx = idx
        GenericUSBFrameGrabber._indices_in_use.add(idx)

    def to_dict(self) -> dict:
        dictionary_config = super().to_dict()
        del dictionary_config["warmup_delay"]
        return dictionary_config

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
        if not self._capture.isOpened():
            self._capture.open(self._idx)

        # OpenCV VideoCapture buffers frames by default. It's usually not possible to turn buffering off.
        # Buffer can be set as low as 1, but even still, if we simply read once, we will get the buffered (stale) frame.
        # Assuming buffer size of 1, we need to read twice to get the current frame.
        for _ in range(2):
            _, frame = self._capture.read()

        return frame

    def release(self) -> None:
        GenericUSBFrameGrabber._indices_in_use.remove(self._idx)
        self._capture.release()

    def apply_options(self, options: dict) -> None:
        super().apply_options(options)
        self._set_cv2_resolution()

        # set the buffer size to 1 to always get the most recent frame
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
    """Handles RTSP streams.

    Can operate in two modes based on the `keep_connection_open` configuration:
        1. If `true`, keeps the connection open for low-latency frame grabbing, but consumes more CPU.  (default)
        2. If `false`, opens the connection only when needed, which is slower but conserves resources.
    """

    rtsp_url: str = Field(..., pattern=r"^rtsp://")
    keep_connection_open: bool = Field(default=True)
    max_fps: int = Field(default=30)

    # private attibutes cannot be overriden
    _lock: Lock = PrivateAttr(default_factory=Lock)
    _run: bool = PrivateAttr(default=True)
    _capture: Optional[cv2.VideoCapture] = PrivateAttr(default=None)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._substitute_rtsp_password()

        if self.keep_connection_open:
            self._open_connection()
            self._init_drain_thread()

    def to_dict(self) -> dict:
        dictionary_config = super().to_dict()

        del dictionary_config["rtsp_url"]
        dictionary_config["id"] = {"rtsp_url": self.rtsp_url}

        dictionary_config["options"]["keep_connection_open"] = self.keep_connection_open
        dictionary_config["options"]["max_fps"] = self.max_fps
        del dictionary_config["keep_connection_open"]
        del dictionary_config["max_fps"]

        return dictionary_config

    @classmethod
    def from_dict(cls, data: dict):
        data = copy.deepcopy(data)
        rtsp_url = data.pop("id").pop("rtsp_url")

        options = data["options"]
        keep_connection_open = options.pop("keep_connection_open")
        max_fps = options.pop("max_fps")

        new_data = {**data, "rtsp_url": rtsp_url, "keep_connection_open": keep_connection_open, "max_fps": max_fps}
        return FrameGrabber.from_dict(dictionary_config=new_data)

    def _autogenerate_name(self) -> None:
        return self.rtsp_url

    def _substitute_rtsp_password(self) -> dict:
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
        matches = re.findall(pattern, self.rtsp_url)

        if len(matches) == 0:
            return  # make no change to config if no password placeholder is found
        if len(matches) > 1:
            raise ValueError("RTSP URL should contain no more than one placeholder for the password.")

        match = matches[0]
        password_env_var = os.environ.get(match)
        if not password_env_var:
            raise ValueError(f"RTSP URL {self.rtsp_url} references environment variable {match} which is not set")

        placeholder = "{{" + match + "}}"
        new_rtsp_url = self.rtsp_url.replace(placeholder, password_env_var)
        self.rtsp_url = new_rtsp_url

    def _open_connection(self):
        self._capture = cv2.VideoCapture(self.rtsp_url)
        if not self._capture.isOpened():
            raise ValueError(
                f"Could not open RTSP stream: {self.rtsp_url}. Is the RTSP URL correct? Is the camera connected to the network?"
            )
        logger.debug(f"Initialized video capture with backend={self._capture.getBackendName()}")

    def _close_connection(self):
        with self._lock:
            if self._capture is not None:
                self._capture.release()

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
        with self._lock:
            ret, frame = self._capture.retrieve() if self.keep_connection_open else self._capture.read()
        if not ret:
            logger.error(f"Could not read frame from {self._capture}")
        return frame

    def release(self) -> None:
        if self.keep_connection_open:
            self.run = False  # to stop the buffer drain thread
            self._close_connection()

    def _init_drain_thread(self):
        if not self.keep_connection_open:
            return  # No need to drain if we're not keeping the connection open

        self.drain_rate = 1 / self.max_fps
        thread = Thread(target=self._drain)
        thread.daemon = True
        thread.start()

    def _drain(self):
        while self.run:
            with self._lock:
                _ = self._capture.grab()
            time.sleep(self.drain_rate)


class BaslerFrameGrabber(WithSerialNumberMixin):
    """Basler USB and Basler GigE Cameras"""

    basler_options: dict = {}

    _serial_numbers_in_use: ClassVar[set] = PrivateAttr(default=set())
    _converter: "pylon.ImageFormatConverter" = PrivateAttr(default=None)
    _camera: Optional["pylon.InstantCamera"] = PrivateAttr(default=None)

    def __init__(self, **kwargs):
        # Basler cameras grab frames in different pixel formats, most of which cannot be displayed directly
        # by OpenCV. self.convert will convert them to BGR which can be used by OpenCV
        super().__init__(**kwargs)
        self._converter = pylon.ImageFormatConverter()
        self._converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self._converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        tlf = pylon.TlFactory.GetInstance()
        devices = tlf.EnumerateDevices()

        if not devices:
            raise ValueError("No Basler cameras were found. Is your camera connected?")

        # Attempt to match the provided serial number with a plugged in device. If no serial number was provided, just
        # pick the first found device that is not currently in use.
        serial_number = self.serial_number
        for device in devices:
            curr_serial_number = device.GetSerialNumber()
            if curr_serial_number in BaslerFrameGrabber._serial_numbers_in_use:
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
        self.serial_number = curr_serial_number

        # A valid camera has been found, remember the serial_number to prevent
        # other FrameGrabbers from using it
        self._camera = camera
        BaslerFrameGrabber._serial_numbers_in_use.add(self.serial_number)

    def _grab_implementation(self) -> np.ndarray:
        with self._camera.GrabOne(2000) as result:
            if result.GrabSucceeded():
                # Convert the image to BGR for OpenCV
                image = self._converter.Convert(result)
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

                camera_name = self.camera_name or "Unnamed Basler Camera"
                logger.warning(
                    f"Could not grab a frame from {camera_name}\n"
                    f"{error_message}\n"
                    f"---------------------------------------------------\n"
                )

                frame = NOISE

        return frame

    def release(self) -> None:
        BaslerFrameGrabber._serial_numbers_in_use.remove(self.serial_number)
        self._camera.Close()

    def apply_options(self, options: dict) -> None:
        super().apply_options(options)
        node_map = self.camera.GetNodeMap()
        for property_name, value in self.basler_options.items():
            node = node_map.GetNode(property_name)
            node.SetValue(value)

    def to_dict(self) -> dict:
        dictionary_config = super().to_dict()
        dictionary_config["options"]["basler_options"] = self.basler_options
        return dictionary_config

    @classmethod
    def from_dict(cls, data: dict):
        data = copy.deepcopy(data)
        basler_options = data.pop("options").pop("basler_options")
        new_data = {**data, "basler_options": basler_options}
        return super().from_dict(new_data)


class RealSenseFrameGrabber(WithSerialNumberAndResolutionMixin):
    """Intel RealSense Depth Camera"""

    side_by_side_depth: bool = Field(default=False)

    _pipeline: Optional["rs.pipeline"] = PrivateAttr(default=None)
    _rs_config: Optional["rs.config"] = PrivateAttr(default=None)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        ctx = rs.context()
        if len(ctx.devices) == 0:
            raise ValueError("No Intel RealSense cameras detected. Is your camera plugged in?")

        provided_serial_number = self.serial_number

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
        self._pipeline = pipeline
        self._rs_config = rs_config

        # In case the serial_number wasn't provided by the user, add it to the config
        self.serial_number = curr_serial_number

    def _grab_implementation(self) -> np.ndarray:
        frames = self._pipeline.wait_for_frames()

        # Convert color images to numpy arrays and convert from RGB to BGR
        color_frame = frames.get_color_frame()
        color_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)

        if self.side_by_side_depth:
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
        self._pipeline.stop()

    def apply_options(self, options: dict) -> None:
        new_width = options.get("resolution", {}).get("width")
        new_height = options.get("resolution", {}).get("height")

        super().apply_options(options)
        if new_width and new_height:
            self._pipeline.stop()  # pipeline needs to be temporarily stopped in order to change the resolution
            self._rs_config.enable_stream(rs.stream.color, new_width, new_height)
            self._rs_config.enable_stream(rs.stream.depth, new_width, new_height)
            self._pipeline.start(self._rs_config)  # Restart the pipeline with the new configuration


class RaspberryPiCSI2FrameGrabber(WithSerialNumberMixin):
    "For CSI2 cameras connected to Raspberry Pis through their dedicated camera port"

    _camera: Optional["Picamera2"] = PrivateAttr(default=None)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
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

        self._camera = picam2

    def _grab_implementation(self) -> np.ndarray:
        frame = self._camera.capture_array()

        # Convert to BGR for opencv
        frame = cv2.cvtColor(np.asanyarray(frame), cv2.COLOR_BGR2RGB)

        return frame

    def release(self) -> None:
        self.camera._close()


class HttpLiveStreamingFrameGrabber(FrameGrabber):
    """Handles Http Live Streaming (HLS)

    Supports two modes:
    1. Keep connection open (default): Opens the connection once and keeps it open for high-fps frame grabbing.
    2. Open connection on every frame: Opens and closes the connection on every captured frame, which conserves
        both CPU and network bandwidth but has higher latency. In practice, roughly 1FPS is achievable with this strategy.
    """

    hls_url: str = Field(..., pattern=r"^https?://")
    keep_connection_open: bool = Field(default=True)

    _lock: Lock = PrivateAttr(default_factory=Lock)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if self.keep_connection_open:
            self._open_connection()

    def _autogenerate_name(self) -> None:
        return self.hls_url

    def _open_connection(self):
        self._capture = cv2.VideoCapture(self.hls_url)
        if not self._capture.isOpened():
            raise ValueError(f"Could not open {type(self)} stream: {self.hls_url}. Is the HLS URL correct?")
        logger.warning(f"Initialized video capture with backend={self._capture.getBackendName()}")

    def _close_connection(self):
        logger.warning(f"Closing connection to {type(self)} stream")
        with self._lock:
            if self._capture is not None:
                self._capture.release()

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
        with self._lock:
            ret, frame = self._capture.read()
        if not ret:
            logger.error(f"Could not read frame from {self._capture}")
        return frame

    def release(self) -> None:
        if self.keep_connection_open:
            self._close_connection()


class YouTubeLiveFrameGrabber(HttpLiveStreamingFrameGrabber):
    """Grabs the most recent frame from a YouTube Live stream (which are HLS streams under the hood)

    Supports two modes:
    1. Keep connection open (default): Opens the connection once and keeps it open for high-fps frame grabbing.
    2. Open connection on every frame: Opens and closes the connection on every captured frame, which conserves
        both CPU and network bandwidth but has higher latency. In practice, roughly 1FPS is achievable with this strategy.
    """

    youtube_url: str = Field(..., pattern=r"^https?://")
    keep_connection_open: bool = Field(default=True)

    def __init__(self, **kwargs):
        hls_url = self._extract_hls_url(kwargs.get("youtube_url"))
        kwargs["hls_url"] = hls_url
        super().__init__(**kwargs)

        if self.keep_connection_open:
            self._open_connection()

    def _autogenerate_name(self) -> None:
        return self.youtube_url

    def _extract_hls_url(self, youtube_url: str) -> str:
        """Extracts the HLS URL from a YouTube Live URL."""
        available_streams = streamlink.streams(youtube_url)
        if "best" not in available_streams:
            raise ValueError(f"No available HLS stream for {youtube_url=}\n{available_streams=}")
        return available_streams["best"].url


class FileStreamFrameGrabber(FrameGrabber):
    """Grabs frames from a video file stream, such as an MP4 or MOV file.

    Supports dropping frames to achieve a target FPS.

    Some of the supported formats: mp4, avi, mov, mjpeg

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

    filename: str = Field(..., pattern=r"^.+\.(mp4|avi|mov|mjpeg)$")
    fps_target: float = Field(default=0, ge=0)

    _remainder: float = PrivateAttr(default=0.0)
    _capture: Optional[cv2.VideoCapture] = PrivateAttr(default=None)
    _fps_source: float = PrivateAttr(default=0.0)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._capture = cv2.VideoCapture(self.filename)
        if not self._capture.isOpened():
            raise ValueError(f"Could not open file {self.filename}. Is it a valid video file?")
        backend = self._capture.getBackendName()
        logger.debug(f"Initialized video capture with {backend=}")

        ret, _ = self._capture.read()
        if not ret:
            self._capture.release()
            raise ValueError(f"Could not read first frame of file {self.filename}. Is it a valid video file?")

        self._fps_source = round(self._capture.get(cv2.CAP_PROP_FPS), 2)
        if self._fps_source <= 0.1:
            logger.warning(f"Captured framerate is very low or zero: {self._fps_source} FPS")
        self._should_drop_frames = self.fps_target > 0 and self.fps_target < self._fps_source
        logger.debug(
            f"Source FPS: {self._fps_source}, Target FPS: {self.fps_target}, Drop Frames: {self._should_drop_frames}"
        )

    def _autogenerate_name(self) -> None:
        return self.filename

    def _grab_implementation(self) -> np.ndarray:
        """Grab a frame from the video file, decimating if needed to match target FPS.

        Returns:
            np.ndarray: The captured frame

        Raises:
            RuntimeWarning: If frame cannot be read, likely end of file
        """
        if self._should_drop_frames:
            self._drop_frames()

        ret, frame = self._capture.read()
        if not ret:
            raise RuntimeWarning("Could not read frame from video file. Possible end of file.")
        return frame

    def _drop_frames(self) -> None:
        """Drop frames to achieve target frame rate using frame position seeking."""
        drop_frames = (self.fps_source / self.fps_target) - 1 + self.remainder
        frames_to_drop = round(drop_frames)

        if frames_to_drop > 0:
            current_pos = self._capture.get(cv2.CAP_PROP_POS_FRAMES)
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, current_pos + frames_to_drop)

        self.remainder = round(drop_frames - frames_to_drop, 2)
        logger.debug(f"Dropped {frames_to_drop} frames to meet {self.fps_target} FPS target")

    def release(self) -> None:
        """Release the video capture resources."""
        if hasattr(self, "capture"):
            self._capture.release()


class MockFrameGrabber(WithSerialNumberAndResolutionMixin):
    """A mock camera class for testing purposes"""

    # Represents the serial numbers of the mock cameras that are discoverable
    available_serial_numbers: ClassVar[tuple] = ("123", "456", "789")

    # Keeps track of the available serial numbers so that we don't try to connect to them twice
    serial_numbers_in_use: ClassVar[set] = set()

    def __post_init__(self):
        provided_serial_number = self.serial_number

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
        self.serial_number = curr_serial_number

    def _grab_implementation(self) -> np.ndarray:
        width = self.resolution.width or 640
        height = self.resolution.height or 480

        return np.zeros((height, width, 3), dtype=np.uint8)

    def release(self) -> None:
        MockFrameGrabber.serial_numbers_in_use.remove(self.serial_number)
