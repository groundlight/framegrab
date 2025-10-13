"""
This module defines the configuration classes for various types of frame grabbers.
These configurations are used to initialize and manage different frame grabber objects.
"""

import copy
import os
import re
from abc import ABC
from enum import Enum
from typing import Any, ClassVar, Dict, Generic, Optional, Tuple, TypeVar, Union

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    GetCoreSchemaHandler,
    PrivateAttr,
    computed_field,
    confloat,
    field_validator,
    model_validator,
)
from pydantic.fields import FieldInfo
from pydantic_core import core_schema

from .unavailable_module import UnavailableModuleOrObject

DIGITAL_ZOOM_MAX = 4
DEFAULT_FOURCC = "MJPG"
DEFAULT_FPS = 30

# Supported video file extensions for FileStreamFrameGrabber
SUPPORTED_VIDEO_EXTENSIONS = ["mp4", "mov", "mjpeg", "avi", "mkv", "webm"]

# Only used for YouTube Live streams, not required otherwise
try:
    import streamlink
except ImportError as e:
    streamlink = UnavailableModuleOrObject(e)


class InputTypes(Enum):
    """Defines the available input types for FrameGrabber objects."""

    GENERIC_USB = "generic_usb"
    RTSP = "rtsp"
    REALSENSE = "realsense"
    BASLER = "basler"
    RPI_CSI2 = "rpi_csi2"
    HLS = "hls"
    YOUTUBE_LIVE = "youtube_live"
    FILE_STREAM = "file"
    MOCK = "mock"
    ROS2 = "ros2"

    def get_options() -> list:
        """Get a list of the available InputType options."""
        output = []
        for attr_name in vars(InputTypes):
            attr_value = getattr(InputTypes, attr_name)
            if "__" not in attr_name and isinstance(attr_value, str):
                output.append(attr_value)
        return output


T = TypeVar("T")


class OptionsField(FieldInfo, Generic[T]):
    """
    Marks a model attribute that belongs under the ``options`` section.

    key: dot-path from ``options`` to the value
         e.g. "zoom.digital" → {"options": {"zoom": {"digital": 2}}}

    Example:
        video_stream: OptionsField[bool] = OptionsField(key="video_stream", default=False)
    """

    def __init__(self, *, key: str, default: T = ..., **kwargs: Any):
        kwargs.setdefault("json_schema_extra", {})["options_key"] = key
        kwargs.setdefault("default", default)
        super().__init__(**kwargs)
        self.key = key

    @classmethod
    def __get_pydantic_core_schema__(cls, _source, handler: GetCoreSchemaHandler) -> core_schema.CoreSchema:
        """
        Delegate validation & schema generation to the inner type `T`.
        Allows OptionsField[T] to be used anywhere a plain `T` could be.
        """
        # `__args__` holds the type parameter, e.g. dict | None
        inner_type = _source.__args__[0]
        return handler.generate_schema(inner_type)


# Private helpers for nested option dict manipulation
def _deep_set(target: dict, parts: list[str], val):
    """Create/update nested dictionaries specified by dot‐path parts."""
    head, *tail = parts
    if not tail:
        target[head] = val
        return
    child = target.setdefault(head, {})
    _deep_set(child, tail, val)


def _deep_get(source: dict, parts: list[str]):
    """Safely fetch nested value addressed by dot‐path parts; returns None if absent."""
    head, *tail = parts
    if head not in source:
        return None
    if not tail:
        return source[head]
    return _deep_get(source[head], tail)


def _deep_delete(target: dict, parts: list[str]):
    """Remove nested value addressed by dot‐path parts."""
    head, *tail = parts
    if head not in target:
        return
    if not tail:
        del target[head]
        return
    _deep_delete(target[head], tail)


def _clean_empty(d: dict):
    """Remove empty dict values recursively."""
    for k in list(d.keys()):
        if isinstance(d[k], dict):
            _clean_empty(d[k])
            if not d[k]:
                del d[k]


class FrameGrabberConfig(ABC, BaseModel, validate_assignment=True):
    """Base configuration class for all frame grabbers."""

    model_config = ConfigDict(extra="forbid", validate_assignment=True)

    crop: OptionsField[Optional[dict]] = OptionsField(key="crop", default=None)
    digital_zoom: OptionsField[Optional[float]] = OptionsField(
        key="zoom.digital", default=None, ge=1, le=DIGITAL_ZOOM_MAX
    )
    num_90_deg_rotations: OptionsField[Optional[int]] = OptionsField(key="rotation.num_90_deg_rotations", default=0)

    name: Optional[str] = None

    _unnamed_grabber_count: ClassVar[int] = PrivateAttr(default=0)

    @model_validator(mode="before")
    def autogenerate_name_if_needed(cls, values: Dict[str, Any]) -> Dict[str, Any]:
        if not values.get("name"):
            input_type = cls.get_input_type()

            id_field_name = cls.get_input_type_to_id_dict()[input_type]
            id_field_value = values.get(id_field_name)
            unnamed_grabber_id = id_field_value if id_field_value else "unnamed"

            unnamed_grabber_count = cls._unnamed_grabber_count
            values["name"] = f"{input_type.value}_{unnamed_grabber_id}_{unnamed_grabber_count}"

            cls._unnamed_grabber_count += 1
        return values

    @field_validator("crop", mode="before")
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
        if crop_type == "pixels":
            for param_name, param_value in crop_dict.items():
                if isinstance(param_value, int) or (hasattr(param_value, "is_integer") and param_value.is_integer()):
                    crop_dict[param_name] = int(param_value)
                else:
                    raise ValueError(
                        f"Pixel cropping parameter ({param_name}) with value ({param_value}) is invalid. "
                        f"All pixel cropping parameters must be integers."
                    )

    @classmethod
    def get_input_type_to_class_dict(cls) -> dict[InputTypes, "FrameGrabberConfig"]:
        """Map input types to their corresponding configuration classes."""
        input_type_to_class = {
            InputTypes.GENERIC_USB: GenericUSBFrameGrabberConfig,
            InputTypes.RTSP: RTSPFrameGrabberConfig,
            InputTypes.BASLER: BaslerFrameGrabberConfig,
            InputTypes.REALSENSE: RealSenseFrameGrabberConfig,
            InputTypes.RPI_CSI2: RaspberryPiCSI2FrameGrabberConfig,
            InputTypes.HLS: HttpLiveStreamingFrameGrabberConfig,
            InputTypes.YOUTUBE_LIVE: YouTubeLiveFrameGrabberConfig,
            InputTypes.FILE_STREAM: FileStreamFrameGrabberConfig,
            InputTypes.MOCK: MockFrameGrabberConfig,
            InputTypes.ROS2: ROS2GrabberConfig,
        }
        return input_type_to_class

    @classmethod
    def get_class_for_input_type(cls, input_type: InputTypes) -> "FrameGrabberConfig":
        """Get the configuration class for a given input type."""
        if type(input_type) != InputTypes:
            raise ValueError(f"Input type must be an instance of InputTypes, not {type(input_type)}")
        if input_type not in cls.get_input_type_to_class_dict():
            raise ValueError(f"Invalid input type: {input_type}")
        return cls.get_input_type_to_class_dict()[input_type]

    @classmethod
    def get_input_type(cls) -> InputTypes:
        """Determine the input type of the current class."""
        input_type_to_class = cls.get_input_type_to_class_dict()
        input_type = next(key for key, value in input_type_to_class.items() if value == cls)
        return input_type

    @classmethod
    def get_input_type_to_id_dict(cls) -> dict[InputTypes, str]:
        """Map input types to their corresponding ID fields."""
        input_type_to_id = {
            InputTypes.GENERIC_USB: "serial_number",
            InputTypes.RTSP: "rtsp_url",
            InputTypes.BASLER: "serial_number",
            InputTypes.REALSENSE: "serial_number",
            InputTypes.RPI_CSI2: None,
            InputTypes.HLS: "hls_url",
            InputTypes.YOUTUBE_LIVE: "youtube_url",
            InputTypes.FILE_STREAM: "filename",
            InputTypes.MOCK: "serial_number",
            InputTypes.ROS2: "topic",
        }
        return input_type_to_id

    @classmethod
    def get_input_type_from_id_field(cls, id_field: str) -> InputTypes:
        """Try to get the input type from an id field. Raise an error if there is ambiguity."""
        input_type_to_id = cls.get_input_type_to_id_dict()
        matching_input_types = [key for key, value in input_type_to_id.items() if value == id_field]

        if len(matching_input_types) > 1:
            raise ValueError(
                f"The id_field '{id_field}' could be mapped to multiple input types: {matching_input_types}"
            )

        if len(matching_input_types) == 0:
            raise KeyError(f"The id_field '{id_field}' could not be mapped to any input types")

        return matching_input_types[0]

    def get_id_field_and_value(self) -> Tuple[str, str]:
        """Get the id field and value for the current class."""
        input_type = self.get_input_type()
        id_field = self.get_input_type_to_id_dict()[input_type]
        id_value = getattr(self, id_field) if id_field else None
        return id_field, id_value

    def to_framegrab_config_dict(self) -> dict:
        """Convert the config to the framegrab standard format."""
        data = super().model_dump()

        # Input type + id handling (unchanged)
        input_type = self.get_input_type()
        data["input_type"] = input_type.value

        id_field, id_value = self.get_id_field_and_value()
        if id_field and id_value:
            data["id"] = {id_field: id_value}
            data.pop(id_field, None)

        # Extract all OptionsField-marked attributes into nested options
        options: dict = {}
        for fname, mfield in self.__class__.model_fields.items():
            opt_key = (mfield.json_schema_extra or {}).get("options_key")
            if not opt_key:
                continue
            val = getattr(self, fname)
            if val is None:
                continue
            _deep_set(options, opt_key.split("."), val)
            data.pop(fname, None)

        data["options"] = options
        return data

    @classmethod
    def get_model_parameters(cls, cfg: dict) -> dict:
        """Inverse of to_framegrab_config_dict: flatten options into kwargs."""
        cfg = copy.deepcopy(cfg)

        # Unpack id section first
        id_section = cfg.pop("id", {})
        id_field_name, id_field_value = next(iter(id_section.items())) if id_section else (None, None)

        options = cfg.pop("options", {})

        # Pull OptionsField values back to top level kwargs
        for fname, mfield in cls.model_fields.items():
            opt_key = (mfield.json_schema_extra or {}).get("options_key")
            if not opt_key:
                continue
            val = _deep_get(options, opt_key.split("."))
            if val is not None:
                cfg[fname] = val
                # Remove the consumed option to track unused keys
                _deep_delete(options, opt_key.split("."))

        _clean_empty(options)
        if options:
            raise ValueError(f"Unexpected option keys for {cls.__name__}: {list(options.keys())}")

        # Reattach id field if present
        if id_field_name:
            cfg[id_field_name] = id_field_value

        return cfg

    @classmethod
    def from_framegrab_config_dict(cls, dictionary_config: dict) -> "FrameGrabber":
        """Create a FrameGrabberConfig instance from a dictionary."""
        dictionary_config = copy.deepcopy(dictionary_config)
        input_type = InputTypes(dictionary_config.pop("input_type"))
        subclass = cls.get_class_for_input_type(input_type)
        kwargs = subclass.get_model_parameters(dictionary_config)
        instance = subclass(**kwargs)
        return instance

    @classmethod
    def create(cls, input_type: Optional[InputTypes] = None, **kwargs) -> "FrameGrabberConfig":
        """Factory method to create an instance of the appropriate subclass based on input_type."""
        if input_type:
            subclass = cls.get_class_for_input_type(input_type)
            return subclass(**kwargs)
        else:  # try and determine the input type
            for arg_name, _ in kwargs.items():
                try:
                    input_type = cls.get_input_type_from_id_field(arg_name)
                    subclass = cls.get_class_for_input_type(input_type)
                    return subclass(**kwargs)
                except ValueError:
                    raise ValueError(
                        f"There are multiple input types that could be mapped to the id field '{arg_name}'. "
                        "You must specifically provide the input_type"
                    )
                except KeyError:
                    continue
            raise ValueError("Could not determine input type from provided arguments")


class WithResolutionMixin(FrameGrabberConfig, ABC):
    """Mixin class to add resolution configuration to FrameGrabberConfig."""

    resolution_width: OptionsField[Optional[int]] = OptionsField(key="resolution.width", default=None)
    resolution_height: OptionsField[Optional[int]] = OptionsField(key="resolution.height", default=None)

    @field_validator("resolution_height", mode="before")
    def validate_resolution(cls, v: int, info: dict):
        """Ensure resolution_height is provided if resolution_width is provided."""
        if info.config.get("resolution_width") is not None and v is None:
            raise ValueError("resolution_height must be provided if resolution_width is provided")
        return v


class WithKeepConnectionOpenMixin(ABC, BaseModel):
    """Mixin class to add keep_connection_open configuration to FrameGrabberConfig."""

    keep_connection_open: OptionsField[bool] = OptionsField(key="keep_connection_open", default=True)


class WithMaxFPSMixin(ABC, BaseModel):
    """Mixin class to add max_fps configuration to FrameGrabberConfig."""

    max_fps: OptionsField[Optional[int]] = OptionsField(key="max_fps", default=30)


class GenericUSBFrameGrabberConfig(WithResolutionMixin):
    """Configuration class for Generic USB Frame Grabber."""

    serial_number: Optional[str] = None
    video_stream: OptionsField[bool] = OptionsField(key="video_stream", default=False)
    fourcc: OptionsField[Optional[str]] = OptionsField(key="fourcc", default=None)
    fps: OptionsField[Optional[int]] = OptionsField(key="fps", default=None)


class RTSPFrameGrabberConfig(FrameGrabberConfig, WithKeepConnectionOpenMixin, WithMaxFPSMixin):
    """Configuration class for RTSP Frame Grabber."""

    rtsp_url: str = Field(..., pattern=r"^rtsp://")

    @field_validator("rtsp_url", mode="before")
    def substitute_rtsp_password(cls, rtsp_url: str) -> str:
        """
        Substitute the password placeholder in the rtsp_url with the actual password
        from an environment variable.
        The URL should take this format:
            Ex: rtsp://admin:{{MY_PASSWORD}}@10.0.0.0/cam/realmonitor?channel=1&subtype=0
        This function looks for an all-uppercase name between {{ and }} to find an environment
        variable with that name. If the environment variable is found, its value will be
        substituted in the rtsp_url.
        NOTE: This can also work for multiple RTSP URLs in the same config file as long
            as each one has a unique password placeholder.
        """
        pattern = r"\{\{([A-Z_][A-Z0-9_]*?)\}\}"
        matches = re.findall(pattern, rtsp_url)

        if len(matches) == 0:
            return rtsp_url  # Make no change to rtsp_url if no password placeholder is found
        elif len(matches) > 1:
            raise ValueError("RTSP URL should contain no more than one placeholder for the password.")

        match = matches[0]
        password_env_var = os.environ.get(match)
        if not password_env_var:
            raise ValueError(f"RTSP URL {rtsp_url} references environment variable {match} which is not set")

        placeholder = "{{" + match + "}}"
        return rtsp_url.replace(placeholder, password_env_var)


class BaslerFrameGrabberConfig(FrameGrabberConfig):
    """Configuration class for Basler Frame Grabber."""

    serial_number: Optional[str] = None
    basler_options: OptionsField[Optional[dict]] = OptionsField(key="basler_options", default=None)


class RealSenseFrameGrabberConfig(WithResolutionMixin):
    """Configuration class for RealSense Frame Grabber."""

    serial_number: Optional[str] = None
    side_by_side_depth: OptionsField[Optional[bool]] = OptionsField(key="depth.side_by_side", default=None)


class HttpLiveStreamingFrameGrabberConfig(FrameGrabberConfig, WithKeepConnectionOpenMixin):
    """Configuration class for HTTP Live Streaming Frame Grabber."""

    hls_url: str = Field(..., pattern=r"^https?://")


class RaspberryPiCSI2FrameGrabberConfig(FrameGrabberConfig):
    """Configuration class for Raspberry Pi CSI-2 Frame Grabber."""


class YouTubeLiveFrameGrabberConfig(FrameGrabberConfig, WithKeepConnectionOpenMixin):
    """Configuration class for YouTube Live Frame Grabber."""

    youtube_url: str = Field(..., pattern=r"^https?://")

    @computed_field
    @property
    def hls_url(self) -> str:
        """Sets the hls_url based on the youtube_url."""
        youtube_url = self.youtube_url
        available_streams = streamlink.streams(youtube_url)
        if "best" not in available_streams:
            raise ValueError(f"No available HLS stream for {youtube_url=}\n{available_streams=}")
        return available_streams["best"].url

    def to_framegrab_config_dict(self) -> dict:
        """Convert the config to the framegrab standard format."""
        base_dict = super().to_framegrab_config_dict()
        del base_dict["hls_url"]
        return base_dict


class FileStreamFrameGrabberConfig(FrameGrabberConfig, WithMaxFPSMixin):
    """Configuration class for File Stream Frame Grabber."""

    filename: str = Field(..., pattern=rf"(?i)^.*\.({'|'.join(SUPPORTED_VIDEO_EXTENSIONS)})$")


class MockFrameGrabberConfig(WithResolutionMixin):
    """Configuration class for Mock Frame Grabber."""

    serial_number: Optional[str] = None


class ROS2GrabberConfig(FrameGrabberConfig):
    """Configuration class for ROS 2 Grabber."""

    topic: str = Field(..., pattern=r"^(~|/)?([A-Za-z_][A-Za-z0-9_]*)(/[A-Za-z_][A-Za-z0-9_]*)*$")
