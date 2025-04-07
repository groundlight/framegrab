"""
This module defines the configuration classes for various types of frame grabbers.
These configurations are used to initialize and manage different frame grabber objects.
"""

import copy
import os
import re
from abc import ABC
from enum import Enum
from typing import ClassVar, Dict, Optional

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    PrivateAttr,
    computed_field,
    confloat,
    field_validator,
)

from .unavailable_module import UnavailableModule

# Only used for YouTube Live streams, not required otherwise
try:
    import streamlink
except ImportError as e:
    streamlink = UnavailableModule(e)


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

    def get_options() -> list:
        """Get a list of the available InputType options."""
        output = []
        for attr_name in vars(InputTypes):
            attr_value = getattr(InputTypes, attr_name)
            if "__" not in attr_name and isinstance(attr_value, str):
                output.append(attr_value)
        return output


DIGITAL_ZOOM_MAX = 4


class FrameGrabberConfig(ABC, BaseModel):
    """Base configuration class for all frame grabbers."""
    model_config = ConfigDict(extra="forbid", validate_assignment=True)

    crop: Optional[Dict[str, Dict[str, float]]] = None
    digital_zoom: Optional[confloat(ge=1, le=DIGITAL_ZOOM_MAX)] = None
    num_90_deg_rotations: Optional[int] = 0
    name: Optional[str] = None

    _id_field_optional: ClassVar[bool] = PrivateAttr(default=False)

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
    def get_input_type_to_class_dict(cls):
        """Map input types to their corresponding configuration classes."""
        input_type_to_class = {
            InputTypes.GENERIC_USB.value: GenericUSBFrameGrabberConfig,
            InputTypes.RTSP.value: RTSPFrameGrabberConfig,
            InputTypes.BASLER.value: BaslerFrameGrabberConfig,
            InputTypes.REALSENSE.value: RealSenseFrameGrabberConfig,
            InputTypes.RPI_CSI2.value: RaspberryPiCSI2FrameGrabberConfig,
            InputTypes.HLS.value: HttpLiveStreamingFrameGrabberConfig,
            InputTypes.YOUTUBE_LIVE.value: YouTubeLiveFrameGrabberConfig,
            InputTypes.FILE_STREAM.value: FileStreamFrameGrabberConfig,
            InputTypes.MOCK.value: MockFrameGrabberConfig,
        }
        return input_type_to_class

    @classmethod
    def get_class_for_input_type(cls, input_type: str):
        """Get the configuration class for a given input type."""
        if input_type not in cls.get_input_type_to_class_dict():
            raise ValueError(f"Invalid input type: {input_type}")
        return cls.get_input_type_to_class_dict()[input_type]

    def get_input_type(self):
        """Determine the input type of the current class."""
        input_type_to_class = self.get_input_type_to_class_dict()
        current_class = type(self)
        input_type = next(key for key, value in input_type_to_class.items() if value == current_class)
        return input_type

    @classmethod
    def get_input_type_to_id_dict(cls):
        """Map input types to their corresponding ID fields."""
        input_type_to_id = {
            InputTypes.GENERIC_USB.value: "serial_number",
            InputTypes.RTSP.value: "rtsp_url",
            InputTypes.BASLER.value: "serial_number",
            InputTypes.REALSENSE.value: "serial_number",
            InputTypes.RPI_CSI2.value: "serial_number",
            InputTypes.HLS.value: "hls_url",
            InputTypes.YOUTUBE_LIVE.value: "youtube_url",
            InputTypes.FILE_STREAM.value: "filename",
            InputTypes.MOCK.value: "serial_number",
        }
        return input_type_to_id

    def to_framegrab_config_dict(self) -> dict:
        """Convert the config to the framegrab standard format."""
        dictionary_config = super().model_dump()

        input_type = self.get_input_type()
        dictionary_config["input_type"] = input_type

        # Structure the id field like this: {"input_type": {"id_field": "id_value"}}
        id_field = self.get_input_type_to_id_dict()[input_type]
        dictionary_config["id"] = {id_field: getattr(self, id_field)}
        del dictionary_config[id_field]

        # These go in the options field
        del dictionary_config["crop"]
        del dictionary_config["digital_zoom"]
        del dictionary_config["num_90_deg_rotations"]

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
    def get_model_parameters(cls, dictionary_config: dict) -> dict:
        """
        Extract the parameters for the pydantic model from the
        framegrab standard format dictionary config and return them as a dictionary.
        """
        dictionary_config = copy.deepcopy(dictionary_config)

        id_field_name = None
        id_field_value = None
        if "id" in dictionary_config or not cls._id_field_optional:
            if "id" not in dictionary_config:
                raise ValueError("The 'id' field is missing in the configuration dictionary.")

            id = dictionary_config.pop("id")
            id_field_name = list(id.keys())[0]
            id_field_value = id[id_field_name]

        options = dictionary_config.pop("options", {})
        crop = options.pop("crop", None)
        digital_zoom = options.pop("zoom", {}).pop("digital", None)
        num_90_deg_rotations = options.pop("rotation", {}).pop("num_90_deg_rotations", 0)

        return {
            "crop": crop,
            "digital_zoom": digital_zoom,
            "num_90_deg_rotations": num_90_deg_rotations,
            **({id_field_name: id_field_value} if id_field_name else {}),
            **dictionary_config,
            **options,  # Theoretically this should be empty. But if it's not, we'll send it downstream so the unexpected option params are validated
        }

    @classmethod
    def from_framegrab_config_dict(cls, dictionary_config: dict) -> "FrameGrabber":
        """Create a FrameGrabberConfig instance from a dictionary."""
        dictionary_config = copy.deepcopy(dictionary_config)
        input_type = dictionary_config["input_type"]
        subclass = cls.get_class_for_input_type(input_type)
        kwargs = subclass.get_model_parameters(dictionary_config)
        instance = subclass(**kwargs)
        return instance

    @classmethod
    def create(cls, input_type: InputTypes, **kwargs) -> "FrameGrabberConfig":
        """Factory method to create an instance of the appropriate subclass based on input_type."""
        subclass = cls.get_class_for_input_type(input_type.value)
        return subclass(**kwargs)


class WithResolutionMixin(FrameGrabberConfig, ABC):
    """Mixin class to add resolution configuration to FrameGrabberConfig."""
    resolution_width: Optional[int] = None
    resolution_height: Optional[int] = None

    @field_validator("resolution_height", mode="before")
    def validate_resolution(cls, v: int, info: dict):
        """Ensure resolution_height is provided if resolution_width is provided."""
        if info.config.get("resolution_width") is not None and v is None:
            raise ValueError("resolution_height must be provided if resolution_width is provided")
        return v

    def to_framegrab_config_dict(self) -> dict:
        """Convert the config to the framegrab standard format."""
        base_dict = super().to_framegrab_config_dict()
        del base_dict["resolution_width"]
        del base_dict["resolution_height"]

        if self.resolution_width is not None and self.resolution_height is not None:
            base_dict["options"]["resolution"] = {"width": self.resolution_width, "height": self.resolution_height}

        return base_dict

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        """Extract model parameters from the framegrab standard format dictionary config."""
        data = copy.deepcopy(config_dict)

        options = data.get("options", {})
        if "resolution" in options:
            resolution = options.pop("resolution")
            data["resolution_width"] = resolution.get("width")
            data["resolution_height"] = resolution.get("height")

        return super().get_model_parameters(data)


class WithKeepConnectionOpenMixin(ABC, BaseModel):
    """Mixin class to add keep_connection_open configuration to FrameGrabberConfig."""
    keep_connection_open: bool = Field(default=True)

    def update_framegrab_config_dict(self, dictionary_config: dict) -> dict:
        """Update the framegrab config dictionary with keep_connection_open option."""
        del dictionary_config["keep_connection_open"]
        dictionary_config.setdefault("options", {})["keep_connection_open"] = self.keep_connection_open
        return dictionary_config

    @classmethod
    def update_model_parameters(cls, model_parameters: dict) -> dict:
        """Update model parameters with keep_connection_open option."""
        data = copy.deepcopy(model_parameters)
        options = data.get("options", {})
        keep_connection_open = options.pop("keep_connection_open", True)
        if keep_connection_open is not None:
            data["keep_connection_open"] = keep_connection_open
        return data


class WithMaxFPSMixin(ABC, BaseModel):
    """Mixin class to add max_fps configuration to FrameGrabberConfig."""
    max_fps: Optional[int] = None

    def update_framegrab_config_dict(self, dictionary_config: dict) -> dict:
        """Update the framegrab config dictionary with max_fps option."""
        del dictionary_config["max_fps"]
        dictionary_config.setdefault("options", {})["max_fps"] = self.max_fps
        return dictionary_config

    @classmethod
    def update_model_parameters(cls, model_parameters: dict) -> dict:
        """Update model parameters with max_fps option."""
        data = copy.deepcopy(model_parameters)
        options = data.get("options", {})
        max_fps = options.pop("max_fps", 30)
        if max_fps:
            data["max_fps"] = max_fps
        return data


class GenericUSBFrameGrabberConfig(WithResolutionMixin):
    """Configuration class for Generic USB Frame Grabber."""
    serial_number: Optional[str] = None
    _id_field_optional: ClassVar[bool] = True


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

    def to_framegrab_config_dict(self) -> dict:
        """Convert the config to the framegrab standard format."""
        base_dict = super().to_framegrab_config_dict()

        with_options_keep_connection_open = WithKeepConnectionOpenMixin.update_framegrab_config_dict(self, base_dict)
        with_options_max_fps = WithMaxFPSMixin.update_framegrab_config_dict(self, with_options_keep_connection_open)

        return with_options_max_fps

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        """Extract model parameters from the framegrab standard format dictionary config."""
        model_parameters_with_keep_connection_open = WithKeepConnectionOpenMixin.update_model_parameters(config_dict)
        model_parameters_with_max_fps = WithMaxFPSMixin.update_model_parameters(
            model_parameters_with_keep_connection_open
        )
        return FrameGrabberConfig.get_model_parameters(model_parameters_with_max_fps)


class BaslerFrameGrabberConfig(FrameGrabberConfig):
    """Configuration class for Basler Frame Grabber."""
    serial_number: Optional[str] = None
    basler_options: Optional[dict] = None
    _id_field_optional: ClassVar[bool] = True

    def to_framegrab_config_dict(self) -> dict:
        """Convert the config to the framegrab standard format."""
        dictionary_config = super().to_framegrab_config_dict()
        dictionary_config["options"]["basler_options"] = self.basler_options
        del dictionary_config["basler_options"]
        return dictionary_config

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        """Extract model parameters from the framegrab standard format dictionary config."""
        data = copy.deepcopy(config_dict)
        options = data.get("options", {})
        basler_options = options.pop("basler_options", {})
        new_data = {**data, "basler_options": basler_options}
        return super().get_model_parameters(new_data)


class RealSenseFrameGrabberConfig(WithResolutionMixin):
    """Configuration class for RealSense Frame Grabber."""
    serial_number: Optional[str] = None
    side_by_side_depth: Optional[bool] = False
    _id_field_optional: ClassVar[bool] = True

    def to_framegrab_config_dict(self) -> dict:
        """Convert the config to the framegrab standard format."""
        base_dict = super().to_framegrab_config_dict()
        base_dict["options"]["depth"] = {"side_by_side": self.side_by_side_depth}
        return base_dict

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        """Extract model parameters from the framegrab standard format dictionary config."""
        data = copy.deepcopy(config_dict)
        options = data.get("options", {})
        side_by_side_depth = options.pop("depth", {}).pop("side_by_side", False)
        if side_by_side_depth:
            data["side_by_side_depth"] = side_by_side_depth
        return super().get_model_parameters(data)


class HttpLiveStreamingFrameGrabberConfig(FrameGrabberConfig, WithKeepConnectionOpenMixin):
    """Configuration class for HTTP Live Streaming Frame Grabber."""
    hls_url: str = Field(..., pattern=r"^https?://")


class RaspberryPiCSI2FrameGrabberConfig(FrameGrabberConfig):
    """Configuration class for Raspberry Pi CSI-2 Frame Grabber."""
    serial_number: Optional[str] = None


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


class FileStreamFrameGrabberConfig(FrameGrabberConfig):
    """Configuration class for File Stream Frame Grabber."""
    filename: str = Field(..., pattern=r"^[\w\-/]+\.mp4|mov|mjpeg$")
    max_fps: float = Field(default=0, ge=0)


class MockFrameGrabberConfig(WithResolutionMixin):
    """Configuration class for Mock Frame Grabber."""
    serial_number: Optional[str] = None
    _id_field_optional: ClassVar[bool] = True
