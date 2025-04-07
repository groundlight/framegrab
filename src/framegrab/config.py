# TODO: figure out name thing
import copy
import pdb
from abc import ABC
from enum import Enum
from typing import ClassVar, Dict, Optional, Tuple

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

# Only used for Youtube Live streams, not required otherwise
try:
    import streamlink
except ImportError as e:
    streamlink = UnavailableModule(e)


class InputTypes(Enum):
    """Defines the available input types from FrameGrabber objects"""

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
        """Get a list of the available InputType options"""
        output = []
        for attr_name in vars(InputTypes):
            attr_value = getattr(InputTypes, attr_name)
            if "__" not in attr_name and isinstance(attr_value, str):
                output.append(attr_value)
        return output


DIGITAL_ZOOM_MAX = 4


class FrameGrabberConfig(ABC, BaseModel):
    # the allowed extra fields are for the fields specific to each input type
    model_config = ConfigDict(extra="forbid", validate_assignment=True)

    input_type: InputTypes
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
        if input_type not in cls.get_input_type_to_class_dict():
            raise ValueError(f"Invalid input type: {input_type}")
        return cls.get_input_type_to_class_dict()[input_type]

    @classmethod
    def get_input_type_to_id_dict(cls):
        """Each class has an id field like serial_number, rtsp_url, etc.
        This dictionary maps the input type to the id field"""
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
        """Convert the config to the framegrab standard format that we've defined"""
        dictionary_config = super().model_dump()
        dictionary_config["input_type"] = self.input_type.value

        # structure the id field like this: {"input_type": {"id_field": "id_value"}}
        id_field = self.get_input_type_to_id_dict()[self.input_type.value]
        dictionary_config["id"] = {id_field: getattr(self, id_field)}
        del dictionary_config[id_field]

        # these go in the options field
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
        framegrab standard format dictionary config and return them as a dictionary
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
            **options,  # theoretically this should be empty. but if it's not, we'll send it downstream so the unexpected option params are validated
        }

    @classmethod
    def from_framegrab_config_dict(cls, dictionary_config: dict) -> "FrameGrabber":
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
    resolution_width: Optional[int] = None
    resolution_height: Optional[int] = None

    @field_validator("resolution_height", mode="before")
    def validate_resolution(cls, v: int, info: dict):
        if info.config.get("resolution_width") is not None and v is None:
            raise ValueError("resolution_height must be provided if resolution_width is provided")
        return v

    def to_framegrab_config_dict(self) -> dict:
        base_dict = super().to_framegrab_config_dict()
        del base_dict["resolution_width"]
        del base_dict["resolution_height"]

        if self.resolution_width is not None and self.resolution_height is not None:
            base_dict["options"]["resolution"] = {"width": self.resolution_width, "height": self.resolution_height}

        return base_dict

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        data = copy.deepcopy(config_dict)

        options = data.get("options", {})
        if "resolution" in options:
            resolution = options.pop("resolution")
            data["resolution_width"] = resolution.get("width")
            data["resolution_height"] = resolution.get("height")

        return super().get_model_parameters(data)


class WithKeepConnectionOpenMixin(ABC, BaseModel):
    keep_connection_open: bool = Field(default=True)

    def update_framegrab_config_dict(self, dictionary_config: dict) -> dict:
        del dictionary_config["keep_connection_open"]
        dictionary_config.setdefault("options", {})["keep_connection_open"] = self.keep_connection_open
        return dictionary_config

    @classmethod
    def update_model_parameters(cls, model_parameters: dict) -> dict:
        data = copy.deepcopy(model_parameters)
        options = data.get("options", {})
        keep_connection_open = options.pop("keep_connection_open", True)
        if keep_connection_open is not None:
            data["keep_connection_open"] = keep_connection_open
        return data


class WithMaxFPSMixin(ABC, BaseModel):
    max_fps: Optional[int] = None

    def update_framegrab_config_dict(self, dictionary_config: dict) -> dict:
        del dictionary_config["max_fps"]
        dictionary_config.setdefault("options", {})["max_fps"] = self.max_fps
        return dictionary_config

    @classmethod
    def update_model_parameters(cls, model_parameters: dict) -> dict:
        data = copy.deepcopy(model_parameters)
        options = data.get("options", {})
        max_fps = options.pop("max_fps", 30)
        if max_fps:
            data["max_fps"] = max_fps
        return data


class GenericUSBFrameGrabberConfig(WithResolutionMixin):
    input_type: InputTypes = InputTypes.GENERIC_USB
    serial_number: Optional[str] = None
    _id_field_optional: ClassVar[bool] = True


class RTSPFrameGrabberConfig(FrameGrabberConfig, WithKeepConnectionOpenMixin, WithMaxFPSMixin):
    input_type: InputTypes = InputTypes.RTSP
    rtsp_url: str = Field(..., pattern=r"^rtsp://")

    def to_framegrab_config_dict(self) -> dict:
        base_dict = super().to_framegrab_config_dict()

        with_options_keep_connection_open = WithKeepConnectionOpenMixin.update_framegrab_config_dict(self, base_dict)
        with_options_max_fps = WithMaxFPSMixin.update_framegrab_config_dict(self, with_options_keep_connection_open)

        return with_options_max_fps

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        model_parameters_with_keep_connection_open = WithKeepConnectionOpenMixin.update_model_parameters(config_dict)
        model_parameters_with_max_fps = WithMaxFPSMixin.update_model_parameters(
            model_parameters_with_keep_connection_open
        )
        return FrameGrabberConfig.get_model_parameters(model_parameters_with_max_fps)


class BaslerFrameGrabberConfig(FrameGrabberConfig):
    input_type: InputTypes = InputTypes.BASLER
    serial_number: Optional[str] = None
    basler_options: Optional[dict] = None
    _id_field_optional: ClassVar[bool] = True

    def to_framegrab_config_dict(self) -> dict:
        dictionary_config = super().to_framegrab_config_dict()
        dictionary_config["options"]["basler_options"] = self.basler_options
        del dictionary_config["basler_options"]
        return dictionary_config

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        data = copy.deepcopy(config_dict)
        options = data.get("options", {})
        basler_options = options.pop("basler_options", {})
        new_data = {**data, "basler_options": basler_options}
        return super().get_model_parameters(new_data)


class RealSenseFrameGrabberConfig(WithResolutionMixin):
    input_type: InputTypes = InputTypes.REALSENSE
    serial_number: Optional[str] = None
    side_by_side_depth: Optional[bool] = False
    _id_field_optional: ClassVar[bool] = True

    def to_framegrab_config_dict(self) -> dict:
        base_dict = super().to_framegrab_config_dict()
        base_dict["options"]["depth"] = {"side_by_side": self.side_by_side_depth}
        return base_dict

    @classmethod
    def get_model_parameters(cls, config_dict: dict) -> dict:
        data = copy.deepcopy(config_dict)
        options = data.get("options", {})
        side_by_side_depth = options.pop("depth", {}).pop("side_by_side", False)
        if side_by_side_depth:
            data["side_by_side_depth"] = side_by_side_depth
        return super().get_model_parameters(data)


class HttpLiveStreamingFrameGrabberConfig(FrameGrabberConfig, WithKeepConnectionOpenMixin):
    input_type: InputTypes = InputTypes.HLS
    hls_url: str = Field(..., pattern=r"^https?://")


class RaspberryPiCSI2FrameGrabberConfig(FrameGrabberConfig):
    input_type: InputTypes = InputTypes.RPI_CSI2
    serial_number: Optional[str] = None


class YouTubeLiveFrameGrabberConfig(FrameGrabberConfig, WithKeepConnectionOpenMixin):
    input_type: InputTypes = InputTypes.YOUTUBE_LIVE
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
        base_dict = super().to_framegrab_config_dict()
        del base_dict["hls_url"]
        return base_dict


class FileStreamFrameGrabberConfig(FrameGrabberConfig):
    input_type: InputTypes = InputTypes.FILE_STREAM
    filename: str = Field(..., pattern=r"^[\w\-/]+\.mp4|mov|mjpeg$")
    max_fps: float = Field(default=0, ge=0)


class MockFrameGrabberConfig(WithResolutionMixin):
    input_type: InputTypes = InputTypes.MOCK
    serial_number: Optional[str] = None
    _id_field_optional: ClassVar[bool] = True
