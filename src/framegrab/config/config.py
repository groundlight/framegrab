from pydantic import BaseModel, validator, create_model, confloat
from typing import Optional, Dict
from enum import Enum
from config.camera_options import (
    CameraOptionsBasler,
    CameraOptionsRealSense,
    CameraOptionsGenericUSB,

)

@dataclass
class IDFieldConfig:
    field_name: str
    is_required: bool

class CameraID(Enum):
    """Enumeration of possible camera identifiers."""
    SERIAL_NUMBER = "serial_number"
    RTSP_URL = "rtsp_url"
    HLS_URL = "hls_url"
    YOUTUBE_URL = "youtube_url"
    FILENAME = "filename"

class InputTypes(str, Enum):
    """Defines the available input types for FrameGrabber objects."""
    GENERIC_USB = "generic_usb"
    RTSP = "rtsp"
    REALSENSE = "realsense"
    BASLER = "basler"
    RPI_CSI2 = "rpi_csi2"
    HLS = "hls"
    YOUTUBE_LIVE = "youtube_live"
    FILE_STREAM = "file_stream"
    MOCK = "mock"

    ID_FIELDS = {
        InputTypes.GENERIC_USB: IDFieldConfig(field_name=CameraID.SERIAL_NUMBER.value, is_required=False),
        InputTypes.RTSP: IDFieldConfig(field_name=CameraID.RTSP_URL.value, is_required=True),
        InputTypes.REALSENSE: IDFieldConfig(field_name=CameraID.SERIAL_NUMBER.value, is_required=True),
        InputTypes.BASLER: IDFieldConfig(field_name=CameraID.SERIAL_NUMBER.value, is_required=False),
        InputTypes.RPI_CSI2: IDFieldConfig(field_name=CameraID.SERIAL_NUMBER.value, is_required=False),
        InputTypes.HLS: IDFieldConfig(field_name=CameraID.HLS_URL.value, is_required=True),
        InputTypes.YOUTUBE_LIVE: IDFieldConfig(field_name=CameraID.YOUTUBE_URL.value, is_required=True),
        InputTypes.FILE_STREAM: IDFieldConfig(field_name=CameraID.FILENAME.value, is_required=True),
        InputTypes.MOCK: IDFieldConfig(field_name=CameraID.SERIAL_NUMBER.value, is_required=False),
    }

    CAMERA_OPTIONS_FOR_INPUT_TYPE = {
        InputTypes.RTSP: CameraOptionsRTSP,
        InputTypes.REALSENSE: CameraOptionsWithResolution,
        InputTypes.BASLER: CameraOptionsBasler,
        InputTypes.RPI_CSI2: CameraOptionsGeneric,
        InputTypes.HLS: CameraOptionsGeneric,
        InputTypes.YOUTUBE_LIVE: CameraOptionsGeneric,
        InputTypes.FILE_STREAM: CameraOptionsRTSP,
        InputTypes.MOCK: CameraOptionsGeneric,
    }

    @staticmethod
    def get_options() -> list:
        """Returns a list of available InputType options."""
        return [item.value for item in InputTypes]


# Dynamically create a model to expose all camera ID fields as potential fields for FrameGrabberConfig
# That way you can specify "rtsp_url" or "serial_number" or "filename" etc on the config model.
CameraIDFields = create_model(
    'CameraIDFields',
    **{id_.value: (Optional[str], None) for id_ in CameraID}
)

class FrameGrabberConfig(BaseModel):
    """Configuration model for FrameGrabber."""
    input_type: InputTypes
    options: Optional[CameraOptions] = None
    name: Optional[str] = None

    # Include all CameraID fields
    __annotations__ = {**CameraIDFields.__annotations__}

    @validator('serial_number', 'rtsp_url', 'hls_url', 'youtube_url', 'filename', pre=True, always=True)
    def validate_id(cls, v, values, field):
        """Validator to ensure the correct ID field is set based on input type."""
        input_type = values.get('input_type')
        id_field_config = InputTypes.ID_FIELDS.get(input_type)

        if field.name == required_field.field_name:
            if required_field.is_required and not v:
                raise ValueError(f"{field.name} must be provided for input type {input_type}")
        else:
            if v is not None:
                raise ValueError(f"{field.name} should not be set for input type {input_type}")

        return v
    
    @validator('options', pre=True, always=True)
    def validate_options(cls, v, values):
        """Validator to ensure options are of the correct type based on input type."""
        input_type = values.get('input_type')
        expected_options_class = InputTypes.CAMERA_OPTIONS_FOR_INPUT_TYPE.get(input_type)
        
        if expected_options_class is None:
            raise ValueError(f"Invalid input type: {input_type}")

        if not isinstance(v, expected_options_class):
            raise ValueError(f"options must be of type {expected_options_class.__name__} for input type {input_type}")
        return v

# Example usage:
# config = FrameGrabberConfig(
#     input_type=InputTypes.RTSP,
#     rtsp_url="rtsp://example.com/stream",
#     options=CameraOptions(resolution={"width": 1920, "height": 1080})
# )