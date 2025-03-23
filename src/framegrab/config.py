from pydantic import BaseModel, Field, validator
from typing import Optional, Dict
from enum import Enum

class InputTypes(str, Enum):
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

    # The field needed in the config for each input type
    REQUIRED_ID_FIELDS = {
        GENERIC_USB: "serial_number",
        RTSP: "rtsp_url",
        REALSENSE: "serial_number",
        BASLER: "serial_number",
        RPI_CSI2: "serial_number",
        HLS: "hls_url",
        YOUTUBE_LIVE: "youtube_url",
        FILE_STREAM: "filename",
        MOCK: "serial_number",
    }

    @staticmethod
    def get_options() -> list:
        """Get a list of the available InputType options"""
        return [item.value for item in InputTypes]

class CameraOptions(BaseModel):
    resolution: Optional[Dict[str, int]] = None
    crop: Optional[Dict[str, Dict[str, float]]] = None
    zoom: Optional[Dict[str, float]] = None
    num_90_deg_rotations: Optional[int] = 0
    keep_connection_open: Optional[bool] = True
    max_fps: Optional[float] = None

class FrameGrabberConfig(BaseModel):
    input_type: InputTypes
    id: CameraID
    options: Optional[CameraOptions] = None
    name: Optional[str] = None

    @validator('id')
    def validate_id(cls, v, values):
        input_type = values.get('input_type')
        required_field = InputTypes.REQUIRED_ID_FIELDS.get(input_type)
        if not required_field:
            raise ValueError(f"No required ID field for input type {input_type}")
        if not v:
            raise ValueError(f"ID must be provided for input type {input_type}")
        return v

