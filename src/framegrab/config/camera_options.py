from enum import Enum
from typing import Dict, Optional

from pydantic import BaseModel, create_model, validator

DIGITAL_ZOOM_MAX = 4


class CameraOptionsGeneric(BaseModel):
    """Configuration options for camera settings."""

    crop: Optional[Dict[str, Dict[str, float]]] = None
    digital_zoom: Optional[confloat(ge=1, le=DIGITAL_ZOOM_MAX)] = None
    num_90_deg_rotations: Optional[int] = 0
    keep_connection_open: Optional[bool] = True

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

    def to_dict(self) -> dict:
        base_dict = self.dict()
        if self.zoom_digital is not None:
            base_dict["digital"] = {"zoom": self.digital_zoom}
            del base_dict["digital_zoom"]
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        if "digital" in data:
            digital = data.pop("digital")
            data["digital_zoom"] = digital.get("zoom")
        return cls(**data)


class CameraOptionsWithResolution(CameraOptionsGeneric):
    resolution_width: Optional[int] = None
    resolution_height: Optional[int] = None

    @validator("resolution_height", always=True)
    def validate_resolution(cls, v, values):
        if values.get("resolution_width") is not None and v is None:
            raise ValueError("resolution_height must be provided if resolution_width is provided")
        return v

    def to_dict(self) -> dict:
        base_dict = super().dict()
        if self.resolution_width is not None and self.resolution_height is not None:
            base_dict["resolution"] = {"width": self.resolution_width, "height": self.resolution_height}
            del base_dict["resolution_width"]
            del base_dict["resolution_height"]
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        if "resolution" in data:
            resolution = data.pop("resolution")
            data["resolution_width"] = resolution.get("width")
            data["resolution_height"] = resolution.get("height")
        return cls(**data)


class CameraOptionsBasler(CameraOptionsGeneric):
    # Should we validate these or let the basler library do it?
    basler: Optional[Dict[str, Any]] = None


class CameraOptionsRTSP(CameraOptionsGeneric):
    max_fps: Optional[confloat(ge=0)] = 30
    keep_connection_open: Optional[bool] = True
