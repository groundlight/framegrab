"""
This script generates a YAML file documenting the available parameters for each camera options Pydantic model.
It is intended for documentation generation purposes, allowing users to understand the configuration options
available when creating a FrameGrabber object. The script writes the schema of each model to a specified output
file, providing a reference for developers and users.
"""

import os
import yaml
from camera_options import (
    RaspberryPiCSI2Options,
    HttpLiveStreamingOptions,
    YouTubeLiveOptions,
    FileStreamOptions,
    RTSPOptions,
    CameraOptionsBasler,
    CameraOptionsRealSense,
    CameraOptionsGenericUSB
)

def write_schema_to_yaml(models, output_file):
    with open(output_file, 'w') as f:
        f.write("# This YAML file documents the avialable parameters for each camera options pydantic model.\n")
        f.write("# When creating a framegrabber object, you can pass in these models as the camera_options parameter.\n\n")
        f.write("# Example RTSPOptions object:\n")
        f.write("# camera_options = RTSPOptions(crop={'relative': {'top': 0.1, 'bottom': 0.1}}, digital_zoom=2)\n")
        f.write("# config = FrameGrabberConfig(input_type=InputTypes.RTSP, rtsp_url='rtsp://example.com/stream', options=camera_options)\n")
        f.write("# grabber = create_grabber(config=config)\n\n")

        for model in models:
            schema = model.model_json_schema()
            yaml.dump({f"{model.__name__}": schema}, f, default_flow_style=False)
            f.write("\n")

if __name__ == "__main__":
    models = [
        RaspberryPiCSI2Options,
        HttpLiveStreamingOptions,
        YouTubeLiveOptions,
        FileStreamOptions,
        RTSPOptions,
        CameraOptionsBasler,
        CameraOptionsRealSense,
        CameraOptionsGenericUSB
    ]
    output_file = "camera_options_schema.yaml"
    write_schema_to_yaml(models, output_file)
    print(f"Schema written to {output_file}")