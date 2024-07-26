# FrameGrab by Groundlight
## A user-friendly library for grabbing images from cameras or streams

FrameGrab is an open-source Python library designed to make it easy to grab frames (images) from cameras or streams. The library supports generic USB cameras (such as webcams), RTSP streams, Basler USB cameras, Basler GigE cameras, and Intel RealSense depth cameras.

FrameGrab also provides basic motion detection functionality. FrameGrab requires Python 3.7 or higher.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

## Installation

To install the FrameGrab library, simply run:

```
pip install framegrab
```

## Optional Dependencies
Certain camera types have additional dependencies that must be installed separately. If you don't intend to use these camera types, you don't need to install these extra packages. 

- To use a Basler USB or GigE camera, you must separately install the `pypylon` package.
- To use Intel RealSense cameras, you must install `pyrealsense2`.
- To use a Raspberry Pi "CSI2" camera (connected with a ribbon cable), you must install the `picamera2` library. See install instructions at the [picamera2 github repository](https://github.com/raspberrypi/picamera2).


## Usage

### Command line interface (CLI)

There is a simple CLI for `framegrab` to discover and preview configurations.

```
framegrab
```

lists the sub-commands, including `autodiscover` and `preview`.

### Frame Grabbing

Frame Grabbers are defined by a configuration dict which is usually stored as YAML.  The configuration combines the camera type, the camera ID, and the camera options.  The configuration is passed to the `FrameGrabber.create_grabber` method to create a grabber object.  The grabber object can then be used to grab frames from the camera.


`config` can contain many details and settings about your camera, but only `input_type` is required. Available `input_type` options are: `generic_usb`, `rtsp`, `realsense`, `basler`, and `rpi_csi2`.

Here's an example of a single USB camera configured with several options:
```python
config = """
name: Raspberry Pi Ribbon Cable Camera
input_type: rpi_csi2
options:
    resolution:
        height: 720
        width: 1280
    zoom:
        digital: 1.5
"""

grabber = FrameGrabber.create_grabber_yaml(config)
```

To get a frame, simply run:
```python
frame = grabber.grab()
```
You can also change the options after the grabber is created.
```python
new_options = {
    'resolution': {
        'height': 480,
        'width': 640,
    },
    'crop': {
        'relative': {
            'top': .1,
            'bottom': .9,
            'left': .1,
            'right': .9,
        }
    }
}

grabber.apply_options(new_options)
```

When you are done with the camera, release the resource by running:
```python
grabber.release()
```

You might have several cameras that you want to use in the same application. In this case, you can load the configurations from a yaml file and use `FrameGrabber.create_grabbers`. Note that currently only a single Raspberry Pi CSI2 camera is supported, but these cameras can be used in conjunction with other types of cameras. 

If you have multiple cameras of the same type plugged in, it's recommended that you include serial numbers in the configurations; this ensures that each configuration is paired with the correct camera. If you don't provide serial numbers in your configurations, configurations will be paired with cameras in a sequential manner.

Below is a sample yaml file containing configurations for three different cameras.
```yaml
image_sources: 
  - name: On Robot Arm
    input_type: basler
    id:
      serial_number: A24P1V4T
    options:
      crop:
        relative:
          top: 0.3
          right: 0.8
  - name: Chip Bin
    input_type: rtsp
    id:
      rtsp_url: rtsp://admin:password@192.168.1.20/cam/realmonitor?channel=1&subtype=0
    options:
      crop:
        pixels:
          top: 350
          bottom: 1100
          left: 1100
          right: 2000
  - name: Over CNC Machine
    input_type: generic_usb
    id:
      serial_number: B77D3A8F
```
You can load the configurations from the yaml file and use the cameras in the following manner.
```python
from framegrab import FrameGrabber

config_path = 'camera_config.yaml'
grabbers = FrameGrabber.from_yaml(config_path)

for grabber in grabbers.values():
    print(grabber.config)
    frame = grabber.grab()
    display_image(frame) # substitute this line for your preferred method of displaying images, such as cv2.imshow
    grabber.release()
```
### Configurations
The table below shows all available configurations and the cameras to which they apply.
| Configuration Name         | Example         | Generic USB     | RTSP      | Basler    | Realsense | Raspberry Pi CSI2 |
|----------------------------|-----------------|------------|-----------|-----------|-----------|-----------|
| name                       | On Robot Arm    | optional   | optional  | optional  | optional  | optional  |
| input_type                 | generic_usb    | required   | required  | required  | required  | required  |
| id.serial_number           | 23458234       | optional   | -         | optional  | optional  | -  |
| id.rtsp_url                | rtsp://…        | -          | required  | -         | -         | -         |
| options.resolution.height  | 480            | optional   | -         | -         | optional  | -  |
| options.resolution.width   | 640            | optional   | -         | -         | optional  | -  |
| options.zoom.digital       | 1.3            | optional   | optional  | optional  | optional  | optional  |
| options.crop.pixels.top    | 100            | optional   | optional  | optional  | optional  | optional  |
| options.crop.pixels.bottom | 400            | optional   | optional  | optional  | optional  | optional  |
| options.crop.pixels.left   | 100            | optional   | optional  | optional  | optional  | optional  |
| options.crop.pixels.right  | 400            | optional   | optional  | optional  | optional  | optional  |
| options.crop.relative.top  | 0.1            | optional   | optional  | optional  | optional  | optional  |
| options.crop.relative.bottom | 0.9          | optional   | optional  | optional  | optional  | optional  |
| options.crop.relative.left | 0.1            | optional   | optional  | optional  | optional  | optional  |
| options.crop.relative.right | 0.9            | optional   | optional  | optional  | optional  | optional  |
| options.depth.side_by_side | 1              | -          | -         | -         | optional  | -  |
| options.num_90_deg_rotations | 2              | optional          | optional         | optional         | optional  | optional  |
| options.keep_connection_open | True              | -          | optional         | -         | -  | - |
| options.max_fps | 30              | -          | optional         | -         | -  | - |




In addition to the configurations in the table above, you can set any Basler camera property by including `options.basler.<BASLER PROPERTY NAME>`. For example, it's common to set `options.basler.PixelFormat` to `RGB8`.

### Autodiscovery
Autodiscovery automatically connects to cameras that are plugged into your machine or discoverable on the network, including `generic_usb`, `realsense`, `basler`, and ONVIF supported `rtsp` cameras. Note that `rpi_csi2` cameras are not yet supported by autodiscover. Default configurations will be loaded for each camera. Note that discovery of RTSP cameras will be disabled by default but can be enabled by setting `rtsp_discover_mode`. Refer to [RTSP Discovery](#rtsp-discovery) section for different options.

Autodiscovery is great for simple applications where you don't need to set any special options on your cameras. It's also a convenient method for finding the serial numbers of your cameras (if the serial number isn't printed on the camera).
```python
grabbers = FrameGrabber.autodiscover()

# Print some information about the discovered cameras
for grabber in grabbers.values():
    print(grabber.config)

    grabber.release()
```

#### RTSP Discovery
RTSP cameras with support for ONVIF can be discovered on your local network in the following way:

```python
from framegrab import RTSPDiscovery, ONVIFDeviceInfo
        
devices = RTSPDiscovery.discover_onvif_devices()
```

The `discover_onvif_devices()` will provide a list of devices that it finds in the `ONVIFDeviceInfo` format. An optional mode `auto_discover_mode` can be used to try different default credentials to fetch RTSP URLs:

- off: No discovery.
- ip_only: Only discover the IP address of the camera.
- light: Only try first two usernames and passwords ("admin:admin" and no username/password).
- complete_fast: Try the entire DEFAULT_CREDENTIALS without delays in between. 
- complete_slow: Try the entire DEFAULT_CREDENTIALS with a delay of 1 seconds in between.


After getting the list and enter the username and password of the camera. Use `generate_rtsp_urls()` to generate RTSP URLs for each devices.

```python
for device in devices:
    RTSPDiscovery.generate_rtsp_urls(device=device)
```

This will generate all the available RTSP URLs and can be used when creating `FrameGrabber.create_grabbers` to grab frames.

```python
config = f"""
name: Front Door Camera
input_type: rtsp
id:
  rtsp_url: {device.rtsp_urls[0]}
"""

grabber = FrameGrabber.create_grabber_yaml(config)
```

### Motion Detection

To use the built-in motion detection functionality, first create a `MotionDetector` object, specifying the percentage threshold for motion detection:

```python
from framegrab import MotionDetector

motion_threshold = 1.0
m = MotionDetector(pct_threshold=motion_threshold)
```

The motion threshold is defined as the detection threshold for motion detection, in terms of the percentage of changed pixels. The default value is 1.0 (which means 1%).

Then, use the `motion_detected()` method with a captured frame to check if motion has been detected:

```python
if m.motion_detected(frame):
    print("Motion detected!")
```

## Examples

Here's an example of using the FrameGrab library to continuously capture frames and detect motion from a video stream:

```python
from framegrab import FrameGrabber, MotionDetector

motion_threshold = 1.0

config = {
    'input_type': 'generic_usb',
}
grabber = FrameGrabber.create_grabber(config)
m = MotionDetector(pct_threshold=motion_threshold)

while True:
    frame = grabber.grab()
    if frame is None:
        print("No frame captured!")
        continue

    if m.motion_detected(frame):
        print("Motion detected!")
```

## Contributing

We welcome contributions to FrameGrab! If you would like to contribute, please follow these steps:

1. Fork the repository
2. Create a new branch for your changes
3. Commit your changes to the branch
4. Open a pull request

## License

FrameGrab is released under the MIT License. For more information, please refer to the [LICENSE.txt](https://github.com/groundlight/framegrab/blob/main/LICENSE.txt) file.


