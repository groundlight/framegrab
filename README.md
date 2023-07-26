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
To use a Basler USB or GigE camera, you must separately install the `pypylon` package.

Similarly, to use Intel RealSense cameras, you must install `pyrealsense2`. 

If you don't intend to use these camera types, you don't need to install these extra packages. 

## Usage

### Frame Grabbing

Simple usage with a single USB camera would look something like the following:

```
from framegrab import FrameGrabber

config = {
    'input_type': 'generic_usb',
}

grabber = FrameGrabber.create_grabber(config)

```
`config` can contain many details and settings about your camera, but only `input_type` is required. Available `input_type` options are: `generic_usb`, `rtsp`, `realsense` and `basler`.

Here's an example of a single USB camera configured with several options:
```
config = {
    'name': 'Front Door Camera',
    'input_type': 'generic_usb',
    'id': {
        'serial_number': 23432570
    },
    'options': {
        'resolution': {
            'height': 1080,
            'width': 1920,
        },
        'zoom': {
            'digital': 1.5
        }

    }
}

grabber = FrameGrabber.create_grabber(config)
```

To get a frame, simply run:
```
frame = grabber.grab()
```
You can also change the options after the grabber is created.
```
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
```
grabber.release()
```

You might have several cameras that you want to use in the same application. In this case, you can load the configurations from a yaml file and use `FrameGrabber.create_grabbers`.

If you have multiple cameras of the same type plugged in, it's recommended that you include serial numbers in the configurations; this ensures that each configuration is paired with the correct camera. If you don't provide serial numbers in your configurations, configurations will be paired with cameras in a sequential manner.

Below is a sample yaml file containing configurations for three different cameras.
```
GL_CAMERAS: |
  - name: on robot arm
    input_type: realsense
    options: 
      depth:
        side_by_side: 1
      crop:
        relative:
          right: .8
  - name: conference room
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
  - name: workshop
    input_type: generic_usb
    id:
      serial_number: B77D3A8F
```
You can load the configurations from the yaml file and use the cameras in the following manner.
```
from framegrab import FrameGrabber
import yaml

# load the configurations from yaml
config_path = 'camera_config.yaml'
with open(config_path, 'r') as f:
    data = yaml.safe_load(f)
    configs = yaml.safe_load(data['GL_CAMERAS'])

# create the grabbers
grabbers = FrameGrabber.create_grabbers(configs)

for grabber in grabbers.values():
    print(grabber.config)
    frame = grabber.grab()
    display_image(frame) # substitute this line for your preferred method of displaying images, such as cv2.imshow
    grabber.release()
```
### Configurations
The table below shows all available configurations and the cameras to which they apply.
| Configuration Name         | Example         | Webcam     | RTSP      | Basler    | Realsense |
|----------------------------|-----------------|------------|-----------|-----------|-----------|
| name                       | On Robot Arm    | optional   | optional  | optional  | optional  |
| input_type                 | generic_usb    | required   | required  | required  | required  |
| id.serial_number           | 23458234       | optional   | -         | optional  | optional  |
| id.rtsp_url                | rtsp://…        | -          | required  | -         | -         |
| options.resolution.height  | 480            | optional   | -         | -         | optional  |
| options.resolution.width   | 640            | optional   | -         | -         | optional  |
| options.zoom.digital       | 1.3            | optional   | optional  | optional  | optional  |
| options.crop.pixels.top    | 100            | optional   | optional  | optional  | optional  |
| options.crop.pixels.bottom | 400            | optional   | optional  | optional  | optional  |
| options.crop.pixels.left   | 100            | optional   | optional  | optional  | optional  |
| options.crop.pixels.right  | 400            | optional   | optional  | optional  | optional  |
| options.crop.relative.top  | 0.1            | optional   | optional  | optional  | optional  |
| options.crop.relative.bottom | 0.9          | optional   | optional  | optional  | optional  |
| options.crop.relative.left | 0.1            | optional   | optional  | optional  | optional  |
| options.crop.relative.right | 0.9            | optional   | optional  | optional  | optional  |
| options.depth.side_by_side | 1              | -          | -         | -         | optional  |
| options.num_90_deg_rotations | 2              | optional          | optional         | optional         | optional  |

In addition to the configurations in the table above, you can set any Basler camera property by including `options.basler.<BASLER PROPERTY NAME>`. For example, it's common to set `options.basler.PixelFormat` to `RGB8`.

### Autodiscovery
Autodiscovery automatically connects to all cameras that are plugged into your machine or discoverable on the network, including `generic_usb`, `realsense` and `basler` cameras. Default configurations will be loaded for each camera. Please note that RTSP streams cannot be discovered in this manner; RTSP URLs must be specified in the configurations.

Autodiscovery is great for simple applications where you don't need to set any special options on your cameras. It's also a convenient method for finding the serial numbers of your cameras (if the serial number isn't printed on the camera).
```
grabbers = FrameGrabber.autodiscover()

# Print some information about the discovered cameras
for grabber in grabbers.values():
    print(grabber.config)

    grabber.release()
```

### Motion Detection

To use the built-in motion detection functionality, first create a `MotionDetector` object, specifying the percentage threshold for motion detection:

```
from framegrab import MotionDetector

motion_threshold = 1.0
m = MotionDetector(pct_threshold=motion_threshold)
```

The motion threshold is defined as the detection threshold for motion detection, in terms of the percentage of changed pixels. The default value is 1.0 (which means 1%).

Then, use the `motion_detected()` method with a captured frame to check if motion has been detected:

```
if m.motion_detected(frame):
    print("Motion detected!")
```

## Examples

Here's an example of using the FrameGrab library to continuously capture frames and detect motion from a video stream:

```
from framegrab import FrameGrabber, MotionDetector

motion_threshold = 1.0

config = {
    'input_type': 'webcam',
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

FrameGrab is released under the MIT License. For more information, please refer to the [LICENSE](LICENSE) file.


