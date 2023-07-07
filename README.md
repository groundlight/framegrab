# FrameGrab by Groundlight
## A user-friendly library for grabbing images from cameras or streams

FrameGrab is an open-source Python library designed to make it easy to grab frames (images) from cameras or streams. The library supports webcams, RTSP streams, Basler USB cameras and Intel RealSense depth cameras.

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

## Usage

### Frame Grabbing

Simple usage with a single webcam would look something like the following:

```
from framegrab import FrameGrabber

config = {
    'input_type': 'webcam',
}

grabber = FrameGrabber.create_grabber(config)

```
`config` can contain many details and settings about your camera, but only `input_type` is required. Available `input_type` options are: `webcam`, `rtsp`, `realsense` and `basler_usb`.

Here's an example of a single webcam configured with several options:
```
config = {
    'name': 'front door camera',
    'input_type': 'webcam',
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

If you have multiple cameras of the same type plugged in, it's recommended to provide serial numbers in the configurations; this ensures that each configuration is paired with the correct camera. If you don't provide serial numbers in your configurations, configurations will be paired with cameras in a sequential manner.

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
      address: 
        rtsp_url: rtsp://admin:password@192.168.1.20/cam/realmonitor?channel=1&subtype=0
      options:
        crop:
          absolute:
            top: 350
            bottom: 1100
            left: 1100
            right: 2000
  - name: workshop
    input_type: webcam
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
    display_image(frame)
    grabber.release()
```
It is also possible to 'autodiscover' cameras. This will automatically connect to all cameras that are plugged into your machine, such as `webcam`, `realsense` and `basler_usb` cameras. Default configurations will be loaded for each camera. Please note that RTSP streams cannot be discovered in this manner; RTSP URLs must be specified in the configurations.
```
grabbers = FrameGrabber.autodiscover()
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


