# FrameGrab by Groundlight
## A user-friendly library for grabbing images from cameras or streams

FrameGrab is an open-source Python library designed to make it easy to grab frames (images) from cameras or streams. The library supports RTSP streams, YouTube streams, or local USB cameras through OpenCV, as well as providing basic motion detection functionality. FrameGrab requires Python 3.7 or higher.

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

To start grabbing frames from a camera or stream, first create a `FrameGrabber` object, specifying the desired stream source and target FPS:

```
from framegrab import FrameGrabber

STREAM = 'your_stream_here'
FPS = 5

grabber = FrameGrabber.create_grabber(stream=STREAM, fps_target=FPS)
```

Replace `'your_stream_here'` with the desired stream source. The `STREAM` variable can be an integer representing the device number, a filename, or a URL of a video stream (e.g., `rtsp://host:port/script?params`, `movie.mp4`, or `*.jpg`).

To grab a frame, simply call the `grab()` method:

```
frame = grabber.grab()
if frame is None:
    print("No frame captured!")
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

STREAM = 'your_stream_here'
FPS = 5
motion_threshold = 1.0

grabber = FrameGrabber.create_grabber(stream=STREAM, fps_target=FPS)
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


