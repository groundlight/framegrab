# FrameGrab by Groundlight
## A user-friendly library for grabbing images from cameras or streams

FrameGrab is an open-source Python library designed to make it easy to grab frames (images) from cameras or streams. The library supports generic USB cameras (such as webcams), RTSP streams, Basler USB cameras, Basler GigE cameras, Intel RealSense depth cameras, and video file streams (mp4, mov, mjpeg, avi, etc.).

FrameGrab also provides basic motion detection functionality. FrameGrab requires Python 3.9 or higher.

## Table of Contents

- [FrameGrab by Groundlight](#framegrab-by-groundlight)
  - [A user-friendly library for grabbing images from cameras or streams](#a-user-friendly-library-for-grabbing-images-from-cameras-or-streams)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
  - [Optional Dependencies](#optional-dependencies)
  - [Usage](#usage)
    - [Command line interface (CLI)](#command-line-interface-cli)
    - [Frame Grabbing](#frame-grabbing)
    - [Configurations](#configurations)
    - [Autodiscovery](#autodiscovery)
      - [RTSP Discovery](#rtsp-discovery)
    - [Motion Detection](#motion-detection)
  - [Examples](#examples)
    - [Generic USB](#generic-usb)
    - [YouTube Live](#youtube-live)
    - [File Stream](#file-stream)
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
- To use a YouTube Live stream, you must install `streamlink`.

We provide optional extras to install these dependencies. For example, to install the Basler camera dependencies, run:
```
pip install framegrab[basler]
```

To install YouTube Live stream dependencies, run:
```
pip install framegrab[youtube]
```

To install all optional dependencies, run:
```
pip install framegrab[all]
```


## Usage

### Command line interface (CLI)

FrameGrab provides a CLI for discovering and previewing cameras.

**Auto-detect source type:**
```bash
framegrab preview rtsp://admin:password@192.168.1.20/stream0    # RTSP camera
framegrab preview camera_config.yaml                            # YAML config file
```

**Explicit input type:**
```bash
framegrab preview 12345678901 -i generic_usb                    # USB camera by serial number
framegrab preview 35432343252 -i basler                         # Basler camera
```

**Discovery:**
```bash
framegrab autodiscover                                          # Find all cameras
```

**Output formats:**
The CLI supports different ways to display camera frames using the `-o/--output` option:

- `imgcat` (default): Displays images directly in compatible terminals like iTerm2
- `cv2`: Opens frames in an OpenCV window (requires GUI)  
- `ascii`: Shows frames as ASCII art in the terminal
- `none`: Captures frames but doesn't display them

```bash
framegrab preview rtsp://camera-url -o cv2                      # OpenCV window
framegrab preview camera_config.yaml -o ascii                   # ASCII art  
framegrab autodiscover -o none                                  # No display
```

**Help:**
```bash
framegrab --help                                                # Show all commands
framegrab preview --help                                        # Show preview options
```

### Frame Grabbing

Frame Grabbers are defined by a configuration parameter. This parameter can take several forms: a python FrameGrabConfig object, a python dictionary object, a yaml string, or a yaml file.  The configuration combines the camera type, the camera ID, and the camera options.  The configuration is passed to the `FrameGrabber.create_grabber` method to create a grabber object.  The grabber object can then be used to grab frames from the camera.

`config` can contain many details and settings about your camera must contain information about the `input_type` of the camera. Available `input_type` options are: `generic_usb`, `rtsp`, `realsense`, `basler`, `rpi_csi2`, `hls`, `youtube_live`, and `file_stream`.

Here's an example of a single USB camera configured with several options:
```python
from framegrab.config import GenericUSBFrameGrabberConfig, InputTypes
config = GenericUSBFrameGrabberConfig(serial_number="1234567890", resolution_width=1280,resolution_height=720, digital_zoom=1.5)
grabber = FrameGrabber.create_grabber(config)
```
You can also create a config using the `FrameGrabberConfig.create` method:
```python
from framegrab import FrameGrabber
from framegrab.config import FrameGrabberConfig
config = FrameGrabberConfig.create(input_type=InputTypes.GENERIC_USB, serial_number="1234567890", resolution_width=1280,resolution_height=720, digital_zoom=1.5)
grabber = FrameGrabber.create_grabber(config)
```

If you'd prefer to store your config as a yaml you can use the framegrab serialized format (we use the same format creating with dictionaries as well)
```python
config = """
name: Generic USB Camera
input_type: generic_usb
id:
    serial_number: 1234567890
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

You can also change the options after the grabber is created. The options passed in
must be a dictionary according to the framegrabber serialized format.
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

Alternatively, you can use a context manager which will automatically release the camera resources:
```python
with FrameGrabber.create_grabber_yaml(config) as grabber:
    frame = grabber.grab()
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
#### Dictionary or YamlFormat
The table below shows all available configurations and the cameras to which they apply. This is the format used when creating grabbers from dictionaries or yaml strings. If you want to use the python FrameGrabberConfig object, the format is slightly different to optimize for python readability (see format and examples). We use the python pydantic model to validate the dictionary and yaml format
so if you want to know the correct format of your parameters, you can read more in config.py, or 
use the python pydantic model to validate your configuration.


| Configuration Name         | Example         | Generic USB     | RTSP      | Basler    | Realsense | Raspberry Pi CSI2 | HLS | YouTube Live | File Stream |
|----------------------------|-----------------|------------|-----------|-----------|-----------|-----------|-----------|-----------|-------------|
| name                       | On Robot Arm    | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| input_type                 | generic_usb    | required   | required  | required  | required  | required  | required | required | required |
| id.serial_number           | 23458234       | optional   | -         | optional  | optional  | -  | - | - | - |
| id.rtsp_url                | rtsp://…        | -          | required  | -         | -         | -         | - | - | - |
| id.hls_url                 | https://.../*.m3u8     | -          | -         | -         | -         | -         | required | - | - |
| id.youtube_url             | https://www.youtube.com/watch?v=...     | -          | -         | -         | -         | -         | - | required | - |
| id.filename               | http://.../*.mp4 | -          | -         | -         | -         | -         | - | - | required |
| options.resolution.height  | 480            | optional   | -         | -         | optional  | -  | - | - | - |
| options.resolution.width   | 640            | optional   | -         | -         | optional  | -  | - | - | - |
| options.zoom.digital       | 1.3            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.pixels.top    | 100            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.pixels.bottom | 400            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.pixels.left   | 100            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.pixels.right  | 400            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.relative.top  | 0.1            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.relative.bottom | 0.9          | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.relative.left | 0.1            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.crop.relative.right | 0.9            | optional   | optional  | optional  | optional  | optional  | optional | optional | optional |
| options.depth.side_by_side | 1              | -          | -         | -         | optional  | -  | - | - | - |
| options.num_90_deg_rotations | 2              | optional          | optional         | optional         | optional  | optional  | optional | optional | optional |
| options.keep_connection_open | True              | -          | optional         | -         | -  | - | optional | optional | - |
| options.max_fps | 30              | -          | optional         | -         | -  | - | - | - | optional |


In addition to the configurations in the table above, you can set any Basler camera property by including `options.basler.<BASLER PROPERTY NAME>`. For example, it's common to set `options.basler.PixelFormat` to `RGB8`.

#### python FrameGrabberConfig format
You can also specify a configuration using the python FrameGrabberConfig object. This is useful if 
you want quick validation of your configuration and your framegrab data stored under a validated, centralized object.

<!-- start configuration table -->

<div style="overflow-x: auto;">

<table>
  <thead>
    <tr>
      <th>Configuration Name</th>
      <th>Type</th>
      <th>generic_usb</th>
      <th>rtsp</th>
      <th>realsense</th>
      <th>basler</th>
      <th>rpi_csi2</th>
      <th>hls</th>
      <th>youtube_live</th>
      <th>file</th>
      <th>mock</th>
      <th>ros2</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>id.filename</td>
      <td>string</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>id.hls_url</td>
      <td>string</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>id.rtsp_url</td>
      <td>string</td>
      <td>-</td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>id.serial_number</td>
      <td>string</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
    </tr>
    <tr>
      <td>id.topic</td>
      <td>string</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
    </tr>
    <tr>
      <td>id.youtube_url</td>
      <td>string</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>input_type</td>
      <td>string</td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
      <td><span style="color: #22c55e; font-weight: bold;">required</span></td>
    </tr>
    <tr>
      <td>name</td>
      <td>string</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
    </tr>
    <tr>
      <td>options.basler_options</td>
      <td>dict</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.crop</td>
      <td>dict</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
    </tr>
    <tr>
      <td>options.depth.side_by_side</td>
      <td>bool</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.fourcc</td>
      <td>string</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.fps</td>
      <td>int</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.keep_connection_open</td>
      <td>bool</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.max_fps</td>
      <td>int</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.resolution.height</td>
      <td>int</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.resolution.width</td>
      <td>int</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.rotation.num_90_deg_rotations</td>
      <td>int</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
    </tr>
    <tr>
      <td>options.video_stream</td>
      <td>bool</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
    <tr>
      <td>options.zoom.digital</td>
      <td>float</td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
      <td><span style="color: #6b7280;">optional</span></td>
    </tr>
  </tbody>
</table>

</div>

