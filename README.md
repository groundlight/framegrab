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

**Note on GStreamer:** The `gstreamer` extra (`pip install framegrab[gstreamer]`) is a placeholder that documents the requirement but doesn't install GStreamer support (which requires system-level OpenCV). See the [GStreamer Support](#gstreamer-support-for-rtsp-streaming) section below for installation instructions.

## GStreamer Support for RTSP Streaming

The RTSP input type in FrameGrab supports two backends:
1. **FFmpeg backend** (default) - Works with standard `pip install opencv-python`
2. **GStreamer backend** - Recommended for low-latency, real-time RTSP streaming

### Why GStreamer for RTSP?

The GStreamer backend provides:
- ✅ **Zero-buffering** with leaky queues for always-fresh frames
- ✅ **Lower latency** (~60-80ms vs 200-500ms with FFmpeg)
- ✅ **Better network handling** with configurable timeouts
- ✅ **Frame rate limiting** at the pipeline level

### Installing OpenCV with GStreamer Support

By default, `pip install opencv-python` includes only FFmpeg support. To use GStreamer, you need OpenCV compiled with GStreamer libraries:

#### Option 1: Using Conda (Recommended - Easiest) ⭐

```bash
# Create a new environment with OpenCV that has GStreamer support
conda create -n framegrab python=3.10
conda activate framegrab
conda install -c conda-forge opencv
pip install framegrab
```

**Why Conda?**
- ✅ Pre-compiled OpenCV with GStreamer support
- ✅ Cross-platform (Mac, Linux, Windows)
- ✅ No manual compilation needed
- ✅ Works consistently

#### Option 2: Docker (Best for Production)

Use the provided Dockerfile which includes GStreamer support:

```bash
# Build the Docker image
docker compose -f docker/docker-compose.yaml build

# Run tests
docker compose -f docker/docker-compose.yaml run --rm framegrab \
  python3 -m pytest test/

# Run your script
docker compose -f docker/docker-compose.yaml run --rm framegrab \
  python3 your_script.py
```

#### Option 3: System OpenCV (Linux)

**Ubuntu/Debian:**
```bash
# Install system OpenCV with GStreamer support
sudo apt-get update
sudo apt-get install -y \
  python3-opencv \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-libav

# Install framegrab
pip install framegrab

# Uninstall pip opencv to use system version
pip uninstall -y opencv-python opencv-python-headless
```

**Note:** System OpenCV may be an older version than the PyPI package.

#### Option 4: macOS with Homebrew

```bash
# Install OpenCV and GStreamer via Homebrew
brew install opencv gstreamer

# Install framegrab
pip install framegrab

# You may need to set PYTHONPATH to find the Homebrew OpenCV
export PYTHONPATH="/opt/homebrew/lib/python3.10/site-packages:$PYTHONPATH"
```

### Using GStreamer Backend

To use the GStreamer backend for RTSP streams, set `backend="gstreamer"` in your config:

```python
from framegrab import FrameGrabber
from framegrab.config import RTSPFrameGrabberConfig

config = RTSPFrameGrabberConfig(
    rtsp_url="rtsp://admin:password@192.168.1.100/stream",
    backend="gstreamer",  # Use GStreamer backend
    max_fps=15,           # Optional: limit frame rate
    timeout=5.0           # Optional: connection timeout (seconds)
)

with FrameGrabber.create_grabber(config) as grabber:
    frame = grabber.grab()
```

**FFmpeg backend (default):**
```python
config = RTSPFrameGrabberConfig(
    rtsp_url="rtsp://admin:password@192.168.1.100/stream",
    backend="ffmpeg",          # Use FFmpeg backend (default)
    keep_connection_open=True, # Keep connection alive
    max_fps=30                 # Drain thread frame rate
)
```

### Verifying GStreamer Support

To check if your OpenCV installation has GStreamer support:

```python
import cv2
build_info = cv2.getBuildInformation()
print("GStreamer support:", "YES" if "GStreamer" in build_info and "YES" in build_info.split("GStreamer")[1][:50] else "NO")
```

Or from the command line:
```bash
python3 -c "import cv2; print('GStreamer: YES' if 'GStreamer' in cv2.getBuildInformation() else 'GStreamer: NO')"
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
| options.keep_connection_open | True              | -          | optional (FFmpeg backend)         | -         | -  | - | optional | optional | - |
| options.max_fps | 30              | -          | optional (both backends)         | -         | -  | - | - | - | optional |
| options.backend | "gstreamer"              | -          | optional ("ffmpeg" or "gstreamer")         | -         | -  | - | - | - | - |
| options.timeout | 5.0              | -          | optional (timeout in seconds)         | -         | -  | - | - | - | - |

**RTSP Backend Options:**
- **FFmpeg backend** (default): Uses OpenCV's FFmpeg backend. Set `backend="ffmpeg"`. Supports `keep_connection_open` and `max_fps` (for drain thread rate).
- **GStreamer backend**: Uses GStreamer pipeline with zero-buffering for low latency. Set `backend="gstreamer"`. Requires OpenCV built with GStreamer support (see [GStreamer Support](#gstreamer-support-for-rtsp-streaming)). Supports `max_fps` (for rate limiting) and `timeout` (for connection timeout).


In addition to the configurations in the table above, you can set any Basler camera property by including `options.basler.<BASLER PROPERTY NAME>`. For example, it's common to set `options.basler.PixelFormat` to `RGB8`.

#### python FrameGrabberConfig format
You can also specify a configuration using the python FrameGrabberConfig object. This is useful if 
you want quick validation of your configuration and your framegrab data stored under a validated, centralized object.

##### Config Schema
```yaml
GenericUSBFrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for Generic USB Frame Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    fourcc:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      options_key: fourcc
      title: Fourcc
    fps:
      anyOf:
      - type: integer
      - type: 'null'
      default: null
      options_key: fps
      title: Fps
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
    resolution_height:
      anyOf:
      - type: integer
      - type: 'null'
      default: null
      options_key: resolution.height
      title: Resolution Height
    resolution_width:
      anyOf:
      - type: integer
      - type: 'null'
      default: null
      options_key: resolution.width
      title: Resolution Width
    serial_number:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Serial Number
    video_stream:
      default: false
      options_key: video_stream
      title: Video Stream
      type: boolean
  title: GenericUSBFrameGrabberConfig
  type: object

RTSPFrameGrabberConfig:
  additionalProperties: false
  description: "Configuration class for RTSP Frame Grabber.\n\nSupports two backends:\n\
    - \"ffmpeg\" (default): Uses OpenCV's default FFmpeg backend with drain thread.\n\
    \  Options: keep_connection_open, max_fps (for drain rate)\n- \"gstreamer\": Uses\
    \ GStreamer backend with zero-buffering (leaky queue) to always\n  get the most\
    \ recent frame without any buffering delay. Requires OpenCV built with\n  GStreamer\
    \ support. Options: max_fps (videorate element), timeout\n\nFFmpeg-specific options:\n\
    - keep_connection_open: If True (default), keeps connection open with drain thread\
    \ for\n  low-latency. If False, opens connection only when needed.\n- max_fps:\
    \ Controls drain thread rate (default: 30)\n\nGStreamer-specific options:\n- max_fps:\
    \ Rate-limit using GStreamer videorate element (default: None = no limit)\n- timeout:\
    \ Connection/data timeout in seconds (default: 5 seconds)\n- protocol: Transport\
    \ protocol (\"tcp\", \"udp\", or \"tcp+udp\"). If not set, uses GStreamer default."
  properties:
    backend:
      default: ffmpeg
      pattern: ^(ffmpeg|gstreamer)$
      title: Backend
      type: string
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    keep_connection_open:
      default: true
      options_key: keep_connection_open
      title: Keep Connection Open
      type: boolean
    max_fps:
      anyOf:
      - type: number
      - type: 'null'
      default: 30
      options_key: max_fps
      title: Max Fps
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
    protocol:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      options_key: protocol
      title: Protocol
    rtsp_url:
      pattern: ^rtsp://
      title: Rtsp Url
      type: string
    timeout:
      anyOf:
      - type: number
      - type: 'null'
      default: 5.0
      options_key: timeout
      title: Timeout
  required:
  - rtsp_url
  title: RTSPFrameGrabberConfig
  type: object

RealSenseFrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for RealSense Frame Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
    resolution_height:
      anyOf:
      - type: integer
      - type: 'null'
      default: null
      options_key: resolution.height
      title: Resolution Height
    resolution_width:
      anyOf:
      - type: integer
      - type: 'null'
      default: null
      options_key: resolution.width
      title: Resolution Width
    serial_number:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Serial Number
    side_by_side_depth:
      anyOf:
      - type: boolean
      - type: 'null'
      default: null
      options_key: depth.side_by_side
      title: Side By Side Depth
  title: RealSenseFrameGrabberConfig
  type: object

BaslerFrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for Basler Frame Grabber.
  properties:
    basler_options:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: basler_options
      title: Basler Options
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
    serial_number:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Serial Number
  title: BaslerFrameGrabberConfig
  type: object

RaspberryPiCSI2FrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for Raspberry Pi CSI-2 Frame Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
  title: RaspberryPiCSI2FrameGrabberConfig
  type: object

HttpLiveStreamingFrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for HTTP Live Streaming Frame Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    hls_url:
      pattern: ^https?://
      title: Hls Url
      type: string
    keep_connection_open:
      default: true
      options_key: keep_connection_open
      title: Keep Connection Open
      type: boolean
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
  required:
  - hls_url
  title: HttpLiveStreamingFrameGrabberConfig
  type: object

YouTubeLiveFrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for YouTube Live Frame Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    keep_connection_open:
      default: true
      options_key: keep_connection_open
      title: Keep Connection Open
      type: boolean
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
    youtube_url:
      pattern: ^https?://
      title: Youtube Url
      type: string
  required:
  - youtube_url
  title: YouTubeLiveFrameGrabberConfig
  type: object

FileStreamFrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for File Stream Frame Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    filename:
      pattern: (?i)^.*\.(mp4|mov|mjpeg|avi|mkv|webm)$
      title: Filename
      type: string
    max_fps:
      anyOf:
      - type: integer
      - type: 'null'
      default: 30
      options_key: max_fps
      title: Max Fps
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
  required:
  - filename
  title: FileStreamFrameGrabberConfig
  type: object

MockFrameGrabberConfig:
  additionalProperties: false
  description: Configuration class for Mock Frame Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
    resolution_height:
      anyOf:
      - type: integer
      - type: 'null'
      default: null
      options_key: resolution.height
      title: Resolution Height
    resolution_width:
      anyOf:
      - type: integer
      - type: 'null'
      default: null
      options_key: resolution.width
      title: Resolution Width
    serial_number:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Serial Number
  title: MockFrameGrabberConfig
  type: object

ROS2GrabberConfig:
  additionalProperties: false
  description: Configuration class for ROS 2 Grabber.
  properties:
    crop:
      anyOf:
      - additionalProperties: true
        type: object
      - type: 'null'
      default: null
      options_key: crop
      title: Crop
    digital_zoom:
      anyOf:
      - maximum: 4
        minimum: 1
        type: number
      - type: 'null'
      default: null
      options_key: zoom.digital
      title: Digital Zoom
    name:
      anyOf:
      - type: string
      - type: 'null'
      default: null
      title: Name
    num_90_deg_rotations:
      anyOf:
      - type: integer
      - type: 'null'
      default: 0
      options_key: rotation.num_90_deg_rotations
      title: Num 90 Deg Rotations
    topic:
      pattern: ^(~|/)?([A-Za-z_][A-Za-z0-9_]*)(/[A-Za-z_][A-Za-z0-9_]*)*$
      title: Topic
      type: string
  required:
  - topic
  title: ROS2GrabberConfig
  type: object

```

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

### RTSP Server
Framegrab provides tools for RTSP stream generation, which can be useful for testing applications.

Basic usage looks like this:
```
server = RTSPServer(port=port)
server.create_stream(get_frame_callback1, width, height, fps, mount_point='/stream0')
server.create_stream(get_frame_callback2, width, height, fps, mount_point='/stream1')
server.start()
time.sleep(n) # keep the server up 
server.stop()
```

Using these tools requires a number of system dependencies, which are listed below:

```
gstreamer1.0-tools
gstreamer1.0-rtsp
gstreamer1.0-plugins-base
gstreamer1.0-plugins-good
gstreamer1.0-plugins-bad
gstreamer1.0-plugins-ugly
libgstreamer1.0-dev
libgirepository1.0-dev
gir1.2-gst-rtsp-server-1.0
gir1.2-gstreamer-1.0
```
We test RTSP server functionality on Ubuntu. It may also work on Mac. It will _not_ work on Windows natively, but you may be able to get it to work with Docker or WSL.

We provide a [Dockerfile](docker/Dockerfile) that contains the necessary packages. 

For inspiration on how to implement an RTSP server, see [sample_scripts/video_to_rtsp.py](sample_scripts/video_to_rtsp.py), which shows can you can convert multiple videos into RTSP streams with a single RTSP server. 


## Examples

### Generic USB
Here's an example of using the FrameGrab library to continuously capture frames and detect motion from a video stream:

```python
from framegrab import FrameGrabber, MotionDetector

motion_threshold = 1.0
m = MotionDetector(pct_threshold=motion_threshold)

config = {
    'input_type': 'generic_usb',
}

with FrameGrabber.create_grabber(config) as grabber:
    while True:
        frame = grabber.grab()
        if frame is None:
            print("No frame captured!")
            continue

        if m.motion_detected(frame):
            print("Motion detected!")
```

### YouTube Live
Here's an example of using FrameGrab to capture frames from a YouTube Live stream:

```python
from framegrab import FrameGrabber
import cv2

config = {
    'input_type': 'youtube_live',
    'id': {
        'youtube_url': 'https://www.youtube.com/watch?v=your_video_id'
    }
}

with FrameGrabber.create_grabber(config) as grabber:
    frame = grabber.grab()
    if frame is None:
        raise Exception("No frame captured")

    # Process the frame as needed
    # For example, display it using cv2.imshow()
    # For example, save it to a file
    cv2.imwrite('youtube_frame.jpg', frame)
```

### File Stream
Here's an example of using FrameGrab to capture frames from a video file:

```python
from framegrab import FrameGrabber
import cv2

config = {
    'input_type': 'file_stream',
    'id': {
        'filename': 'path/to/your/video.mjpeg'  # or .mp4, .avi, .mov, etc.
    },
    'options': {
        'max_fps': 2,  # if a lower FPS than the original video's FPS is specified, Framegrab will skip extra frames as needed.
    }
}

with FrameGrabber.create_grabber(config) as grabber:
  frame = grabber.grab()
  if frame is None:
      raise Exception("No frame captured")

  # Process the frame as needed
  # For example, display it using cv2.imshow()
  # For example, save it to a file
  cv2.imwrite('file_stream_frame.jpg', frame)
```

## Contributing

We welcome contributions to FrameGrab! If you would like to contribute, please follow these steps:

1. Fork the repository
2. Create a new branch for your changes
3. Commit your changes to the branch
4. Open a pull request

## License

FrameGrab is released under the MIT License. For more information, please refer to the [LICENSE.txt](https://github.com/groundlight/framegrab/blob/main/LICENSE.txt) file.
