# Groundlight Stream Processor

A containerized python application that uses the [Groundlight](https://www.groundlight.ai/) [Python SDK](https://github.com/groundlight/python-sdk) to
process frames from a video file, device, or stream.

## Useage

The system is easy to use on any system with Docker installed.  Command line options are displayed like:

``` shell
$ docker run groundlight/stream --help

Captures frames from a video device, file or stream and sends frames as
image queries to a configured detector using the Groundlight API

usage: stream [options] -t TOKEN -d DETECTOR

options:
  -d, --detector=ID      detector id to which the image queries are sent
  -e, --endpoint=URL     api endpoint [default: https://api.groundlight.ai/device-api]
  -f, --fps=FPS          number of frames to capture per second. 0 to use maximum rate possible. [default: 5]
  -h, --help             show this message.
  -s, --stream=STREAM    id, filename or URL of a video stream (e.g. rtsp://host:port/script?params OR video.mp4 OR *.jpg) [default: 0]
  -t, --token=TOKEN      API token to authenticate with the Groundlight API
  -v, --verbose          enable debug logs
  -w, --width=WIDTH      resize images to w pixels wide (and scale height proportionately if not set explicitly)
  -y, --height=HEIGHT    resize images to y pixels high (and scale width proportionately if not set explicitly)
  -m, --motion                 enable motion detection with pixel change threshold percentage (disabled by default)
  -r, --threshold=THRESHOLD    set detection threshold for motion detection [default: 1]
  -p, --postmotion=POSTMOTION  minimum number of seconds to capture for every motion detection [default: 1]
  -i, --maxinterval=MAXINT     maximum number of seconds before sending frames even without motion [default: 1000]
```

Start sending frames and getting predictions and labels using your own API token and detector ID:

``` shell
docker run groundlight/stream \
    -t api_29imEXAMPLE \
    -d det_2MiD5Elu8bza7sil9l7KPpr694a \
    -s https://www.youtube.com/watch?v=210EXAMPLE \
    -f 1
```

## Running with a Local MP4 File

To process frames from a local MP4 file, you need to mount the file from your host machine into the Docker container. Here's how to do it:

1. Place your MP4 file (e.g., `video.mp4`) in a directory on your host machine, such as `/path/to/video`.
2. Run the Docker container, mounting the directory containing the video file:

``` shell
docker run -v /path/to/video:/videos groundlight/stream \
    -t api_29imEXAMPLE \
    -d det_2MiD5Elu8bza7sil9l7KPpr694a \
    -s /videos/video.mp4 \
    -f 1
```

This command mounts the `/path/to/video` directory on your host machine to the `/videos` directory inside the Docker container. The `-s` parameter is then set to the path of the MP4 file inside the container (`/videos/video.mp4`).

## Using a YouTube URL

To use a YouTube video as a source, you first need to obtain a direct URL to the video stream. YouTube does not provide RTSP URLs, so you'll need a tool like `youtube-dl` to extract the direct video URL. Install `youtube-dl` following the instructions on its [GitHub page](https://github.com/ytdl-org/youtube-dl#installation).

Once you have `youtube-dl` installed, extract the direct video URL:

``` shell
youtube-dl -g "https://www.youtube.com/watch?v=210EXAMPLE"
```

Replace the YouTube URL with the video you want to use. The output will be a direct video URL that you can pass to the `-s` parameter:

``` shell
docker run groundlight/stream \
    -t api_29imEXAMPLE \
    -d det_2MiD5Elu8bza7sil9l7KPpr694a \
    -s "<direct_video_url>" \
    -f 1
```

Replace `<direct_video_url>` with the URL obtained from `youtube-dl`.

## Connecting an RTSP Stream

To connect an RTSP stream from a camera or other source, you'll need the RTSP URL specific to your device. Check the instructions provided earlier in this document for obtaining the RTSP URL for your camera.

Once you have the RTSP URL, pass it to the `-s` parameter:

``` shell
docker run groundlight/stream \
    -t api_29imEXAMPLE \
    -d det_2MiD5Elu8bza7sil9l7KPpr694a \
    -s "rtsp://username:password@camera_ip_address:554/path/to/stream" \
    -f 1
```

Replace the RTSP URL with the one specific to your camera or streaming device.


## Further Reading

* [Camera types](https://github.com/groundlight/stream/blob/main/CAMERAS.md) shows how to get RTSP stream URLs for many popular camera brands.
* [Developing](https://github.com/groundlight/stream/blob/main/DEVELOPING.md) discusses how this code is built and maintained.

