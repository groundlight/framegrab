# Groundlight Stream Processor

A containerized python application that uses the [groundlight](https://www.groundlight.ai/) SDK to
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
  -s, --stream=STREAM    id, filename or URL of a video stream (e.g. rtsp://host:port/script?params OR movie.mp4 OR *.jpg) [default: 0]
  -t, --token=TOKEN      api token to authenticate with the groundlight api
  -v, --verbose          enable debug logs
  -w, --width=WIDTH      resize images to w pixels wide (and scale height proportionately if not set explicitly)
  -y, --height=HEIGHT    resize images to y pixels high (and scale width proportionately if not set explicitly)
  -m, --motion                 enable motion detection with pixel change threshold percentage (disabled by default)
  -r, --threshold=THRESHOLD    set detection threshold for motion detection [default: 1]
  -p, --postmotion=POSTMOTION  minimum number of seconds to capture for every motion detection [default: 1]
  -i, --maxinterval=MAXINT     maximum number of seconds before sending frames even without motion [default: 1000]
```

Start sending frames and getting predictions and labels using your own API token and detector ID

``` shell
docker run groundlight/stream \
    -t api_29imEXAMPLE \
    -d det_2MiD5Elu8bza7sil9l7KPpr694a \
    -s https://www.youtube.com/watch?v=210EXAMPLE \
    -f 1
```
