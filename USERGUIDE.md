# Groundlight Stream ingestor

A containerized python application that uses the [groundlight](https://www.groundlight.ai/) SDK to
process frames from a video file, device or stream.

## Useage
The entrypoint of the image is the CLI which will process the input video.
``` shell
$ docker run groundlight/stream -h

Captures frames from a video device, file or stream and sends frames as
image queries to a configured detector using the Groundlight API

usage: streamlight [options] -t TOKEN -d DETECTOR

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
docker run groundlight/stream -t api_29imEXAMPLE -d 772d5b0EXAMPLE -s https://www.youtube.com/watch?v=210EXAMPLE -f 1
```

# License
MIT License

Copyright (c) 2022 Groundlight AI

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

As with all Docker images, this image also contains other software
which may be under other open source licenses along with any direct or
indirect dependencies of the primary software being contained.

As for any pre-built image usage, it is the image user's
responsibility to ensure that any use of this image complies with any
relevant licenses for all software contained within.
