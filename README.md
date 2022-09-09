# streamlight
A containerized python application that uses the groundlight sdk to
process frames from a video stream

## build

``` shell
docker build -t stream:local .
```

## run
Now you can run it

``` shell
docker run stream:local -h

Captures frames from a video device, file or stream and sends frames as
image queries to a configured detector using the Groundlight API

usage: streamlight [options] -t TOKEN -d DETECTOR

options:
  -d, --detector=ID      detector id to which the image queries are sent
  -e, --endpoint=URL     api endpoint [default: https://api.groundlight.ai/device-api]
  -f, --fps=FPS          number of frames to capture per second. 0 to use maximum rate possible. [default: 5]
  -h, --help             show this message.
  -s, --stream=STREAM    id, filename or URL of a video stream (e.g. rtsp://host:port/script?params) [default: 0]
  -t, --token=TOKEN      api token to authenticate with the groundlight api
  -v, --verbose          enable debug logs
  -w, --width=WIDTH      resize images to w pixels wide (and scale height proportionately if not set explicitly)
  -y, --height=HEIGHT    resize images to y pixels high (and scale width proportionately if not set explicitly)
  -m, --motion                 enable motion detection with pixel change threshold percentage (disabled by default)
  -r, --threshold=THRESHOLD    set detection threshold for motion detection [default: 1]
  -p, --postmotion=POSTMOTION  minimum number of seconds to capture for every motion detection [default: 1]
  -i, --maxinterval=MAXINT     maximum number of seconds before sending frames even without motion [default: 1000]
```
and pass an actual token and detector
``` shell
docker run stream:local -t api_29imQxusKndanuiigGzLqAoL3Zj_AD2VFYi191ghbUJeLHJ11GDfVCjfa55JCS -d 772d549499394726b06fd6e36ec41153 -s rtsp://admin:password@10.77.0.29:554/cam/realmonitor?channel=1&subtype=0
```
