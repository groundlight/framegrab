# streamlight
A containerized python application that uses the groundlight sdk to
process frames from a video stream

## build

The docker image uses the groundlight python sdk. We need to make aws
credentials available to the build process so it can authenticate to
aws codeartifact because the python package is private as of this
writing. Set the following env variables with real credentials

``` shell
export AWS_ACCESS_KEY_ID=AKIAIOSFODNN7EXAMPLE
export AWS_SECRET_ACCESS_KEY=wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY
```
Then you can build the docker image exposing these envs as build args.

``` shell
docker build -t stream:local --build-arg AWS_ACCESS_KEY_ID --build-arg AWS_SECRET_ACCESS .
```

## run
Now you can run it

``` shell
docker run stream:local -h

Captures frames from a real-time video stream and sends frames as
image queries to a configured detector using the Groundlight API

usage: streamlight [options] -t TOKEN -d DETECTOR

options:
  -d, --detector=ID      detector id to which the image queries are sent
  -e, --endpoint=URL     api endpoint [default: https://device.positronix.ai/device-api]
  -f, --framerate=FPS    number of frames to capture per second.
  -h, --help             show this message.
  -s, --stream=STREAM    id, filename or URL of a video stream (e.g. rtsp://host:port/script?params) [default: 0]
  -t, --token=TOKEN      api token to authenticate with the groundlight api
  -v, --verbose
```
and pass an actual token and detector
``` shell
docker run stream:local -t api_29imQxusKndanuiigGzLqAoL3Zj_AD2VFYi191ghbUJeLHJ11GDfVCjfa55JCS -d 772d549499394726b06fd6e36ec41153 -s rtsp://admin:password@10.77.0.29:554/cam/realmonitor?channel=1&subtype=0
```
