# Groundlight Stream Processor

A containerized python application that uses the Groundlight SDK to
process frames from a video stream.  This can connect to a web cam over RTSP, a local video file, or a youtube URL.

## Releases

Releases created in Github will automatically get built and pushed to [dockerhub](https://hub.docker.com/r/groundlight/stream/tags).  These are multi-architecture builds including x86 and ARM.

## Test Builds

To build and test locally:

``` shell
docker build -t stream:local .
```

## Run
Now you can run it

``` shell
docker run stream:local -h
```

