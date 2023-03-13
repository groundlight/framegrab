# Groundlight Stream Processor

A containerized python application that uses the groundlight sdk to
process frames from a video stream

## Releases

Releases created in Github will automatically get built and pushed to [dockerhub](https://hub.docker.com/r/groundlight/stream/tags).  These are multi-architecture builds including x86 and ARM.

## Test Builds

To build and test locally:

``` shell
docker build -t stream:local .
```

## run
Now you can run it

``` shell
docker run stream:local -h
```

# Video Stream types

## Unifi Protect Cameras

To get an RTSP stream from a Unifi Protect camera, first open the Unifi Protect web application.
Then select "Unifi Devices", and find the device you want to connect to.  In the right-panel, select "Settings"
Open the Advanced section, and you will find an RTSP section.  Pick a resolution to stream at, and enable the stream.  Then an RTSP url will appear below your selection.

