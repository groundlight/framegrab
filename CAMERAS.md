# RTSP Camera Stream Setup Guide

Real-Time Streaming Protocol (RTSP) is a network control protocol used for streaming media across IP networks. Many WiFi and Ethernet cameras support RTSP streams for remote viewing and integration with third-party applications. This guide provides step-by-step instructions for setting up RTSP streams on various popular camera brands.

## Hikvision Cameras

To get an RTSP stream from a Hikvision camera, follow these steps:

1. Open the camera's web interface by entering its IP address in your web browser.
2. Log in using your camera's username and password.
3. Go to Configuration > Network > Advanced Settings > Integration Protocol.
4. Enable the RTSP protocol and set the Authentication method to "digest/basic".
5. Construct the RTSP URL using the following format:

```
rtsp://<username>:<password>@<camera_ip_address>:554/Streaming/Channels/<channel>
```

   Replace `<username>`, `<password>`, `<camera_ip_address>`, and `<channel>` with the appropriate values. The channel is typically 101 for the main stream and 102 for the substream.

## Axis Cameras

To get an RTSP stream from an Axis camera, follow these steps:

1. Open the camera's web interface by entering its IP address in your web browser.
2. Log in using your camera's username and password.
3. Go to Setup > Video > Stream Profiles.
4. Click "Add" to create a new profile, or edit an existing profile.
5. Configure the desired video and audio settings for the stream, and enable the RTSP protocol.
6. Click "Save" to save the settings.
7. Construct the RTSP URL using the following format:

```
rtsp://<username>:<password>@<camera_ip_address>/axis-media/media.amp?videocodec=h264&streamprofile=<profile_name>
```

   Replace `<username>`, `<password>`, `<camera_ip_address>`, and `<profile_name>` with the appropriate values.

## Foscam Cameras

To get an RTSP stream from a Foscam camera, follow these steps:

1. Open the camera's web interface by entering its IP address in your web browser.
2. Log in using your camera's username and password.
3. Go to Settings > Network > IP Configuration.
4. Note the camera's IP address, HTTP port, and RTSP port.
5. Construct the RTSP URL using the following format:

```
rtsp://<username>:<password>@<camera_ip_address>:<rtsp_port>/videoMain
```

   Replace `<username>`, `<password>`, `<camera_ip_address>`, and `<rtsp_port>` with the appropriate values.

## Amcrest Cameras

To get an RTSP stream from an Amcrest camera, follow these steps:

1. Open the camera's web interface by entering its IP address in your web browser.
2. Log in using your camera's username and password.
3. Go to Setup > Network > Connection.
4. Note the camera's RTSP port.
5. Construct the RTSP URL using the following format:

```
rtsp://<username>:<password>@<camera_ip_address>:<rtsp_port>/cam/realmonitor?channel=1&subtype=<stream_type>
```

   Replace `<username>`, `<password>`, `<camera_ip_address>`, `<rtsp_port>`, and `<stream_type>` with the appropriate values. The stream type is typically 0 for the main stream and 1 for the substream.

## Unifi Protect Cameras

To obtain an RTSP stream from a Unifi Protect camera, follow these steps:

1. Open the Unifi Protect web application by entering its IP address in your web browser.
2. Log in using your Unifi Protect account credentials.
3. Click on "Unifi Devices" and locate the device you want to connect to.
4. In the right-panel, select "Settings" to access the camera's settings.
5. Expand the "Advanced" section, where you will find the RTSP settings.
6. Choose a resolution for the stream and enable it. An RTSP URL will be displayed below your selection.

Use the generated RTSP URL to connect to the camera stream in third-party applications or services.

