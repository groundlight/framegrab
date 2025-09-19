#!/bin/bash

# Build the Docker image
echo "Building RTSP publisher Docker image..."
docker build -t rtsp-publisher .

# Run the container with proper networking
echo "Starting RTSP publisher container..."
echo "RTSP stream will be available at: rtsp://localhost:8554/test"
echo "Press Ctrl+C to stop"

docker run -it --rm \
  --name rtsp-publisher \
  -p 8554:8554 \
  rtsp-publisher
