#!/bin/bash

# Usage: ./run_rtsp_server.sh [config.yaml] [port] [--rebuild]
# Example: ./run_rtsp_server.sh rtsp_camera_config.yaml 8555
# Example: ./run_rtsp_server.sh rtsp_camera_config.yaml 8555 --rebuild

CONFIG_FILE="$1"
RTSP_PORT="${2:-8554}"  # Default to 8554, but allow override
REBUILD=false

# Check for rebuild flag in any position
for arg in "$@"; do
    if [ "$arg" = "--rebuild" ] || [ "$arg" = "--no-cache" ]; then
        REBUILD=true
        break
    fi
done

if [ -z "$CONFIG_FILE" ]; then
    echo "Usage: $0 <config.yaml> [port] [--rebuild]"
    echo "Example: $0 rtsp_camera_config.yaml"
    echo "Example: $0 rtsp_camera_config.yaml 8555"
    echo "Example: $0 rtsp_camera_config.yaml 8555 --rebuild"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file '$CONFIG_FILE' not found!"
    exit 1
fi

# Check if port is in use
if lsof -i :$RTSP_PORT >/dev/null 2>&1; then
    echo "Warning: Port $RTSP_PORT is already in use!"
    echo "You can specify a different port: $0 $CONFIG_FILE 8555"
    echo "Or stop the process using port $RTSP_PORT"
    exit 1
fi

# Get absolute path of config file
CONFIG_PATH=$(realpath "$CONFIG_FILE")
CONFIG_DIR=$(dirname "$CONFIG_PATH")
CONFIG_NAME=$(basename "$CONFIG_PATH")

if [ "$REBUILD" = true ]; then
    echo "Rebuilding RTSP publisher Docker image (no cache)..."
    docker build --no-cache -t rtsp-publisher .
else
    echo "Building RTSP publisher Docker image..."
    docker build -t rtsp-publisher .
fi

echo "Starting RTSP publisher with config: $CONFIG_FILE"
echo "RTSP stream will be available at: rtsp://localhost:$RTSP_PORT/test"
echo "Press Ctrl+C to stop"

# Run container with config file mounted
docker run -it --rm \
  --privileged \
  --name rtsp-publisher \
  -p $RTSP_PORT:8554 \
  -v "$CONFIG_DIR:/app/config:ro" \
  rtsp-publisher \
  python3 rtsp_publisher.py "/app/config/$CONFIG_NAME"
