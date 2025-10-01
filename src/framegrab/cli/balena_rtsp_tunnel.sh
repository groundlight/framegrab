#!/bin/bash

# Simple RTSP tunnel script for Balena devices

# Check arguments
if [ $# -lt 2 ]; then
    echo "Usage: $0 <device-id> <rtsp-ip> [pem-file]"
    echo "Example: $0 abc123def456 192.168.2.219 ~/.ssh/id_ed25519_balena"
    exit 1
fi

DEVICE_ID="$1"
RTSP_IP="$2"
PEM_FILE="${3:-}"

# Check balena CLI
if ! command -v balena &>/dev/null; then
    echo "Error: Balena CLI not found"
    exit 1
fi

# Check login and get username
WHOAMI_OUTPUT=$(balena whoami 2>&1)
if [ $? -ne 0 ]; then
    echo "$WHOAMI_OUTPUT"
    exit 1
fi

USERNAME=$(echo "$WHOAMI_OUTPUT" | grep "USERNAME:" | awk '{print $2}')

# Cleanup function
cleanup() {
    echo "Shutting down..."
    pkill -f "balena device tunnel.*$DEVICE_ID" 2>/dev/null || true
    pkill -f "ssh.*$USERNAME@localhost.*4321" 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Start tunnels
echo "Starting tunnel to $DEVICE_ID..."
balena device tunnel "$DEVICE_ID" -p 22222:4321 &

sleep 3

# Build SSH command
SSH_CMD="ssh -N -L 8554:$RTSP_IP:554 $USERNAME@localhost -p 4321 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
if [ -n "$PEM_FILE" ]; then
    SSH_CMD="$SSH_CMD -i $PEM_FILE"
fi

$SSH_CMD &

echo "RTSP stream available"
echo "Use your camera's credentials and stream path, e.g.:"
echo "  rtsp://admin:123456@localhost:8554/stream0"
echo "Press Ctrl+C to stop"

# Wait
wait
