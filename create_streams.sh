#!/bin/bash

# Simple YouTube to RTSP Stream Generator
# No additional dependencies - uses VLC's built-in YouTube handling

# Check if number of streams and YouTube URL were provided
if [ $# -lt 2 ]; then
    echo "Usage: $0 <number_of_streams> <youtube_url>"
    echo "Example: $0 3 https://www.youtube.com/watch?v=C0HYwEYa8w4"
    exit 1
fi

# Number of streams to create
NUM_STREAMS=$1

# YouTube URL
YOUTUBE_URL=$2

# Base port number (each stream will use port+index)
BASE_PORT=8554

echo "Preparing to stream from YouTube URL: $YOUTUBE_URL"

# Create VLC commands for the specified number of streams
for ((i=0; i<NUM_STREAMS; i++)); do
    # Calculate port for this stream
    PORT=$((BASE_PORT + i))
    
    # Create a unique stream name
    STREAM_NAME="feed_$i"
    
    echo "Starting stream $i on port $PORT..."
    
    # Run VLC in the background with direct YouTube URL
    cvlc -vvv "$YOUTUBE_URL" --sout "#transcode{vcodec=h264,acodec=mpga,ab=128,channels=2,samplerate=44100,scodec=none}:rtp{sdp=rtsp://:$PORT/$STREAM_NAME}" --sout-all --sout-keep > /dev/null 2>&1 &
    
    # Store the process ID
    PID=$!
    echo "Stream $i started with PID $PID on rtsp://localhost:$PORT/$STREAM_NAME"
    
    # Give some time between starting streams
    sleep 2
done

echo "All $NUM_STREAMS streams started successfully."
echo "To stop all streams, run: killall vlc"
