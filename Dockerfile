FROM ubuntu:22.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install GStreamer and Python dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-gi \
    gir1.2-gst-rtsp-server-1.0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install numpy opencv-python framegrab

# Copy the RTSP publisher script
COPY rtsp_publisher.py /app/rtsp_publisher.py

# Set working directory
WORKDIR /app

# Expose RTSP port
EXPOSE 8554

# Default command - can be overridden
CMD ["python3", "rtsp_publisher.py"]
