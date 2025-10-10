FROM ubuntu:22.04

# Install Python and system dependencies for GStreamer and RTSP
RUN apt-get update && apt-get install -y \
    python3.11 \
    python3.11-pip \
    python3.11-venv \
    gstreamer1.0-tools \
    gstreamer1.0-rtsp \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgirepository1.0-dev \
    gir1.2-gst-rtsp-server-1.0 \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for python command
RUN ln -s /usr/bin/python3.11 /usr/bin/python

# Install Poetry
RUN pip install poetry

# Set working directory
WORKDIR /app

# Copy everything
COPY . .

# Install Python dependencies only
RUN poetry config virtualenvs.create false && \
    poetry install --extras youtube --no-root

# Install the project
RUN pip install .
