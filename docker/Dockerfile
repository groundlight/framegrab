FROM python:3.11-slim

# Install system dependencies for GStreamer and RTSP
RUN apt-get update && apt-get install -y \
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
    && rm -rf /var/lib/apt/lists/*

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
