ARG PYTHON_VERSION=3.10
FROM python:${PYTHON_VERSION}-slim

# Install GStreamer/RTSP dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gstreamer1.0-tools \
    gstreamer1.0-rtsp \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgirepository1.0-dev \
    gir1.2-gst-rtsp-server-1.0 \
    gir1.2-gstreamer-1.0 \
    python3-gi \
    && rm -rf /var/lib/apt/lists/*

# Install Poetry
RUN pip install --no-cache-dir poetry

# Set working directory
WORKDIR /app

# Copy everything
COPY . .

# Install Python dependencies only
RUN poetry config virtualenvs.create false && \
    poetry install --extras youtube --no-root

# Install the project
RUN pip install .
