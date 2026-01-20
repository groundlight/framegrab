ARG PYTHON_VERSION=3.10
FROM python:${PYTHON_VERSION}-slim

ENV PYTHONUNBUFFERED=1 \
    PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig

# Install build tools and GStreamer/RTSP dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libffi-dev \
    libcairo2 \
    libcairo2-dev \
    libglib2.0-dev \
    pkg-config \
    gobject-introspection \
    gstreamer1.0-tools \
    gstreamer1.0-rtsp \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgirepository1.0-dev \
    libgirepository-2.0-dev \
    girepository-tools \
    gir1.2-gst-rtsp-server-1.0 \
    gir1.2-gstreamer-1.0 \
    && rm -rf /var/lib/apt/lists/*

# Install Poetry and PyGObject (for gi bindings)
RUN pip install --no-cache-dir poetry PyGObject

# Set working directory
WORKDIR /app

# Copy everything
COPY . .

# Install Python dependencies only
RUN poetry config virtualenvs.create false && \
    poetry install --extras youtube --no-root

# Install the project
RUN pip install .
