#!/usr/bin/env python3
"""Test script for RTSP GStreamer backend.

This script tests the RTSPFrameGrabber with GStreamer backend against provided RTSP stream.
Requires OpenCV built with GStreamer support.

Usage:
    python sample_scripts/stream_rtsp_with_gstreamer.py <rtsp_url> [num_frames] [max_fps] [protocol]

Arguments:
    rtsp_url    - RTSP stream URL (required)
    num_frames  - Number of frames to grab (default: 10)
    max_fps     - Maximum FPS rate limit using GStreamer videorate (default: None = no limit, use 0 to skip)
    protocol    - Transport protocol: "tcp", "udp", or "tcp+udp" (default: GStreamer default)

Examples:
    # Basic usage
    python sample_scripts/stream_rtsp_with_gstreamer.py rtsp://localhost:8554/live/cam1 100

    # With rate limiting
    python sample_scripts/stream_rtsp_with_gstreamer.py rtsp://localhost:8554/live/cam1 100 5

    # Force TCP protocol (use 0 for max_fps to skip rate limiting)
    python sample_scripts/stream_rtsp_with_gstreamer.py rtsp://localhost:8554/live/cam1 50 0 tcp

    # Force UDP protocol
    python sample_scripts/stream_rtsp_with_gstreamer.py rtsp://localhost:8554/live/cam1 50 0 udp

Docker usage (with GStreamer support):
    # Build the Docker image
    docker build -f docker/Dockerfile.gstreamer -t framegrab-gstreamer .

    # Run with an RTSP stream (use host network for local streams)
    docker run -it --rm --network host framegrab-gstreamer \\
        python sample_scripts/stream_rtsp_with_gstreamer.py rtsp://localhost:8554/live/cam1 100

    # Run with TCP protocol
    docker run -it --rm --network host framegrab-gstreamer \\
        python sample_scripts/stream_rtsp_with_gstreamer.py rtsp://localhost:8554/live/cam1 100 0 tcp
"""

import sys

import cv2


def check_gstreamer_support():
    """Check if OpenCV has GStreamer support."""
    build_info = cv2.getBuildInformation()
    has_gstreamer = "GStreamer" in build_info and "YES" in build_info.split("GStreamer")[1][:50]
    
    print("=" * 60)
    print("OpenCV Build Information (GStreamer section)")
    print("=" * 60)
    
    # Extract and print GStreamer-related lines
    for line in build_info.split("\n"):
        if "gstreamer" in line.lower() or "gst" in line.lower():
            print(line)
    
    print("=" * 60)
    print(f"GStreamer support: {'YES ✓' if has_gstreamer else 'NO ✗'}")
    print("=" * 60)
    
    return has_gstreamer


def test_rtsp_stream(rtsp_url: str, num_frames: int = 10, max_fps: float = None, protocol: str = None):
    """Test RTSP stream grabbing with GStreamer backend."""
    from framegrab import FrameGrabber
    from framegrab.config import RTSPFrameGrabberConfig
    
    print(f"\nTesting RTSP stream: {rtsp_url}")
    print("-" * 80)
    
    try:
        config = RTSPFrameGrabberConfig(rtsp_url=rtsp_url, name="test_rtsp", max_fps=max_fps, backend="gstreamer", protocol=protocol)
        grabber = FrameGrabber.create_grabber(config)
        
        print(f"Backend: {grabber.capture.getBackendName()}")
        if protocol:
            print(f"Protocol: {protocol}")
        else:
            print(f"Protocol: default (GStreamer auto-negotiation)")
        if max_fps and max_fps != 30:
            print(f"Rate limit: {max_fps} fps (GStreamer videorate)")
        else:
            print(f"Rate limit: None (source rate)")
        print(f"Grabbing {num_frames} frames...\n")
        
        # Print header
        print(f"{'Frame':>6} | {'Grab Time':>12} | Shape")
        print("-" * 80)
        
        from datetime import datetime
        import time
        
        grab_times = []
        last_frame = None
        last_frame_timestamp = None
        
        for i in range(num_frames):
            try:
                # Measure time for the blocking grab() call
                t0 = time.time()
                frame = grabber.grab()
                t1 = time.time()
                grab_time_ms = (t1 - t0) * 1000
                
                # Capture timestamp immediately after successful grab
                frame_timestamp = datetime.now()
            except Exception as e:
                print(f"{i+1:>6} | FAILED: {e}")
                break
            
            grab_times.append(grab_time_ms)
            last_frame = frame
            last_frame_timestamp = frame_timestamp
            print(f"{i+1:>6} | {grab_time_ms:>10.1f}ms | {frame.shape}")
        
        # Calculate statistics
        avg_time = sum(grab_times) / len(grab_times)
        min_time = min(grab_times)
        max_time = max(grab_times)
        
        print("-" * 80)
        print(f"{'AVG':>6} | {avg_time:>10.1f}ms |")
        print(f"{'MIN':>6} | {min_time:>10.1f}ms |")
        print(f"{'MAX':>6} | {max_time:>10.1f}ms |")
        
        # Summary
        print(f"\n{'='*50}")
        print(f"SUMMARY ({num_frames} frames)")
        print(f"{'='*50}")
        print(f"  Average grab time:   {avg_time:.1f}ms")
        print(f"  Min grab time:       {min_time:.1f}ms")
        print(f"  Max grab time:       {max_time:.1f}ms")
        print(f"  Effective FPS:       {1000/avg_time:.1f}")
        if max_fps:
            print(f"  Target FPS:          {max_fps}")
        
        grabber.release()
        
        # Save the last frame as JPEG to shared output directory with capture timestamp
        if last_frame is not None and last_frame_timestamp is not None:
            import os
            output_dir = "/app/output"
            os.makedirs(output_dir, exist_ok=True)
            # Use the timestamp from when the frame was actually captured
            timestamp = last_frame_timestamp.strftime("%Y-%m-%d_%H-%M-%S")
            output_path = os.path.join(output_dir, f"frame_{timestamp}.jpg")
            cv2.imwrite(output_path, last_frame)
            print(f"\n📷 Last frame saved to: {output_path}")
            print(f"   Captured at: {last_frame_timestamp.strftime('%Y-%m-%d %H:%M:%S')}")
        
        print("\n✓ Test PASSED")
        return True
        
    except Exception as e:
        print(f"\n✗ Test FAILED: {e}")
        return False


def main():
    # Check GStreamer support
    if not check_gstreamer_support():
        print("\nERROR: OpenCV does not have GStreamer support!")
        print("Please use the Docker environment or rebuild OpenCV with GStreamer.")
        sys.exit(1)
    
    # Get RTSP URL from args (required)
    if len(sys.argv) > 1:
        rtsp_url = sys.argv[1]
    else:
        raise ValueError("RTSP URL is required")
    
    # Get number of frames from args or use default
    if len(sys.argv) > 2:
        try:
            num_frames = int(sys.argv[2])
        except ValueError:
            print(f"Invalid num_frames: {sys.argv[2]}. Using default (10).")
            num_frames = 10
    else:
        num_frames = 10
    
    # Get max_fps from args or use default (None = no limit, 0 means no limit)
    max_fps = None
    if len(sys.argv) > 3:
        try:
            max_fps = float(sys.argv[3])
            if max_fps == 0:
                max_fps = None  # 0 means no limit
        except ValueError:
            print(f"Invalid max_fps: {sys.argv[3]}. Using default.")
            max_fps = None
    
    # Get protocol from args (4th argument) or use default (None = GStreamer default)
    protocol = None
    if len(sys.argv) > 4:
        protocol = sys.argv[4].lower()
        valid_protocols = ("tcp", "udp", "tcp+udp")
        if protocol not in valid_protocols:
            print(f"Invalid protocol: {sys.argv[4]}. Must be one of: {', '.join(valid_protocols)}. Using default.")
            protocol = None
    
    # Run the test
    success = test_rtsp_stream(rtsp_url, num_frames=num_frames, max_fps=max_fps, protocol=protocol)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

