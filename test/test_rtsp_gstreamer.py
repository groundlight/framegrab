#!/usr/bin/env python3
"""Test script for RTSP GStreamer backend.

This script tests the RTSPFrameGrabber with GStreamer backend using a public RTSP stream.

Public RTSP test streams (may change availability):
- rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4 (Wowza demo)
- rtsp://rtsp.stream/demo (rtsp.stream demo - requires signup but has free tier)

Usage:
    python test/test_rtsp_gstreamer.py [rtsp_url] [num_frames] [max_fps]

Arguments:
    rtsp_url    - RTSP stream URL (default: Wowza demo stream)
    num_frames  - Number of frames to grab (default: 10)
    max_fps     - Maximum FPS rate limit using GStreamer videorate (default: None = no limit)

Examples:
    python test/test_rtsp_gstreamer.py rtsp://localhost:8554/live/cam1 100
    python test/test_rtsp_gstreamer.py rtsp://localhost:8554/live/cam1 100 5
    python test/test_rtsp_gstreamer.py rtsp://localhost:8554/live/cam1 50 2
"""

import sys
import time

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


def test_rtsp_stream(rtsp_url: str, num_frames: int = 10, max_fps: float = None, backend: str = "gstreamer"):
    """Test RTSP stream grabbing with configurable backend."""
    from framegrab import FrameGrabber
    from framegrab.config import RTSPFrameGrabberConfig
    
    print(f"\nTesting RTSP stream: {rtsp_url}")
    print("-" * 80)
    
    try:
        config = RTSPFrameGrabberConfig(rtsp_url=rtsp_url, name="test_rtsp", max_fps=max_fps, backend=backend)
        grabber = FrameGrabber.create_grabber(config)
        
        print(f"Backend: {grabber.capture.getBackendName()} (config: {backend})")
        if backend == "gstreamer" and max_fps and max_fps != 30:
            print(f"Rate limit: {max_fps} fps (GStreamer videorate)")
        elif backend == "ffmpeg":
            print(f"Drain rate: {max_fps or 30} fps (FFmpeg drain thread)")
        else:
            print(f"Rate limit: None (source rate)")
        print(f"Grabbing {num_frames} frames...\n")
        
        # Print header
        print(f"{'Frame':>6} | {'Grab (wait)':>12} | {'Retrieve':>12} | {'Total':>12} | Shape")
        print("-" * 80)
        
        from datetime import datetime
        
        grab_times = []
        retrieve_times = []
        total_times = []
        last_frame = None
        last_frame_timestamp = None
        
        for i in range(num_frames):
            try:
                frame = grabber.grab()
                # Capture timestamp immediately after successful grab
                frame_timestamp = datetime.now()
            except Exception as e:
                print(f"{i+1:>6} | FAILED: {e}")
                break
            
            # Get timing from grabber
            grab_times.append(grabber.last_grab_time_ms)
            retrieve_times.append(grabber.last_retrieve_time_ms)
            total_times.append(grabber.last_total_time_ms)
            
            last_frame = frame
            last_frame_timestamp = frame_timestamp
            print(f"{i+1:>6} | {grabber.last_grab_time_ms:>10.1f}ms | {grabber.last_retrieve_time_ms:>10.1f}ms | {grabber.last_total_time_ms:>10.1f}ms | {frame.shape}")
        
        # Calculate averages
        avg_grab = sum(grab_times) / len(grab_times)
        avg_retrieve = sum(retrieve_times) / len(retrieve_times)
        avg_total = sum(total_times) / len(total_times)
        
        print("-" * 80)
        print(f"{'AVG':>6} | {avg_grab:>10.1f}ms | {avg_retrieve:>10.1f}ms | {avg_total:>10.1f}ms |")
        
        # Summary
        print(f"\n{'='*40}")
        print(f"SUMMARY ({num_frames} frames)")
        print(f"{'='*40}")
        print(f"  Average total time:  {avg_total:.1f}ms")
        print(f"  Effective FPS:       {1000/avg_total:.1f}")
        if max_fps:
            print(f"  Target FPS:          {max_fps}")
        print(f"  Time waiting:        {avg_grab:.1f}ms ({avg_grab/avg_total*100:.0f}%)")
        print(f"  Time decoding:       {avg_retrieve:.1f}ms ({avg_retrieve/avg_total*100:.0f}%)")
        
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
    # Get backend from args (4th argument) or use default
    backend = "gstreamer"
    if len(sys.argv) > 4:
        backend = sys.argv[4].lower()
        if backend not in ("gstreamer", "ffmpeg"):
            print(f"Invalid backend: {sys.argv[4]}. Using 'gstreamer'.")
            backend = "gstreamer"
    
    # Check GStreamer support only if using GStreamer backend
    if backend == "gstreamer":
        if not check_gstreamer_support():
            print("\nERROR: OpenCV does not have GStreamer support!")
            print("Please use the Docker environment, rebuild OpenCV with GStreamer,")
            print("or use backend='ffmpeg' instead.")
            sys.exit(1)
    else:
        print("\n" + "=" * 60)
        print("Using FFmpeg backend (no GStreamer check needed)")
        print("=" * 60)
    
    # Get RTSP URL from args or use default
    if len(sys.argv) > 1:
        rtsp_url = sys.argv[1]
    else:
        # Wowza public demo stream (Big Buck Bunny video)
        rtsp_url = "rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4"
        print(f"\nNo RTSP URL provided, using Wowza demo stream")
    
    # Get number of frames from args or use default
    if len(sys.argv) > 2:
        try:
            num_frames = int(sys.argv[2])
        except ValueError:
            print(f"Invalid num_frames: {sys.argv[2]}. Using default (10).")
            num_frames = 10
    else:
        num_frames = 10
    
    # Get max_fps from args or use default (None = no limit for GStreamer, 30 for FFmpeg)
    max_fps = None
    if len(sys.argv) > 3:
        try:
            max_fps = float(sys.argv[3])
            if max_fps == 0:
                max_fps = None  # 0 means no limit
        except ValueError:
            print(f"Invalid max_fps: {sys.argv[3]}. Using default.")
            max_fps = None
    
    # Run the test
    success = test_rtsp_stream(rtsp_url, num_frames=num_frames, max_fps=max_fps, backend=backend)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

