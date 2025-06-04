#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Detailed Camera Test Script

This script thoroughly tests camera functionality and helps diagnose
video recording issues for the Story-Switch game.
"""

import cv2
import os
import time
import sys
from datetime import datetime


def test_camera_device(camera_idx: int, backend: int = cv2.CAP_ANY) -> dict:
    """Test a specific camera device.
    
    Args:
        camera_idx (int): Camera index to test
        backend (int): OpenCV backend to use
        
    Returns:
        dict: Test results
    """
    result = {
        "camera_idx": camera_idx,
        "backend": backend,
        "backend_name": get_backend_name(backend),
        "success": False,
        "error": None,
        "properties": {}
    }
    
    try:
        print(f"\nTesting camera {camera_idx} with backend {result['backend_name']}")
        
        # Try to open camera
        cap = cv2.VideoCapture(camera_idx, backend)
        
        if not cap.isOpened():
            result["error"] = "Could not open camera"
            return result
        
        # Get camera properties
        properties = {
            "width": cap.get(cv2.CAP_PROP_FRAME_WIDTH),
            "height": cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            "fps": cap.get(cv2.CAP_PROP_FPS),
            "fourcc": cap.get(cv2.CAP_PROP_FOURCC),
            "brightness": cap.get(cv2.CAP_PROP_BRIGHTNESS),
            "contrast": cap.get(cv2.CAP_PROP_CONTRAST),
            "saturation": cap.get(cv2.CAP_PROP_SATURATION),
            "hue": cap.get(cv2.CAP_PROP_HUE),
        }
        
        result["properties"] = properties
        
        # Try to read a frame
        ret, frame = cap.read()
        
        if not ret or frame is None:
            result["error"] = "Could not read frame"
            cap.release()
            return result
        
        if frame.shape[0] == 0 or frame.shape[1] == 0:
            result["error"] = "Invalid frame dimensions"
            cap.release()
            return result
        
        result["properties"]["actual_width"] = frame.shape[1]
        result["properties"]["actual_height"] = frame.shape[0]
        result["properties"]["channels"] = frame.shape[2] if len(frame.shape) > 2 else 1
        
        print(f"✓ Camera {camera_idx} working!")
        print(f"  Resolution: {frame.shape[1]}x{frame.shape[0]}")
        print(f"  Channels: {frame.shape[2] if len(frame.shape) > 2 else 1}")
        print(f"  FPS: {properties['fps']}")
        
        result["success"] = True
        cap.release()
        
    except Exception as e:
        result["error"] = str(e)
        print(f"✗ Error with camera {camera_idx}: {e}")
    
    return result


def get_backend_name(backend: int) -> str:
    """Get human-readable backend name."""
    backend_names = {
        cv2.CAP_ANY: "CAP_ANY",
        cv2.CAP_V4L2: "CAP_V4L2",
        cv2.CAP_GSTREAMER: "CAP_GSTREAMER",
        cv2.CAP_FFMPEG: "CAP_FFMPEG",
    }
    return backend_names.get(backend, f"Unknown({backend})")


def test_video_recording(camera_idx: int, backend: int = cv2.CAP_ANY, duration: int = 3) -> bool:
    """Test video recording with a specific camera.
    
    Args:
        camera_idx (int): Camera index
        backend (int): OpenCV backend
        duration (int): Recording duration in seconds
        
    Returns:
        bool: True if recording successful
    """
    print(f"\nTesting video recording with camera {camera_idx}")
    
    try:
        # Initialize camera
        cap = cv2.VideoCapture(camera_idx, backend)
        
        if not cap.isOpened():
            print("✗ Could not open camera for recording")
            return False
        
        # Get frame for resolution
        ret, frame = cap.read()
        if not ret:
            print("✗ Could not read frame for recording")
            cap.release()
            return False
        
        resolution = (frame.shape[1], frame.shape[0])
        fps = 30
        
        # Create test output directory
        os.makedirs("test_videos", exist_ok=True)
        output_path = f"test_videos/camera_{camera_idx}_test_{int(time.time())}.mp4"
        
        # Try different codecs
        codecs_to_try = [
            ('mp4v', cv2.VideoWriter_fourcc(*'mp4v')),
            ('XVID', cv2.VideoWriter_fourcc(*'XVID')),
            ('MJPG', cv2.VideoWriter_fourcc(*'MJPG')),
            ('X264', cv2.VideoWriter_fourcc(*'X264')),
        ]
        
        video_writer = None
        working_codec = None
        
        for codec_name, fourcc in codecs_to_try:
            try:
                video_writer = cv2.VideoWriter(output_path, fourcc, fps, resolution)
                if video_writer.isOpened():
                    working_codec = codec_name
                    print(f"✓ Video writer initialized with {codec_name}")
                    break
                else:
                    video_writer.release()
                    video_writer = None
            except Exception as e:
                print(f"✗ Failed to initialize with {codec_name}: {e}")
                if video_writer:
                    video_writer.release()
                    video_writer = None
        
        if not video_writer or not video_writer.isOpened():
            print("✗ Could not initialize video writer with any codec")
            cap.release()
            return False
        
        # Record video
        print(f"Recording {duration} seconds of video...")
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < duration:
            ret, frame = cap.read()
            if not ret:
                print("✗ Failed to read frame during recording")
                break
            
            # Add timestamp to frame
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            cv2.putText(frame, timestamp, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            video_writer.write(frame)
            frame_count += 1
            
            # Control frame rate
            time.sleep(1.0 / fps)
        
        # Cleanup
        video_writer.release()
        cap.release()
        
        # Check if file was created and has content
        if os.path.exists(output_path):
            file_size = os.path.getsize(output_path)
            print(f"✓ Video recorded successfully!")
            print(f"  File: {output_path}")
            print(f"  Size: {file_size} bytes")
            print(f"  Frames: {frame_count}")
            print(f"  Codec: {working_codec}")
            
            if file_size > 1000:  # At least 1KB
                return True
            else:
                print("✗ Video file is too small, likely empty")
                return False
        else:
            print("✗ Video file was not created")
            return False
            
    except Exception as e:
        print(f"✗ Error during video recording: {e}")
        return False


def main():
    """Main test function."""
    print("DETAILED CAMERA TEST FOR STORY-SWITCH GAME")
    print("=" * 60)
    
    # Test available cameras
    print("\n1. TESTING CAMERA DEVICES")
    print("-" * 30)
    
    working_cameras = []
    backends_to_try = [cv2.CAP_V4L2, cv2.CAP_ANY]
    
    for camera_idx in range(6):  # Test cameras 0-5
        for backend in backends_to_try:
            result = test_camera_device(camera_idx, backend)
            if result["success"]:
                working_cameras.append((camera_idx, backend))
                break  # Found working config for this camera
    
    if not working_cameras:
        print("\n✗ No working cameras found!")
        print("Please check:")
        print("- Camera is connected")
        print("- Camera permissions")
        print("- Camera is not being used by another application")
        return 1
    
    print(f"\n✓ Found {len(working_cameras)} working camera(s)")
    
    # Test video recording with working cameras
    print("\n2. TESTING VIDEO RECORDING")
    print("-" * 30)
    
    recording_success = False
    
    for camera_idx, backend in working_cameras:
        print(f"\nTesting recording with camera {camera_idx}, backend {get_backend_name(backend)}")
        if test_video_recording(camera_idx, backend, duration=3):
            recording_success = True
            print(f"✓ Video recording works with camera {camera_idx}!")
            break
    
    if not recording_success:
        print("\n✗ Video recording failed with all cameras")
        print("This might be due to:")
        print("- Codec issues")
        print("- Permission problems")
        print("- Disk space")
        return 1
    
    # Test integration with our VideoRecorder class
    print("\n3. TESTING VIDEORECORDER CLASS")
    print("-" * 30)
    
    try:
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from modules.video_recording import VideoRecorder
        
        recorder = VideoRecorder()
        
        if recorder.is_camera_available():
            print("✓ VideoRecorder initialized successfully")
            print(f"Camera info: {recorder.get_camera_info()}")
            
            # Test short recording
            test_path = "test_videos/videorecorder_test.mp4"
            os.makedirs(os.path.dirname(test_path), exist_ok=True)
            
            if recorder.start_recording(test_path):
                print("✓ Recording started")
                time.sleep(2)
                
                if recorder.stop_recording():
                    print("✓ Recording stopped")
                    
                    if os.path.exists(test_path):
                        file_size = os.path.getsize(test_path)
                        print(f"✓ VideoRecorder test successful! File size: {file_size} bytes")
                    else:
                        print("✗ VideoRecorder test failed - no file created")
                else:
                    print("✗ Failed to stop recording")
            else:
                print("✗ Failed to start recording")
        else:
            print("✗ VideoRecorder could not initialize camera")
        
        recorder.cleanup()
        
    except Exception as e:
        print(f"✗ Error testing VideoRecorder: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("CAMERA TEST COMPLETE")
    print("=" * 60)
    
    if recording_success:
        print("✓ Camera and video recording are working!")
        print("The logging system should now record videos properly.")
        return 0
    else:
        print("✗ Video recording issues detected.")
        print("Please check the errors above and fix camera setup.")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
