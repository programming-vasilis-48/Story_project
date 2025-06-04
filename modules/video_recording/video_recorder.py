#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Video Recorder for the Story-Switch game.

This module provides video recording functionality for capturing
user reactions during emotion elicitation experiments using QT Robot's camera.
"""

import cv2
import threading
import time
import os
from typing import Optional, Tuple
from datetime import datetime
import rospy
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError


class VideoRecorder:
    """Video recorder for capturing user reactions during experiments using QT Robot's camera."""

    def __init__(self, fps: int = 30, resolution: Tuple[int, int] = (1280, 720), existing_node: bool = False):
        """Initialize the video recorder.

        Args:
            fps (int): Frames per second for recording
            resolution (Tuple[int, int]): Video resolution (width, height)
            existing_node (bool): Whether to use an existing ROS node
        """
        self.fps = fps
        self.resolution = resolution

        # Recording state
        self.is_recording = False
        self.current_filename = None
        self.video_writer = None
        self.recording_thread = None

        # ROS and camera state
        self.current_image = None
        self.bridge = CvBridge()
        self.camera_sub = None
        self.ros_available = False

        # Initialize ROS and camera
        self._initialize_ros_camera(existing_node)

    def _initialize_ros_camera(self, existing_node: bool = False) -> bool:
        """Initialize ROS and camera subscription.

        Args:
            existing_node (bool): Whether to use an existing ROS node

        Returns:
            bool: True if ROS camera initialized successfully, False otherwise
        """
        try:
            # Initialize ROS node if needed
            if not existing_node:
                try:
                    if not rospy.get_node_uri():
                        rospy.init_node('video_recorder', anonymous=True)
                except rospy.exceptions.ROSException:
                    # Node already initialized, which is fine
                    pass

            # Subscribe to camera topic (same as live_vision_au_save.py)
            self.camera_sub = rospy.Subscriber('/camera/color/image_raw', RosImage, self._image_callback)

            # Wait a bit for the subscription to establish
            time.sleep(0.5)

            self.ros_available = True
            print("ROS camera subscription initialized successfully")
            print("Subscribed to: /camera/color/image_raw")

            return True

        except Exception as e:
            print(f"Error initializing ROS camera: {str(e)}")
            print("Video recording will not be available")
            self.ros_available = False
            return False

    def _image_callback(self, msg):
        """Callback for camera image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"CV Bridge error in video recorder: {e}")
        except Exception as e:
            print(f"Error in video recorder image callback: {str(e)}")

    def start_recording(self, output_path: str) -> bool:
        """Start video recording.

        Args:
            output_path (str): Full path where to save the video file

        Returns:
            bool: True if recording started successfully, False otherwise
        """
        if not self.ros_available:
            print("Error: ROS camera not available for recording")
            return False

        if self.is_recording:
            print("Warning: Already recording. Stop current recording first.")
            return False

        # Wait for camera to provide images
        print("Waiting for camera images...")
        timeout = time.time() + 5  # 5 seconds timeout
        while self.current_image is None and time.time() < timeout:
            time.sleep(0.1)

        if self.current_image is None:
            print("Error: No images received from camera")
            return False

        # Update resolution based on actual camera image
        if self.current_image is not None:
            h, w = self.current_image.shape[:2]
            self.resolution = (w, h)
            print(f"Using camera resolution: {w}x{h}")

        try:
            # Create output directory if it doesn't exist
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            # Define the codec and create VideoWriter object
            # Try different codecs for better compatibility
            codecs_to_try = [
                cv2.VideoWriter_fourcc(*'mp4v'),  # MPEG-4
                cv2.VideoWriter_fourcc(*'XVID'),  # XVID
                cv2.VideoWriter_fourcc(*'MJPG'),  # Motion JPEG
                cv2.VideoWriter_fourcc(*'X264'),  # H.264
            ]

            self.video_writer = None
            for fourcc in codecs_to_try:
                try:
                    self.video_writer = cv2.VideoWriter(
                        output_path,
                        fourcc,
                        self.fps,
                        self.resolution
                    )

                    if self.video_writer.isOpened():
                        print(f"Video writer initialized with codec: {fourcc}")
                        break
                    else:
                        self.video_writer.release()
                        self.video_writer = None
                except Exception as e:
                    print(f"Failed to initialize video writer with codec {fourcc}: {e}")
                    if self.video_writer:
                        self.video_writer.release()
                        self.video_writer = None

            if not self.video_writer.isOpened():
                print(f"Error: Could not open video writer for {output_path}")
                return False

            self.current_filename = output_path
            self.is_recording = True

            # Start recording thread
            self.recording_thread = threading.Thread(target=self._recording_loop)
            self.recording_thread.daemon = True
            self.recording_thread.start()

            print(f"Video recording started: {output_path}")
            return True

        except Exception as e:
            print(f"Error starting video recording: {str(e)}")
            return False

    def stop_recording(self) -> bool:
        """Stop video recording.

        Returns:
            bool: True if recording stopped successfully, False otherwise
        """
        if not self.is_recording:
            print("Warning: Not currently recording")
            return False

        try:
            self.is_recording = False

            # Wait for recording thread to finish
            if self.recording_thread and self.recording_thread.is_alive():
                self.recording_thread.join(timeout=5.0)

            # Release video writer
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None

            print(f"Video recording stopped: {self.current_filename}")

            # Verify file was created and has content
            if os.path.exists(self.current_filename):
                file_size = os.path.getsize(self.current_filename)
                print(f"Video file size: {file_size} bytes")
                if file_size == 0:
                    print("Warning: Video file is empty")
                    return False
            else:
                print("Warning: Video file was not created")
                return False

            self.current_filename = None
            return True

        except Exception as e:
            print(f"Error stopping video recording: {str(e)}")
            return False

    def _recording_loop(self) -> None:
        """Main recording loop that runs in a separate thread."""
        frame_count = 0
        start_time = time.time()
        last_frame_time = start_time

        while self.is_recording:
            try:
                # Get current frame from ROS callback
                if self.current_image is None:
                    time.sleep(0.01)  # Wait for new frame
                    continue

                # Make a copy of the current frame
                frame = self.current_image.copy()

                # Resize frame if necessary
                if frame.shape[1] != self.resolution[0] or frame.shape[0] != self.resolution[1]:
                    frame = cv2.resize(frame, self.resolution)

                # Add timestamp overlay
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                cv2.putText(frame, timestamp, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Write frame to video file
                self.video_writer.write(frame)
                frame_count += 1

                # Control frame rate
                current_time = time.time()
                elapsed_time = current_time - start_time
                expected_time = frame_count / self.fps
                if elapsed_time < expected_time:
                    time.sleep(expected_time - elapsed_time)

                last_frame_time = current_time

            except Exception as e:
                print(f"Error in recording loop: {str(e)}")
                break

        print(f"Recording loop ended. Frames recorded: {frame_count}")

    def is_camera_available(self) -> bool:
        """Check if camera is available.

        Returns:
            bool: True if camera is available, False otherwise
        """
        return self.ros_available and self.current_image is not None

    def get_camera_info(self) -> dict:
        """Get camera information.

        Returns:
            dict: Camera information
        """
        if not self.ros_available:
            return {"available": False, "reason": "ROS not available"}

        if self.current_image is None:
            return {"available": False, "reason": "No camera images received"}

        h, w = self.current_image.shape[:2]
        return {
            "available": True,
            "source": "ROS topic: /camera/color/image_raw",
            "resolution": (w, h),
            "fps": self.fps,
            "current_width": w,
            "current_height": h,
            "ros_available": self.ros_available
        }

    def test_camera(self) -> bool:
        """Test camera by checking if we can get a frame.

        Returns:
            bool: True if camera test successful, False otherwise
        """
        if not self.ros_available:
            print("Camera test failed: ROS not available")
            return False

        # Wait for a frame
        print("Testing camera by waiting for frame...")
        timeout = time.time() + 5  # 5 seconds timeout
        while self.current_image is None and time.time() < timeout:
            time.sleep(0.1)

        if self.current_image is not None:
            print(f"Camera test successful. Frame shape: {self.current_image.shape}")
            return True
        else:
            print("Camera test failed: No frame received within timeout")
            return False

    def cleanup(self) -> None:
        """Clean up resources."""
        if self.is_recording:
            self.stop_recording()

        if self.camera_sub:
            self.camera_sub.unregister()
            self.camera_sub = None

        self.current_image = None
        print("Video recorder cleaned up")

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()
