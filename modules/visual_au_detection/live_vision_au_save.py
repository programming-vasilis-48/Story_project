#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Live Vision and AU detection script for QTrobot using PyFeat.
This script captures images from the camera, processes them with PyFeat,
and saves the results as images for later viewing.
"""

import rospy
from std_msgs.msg import String
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as RosImage
import os
import threading
import pandas as pd
import datetime

# Import PyFeat
try:
    from feat import Detector
    from feat.utils.io import get_resource_path
    print("PyFeat successfully imported!")
    PYFEAT_AVAILABLE = True
except ImportError as e:
    PYFEAT_AVAILABLE = False
    print(f"PyFeat import error: {e}")
    print("PyFeat not available. Please install with: pip install py-feat")
except Exception as e:
    PYFEAT_AVAILABLE = False
    print(f"Unexpected error importing PyFeat: {e}")
    print("PyFeat not available due to an unexpected error.")

class LiveVisionAUSave:
    """Live Vision and AU detection class for QTrobot using PyFeat with image saving."""

    def __init__(self, existing_node=False, output_dir="/tmp/pyfeat_output"):
        """Initialize the live vision system.

        Args:
            existing_node (bool): Whether to use an existing ROS node.
            output_dir (str): Directory to save output images.
        """
        # Check if PyFeat is available
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Please install it with: pip install py-feat")
            return

        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('live_vision_au_save', anonymous=True)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Set output directory
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        print(f"Output images will be saved to: {self.output_dir}")

        # Initialize PyFeat detector
        print("Initializing PyFeat detector...")
        self.detector = Detector(
            face_model="retinaface",
            landmark_model="mobilenet",
            au_model="xgb",
            emotion_model="resmasknet",
            facepose_model="img2pose"
        )

        # Variables to store image and results
        self.current_image = None
        self.current_results = None
        self.processing = False
        self.display_image = None
        self.running = True
        self.frame_count = 0
        self.last_save_time = time.time()

        # Define emotion and AU columns for easy access
        self.emotion_cols = ['anger', 'disgust', 'fear', 'happiness', 'sadness', 'surprise', 'neutral']
        self.au_cols = ['AU01', 'AU02', 'AU04', 'AU05', 'AU06', 'AU07', 'AU09', 'AU10',
                        'AU11', 'AU12', 'AU14', 'AU15', 'AU17', 'AU20', 'AU23', 'AU24',
                        'AU25', 'AU26', 'AU28', 'AU43']

        # Subscribe to camera topic
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', RosImage, self.image_callback)

        # Start processing thread
        self.process_thread = threading.Thread(target=self.process_frames)
        self.process_thread.daemon = True
        self.process_thread.start()

    def image_callback(self, msg):
        """Callback for camera image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"CV Bridge error: {e}")
        except Exception as e:
            print(f"Error in image callback: {str(e)}")

    def process_frames(self):
        """Process frames continuously in a separate thread."""
        while self.running:
            if self.current_image is not None and not self.processing:
                self.processing = True
                try:
                    # Make a copy of the current image for processing
                    image = self.current_image.copy()

                    # Process the image with PyFeat's full pipeline
                    # This returns a DataFrame with all face data, emotions, and AUs
                    # We'll save the image to a temporary file first
                    temp_img_path = os.path.join(self.output_dir, "temp_frame.jpg")
                    cv2.imwrite(temp_img_path, image)

                    # Use detect_image which runs the full pipeline
                    # Now with the patched library, this should work
                    results = self.detector.detect_image(temp_img_path)

                    # Check if results is a valid DataFrame with face data
                    if isinstance(results, pd.DataFrame) and not results.empty:
                        # Store the results
                        self.current_results = results

                        # Create a visualization of the results
                        self.display_image = self.visualize_results(image, results)

                        # Save the visualization periodically (every 2 seconds)
                        current_time = time.time()
                        if current_time - self.last_save_time >= 2.0:
                            self.save_visualization()
                            self.last_save_time = current_time
                    else:
                        # If no faces detected, just show the original image
                        self.display_image = image.copy()
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(self.display_image, "No face detected", (30, 30),
                                    font, 1, (0, 0, 255), 2, cv2.LINE_AA)

                    # Clean up temporary file
                    if os.path.exists(temp_img_path):
                        try:
                            os.remove(temp_img_path)
                        except:
                            pass

                except Exception as e:
                    print(f"Error processing frame: {str(e)}")
                    import traceback
                    traceback.print_exc()
                    self.display_image = self.current_image.copy()

                self.processing = False

            # Sleep to avoid consuming too much CPU
            time.sleep(0.1)  # 10 FPS is enough for saving images

    def visualize_results(self, image, results):
        """Create a visualization of the PyFeat results."""
        # Make a copy of the image for drawing
        vis_image = image.copy()

        # Add a header with timestamp
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(vis_image, f"PyFeat Analysis - {timestamp}", (10, 30),
                    font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        # Process each face in the results
        for i, (_, row) in enumerate(results.iterrows()):
            # Draw bounding box
            if all(col in results.columns for col in ['FaceRectX', 'FaceRectY', 'FaceRectWidth', 'FaceRectHeight']):
                x = int(row['FaceRectX'])
                y = int(row['FaceRectY'])
                w = int(row['FaceRectWidth'])
                h = int(row['FaceRectHeight'])
                cv2.rectangle(vis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(vis_image, f"Face {i+1}", (x, y - 10),
                            font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

            # Draw landmarks
            for j in range(68):  # 68 facial landmarks
                if f'x_{j}' in results.columns and f'y_{j}' in results.columns:
                    x = int(row[f'x_{j}'])
                    y = int(row[f'y_{j}'])
                    cv2.circle(vis_image, (x, y), 1, (255, 0, 0), -1)

            # Extract and display emotions
            emotions = {}
            for col in self.emotion_cols:
                if col in results.columns:
                    emotions[col] = float(row[col])

            if emotions:
                # Find dominant emotion
                dominant_emotion = max(emotions.items(), key=lambda x: x[1])[0]

                # Display emotions on the left side
                y_pos = 70 + i * 200
                cv2.putText(vis_image, f"Face {i+1} - Dominant: {dominant_emotion}",
                            (10, y_pos), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                y_pos += 25

                for emotion, value in emotions.items():
                    text = f"{emotion}: {value:.2f}"
                    color = (0, 255, 0)  # Default color
                    thickness = 1

                    # Highlight dominant emotion
                    if emotion == dominant_emotion:
                        color = (0, 255, 255)  # Yellow for dominant
                        thickness = 2

                    cv2.putText(vis_image, text, (10, y_pos),
                                font, 0.6, color, thickness, cv2.LINE_AA)
                    y_pos += 20

            # Extract and display AUs
            aus = {}
            for col in self.au_cols:
                if col in results.columns:
                    aus[col] = float(row[col])

            if aus:
                # Display AUs on the right side
                y_pos = 70 + i * 200
                cv2.putText(vis_image, f"Face {i+1} - Action Units",
                            (image.shape[1] - 250, y_pos), font, 0.7, (255, 0, 0), 2, cv2.LINE_AA)
                y_pos += 25

                # Sort AUs by value (descending)
                sorted_aus = sorted(aus.items(), key=lambda x: x[1], reverse=True)

                # Display top 10 AUs
                for au, value in sorted_aus[:10]:
                    text = f"{au}: {value:.2f}"
                    # Color based on intensity
                    intensity = min(value, 1.0)  # Cap at 1.0
                    blue = int(255 * (1 - intensity))
                    red = int(255 * intensity)
                    cv2.putText(vis_image, text, (image.shape[1] - 250, y_pos),
                                font, 0.6, (blue, 0, red), 1, cv2.LINE_AA)
                    y_pos += 20

        return vis_image

    def save_visualization(self):
        """Save the current visualization to a file."""
        if self.display_image is not None:
            # Create a filename with timestamp
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"pyfeat_{timestamp}_{self.frame_count:04d}.jpg"
            filepath = os.path.join(self.output_dir, filename)

            # Save the image
            cv2.imwrite(filepath, self.display_image)
            print(f"Saved visualization to: {filepath}")

            # Increment frame counter
            self.frame_count += 1

    def run(self):
        """Run the live vision system."""
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. System cannot run.")
            return False

        print("Starting Live Vision and AU detection with PyFeat (saving images)...")
        print(f"Images will be saved to: {self.output_dir}")
        print("Press Ctrl+C to stop.")

        # Wait for camera to start providing images
        print("Waiting for camera to provide images...")
        timeout = time.time() + 10  # 10 seconds timeout
        while self.current_image is None and time.time() < timeout:
            time.sleep(0.1)

        if self.current_image is None:
            print("No images received from camera within timeout period.")
            return False

        # Main loop
        try:
            # Keep running until interrupted
            while not rospy.is_shutdown():
                time.sleep(1)

        except KeyboardInterrupt:
            print("Interrupted by user")
        except Exception as e:
            print(f"Error in main loop: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            # Clean up
            self.running = False
            if self.process_thread.is_alive():
                self.process_thread.join(timeout=1.0)
            print("Live Vision and AU Detection stopped.")

        return True

if __name__ == "__main__":
    try:
        live_vision = LiveVisionAUSave()
        success = live_vision.run()
        if not success:
            print("\nLive Vision and AU Detection failed to start! ❌")
    except rospy.ROSInterruptException:
        print("ROS interrupted!")
    except Exception as e:
        print(f"Error during execution: {str(e)}")
        import traceback
        traceback.print_exc()
