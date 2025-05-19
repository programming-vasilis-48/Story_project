#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Live Vision and AU detection script for QTrobot using PyFeat.
This script captures images from the camera, processes them with PyFeat,
and displays the results in real-time with visual feedback.
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

# Import PyFeat
try:
    from feat import Detector
    from feat.utils import get_resource_path
    from feat.plotting import draw_box, draw_landmarks
    PYFEAT_AVAILABLE = True
except ImportError:
    PYFEAT_AVAILABLE = False
    print("PyFeat not available. Please install with: pip install py-feat")

class LiveVisionAU:
    """Live Vision and AU detection class for QTrobot using PyFeat."""

    def __init__(self, existing_node=False):
        """Initialize the live vision system."""
        # Check if PyFeat is available
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Please install it with: pip install py-feat")
            return

        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('live_vision_au', anonymous=True)

        # Initialize CV bridge
        self.bridge = CvBridge()

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
                    
                    # Process the image with PyFeat
                    results = self.detector.detect_faces(image)
                    
                    if results is not None and not results.empty:
                        # Store the results
                        self.current_results = results
                        
                        # Create a visualization of the results
                        self.display_image = self.visualize_results(image, results)
                    else:
                        # If no faces detected, just show the original image
                        self.display_image = image.copy()
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(self.display_image, "No face detected", (30, 30), 
                                    font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                except Exception as e:
                    print(f"Error processing frame: {str(e)}")
                    import traceback
                    traceback.print_exc()
                    self.display_image = self.current_image.copy()
                
                self.processing = False
            
            # Sleep to avoid consuming too much CPU
            time.sleep(0.03)  # ~30 FPS

    def visualize_results(self, image, results):
        """Create a visualization of the PyFeat results."""
        # Make a copy of the image for drawing
        vis_image = image.copy()
        
        # Get the first face (assuming single face for simplicity)
        face_idx = 0
        
        # Draw bounding box
        if 'x' in results.columns and 'y' in results.columns and 'w' in results.columns and 'h' in results.columns:
            x = int(results['x'].iloc[face_idx])
            y = int(results['y'].iloc[face_idx])
            w = int(results['w'].iloc[face_idx])
            h = int(results['h'].iloc[face_idx])
            
            # Draw face bounding box
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Draw facial landmarks if available
        if 'x_0' in results.columns:
            landmarks = []
            for i in range(68):  # Assuming 68 landmarks
                if f'x_{i}' in results.columns and f'y_{i}' in results.columns:
                    x = int(results[f'x_{i}'].iloc[face_idx])
                    y = int(results[f'y_{i}'].iloc[face_idx])
                    landmarks.append((x, y))
                    cv2.circle(vis_image, (x, y), 2, (255, 0, 0), -1)
        
        # Extract emotions
        emotions = {}
        for col in results.columns:
            if col.startswith('emotion_'):
                emotion_name = col.replace('emotion_', '')
                emotions[emotion_name] = float(results[col].iloc[face_idx])
        
        # Extract action units
        aus = {}
        for col in results.columns:
            if col.startswith('AU') or col.lower().startswith('au'):
                aus[col] = float(results[col].iloc[face_idx])
        
        # Determine dominant emotion
        if emotions:
            dominant_emotion = max(emotions.items(), key=lambda x: x[1])[0]
            
            # Display dominant emotion
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(vis_image, f"Emotion: {dominant_emotion}", (10, 30), 
                        font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            
            # Display emotion confidence values
            y_pos = 60
            for emotion, confidence in emotions.items():
                text = f"{emotion}: {confidence:.2f}"
                cv2.putText(vis_image, text, (10, y_pos), 
                            font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                y_pos += 20
        
        # Display action units
        if aus:
            y_pos = 60
            for au, value in list(aus.items())[:8]:  # Show first 8 AUs to avoid cluttering
                text = f"{au}: {value:.2f}"
                cv2.putText(vis_image, text, (vis_image.shape[1] - 150, y_pos), 
                            font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                y_pos += 20
        
        return vis_image

    def run(self):
        """Run the live vision system."""
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. System cannot run.")
            return False

        print("Starting Live Vision and AU detection with PyFeat...")
        print("Press 'q' to quit.")

        # Wait for camera to start providing images
        print("Waiting for camera to provide images...")
        timeout = time.time() + 10  # 10 seconds timeout
        while self.current_image is None and time.time() < timeout:
            time.sleep(0.1)

        if self.current_image is None:
            print("No images received from camera within timeout period.")
            return False

        # Main display loop
        try:
            while not rospy.is_shutdown():
                if self.display_image is not None:
                    # Show the visualization
                    cv2.imshow("Live Vision and AU Detection", self.display_image)
                    
                    # Check for key press
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                
                # Sleep to avoid consuming too much CPU
                time.sleep(0.01)
        
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
            cv2.destroyAllWindows()
            print("Live Vision and AU Detection stopped.")
        
        return True

if __name__ == "__main__":
    try:
        live_vision = LiveVisionAU()
        success = live_vision.run()
        if not success:
            print("\nLive Vision and AU Detection failed to start! ❌")
    except rospy.ROSInterruptException:
        print("ROS interrupted!")
    except Exception as e:
        print(f"Error during execution: {str(e)}")
        import traceback
        traceback.print_exc()
