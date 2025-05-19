#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Direct test script for Vision and AU detection on QTrobot using PyFeat.
This script captures images from the camera and processes them with PyFeat.
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

# Import PyFeat
try:
    from feat import Detector
    PYFEAT_AVAILABLE = True
except ImportError:
    PYFEAT_AVAILABLE = False
    print("PyFeat not available. Please install with: pip install py-feat")

class DirectVisionAUTest:
    """Direct test class for Vision and AU detection on QTrobot using PyFeat."""

    def __init__(self, existing_node=False):
        """Initialize the test."""
        # Check if PyFeat is available
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Please install it with: pip install py-feat")
            return

        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('direct_vision_au_test', anonymous=True)

        # We won't use speech in this test
        self.speech_pub = None

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
        self.face_detected = threading.Event()
        self.face_detection_timeout = threading.Event()

        # Subscribe to camera topic
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', RosImage, self.image_callback)

    def image_callback(self, msg):
        """Callback for camera image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"CV Bridge error: {e}")
        except Exception as e:
            print(f"Error in image callback: {str(e)}")

    def capture_and_save_image(self):
        """Capture an image from the camera and save it to a file."""
        if self.current_image is None:
            print("No image available from camera.")
            return None

        # Create directory if it doesn't exist
        os.makedirs("/tmp/pyfeat_test", exist_ok=True)

        # Save the image to a file
        image_path = "/tmp/pyfeat_test/current_frame.jpg"
        cv2.imwrite(image_path, self.current_image)

        print(f"Image saved to {image_path}")
        return image_path

    def run_test(self):
        """Run the vision and AU detection test."""
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Test cannot run.")
            return False

        print("Testing Vision and AU detection with PyFeat...")

        # Instruct the user (print only, no speech)
        instructions = "I will now test my vision system with PyFeat. Please stand in front of me so I can see your face."
        print(f"\nInstruction: {instructions}")
        time.sleep(2)

        # Wait for camera to start providing images
        print("Waiting for camera to provide images...")
        timeout = time.time() + 10  # 10 seconds timeout
        while self.current_image is None and time.time() < timeout:
            time.sleep(0.1)

        if self.current_image is None:
            print("No images received from camera within timeout period.")
            return False

        # Capture and save the current frame
        print("Capturing and saving current frame...")
        image_path = self.capture_and_save_image()

        if image_path is None:
            print("Failed to save image.")
            return False

        # Process the image with PyFeat
        try:
            print("Processing image with PyFeat...")
            results = self.detector.detect_image(image_path)

            # Check if any faces were detected
            if results.empty:
                print("No faces detected in the image.")
                return False

            print("Face detected! Extracting emotions and AUs...")

            # Print all columns in the results
            print("\nAvailable columns in results:")
            for col in results.columns:
                print(f"  {col}")

            # Extract emotions
            emotions = {}
            for col in results.columns:
                if col.startswith('emotion_'):
                    emotion_name = col.replace('emotion_', '')
                    emotions[emotion_name] = float(results[col].iloc[0])

            # Extract action units
            aus = {}
            for col in results.columns:
                if col.startswith('AU') or col.lower().startswith('au'):
                    aus[col] = float(results[col].iloc[0])

            # Determine dominant emotion
            if emotions:
                dominant_emotion = max(emotions.items(), key=lambda x: x[1])[0]

                # Print results
                print(f"\nDominant Emotion: {dominant_emotion}")

                print("\nEmotion Confidence Values:")
                for emotion, confidence in emotions.items():
                    print(f"  {emotion}: {confidence:.2f}")
            else:
                print("No emotions detected.")
                dominant_emotion = "neutral"

            if aus:
                print("\nAction Units (AUs):")
                for au, value in aus.items():
                    print(f"  {au}: {value:.2f}")
            else:
                print("\nNo Action Units detected.")

            # Provide feedback (print only, no speech)
            feedback = f"I can see you! You appear to be {dominant_emotion}."
            print(f"\nResult: {feedback}")

            return True

        except Exception as e:
            print(f"Error processing image with PyFeat: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

if __name__ == "__main__":
    try:
        test = DirectVisionAUTest()
        success = test.run_test()
        if success:
            print("\nVision and AU Detection test successful! ✅")
        else:
            print("\nVision and AU Detection test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
        import traceback
        traceback.print_exc()
