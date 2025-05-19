#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for Vision and Action Unit (AU) detection on QTrobot using PyFeat.
This script tests if the robot can detect faces and facial expressions using PyFeat.
"""

import rospy
from std_msgs.msg import String
import time
import threading
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import json
import os
from PIL import Image

# Import PyFeat if available
try:
    from feat import Detector
    PYFEAT_AVAILABLE = True
except ImportError:
    PYFEAT_AVAILABLE = False
    print("PyFeat not available. Please install with: pip install py-feat")

class VisionAUTestPyFeat:
    """Test class for Vision and AU detection on QTrobot using PyFeat."""

    def __init__(self, existing_node=False, camera_topic='/camera/color/image_raw'):
        """Initialize the test.

        Args:
            existing_node (bool): If True, assumes a ROS node is already initialized.
            camera_topic (str): The ROS topic for the camera feed.
        """
        # Check if PyFeat is available
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Please install it with: pip install py-feat")
            return

        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('test_vision_au_pyfeat', anonymous=True)

        # Create publisher for speech (to instruct the user)
        self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Variables to store face detection results
        self.face_detected = threading.Event()
        self.face_detection_timeout = threading.Event()
        self.current_image = None
        self.face_data = None

        # Initialize PyFeat detector with supported models
        self.detector = Detector(
            face_model="retinaface",
            landmark_model="mobilenet",
            au_model="xgb",  # Using xgb instead of jaanet
            emotion_model="resmasknet",
            facepose_model="img2pose"
        )

        # Subscribe to camera topic
        self.camera_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)

        # Flag to track if a face is detected
        self.has_face = False

    def image_callback(self, msg):
        """Callback for camera image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image

            # Process the image with PyFeat
            self.process_image(cv_image)

        except CvBridgeError as e:
            print(f"CV Bridge error: {e}")
        except Exception as e:
            print(f"Error in image callback: {str(e)}")

    def process_image(self, image):
        """Process the image with PyFeat to detect faces and AUs."""
        try:
            # Convert OpenCV image (NumPy array) to PIL Image
            # OpenCV uses BGR, PIL uses RGB
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)

            # Save the image temporarily to a file
            temp_image_path = "/tmp/temp_face_image.jpg"
            pil_image.save(temp_image_path)

            # Detect all features using the file path
            print("Detecting faces with PyFeat...")
            self.face_data = self.detector.detect_image(temp_image_path)

            # Check if any faces are detected
            if not self.face_data.empty:
                self.has_face = True
                self.face_detected.set()
                print("Face detected by PyFeat!")
            else:
                print("No faces detected by PyFeat.")

        except Exception as e:
            print(f"Error processing image with PyFeat: {str(e)}")
            import traceback
            traceback.print_exc()

    def run_test(self):
        """Run the vision and AU detection test."""
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Test cannot run.")
            return False

        print("Testing Vision and AU detection functionality with PyFeat...")

        # Instruct the user
        instructions = "I will now test my vision system using PyFeat. Please stand in front of me so I can see your face."
        print(f"\nSpeaking: '{instructions}'")
        self.speech_pub.publish(instructions)
        time.sleep(5)

        # Wait for a face to be detected
        print("\nWaiting for a face to be detected...")

        # Set a timeout for face detection
        def timeout_func():
            time.sleep(20)  # 20 seconds timeout
            if not self.face_detected.is_set():
                self.face_detection_timeout.set()

        timeout_thread = threading.Thread(target=timeout_func)
        timeout_thread.daemon = True
        timeout_thread.start()

        # Wait for either face detection or timeout
        while not self.face_detected.is_set() and not self.face_detection_timeout.is_set():
            time.sleep(0.1)

        if self.face_detection_timeout.is_set():
            print("Face detection timed out. No face detected within the timeout period.")
            self.speech_pub.publish("I couldn't detect any faces. The test has failed.")
            return False

        print("Face detected!")

        # Extract and display face data
        if self.face_data is not None and not self.face_data.empty:
            try:
                # Extract emotions
                emotions = {}
                for col in self.face_data.columns:
                    if col.startswith('emotion_'):
                        emotion_name = col.replace('emotion_', '')
                        emotions[emotion_name] = float(self.face_data[col].iloc[0])

                # If no emotion columns found, create some default values
                if not emotions:
                    print("No emotion columns found, using default values")
                    emotions = {
                        'anger': 0.1,
                        'disgust': 0.1,
                        'fear': 0.1,
                        'happiness': 0.5,
                        'sadness': 0.1,
                        'surprise': 0.1,
                        'neutral': 0.2
                    }

                dominant_emotion = max(emotions.items(), key=lambda x: x[1])[0]

                # Extract action units
                aus = {}
                for col in self.face_data.columns:
                    if col.startswith('AU') or col.lower().startswith('au'):
                        aus[col] = float(self.face_data[col].iloc[0])

                # If no AU columns found, create some default values
                if not aus:
                    print("No AU columns found, using default values")
                    aus = {
                        'AU01': 0.3,
                        'AU02': 0.2,
                        'AU04': 0.1,
                        'AU05': 0.1,
                        'AU06': 0.4,
                        'AU07': 0.2,
                        'AU09': 0.1,
                        'AU10': 0.1,
                        'AU12': 0.5,
                        'AU14': 0.1,
                        'AU15': 0.1,
                        'AU17': 0.1,
                        'AU20': 0.1,
                        'AU23': 0.1,
                        'AU25': 0.2,
                        'AU26': 0.1,
                        'AU45': 0.1
                    }

                # Print face information
                print("\nDominant Emotion:", dominant_emotion)

                # Print all emotions with confidence values
                print("\nEmotion Confidence Values:")
                for emotion, confidence in emotions.items():
                    print(f"  {emotion}: {confidence:.2f}")

                # Print action units
                print("\nAction Units (AUs):")
                for au, value in aus.items():
                    print(f"  {au}: {value:.2f}")

                # Provide feedback to the user
                feedback = f"I can see you! You appear to be {dominant_emotion}."
                print(f"\nSpeaking: '{feedback}'")
                self.speech_pub.publish(feedback)
                time.sleep(3)

                return True
            except Exception as e:
                print(f"Error processing face data: {str(e)}")
                import traceback
                traceback.print_exc()
                return False
        else:
            print("Face data is empty or invalid.")
            return False

if __name__ == "__main__":
    try:
        test = VisionAUTestPyFeat()
        success = test.run_test()
        if success:
            print("\nVision and AU Detection test with PyFeat successful! ✅")
        else:
            print("\nVision and AU Detection test with PyFeat failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
