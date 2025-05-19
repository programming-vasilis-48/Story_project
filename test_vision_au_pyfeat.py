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
        
        # Initialize PyFeat detector
        self.detector = Detector(
            face_model="retinaface",
            landmark_model="mobilenet",
            au_model="jaanet",
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
            # Detect faces and facial expressions
            faces = self.detector.detect_faces(np.array([image]))
            
            # Check if any faces are detected
            if faces and len(faces) > 0:
                # Process the first face
                self.face_data = self.detector.detect_aus(faces)
                self.has_face = True
                self.face_detected.set()
            
        except Exception as e:
            print(f"Error processing image with PyFeat: {str(e)}")
    
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
        if self.face_data is not None:
            # Extract emotions
            emotions = self.face_data.emotions.iloc[0].to_dict()
            dominant_emotion = max(emotions.items(), key=lambda x: x[1])[0]
            
            # Extract action units
            aus = self.face_data.aus.iloc[0].to_dict()
            
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
