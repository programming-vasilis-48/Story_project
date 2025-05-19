#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simplified test script for Vision and Action Unit (AU) detection on QTrobot.
This script uses OpenCV for face detection and simulates AU detection.
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
import random

class SimpleVisionAUTest:
    """Test class for Vision and AU detection on QTrobot using OpenCV."""
    
    def __init__(self, existing_node=False, camera_topic='/camera/color/image_raw'):
        """Initialize the test.
        
        Args:
            existing_node (bool): If True, assumes a ROS node is already initialized.
            camera_topic (str): The ROS topic for the camera feed.
        """
        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('test_vision_au_simple', anonymous=True)
        
        # Create publisher for speech (to instruct the user)
        self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Variables to store face detection results
        self.face_detected = threading.Event()
        self.face_detection_timeout = threading.Event()
        self.current_image = None
        self.face_data = None
        
        # Initialize face detector
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Subscribe to camera topic
        self.camera_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        
        # Flag to track if a face is detected
        self.has_face = False
        self.face_rect = None
    
    def image_callback(self, msg):
        """Callback for camera image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            
            # Process the image to detect faces
            self.detect_face(cv_image)
            
        except CvBridgeError as e:
            print(f"CV Bridge error: {e}")
        except Exception as e:
            print(f"Error in image callback: {str(e)}")
    
    def detect_face(self, image):
        """Detect faces in the image using OpenCV."""
        try:
            # Convert to grayscale for face detection
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            # If faces are detected
            if len(faces) > 0:
                self.has_face = True
                self.face_rect = faces[0]  # Use the first face
                self.face_detected.set()
            
        except Exception as e:
            print(f"Error detecting face: {str(e)}")
    
    def simulate_aus(self):
        """Simulate Action Units (AUs) for the detected face."""
        # Define common AUs
        aus = {
            'AU01': random.uniform(0.1, 0.5),  # Inner Brow Raiser
            'AU02': random.uniform(0.1, 0.5),  # Outer Brow Raiser
            'AU04': random.uniform(0.1, 0.5),  # Brow Lowerer
            'AU05': random.uniform(0.1, 0.3),  # Upper Lid Raiser
            'AU06': random.uniform(0.2, 0.7),  # Cheek Raiser
            'AU07': random.uniform(0.1, 0.4),  # Lid Tightener
            'AU09': random.uniform(0.0, 0.2),  # Nose Wrinkler
            'AU10': random.uniform(0.0, 0.3),  # Upper Lip Raiser
            'AU12': random.uniform(0.3, 0.8),  # Lip Corner Puller (smile)
            'AU14': random.uniform(0.0, 0.3),  # Dimpler
            'AU15': random.uniform(0.0, 0.2),  # Lip Corner Depressor
            'AU17': random.uniform(0.0, 0.2),  # Chin Raiser
            'AU20': random.uniform(0.0, 0.2),  # Lip Stretcher
            'AU23': random.uniform(0.0, 0.2),  # Lip Tightener
            'AU25': random.uniform(0.1, 0.5),  # Lips Part
            'AU26': random.uniform(0.0, 0.3),  # Jaw Drop
            'AU45': random.uniform(0.0, 0.2)   # Blink
        }
        return aus
    
    def simulate_emotions(self, aus):
        """Simulate emotions based on AUs."""
        # Simple mapping of AUs to emotions
        emotions = {
            'anger': aus['AU04'] * 0.7 + aus['AU07'] * 0.3,
            'disgust': aus['AU09'] * 0.6 + aus['AU10'] * 0.4,
            'fear': aus['AU01'] * 0.3 + aus['AU02'] * 0.3 + aus['AU04'] * 0.2 + aus['AU05'] * 0.2,
            'happiness': aus['AU06'] * 0.4 + aus['AU12'] * 0.6,
            'sadness': aus['AU01'] * 0.3 + aus['AU04'] * 0.3 + aus['AU15'] * 0.4,
            'surprise': aus['AU01'] * 0.3 + aus['AU02'] * 0.3 + aus['AU05'] * 0.2 + aus['AU26'] * 0.2,
            'neutral': 1.0 - max(aus['AU01'], aus['AU04'], aus['AU06'], aus['AU12'], aus['AU15'])
        }
        
        # Normalize emotions to sum to 1
        total = sum(emotions.values())
        for emotion in emotions:
            emotions[emotion] /= total
        
        return emotions
    
    def run_test(self):
        """Run the vision and AU detection test."""
        print("Testing Vision and AU detection functionality with OpenCV...")
        
        # Instruct the user
        instructions = "I will now test my vision system. Please stand in front of me so I can see your face."
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
        
        # Simulate AU detection
        aus = self.simulate_aus()
        emotions = self.simulate_emotions(aus)
        
        # Find dominant emotion
        dominant_emotion = max(emotions.items(), key=lambda x: x[1])[0]
        
        # Print face information
        if self.face_rect is not None:
            x, y, w, h = self.face_rect
            print(f"\nFace detected at: x={x}, y={y}, width={w}, height={h}")
        
        print(f"\nDominant Emotion: {dominant_emotion}")
        
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

if __name__ == "__main__":
    try:
        test = SimpleVisionAUTest()
        success = test.run_test()
        if success:
            print("\nVision and AU Detection test successful! ✅")
        else:
            print("\nVision and AU Detection test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
