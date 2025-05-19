#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for Vision and Action Unit (AU) detection on QTrobot.
This script tests if the robot can detect faces and facial expressions.
"""

import rospy
from std_msgs.msg import String
import time
import json
import threading

class VisionAUTest:
    """Test class for Vision and AU detection on QTrobot."""
    
    def __init__(self):
        """Initialize the test."""
        # Initialize ROS node
        rospy.init_node('test_vision_au', anonymous=True)
        
        # Create publisher for speech (to instruct the user)
        self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        
        # Variables to store face detection results
        self.faces_data = None
        self.face_detected = threading.Event()
        self.face_detection_timeout = threading.Event()
        
        # Subscribe to faces topic
        rospy.Subscriber('/qt_nuitrack_app/faces', String, self.faces_callback)
        
        # Flag to track if a face is detected
        self.has_face = False
    
    def faces_callback(self, msg):
        """Callback for faces messages."""
        try:
            # Parse the JSON data
            faces_data = json.loads(msg.data)
            
            # Check if any faces are detected
            if faces_data and len(faces_data) > 0:
                self.faces_data = faces_data
                self.has_face = True
                self.face_detected.set()
            else:
                self.has_face = False
        except json.JSONDecodeError:
            print("Error parsing faces data JSON")
        except Exception as e:
            print(f"Error in faces callback: {str(e)}")
    
    def run_test(self):
        """Run the vision and AU detection test."""
        print("Testing Vision and AU detection functionality...")
        
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
        
        # Extract and display face data
        if self.faces_data:
            face = self.faces_data[0]  # Get the first face
            
            # Extract basic face information
            face_id = face.get('id', 'unknown')
            gender = face.get('gender', 'unknown')
            age = face.get('age', 'unknown')
            
            # Extract emotions if available
            emotions = face.get('emotions', {})
            dominant_emotion = max(emotions.items(), key=lambda x: x[1])[0] if emotions else 'unknown'
            
            # Print face information
            print(f"\nFace ID: {face_id}")
            print(f"Estimated Gender: {gender}")
            print(f"Estimated Age: {age}")
            print(f"Dominant Emotion: {dominant_emotion}")
            
            # Print all emotions with confidence values
            if emotions:
                print("\nEmotion Confidence Values:")
                for emotion, confidence in emotions.items():
                    print(f"  {emotion}: {confidence:.2f}")
            
            # Extract action units if available
            aus = face.get('aus', {})
            if aus:
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
        test = VisionAUTest()
        success = test.run_test()
        if success:
            print("\nVision and AU Detection test successful! ✅")
        else:
            print("\nVision and AU Detection test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
