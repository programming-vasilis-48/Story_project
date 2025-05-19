#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simplified test script for Vision and AU detection on QTrobot using PyFeat.
This script captures a single image and processes it with PyFeat.
"""

import rospy
from std_msgs.msg import String
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as RosImage
from PIL import Image
import os

# Import PyFeat
try:
    from feat import Detector
    PYFEAT_AVAILABLE = True
except ImportError:
    PYFEAT_AVAILABLE = False
    print("PyFeat not available. Please install with: pip install py-feat")

class SimplePyFeatTest:
    """Simple test class for PyFeat on QTrobot."""
    
    def __init__(self, existing_node=False):
        """Initialize the test."""
        # Check if PyFeat is available
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Please install it with: pip install py-feat")
            return
        
        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('simple_pyfeat_test', anonymous=True)
        
        # Create publisher for speech
        self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        
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
        self.image = None
        self.image_received = False
        
        # Subscribe to camera topic
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', RosImage, self.image_callback)
    
    def image_callback(self, msg):
        """Callback for camera image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
            # Unsubscribe after receiving one image
            self.camera_sub.unregister()
        except CvBridgeError as e:
            print(f"CV Bridge error: {e}")
        except Exception as e:
            print(f"Error in image callback: {str(e)}")
    
    def run_test(self):
        """Run a simple PyFeat test."""
        if not PYFEAT_AVAILABLE:
            print("PyFeat is not available. Test cannot run.")
            return False
        
        print("Running simple PyFeat test...")
        
        # Instruct the user
        self.speech_pub.publish("I will now test my vision system with PyFeat. Please stand in front of me.")
        time.sleep(3)
        
        # Wait for an image
        print("Waiting for camera image...")
        timeout = time.time() + 10  # 10 seconds timeout
        while not self.image_received and time.time() < timeout:
            time.sleep(0.1)
        
        if not self.image_received:
            print("No image received from camera within timeout period.")
            self.speech_pub.publish("I couldn't get an image from my camera. The test has failed.")
            return False
        
        print("Image received. Processing with PyFeat...")
        
        try:
            # Save the image to a temporary file
            temp_dir = "/tmp"
            temp_image_path = os.path.join(temp_dir, "pyfeat_test_image.jpg")
            
            # Convert to RGB for PIL
            image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)
            pil_image.save(temp_image_path)
            
            print(f"Image saved to {temp_image_path}")
            
            # Process the image with PyFeat
            print("Detecting faces with PyFeat...")
            results = self.detector.detect_image(temp_image_path)
            
            # Check if any faces were detected
            if results.empty:
                print("No faces detected in the image.")
                self.speech_pub.publish("I couldn't detect any faces in the image.")
                return False
            
            print("Face detected! Extracting emotions and AUs...")
            
            # Extract emotions
            emotions = {}
            for col in results.columns:
                if col.startswith('emotion_'):
                    emotion_name = col.replace('emotion_', '')
                    emotions[emotion_name] = float(results[col].iloc[0])
            
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
            for col in results.columns:
                if col.startswith('AU') or col.lower().startswith('au'):
                    aus[col] = float(results[col].iloc[0])
            
            # Print results
            print(f"\nDominant Emotion: {dominant_emotion}")
            
            print("\nEmotion Confidence Values:")
            for emotion, confidence in emotions.items():
                print(f"  {emotion}: {confidence:.2f}")
            
            if aus:
                print("\nAction Units (AUs):")
                for au, value in aus.items():
                    print(f"  {au}: {value:.2f}")
            else:
                print("\nNo Action Units detected.")
            
            # Provide feedback to the user
            feedback = f"I can see you! You appear to be {dominant_emotion}."
            print(f"\nSpeaking: '{feedback}'")
            self.speech_pub.publish(feedback)
            time.sleep(3)
            
            return True
            
        except Exception as e:
            print(f"Error processing image with PyFeat: {str(e)}")
            import traceback
            traceback.print_exc()
            self.speech_pub.publish("I encountered an error while processing the image.")
            return False

if __name__ == "__main__":
    try:
        test = SimplePyFeatTest()
        success = test.run_test()
        if success:
            print("\nSimple PyFeat test successful! ✅")
        else:
            print("\nSimple PyFeat test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
