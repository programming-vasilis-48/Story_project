#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for Speech Recognition on QTrobot.
This script tests if the robot can recognize speech and transcribe it to text.
"""

import rospy
from std_msgs.msg import String
import time
import threading

class SpeechRecognitionTest:
    """Test class for Speech Recognition on QTrobot."""
    
    def __init__(self):
        """Initialize the test."""
        # Initialize ROS node
        rospy.init_node('test_speech_recognition', anonymous=True)
        
        # Create publisher for speech (to instruct the user)
        self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        
        # Variables to store recognition results
        self.recognized_text = None
        self.recognition_received = threading.Event()
        
        # Subscribe to sound direction topic to detect when someone is speaking
        rospy.Subscriber('/qt_respeaker_app/sound_direction', String, self.sound_direction_callback)
        
        # Subscribe to is_speaking topic
        rospy.Subscriber('/qt_respeaker_app/is_speaking', String, self.is_speaking_callback)
        
        # Flag to track if someone is speaking
        self.is_speaking = False
        self.sound_detected = False
    
    def sound_direction_callback(self, msg):
        """Callback for sound direction messages."""
        self.sound_detected = True
    
    def is_speaking_callback(self, msg):
        """Callback for is_speaking messages."""
        self.is_speaking = (msg.data == "true")
    
    def recognize_speech(self):
        """Call the speech recognition service."""
        try:
            # Call the speech recognition service
            rospy.wait_for_service('/qt_robot/speech/recognize', timeout=5)
            recognize_service = rospy.ServiceProxy('/qt_robot/speech/recognize', rospy.ServiceType('/qt_robot/speech/recognize'))
            
            # Call the service
            response = recognize_service()
            
            # Store the recognized text
            self.recognized_text = response.transcript
            self.recognition_received.set()
            
            return True
        except rospy.ROSException as e:
            print(f"Service call failed: {e}")
            return False
    
    def run_test(self):
        """Run the speech recognition test."""
        print("Testing Speech Recognition functionality...")
        
        # Instruct the user
        instructions = "I will now test my speech recognition. Please say something when I ask you to speak."
        print(f"\nSpeaking: '{instructions}'")
        self.speech_pub.publish(instructions)
        time.sleep(5)
        
        prompt = "Please speak now. Say anything you like."
        print(f"\nSpeaking: '{prompt}'")
        self.speech_pub.publish(prompt)
        
        # Wait for sound to be detected
        print("\nWaiting for sound...")
        timeout = time.time() + 15  # 15 seconds timeout
        while not self.sound_detected and time.time() < timeout:
            time.sleep(0.1)
        
        if not self.sound_detected:
            print("No sound detected within timeout period.")
            return False
        
        print("Sound detected! Recognizing speech...")
        
        # Start speech recognition
        recognition_thread = threading.Thread(target=self.recognize_speech)
        recognition_thread.start()
        
        # Wait for recognition to complete with timeout
        recognition_success = self.recognition_received.wait(timeout=10)
        
        if not recognition_success:
            print("Speech recognition timed out.")
            return False
        
        # Print the recognized text
        print(f"\nRecognized text: '{self.recognized_text}'")
        
        # Confirm what was heard
        confirmation = f"I heard you say: {self.recognized_text}"
        print(f"\nSpeaking: '{confirmation}'")
        self.speech_pub.publish(confirmation)
        time.sleep(5)
        
        return True

if __name__ == "__main__":
    try:
        test = SpeechRecognitionTest()
        success = test.run_test()
        if success:
            print("\nSpeech Recognition test successful! ✅")
        else:
            print("\nSpeech Recognition test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
