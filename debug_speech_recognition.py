#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Debug script for Speech Recognition on QTrobot.
This script provides more detailed debugging information for speech recognition issues.
"""

import rospy
from std_msgs.msg import String
import time
import threading
import sys

class DebugSpeechRecognition:
    """Debug class for Speech Recognition on QTrobot."""
    
    def __init__(self, existing_node=False):
        """Initialize the test."""
        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('debug_speech_recognition', anonymous=True)
        
        # Create publisher for speech (to instruct the user)
        self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        
        # Variables to store recognition results
        self.recognized_text = None
        self.recognition_received = threading.Event()
        
        # Variables to track messages
        self.sound_direction_received = False
        self.is_speaking_received = False
        self.sound_direction_count = 0
        self.is_speaking_count = 0
        self.last_sound_direction = None
        self.last_is_speaking = None
        
        # Subscribe to sound direction topic to detect when someone is speaking
        print("Subscribing to /qt_respeaker_app/sound_direction...")
        rospy.Subscriber('/qt_respeaker_app/sound_direction', String, self.sound_direction_callback)
        
        # Subscribe to is_speaking topic
        print("Subscribing to /qt_respeaker_app/is_speaking...")
        rospy.Subscriber('/qt_respeaker_app/is_speaking', String, self.is_speaking_callback)
        
        # Flag to track if someone is speaking
        self.is_speaking = False
        self.sound_detected = False
    
    def sound_direction_callback(self, msg):
        """Callback for sound direction messages."""
        self.sound_direction_received = True
        self.sound_direction_count += 1
        self.last_sound_direction = msg.data
        self.sound_detected = True
        print(f"Sound direction message received: {msg.data} (Total: {self.sound_direction_count})")
    
    def is_speaking_callback(self, msg):
        """Callback for is_speaking messages."""
        self.is_speaking_received = True
        self.is_speaking_count += 1
        self.last_is_speaking = msg.data
        self.is_speaking = (msg.data == "true")
        print(f"Is speaking message received: {msg.data} (Total: {self.is_speaking_count})")
    
    def check_speech_service(self):
        """Check if the speech recognition service is available."""
        print("\nChecking for speech recognition service...")
        try:
            rospy.wait_for_service('/qt_robot/speech/recognize', timeout=2)
            print("  /qt_robot/speech/recognize: ✅ Available")
            return True
        except rospy.ROSException:
            print("  /qt_robot/speech/recognize: ❌ Not available")
            return False
    
    def recognize_speech(self):
        """Call the speech recognition service."""
        try:
            # Call the speech recognition service
            print("Waiting for speech recognition service...")
            rospy.wait_for_service('/qt_robot/speech/recognize', timeout=5)
            
            print("Creating service proxy...")
            recognize_service = rospy.ServiceProxy('/qt_robot/speech/recognize', rospy.ServiceType('/qt_robot/speech/recognize'))
            
            print("Calling speech recognition service...")
            response = recognize_service()
            
            # Store the recognized text
            self.recognized_text = response.transcript
            print(f"Received transcript: {self.recognized_text}")
            self.recognition_received.set()
            
            return True
        except rospy.ROSException as e:
            print(f"Service call failed: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error in recognize_speech: {e}")
            return False
    
    def run_test(self, verbose=True):
        """Run the speech recognition test with detailed debugging."""
        print("Starting debug speech recognition test...")
        
        # Check if the speech service is available
        if not self.check_speech_service():
            print("Speech recognition service is not available. Test cannot continue.")
            return False
        
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
        print("Please speak loudly and clearly.")
        
        if verbose:
            print("Monitoring for sound direction and speaking messages...")
            print("(This will show messages as they are received)")
        
        timeout = time.time() + 15  # 15 seconds timeout
        while not self.sound_detected and time.time() < timeout:
            time.sleep(0.1)
            if verbose and (time.time() % 1) < 0.1:  # Print a dot every second
                sys.stdout.write(".")
                sys.stdout.flush()
        
        print("")  # New line after dots
        
        if not self.sound_detected:
            print("\nNo sound detected within timeout period.")
            print("\nDebug information:")
            print(f"  Sound direction messages received: {self.sound_direction_count}")
            print(f"  Is speaking messages received: {self.is_speaking_count}")
            
            if self.sound_direction_count == 0 and self.is_speaking_count == 0:
                print("\nPossible issues:")
                print("  1. ReSpeaker microphone is not connected or not working")
                print("  2. ReSpeaker ROS node is not running")
                print("  3. ReSpeaker topics have different names")
                print("\nTry running 'check_ros_topics.py' to verify topic names.")
            
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
        test = DebugSpeechRecognition()
        success = test.run_test()
        if success:
            print("\nSpeech Recognition test successful! ✅")
        else:
            print("\nSpeech Recognition test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
