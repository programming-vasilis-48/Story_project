#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for Text-to-Speech on QTrobot.
This script tests if the robot can speak text using the QTrobot TTS system.
"""

import rospy
from std_msgs.msg import String
import time

def test_tts():
    """Test the Text-to-Speech functionality of QTrobot."""
    print("Testing Text-to-Speech functionality...")
    
    # Initialize ROS node
    rospy.init_node('test_tts', anonymous=True)
    
    # Create publisher for speech
    speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    
    # Wait for publisher to connect
    time.sleep(1)
    
    # Test phrases
    test_phrases = [
        "Hello, I am QT Robot. Can you hear me?",
        "I am testing my speech capabilities.",
        "This is a test of the text to speech system."
    ]
    
    # Publish test phrases
    for i, phrase in enumerate(test_phrases):
        print(f"\nTest {i+1}: Speaking: '{phrase}'")
        speech_pub.publish(phrase)
        
        # Wait for speech to complete (approximate)
        # Adjust the sleep time based on the length of the phrase
        sleep_time = 2 + len(phrase) * 0.07  # Rough estimate
        time.sleep(sleep_time)
    
    print("\nText-to-Speech test completed.")
    return True

if __name__ == "__main__":
    try:
        success = test_tts()
        if success:
            print("\nText-to-Speech test successful! ✅")
        else:
            print("\nText-to-Speech test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
