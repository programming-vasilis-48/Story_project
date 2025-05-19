#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Direct Speech Recognition test for QTrobot using the Sennheiser microphone.
This script bypasses the ReSpeaker and uses the default microphone directly.
"""

import rospy
from std_msgs.msg import String
import time
import threading
import speech_recognition as sr

class DirectSpeechRecognitionTest:
    """Test class for direct Speech Recognition on QTrobot."""
    
    def __init__(self, existing_node=False):
        """Initialize the test."""
        # Initialize ROS node if needed
        if not existing_node:
            rospy.init_node('direct_speech_recognition', anonymous=True)
        
        # Create publisher for speech (to instruct the user)
        self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        
        # Variables to store recognition results
        self.recognized_text = None
        self.recognition_successful = False
    
    def recognize_speech(self):
        """Use the microphone to recognize speech directly."""
        try:
            # Use the default microphone as the audio source
            with sr.Microphone() as source:
                print("Adjusting for ambient noise...")
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
                
                print("Listening...")
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                
                print("Recognizing...")
                # Use Google's speech recognition
                self.recognized_text = self.recognizer.recognize_google(audio)
                print(f"Google Speech Recognition thinks you said: {self.recognized_text}")
                self.recognition_successful = True
                return True
        except sr.WaitTimeoutError:
            print("No speech detected within timeout period")
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")
        except Exception as e:
            print(f"Error in speech recognition: {e}")
        
        return False
    
    def run_test(self):
        """Run the direct speech recognition test."""
        print("Testing Direct Speech Recognition functionality...")
        
        # Instruct the user
        instructions = "I will now test speech recognition using the Sennheiser microphone directly."
        print(f"\nSpeaking: '{instructions}'")
        self.speech_pub.publish(instructions)
        time.sleep(5)
        
        prompt = "Please speak now. Say anything you like."
        print(f"\nSpeaking: '{prompt}'")
        self.speech_pub.publish(prompt)
        time.sleep(2)
        
        # Recognize speech
        success = self.recognize_speech()
        
        if not success:
            print("Speech recognition failed.")
            return False
        
        # Confirm what was heard
        confirmation = f"I heard you say: {self.recognized_text}"
        print(f"\nSpeaking: '{confirmation}'")
        self.speech_pub.publish(confirmation)
        time.sleep(5)
        
        return True

if __name__ == "__main__":
    try:
        test = DirectSpeechRecognitionTest()
        success = test.run_test()
        if success:
            print("\nDirect Speech Recognition test successful! ✅")
        else:
            print("\nDirect Speech Recognition test failed! ❌")
    except rospy.ROSInterruptException:
        print("Test interrupted!")
    except Exception as e:
        print(f"Error during test: {str(e)}")
