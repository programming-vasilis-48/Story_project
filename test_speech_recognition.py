#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for the speech recognition with push-to-talk.
"""

import os
import sys
import time

# Add the modules directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the speech recognizer
from modules.mic_to_text import SpeechRecognizer

def test_push_to_talk():
    """Test the push-to-talk speech recognition."""
    print("=" * 60)
    print("TESTING SPEECH RECOGNITION WITH PUSH-TO-TALK")
    print("=" * 60)
    print("This test verifies that speech recognition with push-to-talk works correctly.")
    print("=" * 60)
    
    # Initialize speech recognizer
    recognizer = SpeechRecognizer()
    
    # Test push-to-talk
    print("\nTesting push-to-talk:")
    print("Press and hold 'V' key while speaking, release when done.")
    print("Press Ctrl+C to exit.")
    
    try:
        while True:
            print("\nWaiting for speech input...")
            text = recognizer.listen_once(push_to_talk=True, push_to_talk_key='v')
            
            if text:
                print(f"Recognized text: '{text}'")
            else:
                print("No speech detected or recognition failed.")
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"Error during test: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_push_to_talk() 