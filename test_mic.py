#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for the microphone-to-text functionality.

This script tests the SpeechRecognizer class from the mic_to_text module.
"""

import os
import time
import argparse
from dotenv import load_dotenv

# Import the SpeechRecognizer from the mic_to_text module
from modules.mic_to_text import SpeechRecognizer

def test_listen_once(device_index=None, language="en-US"):
    """Test the listen_once method of the SpeechRecognizer class.
    
    Args:
        device_index (int, optional): Index of the microphone device to use
        language (str): Language code for speech recognition
    """
    print("Testing SpeechRecognizer.listen_once...")
    
    # Create a SpeechRecognizer instance
    recognizer = SpeechRecognizer(device_index=device_index, language=language)
    
    # Adjust for ambient noise
    recognizer.adjust_for_ambient_noise(duration=1.0)
    
    # Listen for speech once
    print("\nPlease say something...")
    text = recognizer.listen_once(timeout=5.0, phrase_time_limit=10.0)
    
    # Print the recognized text
    print(f"\nRecognized text: {text}")
    
    return text != ""

def test_background_listening(device_index=None, language="en-US", duration=10.0):
    """Test the background listening functionality of the SpeechRecognizer class.
    
    Args:
        device_index (int, optional): Index of the microphone device to use
        language (str): Language code for speech recognition
        duration (float): Duration in seconds to listen for speech
    """
    print("Testing SpeechRecognizer background listening...")
    
    # Create a SpeechRecognizer instance
    recognizer = SpeechRecognizer(device_index=device_index, language=language)
    
    # Adjust for ambient noise
    recognizer.adjust_for_ambient_noise(duration=1.0)
    
    # Define a callback function
    def callback(text):
        print(f"Callback received: {text}")
    
    # Start listening in the background
    recognizer.start_listening(callback)
    
    # Listen for a specified duration
    print(f"\nListening for {duration} seconds. Please speak...")
    time.sleep(duration)
    
    # Stop listening
    recognizer.stop_listening()
    
    print("Background listening test completed")
    return True

def main():
    """Main function to run the tests."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Test the microphone-to-text functionality")
    parser.add_argument("--device", type=int, help="Index of the microphone device to use")
    parser.add_argument("--language", type=str, default="en-US", help="Language code for speech recognition")
    parser.add_argument("--test", type=str, choices=["once", "background", "both"], default="both", 
                        help="Test to run: 'once', 'background', or 'both'")
    parser.add_argument("--duration", type=float, default=10.0, 
                        help="Duration in seconds for background listening test")
    args = parser.parse_args()
    
    # Run the tests
    if args.test == "once" or args.test == "both":
        test_listen_once(device_index=args.device, language=args.language)
    
    if args.test == "background" or args.test == "both":
        test_background_listening(device_index=args.device, language=args.language, duration=args.duration)
    
    print("\nAll tests completed")

if __name__ == "__main__":
    main()
