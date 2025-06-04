#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for the Emotionless voice feature.

This script demonstrates the emotionless voice which uses OpenAI's TTS
but with instructions to speak in a completely neutral, flat tone without emotion.
"""

import os
import sys
import time

# Add the modules directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def main():
    """Main test function."""
    print("=" * 60)
    print("EMOTIONLESS VOICE TEST")
    print("=" * 60)
    print()
    print("This test demonstrates the emotionless voice which uses")
    print("OpenAI's TTS with instructions to speak in a completely")
    print("neutral, flat tone without any emotional expression.")
    print()
    
    # Import the emotionless TTS engine
    from modules.text_to_speech import EmotionlessTTSEngine
    
    # OpenAI API key - use the same one as in the other test scripts
    openai_api_key = "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"
    
    # Initialize TTS engine
    tts_engine = EmotionlessTTSEngine(
        api_key=openai_api_key,
        model="gpt-4o-mini-tts"
    )
    
    print("Initializing Emotionless TTS engine...")
    print()
    
    # Test basic speech
    print("Testing basic speech...")
    tts_engine.speak("Hello, I am speaking with an emotionless voice. I do not express any emotion in my tone or delivery.")
    time.sleep(1)
    
    # Test different emotions (which should be ignored)
    print("\nTesting emotion adjustment (should be ignored)...")
    
    emotions = ["happy", "sad", "neutral"]
    for emotion in emotions:
        print(f"\nTrying to use {emotion} emotion (should be ignored)...")
        tts_engine.adjust_for_emotion(emotion)
        tts_engine.speak(f"This is a test of what should be an emotionless voice even when asked to be {emotion}.")
        time.sleep(1)
        
    # Test with different texts that typically evoke emotion
    print("\nTesting emotionless delivery with emotionally charged text...")
    
    emotional_texts = [
        "This is extremely exciting news! We've won the grand prize!",
        "I'm deeply saddened by the terrible news. It's a tragic loss.",
        "I'm absolutely furious about what happened. This is outrageous!"
    ]
    
    for text in emotional_texts:
        print(f"\nSpeaking emotionally charged text with emotionless voice...")
        tts_engine.speak(text)
        time.sleep(1)
    
    # Final test
    print("\nFinal demonstration...")
    tts_engine.speak("This is the emotionless voice for QT Robot. I speak naturally but with a neutral, flat tone that conveys no emotional expression. This allows for testing whether emotional expression in speech impacts human listeners.")
    
    print("\n" + "=" * 60)
    print("EMOTIONLESS VOICE TEST COMPLETED")
    print("=" * 60)
    print("\nYou can use this voice in your application by:")
    print("1. Importing the EmotionlessTTSEngine class from modules.text_to_speech")
    print("2. Creating an instance with your API key")
    print("3. Using the speak() method to generate emotionless speech")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted by user. Goodbye!")
    except Exception as e:
        print(f"\nError during test: {str(e)}")
        import traceback
        traceback.print_exc() 