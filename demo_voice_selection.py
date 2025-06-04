#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Demo script for voice selection functionality.

This script demonstrates the voice selection system without running the full game.
"""

import os
import sys
import time

# Add the modules directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def demo_voice_selection():
    """Demonstrate the voice selection functionality."""
    print("=" * 60)
    print("VOICE SELECTION DEMO")
    print("=" * 60)

    # Import the voice selection function
    from main import select_voice_type

    print("This demo will show you the voice selection interface.")
    print("You can choose between:")
    print("1 - Emotional voice (OpenAI TTS with emotional expression)")
    print("2 - Emotionless voice (Natural voice without emotion)")
    print()

    # Get voice selection
    voice_choice = select_voice_type()

    # OpenAI API key for TTS
    openai_api_key = "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"

    # Import the appropriate TTS engine
    if voice_choice == 1:
        print("\nInitializing OpenAI TTS (Emotional Voice)...")
        try:
            from modules.text_to_speech import OpenAITTSEngine

            tts_engine = OpenAITTSEngine(
                api_key=openai_api_key,
                model="gpt-4o-mini-tts"
            )

            print("OpenAI TTS initialized successfully!")
            print("\nTesting emotional voices...")

            # Test different emotions
            emotions = ["happy", "sad", "angry"]
            for emotion in emotions:
                print(f"\nTesting {emotion} emotion...")
                tts_engine.adjust_for_emotion(emotion)
                tts_engine.speak(f"This is a test of the {emotion} emotion in the story.")
                time.sleep(0.5)

        except Exception as e:
            print(f"Error initializing OpenAI TTS: {str(e)}")
            
    elif voice_choice == 3:  # This is option 2 in menu but returns 3 for compatibility
        print("\nInitializing Emotionless TTS (Natural Voice Without Emotion)...")
        try:
            from modules.text_to_speech import EmotionlessTTSEngine

            tts_engine = EmotionlessTTSEngine(
                api_key=openai_api_key,
                model="gpt-4o-mini-tts"
            )

            print("Emotionless TTS initialized successfully!")
            print("\nTesting emotionless voice...")

            # Test basic speech
            tts_engine.speak("This is a test of the emotionless voice that sounds natural but expresses no emotion.")
            time.sleep(0.5)

            # Test with emotion adjustment (should be ignored)
            print("\nTesting emotion adjustment (should be ignored)...")
            emotions = ["happy", "sad", "neutral"]
            for emotion in emotions:
                print(f"\nTrying to set {emotion} emotion (should be ignored)...")
                tts_engine.adjust_for_emotion(emotion)
                tts_engine.speak(f"This should still be emotionless even when attempting {emotion} emotion.")
                time.sleep(0.5)

        except Exception as e:
            print(f"Error initializing Emotionless TTS: {str(e)}")

    print("\n" + "=" * 60)
    print("VOICE SELECTION DEMO COMPLETED")
    print("=" * 60)
    print("The selected voice type will be used throughout the story-switch game.")
    print("This allows for comparison between emotional and emotionless voices")
    print("in your experiment to test if emotional expression matters.")

if __name__ == "__main__":
    try:
        demo_voice_selection()
    except KeyboardInterrupt:
        print("\nDemo interrupted by user. Goodbye!")
    except Exception as e:
        print(f"\nError during demo: {str(e)}")
        import traceback
        traceback.print_exc()
