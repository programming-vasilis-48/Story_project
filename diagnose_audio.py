#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Audio diagnosis script for QT Robot.

This script helps diagnose audio routing and find the correct audio devices
to ensure OpenAI TTS uses the same internal speaker as QT Robot's built-in TTS.
"""

import os
import sys
import subprocess
import time

def run_command(cmd):
    """Run a shell command and return the output."""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode, result.stdout, result.stderr
    except Exception as e:
        return -1, "", str(e)

def diagnose_audio_devices():
    """Diagnose available audio devices."""
    print("=" * 60)
    print("AUDIO DEVICE DIAGNOSIS")
    print("=" * 60)
    
    # Check ALSA devices
    print("\n1. ALSA Audio Devices:")
    print("-" * 30)
    returncode, stdout, stderr = run_command("aplay -l")
    if returncode == 0:
        print(stdout)
    else:
        print(f"Error: {stderr}")
    
    # Check PulseAudio sinks
    print("\n2. PulseAudio Sinks:")
    print("-" * 30)
    returncode, stdout, stderr = run_command("pactl list short sinks")
    if returncode == 0:
        print(stdout)
    else:
        print(f"Error: {stderr}")
    
    # Check audio cards
    print("\n3. Audio Cards:")
    print("-" * 30)
    returncode, stdout, stderr = run_command("cat /proc/asound/cards")
    if returncode == 0:
        print(stdout)
    else:
        print(f"Error: {stderr}")

def test_audio_devices():
    """Test different audio devices with a sample sound."""
    print("\n" + "=" * 60)
    print("TESTING AUDIO DEVICES")
    print("=" * 60)
    
    # Create a test audio file
    test_audio = "/tmp/test_beep.wav"
    print(f"Creating test audio file: {test_audio}")
    
    # Generate a simple beep
    returncode, stdout, stderr = run_command(f"speaker-test -t sine -f 1000 -l 1 -s 1 -w {test_audio}")
    if returncode != 0:
        print("Failed to create test audio file")
        return
    
    # Test different ALSA devices
    alsa_devices = ["hw:0,0", "hw:1,0", "hw:2,0", "default", "plughw:0,0", "plughw:1,0"]
    
    for device in alsa_devices:
        print(f"\nTesting ALSA device: {device}")
        print("Press Enter to play test sound, or 's' to skip...")
        user_input = input().strip().lower()
        if user_input == 's':
            continue
            
        returncode, stdout, stderr = run_command(f"aplay -D {device} {test_audio}")
        if returncode == 0:
            print(f"✅ Device {device} works!")
            print("Did you hear the sound from the robot's internal speaker? (y/n)")
            response = input().strip().lower()
            if response == 'y':
                print(f"🎯 FOUND INTERNAL SPEAKER DEVICE: {device}")
        else:
            print(f"❌ Device {device} failed: {stderr}")
    
    # Clean up
    try:
        os.remove(test_audio)
    except:
        pass

def test_qtrobot_tts_routing():
    """Test QT Robot's built-in TTS to see which device it uses."""
    print("\n" + "=" * 60)
    print("TESTING QT ROBOT BUILT-IN TTS ROUTING")
    print("=" * 60)
    
    try:
        # Add the modules directory to the Python path
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from modules.text_to_speech import QTRobotTTSEngine
        
        print("Initializing QT Robot TTS...")
        qtrobot_tts = QTRobotTTSEngine()
        
        print("Playing test message through QT Robot built-in TTS...")
        print("Listen carefully to identify which speaker is being used.")
        qtrobot_tts.speak("This is QT Robot built-in TTS. Listen to which speaker this comes from.")
        
        print("\nDid the sound come from:")
        print("1. Internal speaker (inside the robot)")
        print("2. External speaker (screen/monitor)")
        print("3. Other/Not sure")
        
        choice = input("Enter choice (1/2/3): ").strip()
        if choice == "1":
            print("✅ QT Robot built-in TTS uses INTERNAL speaker")
        elif choice == "2":
            print("⚠️  QT Robot built-in TTS uses EXTERNAL speaker")
        else:
            print("❓ Speaker location unclear")
            
        return choice
        
    except Exception as e:
        print(f"Error testing QT Robot TTS: {str(e)}")
        return None

def test_openai_tts_routing():
    """Test OpenAI TTS routing."""
    print("\n" + "=" * 60)
    print("TESTING OPENAI TTS ROUTING")
    print("=" * 60)
    
    try:
        # Add the modules directory to the Python path
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from modules.text_to_speech import OpenAITTSEngine
        
        # OpenAI API key for TTS
        openai_api_key = "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"
        
        print("Initializing OpenAI TTS with QT Robot audio routing...")
        openai_tts = OpenAITTSEngine(
            api_key=openai_api_key,
            model="gpt-4o-mini-tts",
            use_qtrobot_audio=True
        )
        
        print("Playing test message through OpenAI TTS...")
        print("Listen carefully to identify which speaker is being used.")
        openai_tts.speak("This is OpenAI TTS routed through QT Robot audio system. Listen to which speaker this comes from.")
        
        print("\nDid the sound come from:")
        print("1. Internal speaker (inside the robot)")
        print("2. External speaker (screen/monitor)")
        print("3. Other/Not sure")
        
        choice = input("Enter choice (1/2/3): ").strip()
        if choice == "1":
            print("✅ OpenAI TTS uses INTERNAL speaker")
        elif choice == "2":
            print("⚠️  OpenAI TTS uses EXTERNAL speaker")
        else:
            print("❓ Speaker location unclear")
            
        return choice
        
    except Exception as e:
        print(f"Error testing OpenAI TTS: {str(e)}")
        return None

def main():
    """Main diagnosis function."""
    print("QT ROBOT AUDIO DIAGNOSIS TOOL")
    print("=" * 60)
    print("This tool helps diagnose audio routing issues and find the correct")
    print("audio devices to ensure both TTS systems use the same speaker.")
    print()
    
    # Step 1: Diagnose audio devices
    diagnose_audio_devices()
    
    # Step 2: Test QT Robot built-in TTS
    qtrobot_speaker = test_qtrobot_tts_routing()
    
    # Step 3: Test OpenAI TTS
    openai_speaker = test_openai_tts_routing()
    
    # Step 4: Compare results
    print("\n" + "=" * 60)
    print("DIAGNOSIS SUMMARY")
    print("=" * 60)
    
    if qtrobot_speaker and openai_speaker:
        if qtrobot_speaker == openai_speaker:
            if qtrobot_speaker == "1":
                print("✅ BOTH TTS systems use the INTERNAL speaker - PERFECT!")
            else:
                print("⚠️  Both TTS systems use the same speaker, but it's external")
        else:
            print("❌ TTS systems use DIFFERENT speakers - NEEDS FIXING!")
            print(f"   QT Robot built-in: {'Internal' if qtrobot_speaker == '1' else 'External'}")
            print(f"   OpenAI TTS: {'Internal' if openai_speaker == '1' else 'External'}")
    
    # Step 5: Manual device testing (optional)
    print("\nWould you like to manually test audio devices? (y/n)")
    if input().strip().lower() == 'y':
        test_audio_devices()
    
    print("\n" + "=" * 60)
    print("DIAGNOSIS COMPLETE")
    print("=" * 60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDiagnosis interrupted by user. Goodbye!")
    except Exception as e:
        print(f"\nError during diagnosis: {str(e)}")
        import traceback
        traceback.print_exc()
