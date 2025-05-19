#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to test the microphone directly using PyAudio.
This script helps debug if the microphone hardware is working correctly.
"""

import pyaudio
import numpy as np
import time
import sys

def test_microphone(duration=10, threshold=500, device_index=None):
    """Test the microphone by recording audio and detecting sound.
    
    Args:
        duration (int): Duration to record in seconds.
        threshold (int): Threshold for sound detection.
        device_index (int): Index of the audio device to use. None for default.
    
    Returns:
        bool: True if sound was detected, False otherwise.
    """
    print(f"Testing microphone for {duration} seconds...")
    print(f"Sound detection threshold: {threshold}")
    
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    
    # List available audio devices
    print("\nAvailable audio devices:")
    for i in range(p.get_device_count()):
        dev_info = p.get_device_info_by_index(i)
        print(f"  Device {i}: {dev_info['name']}")
        print(f"    Input channels: {dev_info['maxInputChannels']}")
        print(f"    Output channels: {dev_info['maxOutputChannels']}")
        print(f"    Default sample rate: {dev_info['defaultSampleRate']}")
    
    # Use default device if none specified
    if device_index is None:
        device_index = p.get_default_input_device_info()['index']
        print(f"\nUsing default input device: {device_index}")
    
    # Get device info
    device_info = p.get_device_info_by_index(device_index)
    print(f"Selected device: {device_info['name']}")
    
    # Open stream
    stream = p.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        input_device_index=device_index,
        frames_per_buffer=1024
    )
    
    print("\nRecording... Please speak into the microphone.")
    print("Volume levels:")
    
    # Record for the specified duration
    sound_detected = False
    start_time = time.time()
    
    try:
        while time.time() - start_time < duration:
            # Read audio data
            data = stream.read(1024, exception_on_overflow=False)
            
            # Convert to numpy array
            audio_data = np.frombuffer(data, dtype=np.int16)
            
            # Calculate volume
            volume = np.abs(audio_data).mean()
            
            # Print volume level
            bar_length = int(volume / 100)
            bar = "#" * min(bar_length, 50)
            sys.stdout.write(f"\rVolume: {volume:.0f} {bar}")
            sys.stdout.flush()
            
            # Check if sound is detected
            if volume > threshold:
                sound_detected = True
    
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
    except Exception as e:
        print(f"\nError during recording: {e}")
    finally:
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()
    
    print("\n\nRecording complete!")
    
    if sound_detected:
        print("Sound was detected! ✅")
    else:
        print("No sound was detected. ❌")
        print("Possible issues:")
        print("  1. Microphone is not connected properly")
        print("  2. Microphone is muted")
        print("  3. Sound threshold is too high")
        print("  4. Wrong audio device selected")
    
    return sound_detected

if __name__ == "__main__":
    # Parse command line arguments
    device_index = None
    duration = 10
    threshold = 500
    
    if len(sys.argv) > 1:
        try:
            device_index = int(sys.argv[1])
        except ValueError:
            print(f"Invalid device index: {sys.argv[1]}. Using default.")
    
    if len(sys.argv) > 2:
        try:
            duration = int(sys.argv[2])
        except ValueError:
            print(f"Invalid duration: {sys.argv[2]}. Using default: {duration} seconds.")
    
    if len(sys.argv) > 3:
        try:
            threshold = int(sys.argv[3])
        except ValueError:
            print(f"Invalid threshold: {sys.argv[3]}. Using default: {threshold}.")
    
    try:
        test_microphone(duration, threshold, device_index)
    except Exception as e:
        print(f"Error during microphone test: {e}")
