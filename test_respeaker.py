#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to test the ReSpeaker microphone array directly.
This script helps debug if the ReSpeaker hardware is working correctly.
"""

import pyaudio
import numpy as np
import time
import sys
import webrtcvad
import struct

class ReSpeakerTest:
    """Class to test the ReSpeaker microphone array."""
    
    def __init__(self, device_index=None):
        """Initialize the test.
        
        Args:
            device_index (int): Index of the audio device to use. None for default.
        """
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        
        # Find ReSpeaker device if not specified
        if device_index is None:
            device_index = self.find_respeaker_device()
        
        self.device_index = device_index
        
        # Initialize VAD (Voice Activity Detection)
        self.vad = webrtcvad.Vad(3)  # Aggressiveness level 3 (highest)
        
        # Variables to track sound detection
        self.sound_detected = False
        self.voice_detected = False
        self.max_volume = 0
    
    def find_respeaker_device(self):
        """Find the ReSpeaker device index."""
        print("Looking for ReSpeaker device...")
        
        respeaker_index = None
        
        for i in range(self.p.get_device_count()):
            dev_info = self.p.get_device_info_by_index(i)
            if "respeaker" in dev_info['name'].lower():
                respeaker_index = i
                print(f"Found ReSpeaker device at index {i}: {dev_info['name']}")
                break
        
        if respeaker_index is None:
            print("ReSpeaker device not found. Using default input device.")
            respeaker_index = self.p.get_default_input_device_info()['index']
        
        return respeaker_index
    
    def list_audio_devices(self):
        """List all available audio devices."""
        print("\nAvailable audio devices:")
        for i in range(self.p.get_device_count()):
            dev_info = self.p.get_device_info_by_index(i)
            print(f"  Device {i}: {dev_info['name']}")
            print(f"    Input channels: {dev_info['maxInputChannels']}")
            print(f"    Output channels: {dev_info['maxOutputChannels']}")
            print(f"    Default sample rate: {dev_info['defaultSampleRate']}")
    
    def is_speech(self, audio_data, sample_rate=16000):
        """Detect if audio contains speech using WebRTC VAD."""
        try:
            # WebRTC VAD requires 16-bit PCM audio
            # It also works best with 10, 20, or 30 ms frames
            frame_duration = 30  # ms
            frame_size = int(sample_rate * frame_duration / 1000)
            
            # Ensure we have enough data for a frame
            if len(audio_data) >= frame_size:
                # Extract a frame
                frame = audio_data[:frame_size]
                
                # Convert to bytes
                frame_bytes = struct.pack("h" * frame_size, *frame)
                
                # Check if it's speech
                return self.vad.is_speech(frame_bytes, sample_rate)
            
            return False
        except Exception as e:
            print(f"Error in speech detection: {e}")
            return False
    
    def test_microphone(self, duration=10, threshold=500):
        """Test the microphone by recording audio and detecting sound.
        
        Args:
            duration (int): Duration to record in seconds.
            threshold (int): Threshold for sound detection.
        
        Returns:
            bool: True if sound was detected, False otherwise.
        """
        print(f"Testing ReSpeaker microphone for {duration} seconds...")
        print(f"Sound detection threshold: {threshold}")
        
        # Get device info
        device_info = self.p.get_device_info_by_index(self.device_index)
        print(f"Selected device: {device_info['name']}")
        
        # Open stream
        stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=1024
        )
        
        print("\nRecording... Please speak into the microphone.")
        print("Volume levels:")
        
        # Record for the specified duration
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                # Read audio data
                data = stream.read(1024, exception_on_overflow=False)
                
                # Convert to numpy array
                audio_data = np.frombuffer(data, dtype=np.int16)
                
                # Calculate volume
                volume = np.abs(audio_data).mean()
                self.max_volume = max(self.max_volume, volume)
                
                # Check if sound is detected
                if volume > threshold:
                    self.sound_detected = True
                
                # Check if voice is detected
                if self.is_speech(audio_data):
                    self.voice_detected = True
                    print("\nVoice detected!")
                
                # Print volume level
                bar_length = int(volume / 100)
                bar = "#" * min(bar_length, 50)
                sys.stdout.write(f"\rVolume: {volume:.0f} {bar}")
                sys.stdout.flush()
        
        except KeyboardInterrupt:
            print("\nRecording stopped by user.")
        except Exception as e:
            print(f"\nError during recording: {e}")
        finally:
            # Stop and close the stream
            stream.stop_stream()
            stream.close()
            self.p.terminate()
        
        print("\n\nRecording complete!")
        
        # Print results
        print("\nTest Results:")
        print(f"  Maximum volume: {self.max_volume:.0f}")
        print(f"  Sound detected: {'Yes ✅' if self.sound_detected else 'No ❌'}")
        print(f"  Voice detected: {'Yes ✅' if self.voice_detected else 'No ❌'}")
        
        if not self.sound_detected:
            print("\nPossible issues:")
            print("  1. ReSpeaker microphone is not connected properly")
            print("  2. ReSpeaker microphone is muted")
            print("  3. Sound threshold is too high")
            print("  4. Wrong audio device selected")
        
        return self.sound_detected

if __name__ == "__main__":
    # Parse command line arguments
    device_index = None
    duration = 10
    threshold = 500
    
    if len(sys.argv) > 1:
        try:
            device_index = int(sys.argv[1])
        except ValueError:
            print(f"Invalid device index: {sys.argv[1]}. Will try to auto-detect.")
    
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
        tester = ReSpeakerTest(device_index)
        tester.list_audio_devices()
        tester.test_microphone(duration, threshold)
    except Exception as e:
        print(f"Error during ReSpeaker test: {e}")
