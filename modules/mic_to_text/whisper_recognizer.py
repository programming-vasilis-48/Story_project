#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Whisper-based Speech Recognition module.

This module provides functionality for capturing audio from a microphone
and converting it to text using OpenAI's Whisper API.
"""

import os
import time
import sys
import threading
import subprocess
import tempfile
from typing import Optional, Dict, Any

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found")
    print("Install it with: pip install pynput")
    keyboard = None

try:
    import openai
except ImportError:
    print("Error: openai library not found")
    print("Install it with: pip install openai")
    openai = None

class WhisperRecognizer:
    """Speech Recognizer class using OpenAI's Whisper API."""

    def __init__(self, api_key: str, language: str = "en"):
        """Initialize the Whisper speech recognizer.

        Args:
            api_key (str): OpenAI API key
            language (str): Language code for speech recognition
        """
        self.language = language
        
        # Check if required libraries are available
        if openai is None:
            print("Error: openai library not available")
            return
            
        if keyboard is None:
            print("Error: pynput library not available")
            return
        
        # Set up OpenAI client
        self.client = openai.OpenAI(api_key=api_key)
        
        # Check if microphone is available
        self._check_microphone()

    def _check_microphone(self):
        """Check if a microphone is available."""
        try:
            # Run arecord -l to list capture devices
            result = subprocess.run(["arecord", "-l"], 
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE,
                                  text=True)
            
            if "card" in result.stdout:
                self.microphone = True
                print("Microphone detected:")
                print(result.stdout)
            else:
                self.microphone = False
                print("No microphone detected")
        except Exception as e:
            print(f"Error checking microphone: {e}")
            self.microphone = False

    def listen_once(self, timeout: Optional[float] = None, 
                   push_to_talk: bool = False, push_to_talk_key: str = 'v'):
        """Listen for speech once and return the recognized text.

        Args:
            timeout: Maximum recording time (if not push-to-talk)
            push_to_talk: Whether to use push-to-talk
            push_to_talk_key: Key to use for push-to-talk

        Returns:
            Recognized text or empty string
        """
        if not self.microphone:
            print("Error: Microphone not available")
            return ""

        if push_to_talk:
            return self._listen_push_to_talk(push_to_talk_key)
        else:
            # Normal listening with timeout
            timeout = timeout or 10  # Default to 10 seconds if not specified
            return self._listen_with_timeout(timeout)

    def _listen_with_timeout(self, timeout: float):
        """Listen for a fixed duration."""
        print(f"Recording for {timeout} seconds...")
        
        # Create a temporary file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            temp_path = temp_file.name
        
        try:
            # Record audio with arecord
            cmd = ["arecord", "-f", "cd", "-t", "wav", "-d", str(int(timeout)), temp_path]
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            print("Processing speech...")
            return self._process_audio_file(temp_path)
        finally:
            # Clean up the temporary file
            try:
                if os.path.exists(temp_path):
                    os.remove(temp_path)
            except:
                pass

    def _listen_push_to_talk(self, key: str = 'v'):
        """Listen using push-to-talk mode."""
        if keyboard is None:
            print("Error: pynput library not available")
            return ""
            
        print(f"Push-to-talk mode: Press and hold '{key.upper()}' key while speaking, release when done.")
        print("Press ESC to cancel.")
        
        # Create a temporary file for the recording
        temp_path = tempfile.mktemp(suffix='.wav')
        key_pressed = threading.Event()
        recording_process = None
        cancelled = False
        
        # Define key handlers
        def on_press(k):
            nonlocal recording_process, cancelled
            
            try:
                # Check if the pressed key matches our target key
                if hasattr(k, 'char') and k.char.lower() == key.lower():
                    if not key_pressed.is_set():
                        key_pressed.set()
                        # Start recording with arecord - no duration limit
                        cmd = ["arecord", "-f", "cd", "-t", "wav", temp_path]
                        recording_process = subprocess.Popen(cmd, 
                                                          stdout=subprocess.DEVNULL,
                                                          stderr=subprocess.DEVNULL)
                        print("Recording...")
            except AttributeError:
                # For special keys like ESC
                if k == keyboard.Key.esc:
                    cancelled = True
                    if recording_process:
                        recording_process.terminate()
                    key_pressed.clear()
                    return False  # Stop listener
            return True
            
        def on_release(k):
            nonlocal recording_process
            
            try:
                # Check if the released key matches our target key
                if hasattr(k, 'char') and k.char.lower() == key.lower():
                    if key_pressed.is_set() and recording_process:
                        key_pressed.clear()
                        # Stop the recording process
                        recording_process.terminate()
                        recording_process.wait()
                        recording_process = None
                        print(f"Key '{key.upper()}' released. Processing speech...")
            except AttributeError:
                pass
            return True
            
        # Start keyboard listener
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        
        try:
            print(f"Waiting for '{key.upper()}' key press...")
            
            # Wait for key press
            while not key_pressed.is_set() and not cancelled:
                time.sleep(0.1)
                
            if cancelled:
                print("Cancelled by user")
                return ""
                
            # Wait for key release
            while key_pressed.is_set() and not cancelled:
                time.sleep(0.1)
                
            if cancelled:
                print("Cancelled by user")
                return ""
                
            # Process the recorded audio
            if os.path.exists(temp_path) and os.path.getsize(temp_path) > 1000:  # Check if file exists and has content
                return self._process_audio_file(temp_path)
            else:
                print("No audio recorded or file too small")
                return ""
                
        finally:
            # Clean up
            listener.stop()
            if recording_process and recording_process.poll() is None:
                recording_process.terminate()
                
            # Remove temporary file
            try:
                if os.path.exists(temp_path):
                    os.remove(temp_path)
            except:
                pass

    def _process_audio_file(self, audio_path: str):
        """Process an audio file using Whisper API."""
        if not os.path.exists(audio_path) or os.path.getsize(audio_path) < 1000:
            print("Audio file missing or too small")
            return ""
            
        try:
            print("Sending audio to Whisper API...")
            
            with open(audio_path, "rb") as audio_file:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file,
                    language=self.language
                )
            
            if transcript and hasattr(transcript, 'text'):
                print(f"Recognized: {transcript.text}")
                return transcript.text
            else:
                print("No speech recognized")
                return ""
                
        except Exception as e:
            print(f"Error processing audio with Whisper API: {e}")
            return "" 