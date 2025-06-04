#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Speech Recognition module.

This module provides functionality for capturing audio from a microphone
and converting it to text using speech recognition.
"""

import os
import time
import sys
import threading
from typing import Optional, Dict, Any
import subprocess

try:
    import speech_recognition as sr
except ImportError:
    print("Error: speech_recognition library not found")
    print("Install it with: pip install SpeechRecognition")
    sr = None

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found")
    print("Install it with: pip install pynput")
    keyboard = None

class SpeechRecognizer:
    """Simple Speech Recognizer class using the SpeechRecognition library."""

    def __init__(self, device_index: Optional[int] = None, language: str = "en-US"):
        """Initialize the speech recognizer.

        Args:
            device_index (int, optional): Index of the microphone device to use
            language (str): Language code for speech recognition
        """
        self.device_index = device_index
        self.language = language
        
        # Check if required libraries are available
        if sr is None:
            print("Error: speech_recognition library not available")
            return
            
        if keyboard is None:
            print("Error: pynput library not available")
            return
            
        # Initialize recognizer
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 300
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8  # More responsive

        # Print available microphones
        self._print_available_microphones()

        # Initialize the microphone
        try:
            if device_index is not None:
                self.microphone = sr.Microphone(device_index=device_index)
                print(f"Initialized microphone with device index {device_index}")
            else:
                # Try to find a Sennheiser microphone
                mics = sr.Microphone.list_microphone_names()
                for i, name in enumerate(mics):
                    if "sennheiser" in name.lower():
                        try:
                            self.microphone = sr.Microphone(device_index=i)
                            print(f"Initialized Sennheiser microphone (index {i}): {name}")
                            break
                        except Exception as e:
                            print(f"Error initializing Sennheiser: {e}")
                else:
                    # If no Sennheiser found, use default
                    self.microphone = sr.Microphone()
                    print("Initialized default microphone")
        except Exception as e:
            print(f"Error initializing microphone: {e}")
            self.microphone = None

    def _print_available_microphones(self):
        """Print available microphones."""
        if sr is None:
            return
            
        try:
            mics = sr.Microphone.list_microphone_names()
            print(f"Available microphones ({len(mics)}):")
            for i, name in enumerate(mics):
                print(f"  {i}: {name}")
        except Exception as e:
            print(f"Error listing microphones: {e}")

    def _suppress_alsa_errors(self, func):
        """Suppress ALSA error messages when running a function."""
        # Save stderr
        old_stderr = sys.stderr
        # Redirect stderr to /dev/null
        with open(os.devnull, 'w') as devnull:
            sys.stderr = devnull
            try:
                result = func()
            finally:
                # Restore stderr
                sys.stderr = old_stderr
        return result

    def adjust_for_ambient_noise(self, duration: float = 1.0):
        """Adjust recognizer for ambient noise."""
        if self.microphone is None:
            print("Error: Microphone not available")
            return
            
        print(f"Adjusting for ambient noise (duration: {duration}s)...")
        
        def _adjust():
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=duration)
            print(f"Ambient noise adjustment complete. Energy threshold: {self.recognizer.energy_threshold}")
            
        self._suppress_alsa_errors(_adjust)

    def listen_once(self, timeout: Optional[float] = None, phrase_time_limit: Optional[float] = None,
                   push_to_talk: bool = False, push_to_talk_key: str = 'v'):
        """Listen for speech once and return the recognized text.

        Args:
            timeout: How long to wait for phrase to start
            phrase_time_limit: Max time for a phrase
            push_to_talk: Whether to use push-to-talk
            push_to_talk_key: Key to use for push-to-talk

        Returns:
            Recognized text or empty string
        """
        if self.microphone is None:
            print("Error: Microphone not available")
            return ""

        if push_to_talk:
            return self._listen_push_to_talk(push_to_talk_key)
        else:
            # Normal listening
            def _listen():
                print("Listening for speech...")
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=timeout,
                                                phrase_time_limit=phrase_time_limit)
                
                print("Got audio, recognizing...")
                try:
                    text = self.recognizer.recognize_google(audio, language=self.language)
                    return text
                except sr.UnknownValueError:
                    print("Speech not recognized")
                    return ""
                except sr.RequestError:
                    print("Recognition service error")
                    return ""
                except Exception as e:
                    print(f"Recognition error: {e}")
                    return ""
            
            try:
                return self._suppress_alsa_errors(_listen)
            except Exception as e:
                print(f"Error listening: {e}")
                return ""

    def _listen_push_to_talk(self, key: str = 'v'):
        """Listen using push-to-talk mode."""
        if keyboard is None or self.microphone is None:
            print("Error: Required libraries not available")
            return ""
            
        print(f"Push-to-talk mode: Press and hold '{key.upper()}' key while speaking, release when done.")
        print("Press ESC to cancel.")
        
        # This is a simpler implementation using system tools
        # Use arecord to capture audio to a temporary file while key is pressed
        
        temp_file = "/tmp/speech_audio.wav"
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
                        cmd = ["arecord", "-f", "cd", "-t", "wav", temp_file]
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
            if os.path.exists(temp_file) and os.path.getsize(temp_file) > 1000:  # Check if file exists and has content
                try:
                    with sr.AudioFile(temp_file) as source:
                        audio = self.recognizer.record(source)
                        
                    # Recognize speech
                    try:
                        text = self.recognizer.recognize_google(audio, language=self.language)
                        print(f"Recognized: {text}")
                        return text
                    except sr.UnknownValueError:
                        print("Speech not recognized")
                        return ""
                    except sr.RequestError:
                        print("Recognition service error")
                        return ""
                except Exception as e:
                    print(f"Error processing audio: {e}")
                    return ""
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
                if os.path.exists(temp_file):
                    os.remove(temp_file)
            except:
                pass
