"""
Microphone to Text module.

This module provides functionality for capturing audio from a microphone
and converting it to text using speech recognition.

Main components:
- Microphone input handling
- Speech recognition
- Text output processing
"""

from modules.mic_to_text.speech_recognizer import SpeechRecognizer
from modules.mic_to_text.whisper_recognizer import WhisperRecognizer

__all__ = ["SpeechRecognizer", "WhisperRecognizer"]
