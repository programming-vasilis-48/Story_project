"""
Text-to-Speech module.

This module provides functionality for converting text to speech using OpenAI's TTS API.

Main components:
- Text-to-speech conversion
- Voice selection and customization
- Audio output handling
- Emotion-based voice adjustment
"""

from .openai_tts_engine import OpenAITTSEngine
from .qtrobot_tts_engine import QTRobotTTSEngine
from .emotionless_tts_engine import EmotionlessTTSEngine

__all__ = ['OpenAITTSEngine', 'QTRobotTTSEngine', 'EmotionlessTTSEngine']
