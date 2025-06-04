#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenAI Text-to-Speech Engine module.

This module provides functionality for converting text to speech using OpenAI's TTS API,
which produces high-quality, natural-sounding voices.
"""

import os
import tempfile
import time
import subprocess
from typing import Optional, Dict, Any, List
import pygame
from openai import OpenAI

# Import ROS modules conditionally for QT Robot audio routing
try:
    import rospy
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    rospy = None
    String = None

class OpenAITTSEngine:
    """Text-to-Speech Engine class using OpenAI Text-to-Speech."""

    def __init__(self, api_key: str, model: str = "gpt-4o-mini-tts",
                 rate: float = 1.0, volume: float = 1.0, use_qtrobot_audio: bool = False):
        """Initialize the OpenAI TTS engine.

        Args:
            api_key (str): OpenAI API key
            model (str): TTS model to use (default: gpt-4o-mini-tts)
            rate (float): Speech rate multiplier (1.0 is normal speed)
            volume (float): Volume level (0.0 to 1.0)
            use_qtrobot_audio (bool): Whether to use QT Robot's audio system (default: False)
        """
        # Set properties
        self.api_key = api_key
        self.model = model
        self.voice = "nova"  # Always use Nova voice
        self.rate = rate
        self.volume = volume
        self.instructions = ""  # Default empty instructions
        self.use_qtrobot_audio = use_qtrobot_audio

        # Emotion instructions for the TTS API
        self.emotion_instructions = {
            'happy': 'Speak in a cheerful, upbeat, and enthusiastic tone. Express joy and excitement.',
            'sad': 'Speak in a somber, melancholic tone with a slight tremor. Express sadness and grief.',
            'angry': 'Speak in a stern, intense tone with emphasis on certain words. Express frustration and anger.',
            'fear': 'Speak in a trembling, hesitant voice with occasional pauses. Express anxiety and fear.',
            'surprise': 'Speak with a tone of astonishment and wonder. Express shock and amazement.',
            'disgust': 'Speak with a tone of revulsion and disapproval. Express distaste and disgust.',
            'neutral': 'Speak in a balanced, even tone without strong emotional coloring.'
        }

        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.api_key)

        # Initialize QT Robot audio if requested
        self.ros_available = False
        self.speech_pub = None
        
        if self.use_qtrobot_audio and ROS_AVAILABLE:
            try:
                # Try to initialize ROS node if not already initialized
                try:
                    if not rospy.get_node_uri():
                        rospy.init_node('openai_tts_client', anonymous=True)
                except rospy.exceptions.ROSException:
                    # Node already initialized, which is fine
                    pass

                # Create publisher for QT Robot speech
                self.speech_pub = rospy.Publisher('/qt_robot/speech/play', String, queue_size=10)

                # Wait for publisher to connect
                time.sleep(0.5)

                self.ros_available = True
                print("OpenAI TTS configured to use QT Robot's internal speaker (not recommended)")
                print("Warning: QT Robot audio routing often doesn't work for OpenAI TTS")

            except Exception as e:
                print(f"Warning: Failed to initialize ROS for QT Robot audio: {str(e)}")
                print("Falling back to standard audio output...")
                self.ros_available = False
                self.speech_pub = None
                self.use_qtrobot_audio = False

        # Initialize pygame for standard audio playback
        try:
            pygame.mixer.init()
            self.pygame_available = True
            print("OpenAI TTS configured to use standard audio output")
        except Exception as e:
            self.pygame_available = False
            print(f"Pygame initialization failed: {str(e)}")
            print("Will use alternative audio playback methods")

    def get_available_voices(self) -> Dict[str, Dict[str, str]]:
        """Get available voices (only Nova).

        Returns:
            Dict[str, Dict[str, str]]: Dictionary of available voices
        """
        # Only return Nova voice
        return {
            0: {
                'id': 'nova',
                'name': 'Nova - Bright and energetic'
            }
        }

    def speak(self, text: str) -> None:
        """Convert text to speech and play it.

        Args:
            text (str): Text to convert to speech
        """
        try:
            # Create a temporary file for the audio
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_file:
                temp_file_path = temp_file.name

            # Generate speech
            self._generate_speech(text, temp_file_path)

            # Play the audio using the appropriate method
            if self.use_qtrobot_audio and self.ros_available:
                # Warning: This method often doesn't work
                self._play_audio_qtrobot(temp_file_path)
            else:
                self._play_audio(temp_file_path)

            # Clean up
            os.unlink(temp_file_path)

        except Exception as e:
            print(f"Error: Failed to speak text: {e}")

    def _generate_speech(self, text: str, output_file: str) -> None:
        """Generate speech using OpenAI TTS.

        Args:
            text (str): Text to convert to speech
            output_file (str): Path to save the audio file
        """
        try:
            # Generate speech using OpenAI API
            if self.instructions:
                print(f"Using instructions for emotional tone: '{self.instructions}'")
                response = self.client.audio.speech.create(
                    model=self.model,
                    voice=self.voice,
                    input=text,
                    speed=self.rate,
                    instructions=self.instructions
                )
            else:
                response = self.client.audio.speech.create(
                    model=self.model,
                    voice=self.voice,
                    input=text,
                    speed=self.rate
                )

            # Save the audio to file
            response.stream_to_file(output_file)

        except Exception as e:
            print(f"Error generating speech with OpenAI TTS: {str(e)}")
            raise

    def _play_audio_qtrobot(self, audio_file: str) -> None:
        """Play audio file through QT Robot's internal speaker.

        Args:
            audio_file (str): Path to audio file
        """
        if not self.ros_available or not self.speech_pub:
            print("Warning: QT Robot audio routing not available, using standard output")
            self._play_audio(audio_file)
            return

        try:
            # Use QT Robot's audio system via the /qt_robot/speech/play topic
            if String is not None:
                msg = String()
                msg.data = audio_file
                self.speech_pub.publish(msg)
                print(f"Published audio file path to QT Robot: {audio_file}")

                # Wait for the audio to finish playing (estimation based on file size)
                file_size = os.path.getsize(audio_file)
                # Rough estimate: 1MB = 60 seconds of speech
                estimated_duration = max(1.0, file_size / (1024 * 1024) * 60)
                time.sleep(estimated_duration)

            else:
                print("ROS message type not available, using standard output")
                self._play_audio(audio_file)

        except Exception as e:
            print(f"Error playing audio through QT Robot: {str(e)}")
            # Fall back to standard audio if QT Robot fails
            self._play_audio(audio_file)

    def _play_audio(self, audio_file: str) -> None:
        """Play audio file using standard output methods.

        Args:
            audio_file (str): Path to audio file
        """
        if self.pygame_available:
            try:
                pygame.mixer.music.load(audio_file)
                pygame.mixer.music.set_volume(self.volume)
                pygame.mixer.music.play()

                # Wait for playback to finish
                while pygame.mixer.music.get_busy():
                    time.sleep(0.1)

            except Exception as e:
                print(f"Error playing audio with pygame: {str(e)}")
                self._play_audio_fallback(audio_file)
        else:
            self._play_audio_fallback(audio_file)

    def _play_audio_fallback(self, audio_file: str) -> None:
        """Fallback method to play audio file using system commands.

        Args:
            audio_file (str): Path to audio file
        """
        try:
            # Try using aplay (Linux)
            subprocess.run(['aplay', '-q', audio_file], check=True)
        except Exception:
            try:
                # Try using mpg123 (Linux)
                subprocess.run(['mpg123', '-q', audio_file], check=True)
            except Exception:
                try:
                    # Try using mplayer (Linux)
                    subprocess.run(['mplayer', '-really-quiet', audio_file], check=True)
                except Exception as e:
                    print(f"Error playing audio with fallback methods: {str(e)}")

    def save_to_file(self, text: str, output_file: str) -> None:
        """Convert text to speech and save it to a file.

        Args:
            text (str): Text to convert to speech
            output_file (str): Path to save the audio file
        """
        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)

        try:
            # Generate speech using the async method
            self._generate_speech(text, output_file)
            print(f"Speech saved to {output_file}")
        except Exception as e:
            print(f"Error: Failed to save speech to file: {e}")

    def adjust_for_emotion(self, emotion: str) -> None:
        """Adjust voice properties based on emotion.

        Args:
            emotion (str): Emotion to adjust for (happy, sad, angry, fear, surprise, disgust, neutral)
        """
        # Default settings
        self.rate = 1.0

        # Always use Nova voice
        self.voice = "nova"

        # Set the instructions based on the emotion
        emotion_lower = emotion.lower()
        if emotion_lower in self.emotion_instructions:
            self.instructions = self.emotion_instructions[emotion_lower]
        else:
            # Default to neutral if emotion not found
            self.instructions = self.emotion_instructions['neutral']

        # Adjust rate based on emotion for additional effect
        if emotion_lower == "happy":
            self.rate = 1.1  # Slightly faster
        elif emotion_lower == "sad":
            self.rate = 0.9  # Slightly slower
        elif emotion_lower == "angry":
            self.rate = 1.15  # Faster
        elif emotion_lower == "fear":
            self.rate = 1.2  # Faster (anxious)
        elif emotion_lower == "surprise":
            self.rate = 1.1  # Slightly faster
        elif emotion_lower == "disgust":
            self.rate = 0.95  # Slightly slower
        elif emotion_lower == "neutral":
            self.rate = 1.0  # Normal speed

        print(f"Adjusted for {emotion} emotion: instructions='{self.instructions}', rate={self.rate}")

    def set_voice(self, voice: str) -> None:
        """Set the voice to use (always uses Nova).

        Args:
            voice (str): Voice to use (ignored, always uses Nova)
        """
        # Always use Nova voice
        self.voice = "nova"
        print("Using Nova voice (other voices are not supported)")

    def set_rate(self, rate: float) -> None:
        """Set the speech rate.

        Args:
            rate (float): Speech rate multiplier (0.5 to 2.0)
        """
        # Ensure rate is within valid range
        rate = max(0.5, min(2.0, rate))
        self.rate = rate
        print(f"Speech rate set to {rate}")

    def set_volume(self, volume: float) -> None:
        """Set the speech volume.

        Args:
            volume (float): Speech volume (0.0 to 1.0)
        """
        # Ensure volume is within valid range
        volume = max(0.0, min(1.0, volume))
        self.volume = volume
        print(f"Speech volume set to {volume}")

    def set_qtrobot_audio(self, use_qtrobot_audio: bool) -> None:
        """Set whether to use QT Robot's audio system.

        Args:
            use_qtrobot_audio (bool): Whether to use QT Robot's audio system
        """
        if use_qtrobot_audio == self.use_qtrobot_audio:
            return

        self.use_qtrobot_audio = use_qtrobot_audio

        if use_qtrobot_audio:
            if ROS_AVAILABLE:
                try:
                    # Try to initialize ROS node if not already initialized
                    try:
                        if not rospy.get_node_uri():
                            rospy.init_node('openai_tts_client', anonymous=True)
                    except rospy.exceptions.ROSException:
                        # Node already initialized, which is fine
                        pass

                    # Create publisher for QT Robot speech
                    self.speech_pub = rospy.Publisher('/qt_robot/speech/play', String, queue_size=10)
                    time.sleep(0.5)
                    self.ros_available = True
                    print("OpenAI TTS switched to use QT Robot's internal speaker")
                    print("Warning: This option often doesn't work for OpenAI TTS")
                except Exception as e:
                    print(f"Warning: Failed to initialize ROS for QT Robot audio: {str(e)}")
                    self.ros_available = False
                    self.use_qtrobot_audio = False
                    print("Falling back to standard audio output")
            else:
                print("Warning: ROS is not available for QT Robot audio routing")
                self.use_qtrobot_audio = False
        else:
            # Switch to standard audio
            if not self.pygame_available:
                try:
                    pygame.mixer.init()
                    self.pygame_available = True
                except Exception as e:
                    print(f"Pygame initialization failed: {str(e)}")
                    
            print("OpenAI TTS switched to use standard audio output")
