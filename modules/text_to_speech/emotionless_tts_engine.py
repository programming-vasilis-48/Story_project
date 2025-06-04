#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Emotionless Text-to-Speech Engine module.

This module provides functionality for converting text to speech using OpenAI's TTS API
but with instructions to speak in a completely neutral, flat tone without emotional expression.
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

class EmotionlessTTSEngine:
    """Text-to-Speech Engine class using OpenAI TTS with emotionless delivery."""

    def __init__(self, api_key: str, model: str = "gpt-4o-mini-tts",
                 rate: float = 1.0, volume: float = 1.0, use_qtrobot_audio: bool = False):
        """Initialize the Emotionless TTS engine.

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
        self.volume = min(1.0, max(0.0, volume))  # Clamp between 0.0 and 1.0
        self.use_qtrobot_audio = use_qtrobot_audio
        
        # Fixed instruction for emotionless speech
        self.instructions = "Speak in a completely flat, neutral tone without any emotional expression. Maintain the same pitch and rhythm throughout. Do not emphasize any words or use vocal inflections that would convey emotions."

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
                        rospy.init_node('emotionless_tts_client', anonymous=True)
                except rospy.exceptions.ROSException:
                    # Node already initialized, which is fine
                    pass

                # Create publisher for QT Robot speech
                self.speech_pub = rospy.Publisher('/qt_robot/speech/play', String, queue_size=10)

                # Wait for publisher to connect
                time.sleep(0.5)

                self.ros_available = True
                print("Emotionless TTS configured to use QT Robot's internal speaker")

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
            print("Emotionless TTS configured to use standard audio output")
        except Exception as e:
            self.pygame_available = False
            print(f"Pygame initialization failed: {str(e)}")
            print("Will use alternative audio playback methods")

    def get_available_voices(self) -> Dict[str, Dict[str, str]]:
        """Get available voices (only Emotionless Nova).

        Returns:
            Dict[str, Dict[str, str]]: Dictionary of available voices
        """
        return {
            0: {
                'id': 'emotionless_nova',
                'name': 'Emotionless Nova - Natural voice without emotion'
            }
        }

    def speak(self, text: str) -> None:
        """Convert text to speech and play it with emotionless delivery.

        Args:
            text (str): Text to convert to speech
        """
        try:
            # Create a temporary file for the audio
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_file:
                temp_file_path = temp_file.name

            # Generate speech
            self._generate_speech(text, temp_file_path)

            # Process to ensure consistent volume
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as processed_file:
                processed_file_path = processed_file.name
            
            # Apply volume normalization
            self._normalize_volume(temp_file_path, processed_file_path)

            # Play the audio using the appropriate method
            if self.use_qtrobot_audio and self.ros_available:
                self._play_audio_qtrobot(processed_file_path)
            else:
                self._play_audio(processed_file_path)

            # Clean up
            os.unlink(temp_file_path)
            os.unlink(processed_file_path)

        except Exception as e:
            print(f"Error: Failed to speak text: {e}")

    def _generate_speech(self, text: str, output_file: str) -> None:
        """Generate speech using OpenAI TTS with emotionless instructions.

        Args:
            text (str): Text to convert to speech
            output_file (str): Path to save the audio file
        """
        try:
            # Generate speech using OpenAI API with emotionless instructions
            response = self.client.audio.speech.create(
                model=self.model,
                voice=self.voice,
                input=text,
                speed=self.rate,
                instructions=self.instructions
            )

            # Save the audio to file
            response.stream_to_file(output_file)

        except Exception as e:
            print(f"Error generating speech with OpenAI TTS: {str(e)}")
            raise

    def _normalize_volume(self, input_file: str, output_file: str) -> None:
        """Normalize the volume of the audio file for consistent loudness.

        Args:
            input_file (str): Path to input audio file
            output_file (str): Path to output audio file
        """
        try:
            # Apply volume normalization with ffmpeg
            # Reduce the volume to match OpenAI TTS engine (volume=1.0 instead of 3.0)
            cmd = [
                'ffmpeg', '-y', '-i', input_file,
                '-af', 'loudnorm=I=-16:LRA=1,volume=1.0',
                output_file
            ]
            
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
        except Exception as e:
            print(f"Error normalizing volume: {str(e)}")
            print("Using original audio without normalization...")
            
            # Just copy the original file if normalization fails
            try:
                subprocess.run(['cp', input_file, output_file], check=True)
            except Exception as copy_error:
                print(f"Error copying original file: {str(copy_error)}")

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
                # Always use maximum volume for playback
                pygame.mixer.music.set_volume(1.0)
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
            # Create temporary file for original speech
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_file:
                temp_file_path = temp_file.name
                
            # Generate speech using OpenAI
            self._generate_speech(text, temp_file_path)
            
            # Normalize volume and save to final file
            self._normalize_volume(temp_file_path, output_file)
            
            # Clean up
            os.unlink(temp_file_path)
            
            print(f"Emotionless speech saved to {output_file}")
            
        except Exception as e:
            print(f"Error: Failed to save speech to file: {e}")

    def adjust_for_emotion(self, emotion: str) -> None:
        """Override emotional adjustment to always remain emotionless.

        Args:
            emotion (str): Emotion to adjust for (ignored)
        """
        # Always use emotionless instructions regardless of requested emotion
        print(f"Note: Ignoring {emotion} emotion - maintaining emotionless delivery")
        # Keep the fixed emotionless instruction
        pass

    def set_voice(self, voice: str) -> None:
        """Set the voice to use (always uses Nova with emotionless instructions).

        Args:
            voice (str): Voice to use (ignored, always uses Nova)
        """
        # Always use Nova voice
        self.voice = "nova"
        print("Using Nova voice with emotionless delivery")

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