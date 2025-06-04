#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Semi-Robotic Text-to-Speech Engine module.

This module provides functionality for converting text to speech using OpenAI's TTS API
with added robotic effects to create a voice that's 50% natural and 50% robotic.
"""

import os
import tempfile
import time
import subprocess
import wave
import array
import math
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

class SemiRoboticTTSEngine:
    """Text-to-Speech Engine class combining OpenAI TTS with robotic effects."""

    def __init__(self, api_key: str, model: str = "gpt-4o-mini-tts",
                 rate: float = 1.0, volume: float = 1.0, use_qtrobot_audio: bool = False,
                 robotic_effect_level: float = 0.7):
        """Initialize the Semi-Robotic TTS engine.

        Args:
            api_key (str): OpenAI API key
            model (str): TTS model to use (default: gpt-4o-mini-tts)
            rate (float): Speech rate multiplier (1.0 is normal speed)
            volume (float): Volume level (0.0 to 1.0)
            use_qtrobot_audio (bool): Whether to use QT Robot's audio system (default: False)
            robotic_effect_level (float): Level of robotic effect (0.0 to 1.0, default: 0.7)
        """
        # Set properties
        self.api_key = api_key
        self.model = model
        self.voice = "nova"  # Always use Nova voice
        self.rate = rate
        self.volume = min(1.0, max(0.0, volume))  # Clamp between 0.0 and 1.0
        self.instructions = ""
        self.use_qtrobot_audio = use_qtrobot_audio
        self.robotic_effect_level = max(0.0, min(1.0, robotic_effect_level))  # Clamp between 0.0 and 1.0

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
                        rospy.init_node('semi_robotic_tts_client', anonymous=True)
                except rospy.exceptions.ROSException:
                    # Node already initialized, which is fine
                    pass

                # Create publisher for QT Robot speech
                self.speech_pub = rospy.Publisher('/qt_robot/speech/play', String, queue_size=10)

                # Wait for publisher to connect
                time.sleep(0.5)

                self.ros_available = True
                print("Semi-Robotic TTS configured to use QT Robot's internal speaker")

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
            print("Semi-Robotic TTS configured to use standard audio output")
        except Exception as e:
            self.pygame_available = False
            print(f"Pygame initialization failed: {str(e)}")
            print("Will use alternative audio playback methods")

    def _is_command_available(self, cmd):
        """Check if a command is available on the system."""
        try:
            subprocess.run(['which', cmd], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return True
        except subprocess.CalledProcessError:
            return False

    def get_available_voices(self) -> Dict[str, Dict[str, str]]:
        """Get available voices (only Semi-Robotic Nova).

        Returns:
            Dict[str, Dict[str, str]]: Dictionary of available voices
        """
        return {
            0: {
                'id': 'semi_robotic_nova',
                'name': 'Semi-Robotic Nova - 50% Natural, 50% Robotic'
            }
        }

    def speak(self, text: str) -> None:
        """Convert text to speech and play it with robotic effects.

        Args:
            text (str): Text to convert to speech
        """
        try:
            # Create temporary files for the audio
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_file:
                original_file_path = temp_file.name
                
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                processed_file_path = temp_file.name

            # Generate speech
            self._generate_speech(text, original_file_path)
            
            # Apply robotic effects
            self._apply_robotic_effects(original_file_path, processed_file_path)

            # Play the audio using the appropriate method
            if self.use_qtrobot_audio and self.ros_available:
                self._play_audio_qtrobot(processed_file_path)
            else:
                self._play_audio(processed_file_path)

            # Clean up
            os.unlink(original_file_path)
            os.unlink(processed_file_path)

        except Exception as e:
            print(f"Error: Failed to speak text: {e}")

    def _generate_speech(self, text: str, output_file: str) -> None:
        """Generate speech using OpenAI TTS.

        Args:
            text (str): Text to convert to speech
            output_file (str): Path to save the audio file
        """
        try:
            # Set robotic instructions based on effect level
            if self.robotic_effect_level > 0.2:
                robotic_instruction = "Speak in a slightly monotone, precise manner with minimal emotional inflection."
                if self.instructions:
                    self.instructions += " " + robotic_instruction
                else:
                    self.instructions = robotic_instruction

            # Generate speech using OpenAI API
            if self.instructions:
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

    def _apply_robotic_effects(self, input_file: str, output_file: str) -> None:
        """Apply robotic effects to the audio file.

        Args:
            input_file (str): Path to input audio file
            output_file (str): Path to output audio file
        """
        # Use ffmpeg for audio processing with a simple but effective robotic effect
        try:
            # Calculate effect parameters based on robotic_effect_level
            echo_delay = 0.03  # Short delay for robotic effect
            echo_decay = 0.7 * self.robotic_effect_level  # Stronger echo for more robotic effect
            
            # Simple robotic effect using just echo with multiple short delays
            filter_str = f"aecho=0.8:{echo_delay}:{echo_delay*2}:{echo_decay}"
            
            # Add flanger for metallic effect if robotic level is high enough
            if self.robotic_effect_level > 0.4:
                filter_str += ",flanger=delay=1:depth=0.5:speed=2"
            
            # First apply normalization to make the input consistent
            filter_str = f"loudnorm=I=-16:LRA=1,{filter_str}"
            
            # Then apply a simple static volume boost at the end
            filter_str += ",volume=3.0"
            
            # Apply the effects with ffmpeg
            cmd = [
                'ffmpeg', '-y', '-i', input_file,
                '-af', filter_str,
                output_file
            ]
            
            print(f"Applying robotic effects with consistent volume (level: {self.robotic_effect_level:.2f})")
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
        except Exception as e:
            print(f"Error applying robotic effects: {str(e)}")
            print("Falling back to original audio without effects...")
            
            # Just copy the original file if effects fail
            try:
                # Apply only simple volume boost when falling back
                subprocess.run(['ffmpeg', '-y', '-i', input_file, '-af', 'loudnorm=I=-16:LRA=1,volume=3.0', output_file], 
                             check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                print("Applied volume boost to original audio")
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
        """Convert text to speech with robotic effects and save it to a file.

        Args:
            text (str): Text to convert to speech
            output_file (str): Path to save the audio file
        """
        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)

        try:
            # Create temporary file for original speech
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_file:
                original_file_path = temp_file.name
                
            # Generate speech using OpenAI
            self._generate_speech(text, original_file_path)
            
            # Apply robotic effects and save to final file
            self._apply_robotic_effects(original_file_path, output_file)
            
            # Clean up
            os.unlink(original_file_path)
            
            print(f"Semi-robotic speech saved to {output_file}")
            
        except Exception as e:
            print(f"Error: Failed to save speech to file: {e}")

    def adjust_for_emotion(self, emotion: str) -> None:
        """Adjust voice properties based on emotion.

        Args:
            emotion (str): Emotion to adjust for (happy, sad, angry, fear, surprise, disgust, neutral)
        """
        # Emotion instructions mapping
        emotion_instructions = {
            'happy': 'Speak with a hint of cheerfulness and slight enthusiasm.',
            'sad': 'Speak with a hint of solemnity and slight melancholy.',
            'angry': 'Speak with a hint of intensity and slight sternness.',
            'fear': 'Speak with a hint of caution and slight hesitation.',
            'surprise': 'Speak with a hint of wonder and slight astonishment.',
            'disgust': 'Speak with a hint of disapproval.',
            'neutral': 'Speak in a balanced, even tone with minimal emotional coloring.'
        }

        # Default settings
        self.rate = 1.0

        # Always use Nova voice
        self.voice = "nova"

        # Set the instructions based on the emotion
        emotion_lower = emotion.lower()
        if emotion_lower in emotion_instructions:
            # Mix with robotic instructions
            self.instructions = emotion_instructions[emotion_lower]
            if self.robotic_effect_level > 0.3:
                self.instructions += " Maintain a somewhat precise, semi-mechanical delivery."
        else:
            # Default to neutral if emotion not found
            self.instructions = emotion_instructions['neutral']
            if self.robotic_effect_level > 0.3:
                self.instructions += " Maintain a somewhat precise, semi-mechanical delivery."

        # Adjust rate based on emotion but moderated for robotic effect
        rate_adjustments = {
            'happy': 1.1,   # Slightly faster
            'sad': 0.9,     # Slightly slower
            'angry': 1.15,  # Faster
            'fear': 1.2,    # Faster (anxious)
            'surprise': 1.1,# Slightly faster
            'disgust': 0.95,# Slightly slower
            'neutral': 1.0  # Normal speed
        }
        
        if emotion_lower in rate_adjustments:
            # Calculate rate with reduced emotion effect based on robotic level
            emotion_rate = rate_adjustments[emotion_lower]
            neutral_rate = 1.0
            self.rate = neutral_rate + (emotion_rate - neutral_rate) * (1 - self.robotic_effect_level)
        else:
            self.rate = 1.0

        print(f"Adjusted for {emotion} emotion with robotic level {self.robotic_effect_level:.2f}")
        print(f"Instructions: '{self.instructions}', rate: {self.rate:.2f}")

    def set_voice(self, voice: str) -> None:
        """Set the voice to use (always uses Nova with robotic effects).

        Args:
            voice (str): Voice to use (ignored, always uses Nova with robotic effects)
        """
        # Always use Nova voice
        self.voice = "nova"
        print("Using Nova voice with robotic effects")

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
        
    def set_robotic_effect_level(self, level: float) -> None:
        """Set the robotic effect level.
        
        Args:
            level (float): Robotic effect level (0.0 to 1.0)
        """
        # Ensure level is within valid range
        level = max(0.0, min(1.0, level))
        self.robotic_effect_level = level
        print(f"Robotic effect level set to {level:.2f}") 