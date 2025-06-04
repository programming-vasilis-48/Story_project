#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
QT Robot Text-to-Speech Engine module.

This module provides functionality for converting text to speech using QT Robot's
built-in speech synthesis system via ROS topics.
"""

import os
import time
import tempfile
import subprocess
from typing import Optional, Dict, Any

# Import pygame for external speaker audio playback
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

# Import ROS modules conditionally
try:
    import rospy
    from std_msgs.msg import String
    from qt_robot_interface.srv import setting_setVolume
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    rospy = None
    String = None
    setting_setVolume = None


class QTRobotTTSEngine:
    """Text-to-Speech Engine class using QT Robot's built-in speech synthesis."""

    def __init__(self, rate: float = 1.0, volume: float = 1.0, use_external_speaker: bool = False):
        """Initialize the QT Robot TTS engine.

        Args:
            rate (float): Speech rate (not used for QT Robot, kept for interface compatibility)
            volume (float): Speech volume (0.0 to 1.0)
            use_external_speaker (bool): Whether to use external speaker instead of QT Robot's internal speaker
        """
        self.rate = rate
        self.volume = volume
        self.voice = "qtrobot_builtin"  # Identifier for the built-in voice
        self.instructions = None  # QT Robot doesn't support emotional instructions
        self.use_external_speaker = use_external_speaker

        # Initialize pygame for external speaker playback if needed
        self.pygame_available = False
        if self.use_external_speaker and PYGAME_AVAILABLE:
            try:
                pygame.mixer.init()
                self.pygame_available = True
                print("Pygame initialized for external speaker audio playback")
            except Exception as e:
                print(f"Pygame initialization failed: {str(e)}")
                print("Will use alternative audio playback methods for external speaker")

        # Initialize ROS if available
        self.ros_available = False
        self.speech_pub = None

        if not ROS_AVAILABLE:
            print("Warning: ROS not available. QT Robot TTS will run in simulation mode.")
            print("Make sure ROS is installed and sourced for actual QT Robot functionality.")
            return

        try:
            # Try to initialize ROS node if not already initialized
            try:
                if not rospy.get_node_uri():
                    rospy.init_node('qtrobot_tts_client', anonymous=True)
            except rospy.exceptions.ROSException:
                # Node already initialized, which is fine
                pass

            # Create publisher for QT Robot speech
            self.speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)

            # Wait for publisher to connect
            time.sleep(0.5)

            # Set volume to 68% for the experiment (if using internal speaker)
            if not self.use_external_speaker:
                self._set_qtrobot_volume(68)
            else:
                print("Using external speaker for QT Robot TTS output")
                # Check if we have espeak or pico2wave installed
                self._check_external_tts_tools()

            self.ros_available = True
            
            if self.use_external_speaker:
                print("QT Robot TTS engine initialized with external speaker output")
            else:
                print("QT Robot TTS engine initialized with internal speaker (default)")

        except Exception as e:
            print(f"Warning: Failed to initialize ROS for QT Robot TTS: {str(e)}")
            print("QT Robot TTS will run in simulation mode.")
            self.speech_pub = None

    def _check_external_tts_tools(self):
        """Check if necessary external TTS tools are available."""
        self.has_espeak = self._is_command_available("espeak")
        self.has_pico2wave = self._is_command_available("pico2wave")
        self.has_festival = self._is_command_available("festival")
        
        if self.has_espeak:
            print("Using 'espeak' for external speaker TTS")
        elif self.has_pico2wave:
            print("Using 'pico2wave' for external speaker TTS")
        elif self.has_festival:
            print("Using 'festival' for external speaker TTS")
        else:
            print("Warning: No external TTS tools found (espeak, pico2wave, or festival)")
            print("External speaker TTS may not work properly")
            
    def _is_command_available(self, cmd):
        """Check if a command is available on the system."""
        try:
            subprocess.run(['which', cmd], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return True
        except subprocess.CalledProcessError:
            return False

    def get_available_voices(self) -> Dict[str, Dict[str, str]]:
        """Get available voices (only QT Robot built-in).

        Returns:
            Dict[str, Dict[str, str]]: Dictionary of available voices
        """
        return {
            0: {
                'id': 'qtrobot_builtin',
                'name': 'QT Robot Built-in - Robotic voice'
            }
        }

    def speak(self, text: str) -> None:
        """Convert text to speech using QT Robot's built-in TTS.

        Args:
            text (str): Text to convert to speech
        """
        if not self.ros_available and not self.use_external_speaker:
            print(f"QT Robot TTS (simulated): {text}")
            # Simulate speech duration
            words = len(text.split())
            estimated_duration = max(1.0, words / 2.5)  # Minimum 1 second
            time.sleep(estimated_duration)
            return

        try:
            if self.use_external_speaker:
                # Use external TTS tools that output to default audio device
                self._speak_external(text)
            else:
                # Use QT Robot's internal speaker via direct ROS topic
                if String is not None and self.speech_pub is not None:
                    msg = String()
                    msg.data = text
                    self.speech_pub.publish(msg)
                    print(f"Published to QT Robot internal speaker: '{text}'")
                else:
                    print(f"QT Robot TTS (ROS unavailable): {text}")

                # Wait a bit to allow the speech to be processed
                # Estimate speech duration (rough approximation: 150 words per minute)
                words = len(text.split())
                estimated_duration = max(1.0, words / 2.5)  # Minimum 1 second
                time.sleep(estimated_duration)

        except Exception as e:
            print(f"Error: Failed to speak text with QT Robot: {e}")
            print(f"QT Robot TTS (fallback): {text}")

    def _speak_external(self, text: str) -> None:
        """Use external TTS tools to output speech to the external speaker.
        
        This method uses available TTS tools like festival, pico2wave, or espeak
        to generate speech that plays through the system's default audio device.
        
        Args:
            text (str): Text to convert to speech
        """
        try:
            # Create a temporary file for the audio if needed
            temp_file_path = None
            
            # Try different TTS tools in order of preference
            if self.has_festival:
                # Use festival with the SLT female voice (similar to QT Robot voice)
                # Create a temporary script for festival
                with tempfile.NamedTemporaryFile(mode='w+', suffix='.scm', delete=False) as script_file:
                    script_file_path = script_file.name
                    # Set voice to nicer female voice and add the text
                    script_file.write(f'(voice_cmu_us_slt_arctic_hts)\n(SayText "{text}")')
                
                # Run festival with the script
                cmd = ['festival', '-b', script_file_path]
                print(f"Using festival with SLT voice for external audio: {' '.join(cmd)}")
                process = subprocess.Popen(cmd)
                process.wait()
                
                # Clean up the script file
                if os.path.exists(script_file_path):
                    os.unlink(script_file_path)
                success = True
            
            elif self.has_pico2wave:
                # pico2wave needs to write to a file first, then we play it
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                    temp_file_path = temp_file.name
                    
                cmd = ['pico2wave', '-w', temp_file_path, '-l', 'en-US', text]
                print(f"Generating speech with pico2wave: {' '.join(cmd)}")
                subprocess.run(cmd, check=True)
                
                # Play the generated file
                play_cmd = ['aplay', temp_file_path]
                print(f"Playing with aplay: {' '.join(play_cmd)}")
                subprocess.run(play_cmd, check=True)
                success = True
                
            elif self.has_espeak:
                # Use espeak as last resort for external TTS tools
                cmd = ['espeak', '-v', 'en-us+f3', '-s', '130', '-p', '60', '-a', '200', text]
                print(f"Using espeak for external audio: {' '.join(cmd)}")
                process = subprocess.Popen(cmd)
                process.wait()
                success = True
                
            elif self.pygame_available:
                # As a last resort, try to synthesize speech with QT Robot and play with pygame
                print("Using fallback pygame audio playback")
                temp_msg = "Unable to use external TTS tools. Defaulting to internal speaker."
                if String is not None and self.speech_pub is not None:
                    msg = String()
                    msg.data = text
                    self.speech_pub.publish(msg)
                    
                    # Wait for speech to finish
                    words = len(text.split())
                    estimated_duration = max(1.0, words / 2.5)
                    time.sleep(estimated_duration)
                success = True
                
            else:
                # None of the TTS tools are available, fall back to QT Robot TTS
                print("No external TTS tools available. Falling back to internal speaker.")
                if String is not None and self.speech_pub is not None:
                    msg = String()
                    msg.data = text
                    self.speech_pub.publish(msg)
                    
                    # Wait for speech to finish
                    words = len(text.split())
                    estimated_duration = max(1.0, words / 2.5)
                    time.sleep(estimated_duration)
                success = True

            # Clean up any temporary files
            if temp_file_path and os.path.exists(temp_file_path):
                os.unlink(temp_file_path)

        except Exception as e:
            print(f"Error in external speaker playback: {str(e)}")
            # Fall back to standard QT Robot TTS method
            if String is not None and self.speech_pub is not None:
                msg = String()
                msg.data = text
                self.speech_pub.publish(msg)
                
                # Wait for speech to finish
                words = len(text.split())
                estimated_duration = max(1.0, words / 2.5)
                time.sleep(estimated_duration)

    def save_to_file(self, text: str, output_file: str) -> None:
        """Save text to speech file.

        Args:
            text (str): Text to convert to speech
            output_file (str): Path to save the audio file
        """
        try:
            if self.has_pico2wave:
                # pico2wave can generate nice quality audio files
                cmd = ['pico2wave', '-w', output_file, '-l', 'en-US', text]
                subprocess.run(cmd, check=True)
                print(f"Speech saved to {output_file} using pico2wave")
                return
            elif self.has_espeak:
                # espeak can also generate audio files
                cmd = ['espeak', '-v', 'en-us', '-w', output_file, text]
                subprocess.run(cmd, check=True)
                print(f"Speech saved to {output_file} using espeak")
                return
                
            print("Warning: Could not save speech to file")
            print(f"Text that would be spoken: {text}")
        except Exception as e:
            print(f"Error saving speech to file: {e}")
            print(f"Text that would be spoken: {text}")

    def adjust_for_emotion(self, emotion: str) -> None:
        """Adjust voice properties based on emotion.

        Note: QT Robot's built-in TTS does not support emotional adjustments.
        This method is kept for interface compatibility.

        Args:
            emotion (str): Emotion to adjust for (ignored for QT Robot)
        """
        # QT Robot's built-in TTS doesn't support emotional adjustments
        # This method is kept for interface compatibility
        print(f"Note: QT Robot built-in voice does not support emotional adjustment for '{emotion}'")
        self.instructions = None  # Always None for QT Robot

    def set_voice(self, voice_id: str) -> None:
        """Set the voice to use for speech synthesis.

        Note: QT Robot only has one built-in voice.

        Args:
            voice_id (str): Voice identifier (ignored for QT Robot)
        """
        # QT Robot only has one built-in voice
        self.voice = "qtrobot_builtin"
        print("Note: QT Robot only supports its built-in robotic voice")

    def set_rate(self, rate: float) -> None:
        """Set the speech rate.

        Note: QT Robot's built-in TTS rate is not adjustable.

        Args:
            rate (float): Speech rate (ignored for QT Robot)
        """
        self.rate = rate
        print(f"Note: QT Robot built-in TTS rate is not adjustable (requested: {rate})")

    def set_volume(self, volume: float) -> None:
        """Set the speech volume.

        Note: QT Robot's built-in TTS volume is controlled by system settings.

        Args:
            volume (float): Speech volume (0.0 to 1.0)
        """
        self.volume = volume
        # Actually set the QT Robot volume if ROS is available and using internal speaker
        if self.ros_available and not self.use_external_speaker:
            # Convert 0.0-1.0 range to 0-100 percentage
            volume_percentage = int(volume * 100)
            self._set_qtrobot_volume(volume_percentage)
        else:
            print(f"Volume set to {volume:.1f} for external speaker playback")

    def _set_qtrobot_volume(self, volume_percentage: int) -> None:
        """Set QT Robot speaker volume.

        Args:
            volume_percentage (int): Volume level in percentage (0-100)
        """
        if not self.ros_available or setting_setVolume is None:
            print(f"QT Robot volume control not available (requested: {volume_percentage}%)")
            return

        try:
            # Wait for the service to be available
            rospy.wait_for_service('/qt_robot/setting/setVolume', timeout=5.0)

            # Create service proxy
            set_volume_service = rospy.ServiceProxy('/qt_robot/setting/setVolume', setting_setVolume)

            # Call the service
            response = set_volume_service(volume=volume_percentage)
            print(f"QT Robot volume set to {volume_percentage}%")

        except rospy.ServiceException as e:
            print(f"Failed to set QT Robot volume: {str(e)}")
        except rospy.ROSException as e:
            print(f"QT Robot volume service not available: {str(e)}")
        except Exception as e:
            print(f"Error setting QT Robot volume: {str(e)}")
            
    def set_external_speaker(self, use_external_speaker: bool) -> None:
        """Set whether to use the external speaker.
        
        Args:
            use_external_speaker (bool): Whether to use external speaker
        """
        if use_external_speaker == self.use_external_speaker:
            return
            
        self.use_external_speaker = use_external_speaker
        
        if use_external_speaker:
            # Check for external TTS tools
            self._check_external_tts_tools()
            
            # Initialize pygame if needed
            if PYGAME_AVAILABLE and not self.pygame_available:
                try:
                    pygame.mixer.init()
                    self.pygame_available = True
                    print("Pygame initialized for external speaker audio playback")
                except Exception as e:
                    print(f"Pygame initialization failed: {str(e)}")
            print("QT Robot TTS switched to use external speaker")
        else:
            print("QT Robot TTS switched to use internal speaker")
