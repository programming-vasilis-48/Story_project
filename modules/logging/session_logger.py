#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Session Logger for the Story-Switch game.

This module provides comprehensive logging functionality for tracking
user interactions, robot speech, emotions, and session metadata.
"""

import os
import json
import time
from datetime import datetime
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict


@dataclass
class SpeechEvent:
    """Data class for speech events."""
    timestamp: float
    speaker: str  # "robot" or "user"
    text: str
    emotion_attempted: Optional[str] = None  # Only for robot speech
    duration: Optional[float] = None  # Speech duration in seconds
    exchange_number: Optional[int] = None
    voice_type: Optional[str] = None  # Track which TTS was used


@dataclass
class VideoEvent:
    """Data class for video recording events."""
    timestamp: float
    event_type: str  # "start" or "stop"
    filename: str
    exchange_number: Optional[int] = None
    emotion_attempted: Optional[str] = None  # Only for start events
    voice_type: Optional[str] = None  # Track which TTS was used


@dataclass
class SessionMetadata:
    """Data class for session metadata."""
    session_id: str
    person_name: str
    voice_type: str  # "Emotional OpenAI TTS" or "QT Robot Built-in Voice"
    start_time: float
    end_time: Optional[float] = None
    total_exchanges: int = 0
    emotions_used: Dict[str, int] = None


class SessionLogger:
    """Comprehensive session logger for the Story-Switch game."""

    def __init__(self, person_name: str, voice_type: str, base_log_dir: str = "logs"):
        """Initialize the session logger.
        
        Args:
            person_name (str): Name of the person participating
            voice_type (str): Type of voice being used
            base_log_dir (str): Base directory for logs
        """
        self.person_name = person_name.replace(" ", "_")  # Replace spaces with underscores
        self.voice_type = voice_type
        self.base_log_dir = base_log_dir
        
        # Create session ID with timestamp
        self.session_start_time = time.time()
        timestamp_str = datetime.fromtimestamp(self.session_start_time).strftime("%Y-%m-%d_%H-%M-%S")
        self.session_id = f"{timestamp_str}_{self.person_name}"
        
        # Create session directory
        self.session_dir = os.path.join(base_log_dir, "sessions", self.session_id)
        self.videos_dir = os.path.join(self.session_dir, "videos")
        os.makedirs(self.videos_dir, exist_ok=True)
        
        # Initialize data structures
        self.speech_events: List[SpeechEvent] = []
        self.video_events: List[VideoEvent] = []
        self.exchange_counter = 0
        
        # Initialize session metadata
        self.metadata = SessionMetadata(
            session_id=self.session_id,
            person_name=person_name,
            voice_type=voice_type,
            start_time=self.session_start_time,
            emotions_used={}
        )
        
        print(f"Session logger initialized for {person_name}")
        print(f"Session ID: {self.session_id}")
        print(f"Logs will be saved to: {self.session_dir}")

        # Save initial metadata and summary
        self._save_metadata()
        self._save_summary()

    def log_robot_speech(self, text: str, emotion_attempted: str, duration: Optional[float] = None, voice_type: Optional[str] = None) -> None:
        """Log robot speech event.
        
        Args:
            text (str): Text spoken by the robot
            emotion_attempted (str): Emotion being attempted to convey
            duration (float, optional): Duration of the speech
            voice_type (str, optional): Type of voice used for this speech event
        """
        # Get the current timestamp for the speech event
        current_time = time.time()
        formatted_time = datetime.fromtimestamp(current_time).strftime("%Y-%m-%d %H:%M:%S")
        
        event = SpeechEvent(
            timestamp=current_time,
            speaker="robot",
            text=text,
            emotion_attempted=emotion_attempted,
            duration=duration,
            exchange_number=self.exchange_counter,
            voice_type=voice_type or self.voice_type
        )
        
        self.speech_events.append(event)
        
        # Add exchange counter
        self.exchange_counter += 1
        
        # Track emotions used in metadata
        if emotion_attempted:
            if not self.metadata.emotions_used:
                self.metadata.emotions_used = {}
            
            self.metadata.emotions_used[emotion_attempted] = self.metadata.emotions_used.get(emotion_attempted, 0) + 1
        
        # Update metadata
        self.metadata.total_exchanges = self.exchange_counter
        
        print(f"[LOG] Robot speech: Exchange {self.exchange_counter-1}")
        print(f"[LOG] Timestamp: {formatted_time}, Duration: {duration:.2f}s")
        print(f"[LOG] Emotion: {emotion_attempted}")
        if voice_type:
            print(f"[LOG] Voice type: {voice_type}")
        print(f"[LOG] Content: \"{text}\"")

        # Save updates to disk after each interaction
        self._save_session_data()

    def log_user_speech(self, text: str, duration: Optional[float] = None) -> None:
        """Log user speech event.
        
        Args:
            text (str): Text spoken by the user
            duration (float, optional): Duration of the speech
        """
        # Get the current timestamp for the speech event
        current_time = time.time()
        formatted_time = datetime.fromtimestamp(current_time).strftime("%Y-%m-%d %H:%M:%S")
        
        event = SpeechEvent(
            timestamp=current_time,
            speaker="user",
            text=text,
            duration=duration,
            exchange_number=self.exchange_counter
        )
        
        self.speech_events.append(event)
        print(f"[LOG] User speech: Exchange {self.exchange_counter}")
        print(f"[LOG] Timestamp: {formatted_time}, Duration: {duration:.2f}s")
        print(f"[LOG] Content: \"{text}\"")

        # Save updates to disk after each interaction
        self._save_session_data()

    def log_video_start(self, emotion_attempted: str, filename: Optional[str] = None, voice_type: Optional[str] = None) -> str:
        """Log video recording start.
        
        Args:
            emotion_attempted (str): Emotion being attempted in this exchange
            filename (str, optional): Filename for the video, if None one will be generated
            voice_type (str, optional): Type of voice used for this exchange
            
        Returns:
            str: Generated or provided filename for the video
        """
        # Generate a filename if one wasn't provided
        if filename is None:
            voice_type_part = ""
            if voice_type:
                voice_type_part = f"_{voice_type.replace(' ', '_')}"
            filename = f"{self.person_name}{voice_type_part}_{emotion_attempted}_exchange_{self.exchange_counter}.mp4"
        
        # Get current timestamp
        current_time = time.time()
        formatted_time = datetime.fromtimestamp(current_time).strftime("%Y-%m-%d %H:%M:%S")
        
        event = VideoEvent(
            timestamp=current_time,
            event_type="start",
            filename=filename,
            exchange_number=self.exchange_counter,
            emotion_attempted=emotion_attempted,
            voice_type=voice_type or self.voice_type
        )
        
        self.video_events.append(event)
        print(f"[LOG] Video recording started: {filename}")
        print(f"[LOG] Timestamp: {formatted_time}, Emotion: {emotion_attempted}")
        if voice_type:
            print(f"[LOG] Voice type: {voice_type}")
        
        # Save updates to disk after each interaction
        self._save_session_data()
        
        return filename

    def log_video_stop(self, filename: str) -> None:
        """Log video recording stop.
        
        Args:
            filename (str): Filename of the video that was recorded
        """
        # Get current timestamp
        current_time = time.time()
        formatted_time = datetime.fromtimestamp(current_time).strftime("%Y-%m-%d %H:%M:%S")
        
        event = VideoEvent(
            timestamp=current_time,
            event_type="stop",
            filename=filename,
            exchange_number=self.exchange_counter
        )
        
        self.video_events.append(event)
        print(f"[LOG] Video recording stopped: {filename}")
        print(f"[LOG] Timestamp: {formatted_time}")
        
        # Save updates to disk after each interaction
        self._save_session_data()

    def get_video_path(self, filename: str) -> str:
        """Get full path for a video file.
        
        Args:
            filename (str): Video filename
            
        Returns:
            str: Full path to the video file
        """
        return os.path.join(self.videos_dir, filename)

    def _save_metadata(self) -> None:
        """Save metadata to a JSON file."""
        # Update end_time to current time
        self.metadata.end_time = time.time()
        
        # Save metadata
        metadata_file_path = os.path.join(self.session_dir, "metadata.json")
        with open(metadata_file_path, 'w', encoding='utf-8') as f:
            json.dump(asdict(self.metadata), f, indent=2, ensure_ascii=False)
            
        print(f"[LOG] Metadata saved to: {metadata_file_path}")
    
    def _save_session_log(self) -> None:
        """Save the complete session log to JSON file."""
        # Update end_time to current time
        self.metadata.end_time = time.time()
        
        # Prepare data for JSON serialization
        log_data = {
            "metadata": asdict(self.metadata),
            "speech_events": [asdict(event) for event in self.speech_events],
            "video_events": [asdict(event) for event in self.video_events]
        }
        
        # Save session log
        log_file_path = os.path.join(self.session_dir, "session_log.json")
        with open(log_file_path, 'w', encoding='utf-8') as f:
            json.dump(log_data, f, indent=2, ensure_ascii=False)
        
        print(f"[LOG] Session log saved to: {log_file_path}")
    
    def _save_summary(self) -> None:
        """Save a human-readable summary file."""
        summary_path = os.path.join(self.session_dir, "summary.txt")
        with open(summary_path, 'w', encoding='utf-8') as f:
            f.write(f"SESSION SUMMARY\n")
            f.write(f"==============\n\n")
            f.write(f"Participant: {self.metadata.person_name}\n")
            f.write(f"Voice Type: {self.voice_type}\n")
            f.write(f"Session Time: {datetime.fromtimestamp(self.metadata.start_time).strftime('%Y-%m-%d %H:%M:%S')}\n")
            
            # Calculate duration using current time for live updates
            current_duration = (time.time() - self.metadata.start_time) / 60
            f.write(f"Duration: {current_duration:.2f} minutes\n")
            f.write(f"Total Exchanges: {self.exchange_counter}\n\n")
            
            f.write(f"EMOTION USAGE\n")
            f.write(f"============\n\n")
            for emotion, count in self.metadata.emotions_used.items():
                f.write(f"{emotion}: {count}\n")
            
            f.write(f"\nSPEECH EVENTS\n")
            f.write(f"============\n\n")
            for event in self.speech_events:
                time_str = datetime.fromtimestamp(event.timestamp).strftime('%Y-%m-%d %H:%M:%S')
                f.write(f"[{time_str}] {event.speaker.upper()}")
                if event.emotion_attempted:
                    f.write(f" ({event.emotion_attempted})")
                f.write(f": {event.text}\n\n")
            
            f.write(f"\nVIDEO RECORDINGS\n")
            f.write(f"==============\n\n")
            # Group video events by filename to show start/stop pairs
            video_pairs = {}
            for event in self.video_events:
                if event.filename not in video_pairs:
                    video_pairs[event.filename] = {"start": None, "stop": None, "emotion": None}
                
                if event.event_type == "start":
                    video_pairs[event.filename]["start"] = event.timestamp
                    video_pairs[event.filename]["emotion"] = event.emotion_attempted
                else:
                    video_pairs[event.filename]["stop"] = event.timestamp
            
            # Write video recording details
            for filename, data in video_pairs.items():
                f.write(f"File: {filename}\n")
                f.write(f"Emotion: {data['emotion']}\n")
                if data["start"]:
                    start_time = datetime.fromtimestamp(data["start"]).strftime('%Y-%m-%d %H:%M:%S')
                    f.write(f"Start: {start_time}\n")
                if data["start"] and data["stop"]:
                    duration = data["stop"] - data["start"]
                    f.write(f"Duration: {duration:.2f} seconds\n")
                elif data["start"]:
                    f.write(f"Status: Recording in progress\n")
                f.write("\n")
        
        print(f"[LOG] Human-readable summary saved to: {summary_path}")
    
    def _save_session_data(self) -> None:
        """Save all session data after each interaction."""
        try:
            # Update and save metadata
            self._save_metadata()
            
            # Save complete session log
            self._save_session_log()
            
            # Save human-readable summary
            self._save_summary()
            
            print(f"[LOG] Session data saved incrementally (exchange {self.exchange_counter})")
            
        except Exception as e:
            print(f"[ERROR] Error saving session data: {str(e)}")
            import traceback
            traceback.print_exc()

    def save_session_log(self) -> None:
        """Save the complete session log to JSON file (for backward compatibility)."""
        self._save_session_data()

    def get_session_summary(self) -> Dict[str, Any]:
        """Get a summary of the current session.
        
        Returns:
            Dict[str, Any]: Session summary
        """
        duration = time.time() - self.session_start_time
        
        return {
            "session_id": self.session_id,
            "person_name": self.metadata.person_name,
            "voice_type": self.voice_type,
            "duration_minutes": round(duration / 60, 2),
            "total_exchanges": self.exchange_counter,
            "emotions_used": self.metadata.emotions_used,
            "total_speech_events": len(self.speech_events),
            "total_video_events": len(self.video_events)
        }

    def log_event(self, message: str) -> None:
        """Log a general event with a message.
        
        Args:
            message (str): Event message to log
        """
        print(f"[LOG] {message}")
        # This could also be saved to a separate events log file if needed
