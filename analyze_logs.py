#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Log Analysis Tool for Story-Switch Game

This script analyzes session logs and provides insights about
emotion elicitation experiments.
"""

import os
import json
import sys
from datetime import datetime
from typing import Dict, List, Any
import glob


def load_session_data(session_dir: str) -> Dict[str, Any]:
    """Load session data from a session directory.
    
    Args:
        session_dir (str): Path to session directory
        
    Returns:
        Dict[str, Any]: Session data or None if failed
    """
    try:
        session_log_path = os.path.join(session_dir, "session_log.json")
        
        if not os.path.exists(session_log_path):
            print(f"Warning: No session log found in {session_dir}")
            return None
        
        with open(session_log_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    
    except Exception as e:
        print(f"Error loading session data from {session_dir}: {str(e)}")
        return None


def analyze_session(session_data: Dict[str, Any]) -> Dict[str, Any]:
    """Analyze a single session.
    
    Args:
        session_data (Dict[str, Any]): Session data
        
    Returns:
        Dict[str, Any]: Analysis results
    """
    metadata = session_data.get("metadata", {})
    speech_events = session_data.get("speech_events", [])
    video_events = session_data.get("video_events", [])
    
    # Basic statistics
    total_duration = metadata.get("end_time", 0) - metadata.get("start_time", 0)
    total_exchanges = metadata.get("total_exchanges", 0)
    emotions_used = metadata.get("emotions_used", {})
    
    # Speech analysis
    robot_speeches = [e for e in speech_events if e["speaker"] == "robot"]
    user_speeches = [e for e in speech_events if e["speaker"] == "user"]
    
    # Calculate average speech durations
    robot_durations = [e.get("duration", 0) for e in robot_speeches if e.get("duration")]
    user_durations = [e.get("duration", 0) for e in user_speeches if e.get("duration")]
    
    avg_robot_duration = sum(robot_durations) / len(robot_durations) if robot_durations else 0
    avg_user_duration = sum(user_durations) / len(user_durations) if user_durations else 0
    
    # Video analysis
    video_recordings = len([e for e in video_events if e["event_type"] == "start"])
    
    # Exchange timing analysis
    exchange_timings = []
    for i in range(1, total_exchanges + 1):
        exchange_events = [e for e in speech_events if e.get("exchange_number") == i]
        if len(exchange_events) >= 2:
            robot_event = next((e for e in exchange_events if e["speaker"] == "robot"), None)
            user_event = next((e for e in exchange_events if e["speaker"] == "user"), None)
            
            if robot_event and user_event:
                response_time = user_event["timestamp"] - robot_event["timestamp"]
                exchange_timings.append({
                    "exchange": i,
                    "emotion": robot_event.get("emotion_attempted"),
                    "response_time": response_time,
                    "robot_duration": robot_event.get("duration", 0),
                    "user_duration": user_event.get("duration", 0)
                })
    
    return {
        "session_id": metadata.get("session_id"),
        "participant": metadata.get("person_name"),
        "voice_type": metadata.get("voice_type"),
        "total_duration_minutes": round(total_duration / 60, 2),
        "total_exchanges": total_exchanges,
        "emotions_used": emotions_used,
        "speech_stats": {
            "total_robot_speeches": len(robot_speeches),
            "total_user_speeches": len(user_speeches),
            "avg_robot_duration": round(avg_robot_duration, 2),
            "avg_user_duration": round(avg_user_duration, 2)
        },
        "video_stats": {
            "total_recordings": video_recordings
        },
        "exchange_timings": exchange_timings
    }


def find_all_sessions(logs_dir: str = "logs") -> List[str]:
    """Find all session directories.
    
    Args:
        logs_dir (str): Base logs directory
        
    Returns:
        List[str]: List of session directory paths
    """
    sessions_dir = os.path.join(logs_dir, "sessions")
    
    if not os.path.exists(sessions_dir):
        return []
    
    session_dirs = []
    for item in os.listdir(sessions_dir):
        item_path = os.path.join(sessions_dir, item)
        if os.path.isdir(item_path):
            session_dirs.append(item_path)
    
    return sorted(session_dirs)


def print_session_analysis(analysis: Dict[str, Any]) -> None:
    """Print analysis results for a session.
    
    Args:
        analysis (Dict[str, Any]): Analysis results
    """
    print(f"\n{'='*60}")
    print(f"SESSION: {analysis['session_id']}")
    print(f"{'='*60}")
    print(f"Participant: {analysis['participant']}")
    print(f"Voice Type: {analysis['voice_type']}")
    print(f"Duration: {analysis['total_duration_minutes']} minutes")
    print(f"Total Exchanges: {analysis['total_exchanges']}")
    print(f"Video Recordings: {analysis['video_stats']['total_recordings']}")
    
    print(f"\nEmotions Used:")
    for emotion, count in analysis['emotions_used'].items():
        print(f"  {emotion}: {count}")
    
    print(f"\nSpeech Statistics:")
    stats = analysis['speech_stats']
    print(f"  Robot speeches: {stats['total_robot_speeches']}")
    print(f"  User speeches: {stats['total_user_speeches']}")
    print(f"  Avg robot duration: {stats['avg_robot_duration']}s")
    print(f"  Avg user duration: {stats['avg_user_duration']}s")
    
    if analysis['exchange_timings']:
        print(f"\nExchange Timings:")
        print(f"  {'Ex#':<4} {'Emotion':<10} {'Response Time':<15} {'Robot Dur':<12} {'User Dur':<10}")
        print(f"  {'-'*4} {'-'*10} {'-'*15} {'-'*12} {'-'*10}")
        
        for timing in analysis['exchange_timings']:
            print(f"  {timing['exchange']:<4} "
                  f"{timing['emotion']:<10} "
                  f"{timing['response_time']:.2f}s{'':<10} "
                  f"{timing['robot_duration']:.2f}s{'':<7} "
                  f"{timing['user_duration']:.2f}s")


def analyze_all_sessions(logs_dir: str = "logs") -> None:
    """Analyze all sessions and print results.
    
    Args:
        logs_dir (str): Base logs directory
    """
    session_dirs = find_all_sessions(logs_dir)
    
    if not session_dirs:
        print(f"No sessions found in {logs_dir}/sessions/")
        return
    
    print(f"Found {len(session_dirs)} session(s)")
    
    all_analyses = []
    
    for session_dir in session_dirs:
        session_data = load_session_data(session_dir)
        if session_data:
            analysis = analyze_session(session_data)
            all_analyses.append(analysis)
            print_session_analysis(analysis)
    
    # Overall summary
    if len(all_analyses) > 1:
        print(f"\n{'='*60}")
        print(f"OVERALL SUMMARY ({len(all_analyses)} sessions)")
        print(f"{'='*60}")
        
        total_participants = len(set(a['participant'] for a in all_analyses))
        total_exchanges = sum(a['total_exchanges'] for a in all_analyses)
        total_duration = sum(a['total_duration_minutes'] for a in all_analyses)
        
        print(f"Total participants: {total_participants}")
        print(f"Total exchanges: {total_exchanges}")
        print(f"Total duration: {total_duration:.2f} minutes")
        
        # Emotion usage across all sessions
        all_emotions = {}
        for analysis in all_analyses:
            for emotion, count in analysis['emotions_used'].items():
                all_emotions[emotion] = all_emotions.get(emotion, 0) + count
        
        print(f"\nEmotion usage across all sessions:")
        for emotion, count in sorted(all_emotions.items()):
            print(f"  {emotion}: {count}")


def list_video_files(logs_dir: str = "logs") -> None:
    """List all video files with their details.
    
    Args:
        logs_dir (str): Base logs directory
    """
    print(f"\n{'='*60}")
    print("VIDEO FILES")
    print(f"{'='*60}")
    
    video_pattern = os.path.join(logs_dir, "sessions", "*", "videos", "*.mp4")
    video_files = glob.glob(video_pattern)
    
    if not video_files:
        print("No video files found.")
        return
    
    print(f"Found {len(video_files)} video file(s):")
    
    for video_file in sorted(video_files):
        filename = os.path.basename(video_file)
        file_size = os.path.getsize(video_file)
        file_size_mb = file_size / (1024 * 1024)
        
        # Parse filename for details
        parts = filename.replace('.mp4', '').split('_')
        if len(parts) >= 4:
            participant = '_'.join(parts[:-3])
            emotion = parts[-3]
            exchange = parts[-1]
            print(f"  {filename}")
            print(f"    Participant: {participant}")
            print(f"    Emotion: {emotion}")
            print(f"    Exchange: {exchange}")
            print(f"    Size: {file_size_mb:.2f} MB")
            print()
        else:
            print(f"  {filename} (Size: {file_size_mb:.2f} MB)")


def main():
    """Main function."""
    print("STORY-SWITCH GAME LOG ANALYZER")
    print("="*60)
    
    # Check if logs directory exists
    if not os.path.exists("logs"):
        print("No logs directory found. Run the game first to generate logs.")
        return
    
    # Analyze all sessions
    analyze_all_sessions()
    
    # List video files
    list_video_files()


if __name__ == "__main__":
    main()
