"""
Data loading utilities for video emotion analysis
"""

import os
import re
import json
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import pandas as pd

from .config import SESSIONS_PATH, EMOTIONS, VOICE_TYPES


class VideoDataLoader:
    """Load and organize video data from experiment sessions"""
    
    def __init__(self):
        self.sessions_path = SESSIONS_PATH
        self.video_pattern = re.compile(
            r'(?P<participant>\w+)_(?P<voice_type>Emotional_OpenAI_TTS|Emotionless_Voice)_(?P<emotion>\w+)_exchange_(?P<exchange>\d+)\.mp4'
        )
    
    def get_all_sessions(self) -> List[str]:
        """Get list of all session directories"""
        if not self.sessions_path.exists():
            raise FileNotFoundError(f"Sessions path not found: {self.sessions_path}")
        
        sessions = [d.name for d in self.sessions_path.iterdir() if d.is_dir()]
        return sorted(sessions)
    
    def get_session_videos(self, session_id: str) -> List[Dict]:
        """Get all videos for a specific session with metadata"""
        session_path = self.sessions_path / session_id / "videos"
        
        if not session_path.exists():
            raise FileNotFoundError(f"Videos path not found: {session_path}")
        
        videos = []
        for video_file in session_path.glob("*.mp4"):
            match = self.video_pattern.match(video_file.name)
            if match:
                video_info = {
                    'session_id': session_id,
                    'file_path': str(video_file),
                    'filename': video_file.name,
                    'participant': match.group('participant'),
                    'voice_type': match.group('voice_type'),
                    'target_emotion': match.group('emotion'),
                    'exchange_number': int(match.group('exchange')),
                    'file_size': video_file.stat().st_size
                }
                videos.append(video_info)
        
        return sorted(videos, key=lambda x: (x['voice_type'], x['exchange_number']))
    
    def get_all_videos(self) -> List[Dict]:
        """Get all videos from all sessions"""
        all_videos = []
        sessions = self.get_all_sessions()
        
        for session_id in sessions:
            try:
                session_videos = self.get_session_videos(session_id)
                all_videos.extend(session_videos)
            except FileNotFoundError as e:
                print(f"Warning: {e}")
                continue
        
        return all_videos
    
    def get_session_metadata(self, session_id: str) -> Optional[Dict]:
        """Load session metadata if available"""
        metadata_path = self.sessions_path / session_id / "metadata.json"
        
        if metadata_path.exists():
            with open(metadata_path, 'r') as f:
                return json.load(f)
        return None
    
    def create_analysis_dataframe(self) -> pd.DataFrame:
        """Create a pandas DataFrame with all video information for analysis"""
        videos = self.get_all_videos()
        
        if not videos:
            raise ValueError("No videos found in sessions")
        
        df = pd.DataFrame(videos)
        
        # Add derived columns for analysis
        df['voice_condition'] = df['voice_type'].map({
            'Emotional_OpenAI_TTS': 'Emotional',
            'Emotionless_Voice': 'Emotionless'
        })
        
        # Ensure target emotions are in our expected list
        df = df[df['target_emotion'].isin(EMOTIONS)]
        
        return df
    
    def get_participant_summary(self) -> pd.DataFrame:
        """Get summary statistics per participant"""
        df = self.create_analysis_dataframe()
        
        summary = df.groupby(['participant', 'voice_condition']).agg({
            'target_emotion': 'count',
            'file_size': 'mean'
        }).reset_index()
        
        summary.columns = ['participant', 'voice_condition', 'video_count', 'avg_file_size']
        
        return summary
    
    def validate_data_completeness(self) -> Dict:
        """Validate that we have complete data for analysis"""
        df = self.create_analysis_dataframe()
        
        expected_videos_per_participant = len(EMOTIONS) * len(VOICE_TYPES)
        
        participant_counts = df.groupby('participant').size()
        complete_participants = participant_counts[participant_counts == expected_videos_per_participant]
        incomplete_participants = participant_counts[participant_counts != expected_videos_per_participant]
        
        validation_report = {
            'total_participants': len(participant_counts),
            'complete_participants': len(complete_participants),
            'incomplete_participants': len(incomplete_participants),
            'total_videos': len(df),
            'expected_videos': len(participant_counts) * expected_videos_per_participant,
            'missing_videos': len(participant_counts) * expected_videos_per_participant - len(df),
            'participants_with_complete_data': complete_participants.index.tolist(),
            'participants_with_incomplete_data': incomplete_participants.to_dict()
        }
        
        return validation_report


def load_video_data() -> Tuple[pd.DataFrame, Dict]:
    """Convenience function to load all video data and validation report"""
    loader = VideoDataLoader()
    df = loader.create_analysis_dataframe()
    validation = loader.validate_data_completeness()
    
    return df, validation


if __name__ == "__main__":
    # Test the data loader
    loader = VideoDataLoader()
    
    print("Available sessions:")
    sessions = loader.get_all_sessions()
    for session in sessions:
        print(f"  - {session}")
    
    print(f"\nTotal sessions: {len(sessions)}")
    
    # Load all data
    df, validation = load_video_data()
    print(f"\nLoaded {len(df)} videos")
    print(f"Participants with complete data: {validation['complete_participants']}")
    
    if validation['incomplete_participants']:
        print(f"Participants with incomplete data: {validation['participants_with_incomplete_data']}")
