"""
Core emotion detection using py-feat library
"""

import cv2
import numpy as np
import pandas as pd
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import warnings
warnings.filterwarnings('ignore')

try:
    from feat import Detector
except ImportError:
    print("Warning: py-feat not installed. Install with: pip install py-feat")
    Detector = None

from ..utils.config import FEAT_CONFIG, VIDEO_CONFIG, EMOTIONS


class EmotionDetector:
    """Facial emotion detection using py-feat"""
    
    def __init__(self):
        if Detector is None:
            raise ImportError("py-feat is required. Install with: pip install py-feat")
        
        # Initialize py-feat detector
        self.detector = Detector(
            face_model=FEAT_CONFIG['face_model'],
            landmark_model=FEAT_CONFIG['landmark_model'],
            au_model=FEAT_CONFIG['au_model'],
            emotion_model=FEAT_CONFIG['emotion_model'],
            facepose_model=FEAT_CONFIG['facepose_model']
        )
        
        self.emotion_columns = [
            'anger', 'disgust', 'fear', 'happiness', 'sadness', 'surprise', 'neutral'
        ]
        
        # Mapping from our experiment emotions to py-feat emotions
        self.emotion_mapping = {
            'angry': 'anger',
            'disgust': 'disgust',
            'fear': 'fear',
            'happy': 'happiness',
            'sad': 'sadness',
            'surprise': 'surprise',
            'neutral': 'neutral'
        }
    
    def detect_emotions_from_video(self, video_path: str) -> Optional[pd.DataFrame]:
        """
        Detect emotions from a video file
        
        Args:
            video_path: Path to video file
            
        Returns:
            DataFrame with emotion predictions per frame
        """
        try:
            # Use py-feat to detect emotions from video
            results = self.detector.detect_video(video_path)
            
            if results is None or len(results) == 0:
                print(f"Warning: No faces detected in {video_path}")
                return None
            
            return results
            
        except Exception as e:
            print(f"Error processing video {video_path}: {str(e)}")
            return None
    
    def extract_emotion_features(self, results: pd.DataFrame, target_emotion: str) -> Dict:
        """
        Extract key emotion features from py-feat results
        
        Args:
            results: py-feat detection results
            target_emotion: The emotion that was supposed to be elicited
            
        Returns:
            Dictionary with emotion analysis features
        """
        if results is None or len(results) == 0:
            return self._get_empty_features()
        
        # Get emotion columns (they should be in the results)
        emotion_cols = [col for col in self.emotion_columns if col in results.columns]
        
        if not emotion_cols:
            print("Warning: No emotion columns found in results")
            return self._get_empty_features()
        
        # Calculate emotion statistics
        emotion_stats = {}
        
        for emotion in emotion_cols:
            emotion_values = results[emotion].dropna()
            if len(emotion_values) > 0:
                emotion_stats[f'{emotion}_mean'] = emotion_values.mean()
                emotion_stats[f'{emotion}_max'] = emotion_values.max()
                emotion_stats[f'{emotion}_std'] = emotion_values.std()
                emotion_stats[f'{emotion}_median'] = emotion_values.median()
        
        # Target emotion analysis
        target_feat_emotion = self.emotion_mapping.get(target_emotion, target_emotion)
        
        if target_feat_emotion in results.columns:
            target_values = results[target_feat_emotion].dropna()
            
            if len(target_values) > 0:
                emotion_stats.update({
                    'target_emotion_intensity': target_values.mean(),
                    'target_emotion_peak': target_values.max(),
                    'target_emotion_std': target_values.std(),
                    'frames_with_target_emotion': len(target_values[target_values > 0.5]),
                    'peak_frame_index': target_values.idxmax() if len(target_values) > 0 else -1
                })
        
        # Classification accuracy
        predicted_emotion = self._get_dominant_emotion(results)
        emotion_stats['predicted_emotion'] = predicted_emotion
        emotion_stats['classification_correct'] = (predicted_emotion == target_feat_emotion)
        
        # Overall statistics
        emotion_stats.update({
            'total_frames': len(results),
            'frames_with_face': len(results.dropna()),
            'face_detection_rate': len(results.dropna()) / len(results) if len(results) > 0 else 0,
            'target_emotion': target_emotion,
            'video_duration_frames': len(results)
        })
        
        return emotion_stats
    
    def _get_dominant_emotion(self, results: pd.DataFrame) -> str:
        """Get the dominant emotion across all frames"""
        emotion_cols = [col for col in self.emotion_columns if col in results.columns]
        
        if not emotion_cols:
            return 'unknown'
        
        # Calculate mean emotion intensities
        emotion_means = results[emotion_cols].mean()
        
        # Return emotion with highest mean intensity
        return emotion_means.idxmax()
    
    def _get_empty_features(self) -> Dict:
        """Return empty feature dictionary when no faces detected"""
        features = {}
        
        # Initialize all emotion statistics to 0
        for emotion in self.emotion_columns:
            features.update({
                f'{emotion}_mean': 0.0,
                f'{emotion}_max': 0.0,
                f'{emotion}_std': 0.0,
                f'{emotion}_median': 0.0
            })
        
        features.update({
            'target_emotion_intensity': 0.0,
            'target_emotion_peak': 0.0,
            'target_emotion_std': 0.0,
            'frames_with_target_emotion': 0,
            'peak_frame_index': -1,
            'predicted_emotion': 'none',
            'classification_correct': False,
            'total_frames': 0,
            'frames_with_face': 0,
            'face_detection_rate': 0.0,
            'target_emotion': '',
            'video_duration_frames': 0
        })
        
        return features
    
    def analyze_single_video(self, video_path: str, target_emotion: str, 
                           participant: str, voice_type: str) -> Dict:
        """
        Complete analysis of a single video
        
        Args:
            video_path: Path to video file
            target_emotion: Expected emotion
            participant: Participant ID
            voice_type: Voice condition
            
        Returns:
            Complete analysis results
        """
        print(f"Analyzing: {Path(video_path).name}")
        
        # Detect emotions
        results = self.detect_emotions_from_video(video_path)
        
        # Extract features
        features = self.extract_emotion_features(results, target_emotion)
        
        # Add metadata
        features.update({
            'video_path': video_path,
            'participant': participant,
            'voice_type': voice_type,
            'filename': Path(video_path).name
        })
        
        return features, results


def test_emotion_detector():
    """Test the emotion detector with a sample video"""
    from ..utils.data_loader import VideoDataLoader
    
    # Load sample video
    loader = VideoDataLoader()
    videos = loader.get_all_videos()
    
    if not videos:
        print("No videos found for testing")
        return
    
    # Test with first video
    sample_video = videos[0]
    
    detector = EmotionDetector()
    features, raw_results = detector.analyze_single_video(
        sample_video['file_path'],
        sample_video['target_emotion'],
        sample_video['participant'],
        sample_video['voice_type']
    )
    
    print("Sample analysis results:")
    for key, value in features.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")


if __name__ == "__main__":
    test_emotion_detector()
