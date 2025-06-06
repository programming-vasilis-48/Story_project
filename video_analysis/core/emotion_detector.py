"""
Core emotion detection using py-feat library
"""
import torch
import cv2
import numpy as np
import pandas as pd
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import warnings
from feat import Detector # <-- Cleaned up import

# It's good practice to keep this here if you want to ignore warnings from libraries
warnings.filterwarnings('ignore')

from ..utils.config import FEAT_CONFIG, VIDEO_CONFIG, EMOTIONS


class EmotionDetector:
    """Facial emotion detection using py-feat"""
    
    def __init__(self):
        """Initializes the detector and sets the device to CUDA if available."""
        
        # Check for CUDA availability and set the device
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"INFO: EmotionDetector will use device: {device}")
        
        # Initialize py-feat detector and pass it the device
        self.detector = Detector(
            face_model=FEAT_CONFIG['face_model'],
            landmark_model=FEAT_CONFIG['landmark_model'],
            au_model=FEAT_CONFIG['au_model'],
            emotion_model=FEAT_CONFIG['emotion_model'],
            device=device
        )
        
        self.emotion_columns = [
            'anger', 'disgust', 'fear', 'happiness', 'sadness', 'surprise', 'neutral'
        ]
        
        self.emotion_mapping = {
            'angry': 'anger', 'disgust': 'disgust', 'fear': 'fear',
            'happy': 'happiness', 'sad': 'sadness', 'surprise': 'surprise',
            'neutral': 'neutral'
        }
    
    def detect_emotions_from_video(self, video_path: str) -> Optional[pd.DataFrame]:
        """Detects emotions from a video file."""
        try:
            return self.detector.detect_video(video_path)
        except Exception as e:
            print(f"Error processing video {video_path}: {str(e)}")
            return None
    
    # ... The rest of your file (extract_emotion_features, etc.) remains the same ...
    # Make sure it's included below this point. The following is a placeholder.

    def extract_emotion_features(self, results: pd.DataFrame, target_emotion: str) -> Dict:
        # This method and the others below it remain unchanged
        if results is None or len(results) == 0:
            return self._get_empty_features()
        # ... (rest of the method) ...
        emotion_cols = [col for col in self.emotion_columns if col in results.columns]
        if not emotion_cols: return self._get_empty_features()
        emotion_stats = {}
        for emotion in emotion_cols:
            emotion_values = results[emotion].dropna()
            if len(emotion_values) > 0:
                emotion_stats[f'{emotion}_mean'] = emotion_values.mean()
                emotion_stats[f'{emotion}_max'] = emotion_values.max()
                emotion_stats[f'{emotion}_std'] = emotion_values.std()
                emotion_stats[f'{emotion}_median'] = emotion_values.median()
        target_feat_emotion = self.emotion_mapping.get(target_emotion, target_emotion)
        if target_feat_emotion in results.columns:
            target_values = results[target_feat_emotion].dropna()
            if len(target_values) > 0:
                emotion_stats.update({
                    'target_emotion_intensity': target_values.mean(), 'target_emotion_peak': target_values.max(),
                    'target_emotion_std': target_values.std(),
                    'frames_with_target_emotion': len(target_values[target_values > 0.5]),
                    'peak_frame_index': target_values.idxmax() if len(target_values) > 0 else -1
                })
        predicted_emotion = self._get_dominant_emotion(results)
        emotion_stats['predicted_emotion'] = predicted_emotion
        emotion_stats['classification_correct'] = (predicted_emotion == target_feat_emotion)
        emotion_stats.update({
            'total_frames': len(results), 'frames_with_face': len(results.dropna()),
            'face_detection_rate': len(results.dropna()) / len(results) if len(results) > 0 else 0,
            'target_emotion': target_emotion, 'video_duration_frames': len(results)
        })
        return emotion_stats

    def _get_dominant_emotion(self, results: pd.DataFrame) -> str:
        emotion_cols = [col for col in self.emotion_columns if col in results.columns]
        if not emotion_cols: return 'unknown'
        emotion_means = results[emotion_cols].mean()
        return emotion_means.idxmax()

    def _get_empty_features(self) -> Dict:
        features = {}
        for emotion in self.emotion_columns:
            features.update({ f'{emotion}_mean': 0.0, f'{emotion}_max': 0.0, f'{emotion}_std': 0.0, f'{emotion}_median': 0.0})
        features.update({
            'target_emotion_intensity': 0.0, 'target_emotion_peak': 0.0, 'target_emotion_std': 0.0,
            'frames_with_target_emotion': 0, 'peak_frame_index': -1, 'predicted_emotion': 'none',
            'classification_correct': False, 'total_frames': 0, 'frames_with_face': 0,
            'face_detection_rate': 0.0, 'target_emotion': '', 'video_duration_frames': 0
        })
        return features

    def analyze_single_video(self, video_path: str, target_emotion: str, participant: str, voice_type: str) -> Dict:
        print(f"Analyzing: {Path(video_path).name}")
        results = self.detect_emotions_from_video(video_path)
        features = self.extract_emotion_features(results, target_emotion)
        features.update({
            'video_path': video_path, 'participant': participant,
            'voice_type': voice_type, 'filename': Path(video_path).name
        })
        return features, results