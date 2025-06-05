"""
Configuration settings for video emotion analysis
"""

import os
from pathlib import Path

# Base paths
PROJECT_ROOT = Path(__file__).parent.parent.parent
SESSIONS_PATH = PROJECT_ROOT / "logs" / "sessions"
RESULTS_PATH = PROJECT_ROOT / "video_analysis" / "results"

# Analysis settings
EMOTIONS = ['neutral', 'surprise', 'fear', 'disgust', 'angry', 'happy', 'sad']
VOICE_TYPES = ['Emotional_OpenAI_TTS', 'Emotionless_Voice']

# py-feat settings
FEAT_CONFIG = {
    'face_model': 'retinaface',
    'landmark_model': 'mobilefacenet',
    'au_model': 'xgb',
    'emotion_model': 'resmasknet',
    'facepose_model': 'img2pose'
}

# Video processing settings
VIDEO_CONFIG = {
    'fps_target': 30,  # Target FPS for analysis
    'face_detection_confidence': 0.5,
    'min_face_size': 50,  # Minimum face size in pixels
    'max_frames_per_video': 3000  # Limit for very long videos
}

# Statistical analysis settings
STATS_CONFIG = {
    'alpha': 0.05,  # Significance level
    'bonferroni_correction': True,
    'effect_size_thresholds': {
        'small': 0.2,
        'medium': 0.5,
        'large': 0.8
    }
}

# Output settings
OUTPUT_CONFIG = {
    'save_individual_frames': False,
    'save_emotion_timelines': True,
    'generate_plots': True,
    'export_formats': ['csv', 'json', 'xlsx']
}

# Ensure results directories exist
for subdir in ['raw_data', 'statistical_results', 'visualizations', 'reports']:
    (RESULTS_PATH / subdir).mkdir(parents=True, exist_ok=True)
