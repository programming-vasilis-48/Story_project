"""
Video Analysis Package for Emotion Elicitation Experiment

This package provides comprehensive video analysis tools for studying
emotional responses in human-robot interaction experiments.
"""

from .batch_processor import run_complete_analysis, BatchVideoProcessor
from .utils.data_loader import load_video_data, VideoDataLoader
from .core.emotion_detector import EmotionDetector
from .analysis.statistical_analyzer import StatisticalAnalyzer, run_statistical_analysis
from .visualization.emotion_plots import create_all_visualizations, EmotionVisualizer

__version__ = "1.0.0"
__author__ = "Emotion Analysis Team"

__all__ = [
    'run_complete_analysis',
    'BatchVideoProcessor',
    'load_video_data',
    'VideoDataLoader',
    'EmotionDetector',
    'StatisticalAnalyzer',
    'run_statistical_analysis',
    'create_all_visualizations',
    'EmotionVisualizer'
]
