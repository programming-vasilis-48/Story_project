#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Streamlined Video Emotion Analysis using PyFeat

This script analyzes pre-recorded videos to detect facial emotions,
using the PyFeat library. It processes a single session with very few frames
to make the analysis more manageable.
"""

import cv2
import numpy as np
import pandas as pd
import os
import time
import argparse
from pathlib import Path
import logging
from datetime import datetime
import matplotlib.pyplot as plt
import seaborn as sns

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("streamlined_analysis")

# Try to import PyFeat
try:
    from feat import Detector
    logger.info("PyFeat successfully imported!")
    PYFEAT_AVAILABLE = True
except ImportError as e:
    PYFEAT_AVAILABLE = False
    logger.error(f"PyFeat import error: {e}")
    logger.error("PyFeat not available. Please install with: pip install py-feat")
except Exception as e:
    PYFEAT_AVAILABLE = False
    logger.error(f"Unexpected error importing PyFeat: {e}")

def extract_metadata(filename):
    """Extract metadata from video filename."""
    metadata = {
        "participant_id": "unknown",
        "voice_type": "unknown",
        "emotion": "unknown",
        "exchange_number": -1
    }
    
    try:
        name = Path(filename).stem
        parts = name.split('_')
        
        metadata["participant_id"] = parts[0]
        
        for part in reversed(parts):
            if part.isdigit():
                metadata["exchange_number"] = int(part)
                break
        
        if "emotionless" in name.lower():
            metadata["voice_type"] = "Emotionless Voice"
        elif "emotional" in name.lower():
            metadata["voice_type"] = "Emotional OpenAI TTS"
            
        emotions = ["angry", "happy", "sad", "fear", "disgust", "surprise", "neutral"]
        for emotion in emotions:
            if emotion.lower() in name.lower():
                metadata["emotion"] = emotion
                break
    
    except Exception as e:
        logger.warning(f"Error extracting metadata: {str(e)}")
    
    return metadata

def analyze_video(video_path, detector, frames_to_sample=5):
    """Process a video file to extract facial emotions from specific frames."""
    video_path = Path(video_path)
    logger.info(f"Processing video: {video_path}")
    
    # Extract metadata from filename
    metadata = extract_metadata(video_path.name)
    
    # Open the video file
    video = cv2.VideoCapture(str(video_path))
    if not video.isOpened():
        logger.error(f"Could not open video file: {video_path}")
        return None
    
    # Get video properties
    fps = video.get(cv2.CAP_PROP_FPS)
    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = frame_count / fps
    
    logger.info(f"Video info: {fps} fps, {frame_count} frames, {duration:.2f} seconds")
    
    # Calculate frame indices to sample (evenly spaced)
    if frame_count <= frames_to_sample:
        frame_indices = list(range(frame_count))
    else:
        step = frame_count // frames_to_sample
        frame_indices = [i * step for i in range(frames_to_sample)]
    
    logger.info(f"Will sample {len(frame_indices)} frames from video")
    
    # Create a temporary directory for storing frames
    temp_dir = Path("./temp_frames")
    temp_dir.mkdir(exist_ok=True)
    
    # Results storage
    all_results = []
    
    # Process specific frames
    for i, frame_idx in enumerate(frame_indices):
        # Set frame position
        video.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = video.read()
        
        if not ret:
            logger.warning(f"Could not read frame {frame_idx}")
            continue
        
        # Save frame to temporary file
        temp_file = temp_dir / f"frame_{i:04d}.jpg"
        cv2.imwrite(str(temp_file), frame)
        
        try:
            # Process the frame with PyFeat
            results = detector.detect_image(str(temp_file))
            
            # Check if faces were detected
            if isinstance(results, pd.DataFrame) and not results.empty:
                # Add frame information
                results['frame_idx'] = frame_idx
                results['timestamp'] = frame_idx / fps
                results['video_file'] = str(video_path)
                
                # Add metadata
                for key, value in metadata.items():
                    results[key] = value
                
                # Append to results list
                all_results.append(results)
                logger.info(f"Frame {i+1}/{len(frame_indices)} - Found {len(results)} faces")
            else:
                logger.info(f"Frame {i+1}/{len(frame_indices)} - No faces detected")
        
        except Exception as e:
            logger.error(f"Error processing frame {frame_idx}: {str(e)}")
        
        # Clean up temporary file
        os.remove(temp_file)
    
    # Release video
    video.release()
    
    # Clean up temporary directory
    try:
        os.rmdir(temp_dir)
    except:
        pass
    
    # Combine all results
    if all_results:
        final_results = pd.concat(all_results, ignore_index=True)
        logger.info(f"Found faces in {len(all_results)} of {len(frame_indices)} sampled frames")
        return final_results
    else:
        logger.warning("No faces detected in the sampled frames")
        return None

def create_plots(results_df, session_name, output_dir):
    """Create emotion plots for the session."""
    if results_df is None or results_df.empty:
        logger.warning("No data to create plots")
        return
    
    plots_dir = Path(output_dir) / "emotion_plots"
    plots_dir.mkdir(exist_ok=True, parents=True)
    
    try:
        # Set plot style
        plt.style.use('seaborn-v0_8-whitegrid')
        
        # Define emotion columns
        emotion_cols = ['anger', 'disgust', 'fear', 'happiness', 'sadness', 'surprise', 'neutral']
        
        # Prepare data for plotting - aggregate emotions by voice type
        emotion_data = []
        
        for emotion in emotion_cols:
            if emotion in results_df.columns:
                for voice_type in results_df['voice_type'].unique():
                    voice_data = results_df[results_df['voice_type'] == voice_type]
                    if not voice_data.empty:
                        # Calculate mean emotion value
                        mean_value = voice_data[emotion].mean()
                        emotion_data.append({
                            'emotion': emotion,
                            'voice_type': voice_type,
                            'mean_value': mean_value
                        })
        
        if emotion_data:
            emotion_df = pd.DataFrame(emotion_data)
            
            # Create plot
            plt.figure(figsize=(12, 8))
            sns.barplot(x="emotion", y="mean_value", hue="voice_type", data=emotion_df)
            plt.title(f"Mean Emotion Values by Voice Type - {session_name}")
            plt.xlabel("Emotion")
            plt.ylabel("Mean Value")
            plt.xticks(rotation=45)
            plt.tight_layout()
            plot_path = plots_dir / f"{session_name}_emotions_by_voice_type.png"
            plt.savefig(str(plot_path), dpi=300)
            plt.close()
            logger.info(f"Created emotion plot: {plot_path}")
            
            return str(plot_path)
    
    except Exception as e:
        logger.error(f"Error creating plots: {str(e)}")
        return None

def create_html_report(session_name, plot_path, output_dir):
    """Create a simple HTML report."""
    try:
        report_path = Path(output_dir) / f"{session_name}_report.html"
        
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Facial Emotion Analysis - {session_name}</title>
            <style>
                body {{ font-family: Arial, sans-serif; margin: 20px; }}
                h1 {{ color: #2c3e50; }}
                .plot {{ margin: 20px 0; }}
                img {{ max-width: 100%; border: 1px solid #ddd; }}
            </style>
        </head>
        <body>
            <h1>Facial Emotion Analysis - {session_name}</h1>
            <p>This report shows the facial emotions detected in the participant videos, comparing responses to two voice types.</p>
            
            <div class="plot">
                <h2>Emotion Distribution by Voice Type</h2>
                <img src="{os.path.relpath(plot_path, str(report_path.parent))}" alt="Emotion Distribution">
            </div>
        </body>
        </html>
        """
        
        with open(report_path, 'w') as f:
            f.write(html)
        
        logger.info(f"Created HTML report: {report_path}")
        return str(report_path)
    
    except Exception as e:
        logger.error(f"Error creating HTML report: {str(e)}")
        return None

def main():
    parser = argparse.ArgumentParser(description="Analyze facial emotions using PyFeat")
    parser.add_argument("--session", type=str, default="../logs/sessions/2025-06-04_17-17-41_andreea1", 
                      help="Path to session directory")
    parser.add_argument("--output", type=str, default="./streamlined_results", 
                      help="Output directory for results")
    parser.add_argument("--max_videos", type=int, default=2, 
                      help="Maximum number of videos to process")
    parser.add_argument("--frames", type=int, default=5, 
                      help="Number of frames to sample per video")
    args = parser.parse_args()
    
    if not PYFEAT_AVAILABLE:
        logger.error("PyFeat is not available. Cannot continue.")
        return 1
    
    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(exist_ok=True, parents=True)
    
    try:
        # Initialize PyFeat detector
        logger.info("Initializing PyFeat detector...")
        detector = Detector(
            face_model="retinaface",
            landmark_model="mobilenet",
            au_model="xgb",
            emotion_model="resmasknet",
            identity_model=None
        )
        
        # Find video files in the session directory
        session_dir = Path(args.session)
        videos_dir = session_dir / "videos"
        
        if not videos_dir.exists():
            logger.error(f"Videos directory not found: {videos_dir}")
            return 1
        
        video_files = list(videos_dir.glob("*.mp4"))
        logger.info(f"Found {len(video_files)} video files in {videos_dir}")
        
        if args.max_videos > 0 and args.max_videos < len(video_files):
            video_files = video_files[:args.max_videos]
            logger.info(f"Limiting analysis to {args.max_videos} videos")
        
        # Process each video
        all_results = []
        
        for i, video_file in enumerate(video_files):
            logger.info(f"Processing video {i+1}/{len(video_files)}: {video_file.name}")
            results = analyze_video(video_file, detector, frames_to_sample=args.frames)
            
            if results is not None:
                all_results.append(results)
        
        # Combine all results
        if all_results:
            combined_results = pd.concat(all_results, ignore_index=True)
            
            # Save results to CSV
            csv_path = output_dir / f"{session_dir.name}_emotions.csv"
            combined_results.to_csv(csv_path, index=False)
            logger.info(f"Saved results to {csv_path}")
            
            # Create plots
            plot_path = create_plots(combined_results, session_dir.name, output_dir)
            
            # Create HTML report
            if plot_path:
                report_path = create_html_report(session_dir.name, plot_path, output_dir)
                logger.info(f"Analysis complete! Open {report_path} to view results.")
            
            return 0
        else:
            logger.error("No results obtained from any videos")
            return 1
    
    except Exception as e:
        logger.error(f"Error during analysis: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    main() 