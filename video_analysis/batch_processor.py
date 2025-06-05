"""
Batch processor for analyzing all videos in the experiment
"""

import pandas as pd
import json
import pickle
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

from .utils.data_loader import VideoDataLoader, load_video_data
from .utils.config import RESULTS_PATH, OUTPUT_CONFIG
from .core.emotion_detector import EmotionDetector
from .analysis.statistical_analyzer import StatisticalAnalyzer
from .visualization.emotion_plots import create_all_visualizations


class BatchVideoProcessor:
    """Process all videos and generate comprehensive analysis"""
    
    def __init__(self, save_individual_results: bool = True):
        self.data_loader = VideoDataLoader()
        self.emotion_detector = EmotionDetector()
        self.statistical_analyzer = StatisticalAnalyzer()
        self.save_individual_results = save_individual_results
        
        # Results storage
        self.results_path = RESULTS_PATH
        self.individual_results = []
        self.analysis_summary = {}
        
        # Progress tracking
        self.total_videos = 0
        self.processed_videos = 0
        self.failed_videos = []
    
    def process_all_videos(self) -> Dict:
        """
        Process all videos in the experiment
        
        Returns:
            Complete analysis results dictionary
        """
        print("Starting batch video processing...")
        
        # Load video data
        video_df, validation = load_video_data()
        print(f"Found {len(video_df)} videos from {validation['total_participants']} participants")
        
        if validation['incomplete_participants']:
            print(f"Warning: {len(validation['participants_with_incomplete_data'])} participants have incomplete data")
        
        self.total_videos = len(video_df)
        
        # Process each video
        print("\nProcessing videos...")
        for idx, video_info in video_df.iterrows():
            self._process_single_video(video_info)
            self._update_progress()
        
        # Create results DataFrame
        results_df = pd.DataFrame(self.individual_results)
        
        # Add voice condition mapping
        if 'voice_condition' not in results_df.columns:
            results_df['voice_condition'] = results_df['voice_type'].map({
                'Emotional_OpenAI_TTS': 'Emotional',
                'Emotionless_Voice': 'Emotionless'
            })
        
        # Perform statistical analysis
        print("\nPerforming statistical analysis...")
        statistical_results = self.statistical_analyzer.generate_statistical_summary(results_df)
        
        # Create visualizations
        print("\nGenerating visualizations...")
        pairwise_results = statistical_results.get('pairwise_comparisons', {})
        figures = create_all_visualizations(results_df, pairwise_results)
        
        # Compile complete analysis
        complete_analysis = {
            'metadata': {
                'analysis_date': datetime.now().isoformat(),
                'total_videos_processed': self.processed_videos,
                'failed_videos': len(self.failed_videos),
                'participants': results_df['participant'].nunique(),
                'data_validation': validation
            },
            'raw_results': results_df,
            'statistical_analysis': statistical_results,
            'visualizations_created': list(figures.keys()),
            'failed_videos': self.failed_videos
        }
        
        # Save results
        self._save_results(complete_analysis, results_df)
        
        print(f"\nAnalysis complete!")
        print(f"Processed: {self.processed_videos}/{self.total_videos} videos")
        print(f"Failed: {len(self.failed_videos)} videos")
        print(f"Results saved to: {self.results_path}")
        
        return complete_analysis
    
    def _process_single_video(self, video_info: pd.Series) -> None:
        """Process a single video file"""
        try:
            # Extract video information
            video_path = video_info['file_path']
            participant = video_info['participant']
            voice_type = video_info['voice_type']
            target_emotion = video_info['target_emotion']
            
            # Analyze video
            features, raw_results = self.emotion_detector.analyze_single_video(
                video_path, target_emotion, participant, voice_type
            )
            
            # Add additional metadata
            features.update({
                'session_id': video_info['session_id'],
                'exchange_number': video_info['exchange_number'],
                'file_size': video_info['file_size']
            })
            
            # Store results
            self.individual_results.append(features)
            
            # Save individual results if requested
            if self.save_individual_results:
                self._save_individual_result(features, raw_results, video_info['filename'])
            
            self.processed_videos += 1
            
        except Exception as e:
            error_info = {
                'filename': video_info['filename'],
                'participant': video_info['participant'],
                'error': str(e)
            }
            self.failed_videos.append(error_info)
            print(f"Error processing {video_info['filename']}: {str(e)}")
    
    def _save_individual_result(self, features: Dict, raw_results: Optional[pd.DataFrame], 
                              filename: str) -> None:
        """Save individual video analysis results"""
        individual_path = self.results_path / "raw_data" / f"{filename}_analysis.json"
        
        # Prepare data for JSON serialization
        json_features = {}
        for key, value in features.items():
            if pd.isna(value):
                json_features[key] = None
            elif isinstance(value, (int, float, str, bool)):
                json_features[key] = value
            else:
                json_features[key] = str(value)
        
        # Save features
        with open(individual_path, 'w') as f:
            json.dump(json_features, f, indent=2)
        
        # Save raw results if available
        if raw_results is not None and len(raw_results) > 0:
            raw_path = self.results_path / "raw_data" / f"{filename}_raw_results.csv"
            raw_results.to_csv(raw_path, index=False)
    
    def _save_results(self, complete_analysis: Dict, results_df: pd.DataFrame) -> None:
        """Save complete analysis results"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save main results DataFrame
        for format_type in OUTPUT_CONFIG['export_formats']:
            if format_type == 'csv':
                results_df.to_csv(self.results_path / f"emotion_analysis_results_{timestamp}.csv", index=False)
            elif format_type == 'json':
                # Convert DataFrame to JSON-serializable format
                json_data = results_df.to_dict('records')
                with open(self.results_path / f"emotion_analysis_results_{timestamp}.json", 'w') as f:
                    json.dump(json_data, f, indent=2, default=str)
            elif format_type == 'xlsx':
                try:
                    results_df.to_excel(self.results_path / f"emotion_analysis_results_{timestamp}.xlsx", index=False)
                except ImportError:
                    print("Warning: openpyxl not installed, skipping Excel export")
        
        # Save statistical results
        stats_path = self.results_path / "statistical_results" / f"statistical_analysis_{timestamp}.json"
        with open(stats_path, 'w') as f:
            json.dump(complete_analysis['statistical_analysis'], f, indent=2, default=str)
        
        # Save complete analysis (pickle for complex objects)
        complete_path = self.results_path / f"complete_analysis_{timestamp}.pkl"
        with open(complete_path, 'wb') as f:
            pickle.dump(complete_analysis, f)
        
        # Save summary report
        self._generate_summary_report(complete_analysis, timestamp)
    
    def _generate_summary_report(self, complete_analysis: Dict, timestamp: str) -> None:
        """Generate a human-readable summary report"""
        report_path = self.results_path / "reports" / f"analysis_summary_{timestamp}.txt"
        
        stats = complete_analysis['statistical_analysis']
        metadata = complete_analysis['metadata']
        
        with open(report_path, 'w') as f:
            f.write("EMOTION ELICITATION EXPERIMENT ANALYSIS REPORT\n")
            f.write("=" * 50 + "\n\n")
            
            # Metadata
            f.write(f"Analysis Date: {metadata['analysis_date']}\n")
            f.write(f"Videos Processed: {metadata['total_videos_processed']}\n")
            f.write(f"Participants: {metadata['participants']}\n")
            f.write(f"Failed Videos: {metadata['failed_videos']}\n\n")
            
            # Descriptive Statistics
            f.write("DESCRIPTIVE STATISTICS\n")
            f.write("-" * 25 + "\n")
            
            if 'descriptive_statistics' in stats:
                desc_stats = stats['descriptive_statistics']
                
                if 'voice_condition_stats' in desc_stats:
                    f.write("\nOverall Voice Condition Comparison:\n")
                    voice_stats = desc_stats['voice_condition_stats']
                    for condition in voice_stats.index:
                        f.write(f"  {condition} TTS:\n")
                        f.write(f"    Mean Intensity: {voice_stats.loc[condition, 'mean']:.3f}\n")
                        f.write(f"    Std Deviation: {voice_stats.loc[condition, 'std']:.3f}\n")
                        f.write(f"    Sample Size: {voice_stats.loc[condition, 'count']}\n")
            
            # ANOVA Results
            f.write("\nANOVA RESULTS\n")
            f.write("-" * 15 + "\n")
            
            if 'anova_results' in stats and 'effects' in stats['anova_results']:
                effects = stats['anova_results']['effects']
                for effect_name, effect_data in effects.items():
                    f.write(f"\n{effect_name}:\n")
                    f.write(f"  F-statistic: {effect_data['F']:.3f}\n")
                    f.write(f"  p-value: {effect_data['p_value']:.6f}\n")
                    f.write(f"  Significant: {'Yes' if effect_data['significant'] else 'No'}\n")
            
            # Pairwise Comparisons
            f.write("\nPAIRWISE COMPARISONS (Emotional vs Emotionless TTS)\n")
            f.write("-" * 50 + "\n")
            
            if 'pairwise_comparisons' in stats:
                pairwise = stats['pairwise_comparisons']
                for emotion, results in pairwise.items():
                    f.write(f"\n{emotion.capitalize()}:\n")
                    f.write(f"  Emotional TTS Mean: {results['emotional_mean']:.3f}\n")
                    f.write(f"  Emotionless TTS Mean: {results['emotionless_mean']:.3f}\n")
                    f.write(f"  Cohen's d: {results['cohens_d']:.3f} ({results['effect_size_interpretation']})\n")
                    f.write(f"  p-value: {results['corrected_p_value']:.6f}\n")
                    f.write(f"  Significant: {'Yes' if results['significant'] else 'No'}\n")
            
            # Key Findings
            f.write("\nKEY FINDINGS\n")
            f.write("-" * 12 + "\n")
            
            # Count significant effects
            if 'pairwise_comparisons' in stats:
                significant_emotions = [emotion for emotion, results in stats['pairwise_comparisons'].items() 
                                      if results['significant']]
                f.write(f"• {len(significant_emotions)} out of {len(stats['pairwise_comparisons'])} emotions showed significant differences\n")
                
                if significant_emotions:
                    f.write(f"• Significant emotions: {', '.join([e.capitalize() for e in significant_emotions])}\n")
                
                # Effect sizes
                large_effects = [emotion for emotion, results in stats['pairwise_comparisons'].items() 
                               if results['effect_size_interpretation'] == 'large']
                if large_effects:
                    f.write(f"• Large effect sizes found for: {', '.join([e.capitalize() for e in large_effects])}\n")
            
            f.write(f"\n• Visualizations saved to: {self.results_path / 'visualizations'}\n")
            f.write(f"• Detailed results saved to: {self.results_path}\n")
    
    def _update_progress(self) -> None:
        """Update and display progress"""
        if self.total_videos > 0:
            progress = (self.processed_videos / self.total_videos) * 100
            print(f"Progress: {self.processed_videos}/{self.total_videos} ({progress:.1f}%)")


def run_complete_analysis(save_individual: bool = True) -> Dict:
    """
    Run complete video analysis pipeline
    
    Args:
        save_individual: Whether to save individual video results
        
    Returns:
        Complete analysis results
    """
    processor = BatchVideoProcessor(save_individual_results=save_individual)
    return processor.process_all_videos()


if __name__ == "__main__":
    # Run the complete analysis
    print("Starting emotion elicitation video analysis...")
    results = run_complete_analysis()
    print("Analysis completed successfully!")
