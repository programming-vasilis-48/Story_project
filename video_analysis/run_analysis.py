"""
Main script to run the complete emotion analysis pipeline

This script provides a simple interface to run the entire video analysis
pipeline for the emotion elicitation experiment.
"""

import sys
import argparse
from pathlib import Path

# Add the parent directory to the path so we can import the video_analysis package
sys.path.append(str(Path(__file__).parent.parent))

from video_analysis import run_complete_analysis
from video_analysis.utils.data_loader import load_video_data


def main():
    """Main function to run the analysis"""
    parser = argparse.ArgumentParser(
        description="Run emotion analysis on video data from storytelling experiment"
    )
    
    parser.add_argument(
        '--no-individual',
        action='store_true',
        help='Skip saving individual video analysis results (saves disk space)'
    )
    
    parser.add_argument(
        '--test-data',
        action='store_true',
        help='Test data loading without running full analysis'
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("EMOTION ELICITATION VIDEO ANALYSIS")
    print("=" * 60)
    print()
    
    if args.test_data:
        print("Testing data loading...")
        try:
            video_df, validation = load_video_data()
            print(f"✓ Successfully loaded {len(video_df)} videos")
            print(f"✓ Found {validation['total_participants']} participants")
            print(f"✓ Complete data for {validation['complete_participants']} participants")
            
            if validation['incomplete_participants']:
                print(f"⚠ Warning: {len(validation['participants_with_incomplete_data'])} participants have incomplete data")
                for participant, count in validation['participants_with_incomplete_data'].items():
                    print(f"  - {participant}: {count} videos (expected 14)")
            
            print("\nData validation successful! Ready to run full analysis.")
            
        except Exception as e:
            print(f"✗ Error loading data: {str(e)}")
            return 1
        
        return 0
    
    # Run complete analysis
    try:
        print("Starting complete video analysis pipeline...")
        print("This may take several minutes depending on the number of videos.")
        print()
        
        # Run analysis
        results = run_complete_analysis(save_individual=not args.no_individual)
        
        print()
        print("=" * 60)
        print("ANALYSIS COMPLETED SUCCESSFULLY!")
        print("=" * 60)
        
        # Print summary
        metadata = results['metadata']
        stats = results['statistical_analysis']
        
        print(f"Videos processed: {metadata['total_videos_processed']}")
        print(f"Participants: {metadata['participants']}")
        print(f"Failed videos: {metadata['failed_videos']}")
        
        if 'pairwise_comparisons' in stats:
            significant_emotions = [
                emotion for emotion, results in stats['pairwise_comparisons'].items() 
                if results['significant']
            ]
            print(f"Significant emotion differences: {len(significant_emotions)}/7")
            if significant_emotions:
                print(f"Significant emotions: {', '.join([e.capitalize() for e in significant_emotions])}")
        
        print()
        print("Results saved to:")
        print(f"  - Main results: video_analysis/results/")
        print(f"  - Visualizations: video_analysis/results/visualizations/")
        print(f"  - Statistical analysis: video_analysis/results/statistical_results/")
        print(f"  - Summary report: video_analysis/results/reports/")
        
        return 0
        
    except Exception as e:
        print(f"✗ Error during analysis: {str(e)}")
        print("\nTroubleshooting tips:")
        print("1. Ensure py-feat is installed: pip install py-feat")
        print("2. Check that video files exist in logs/sessions/")
        print("3. Verify all required dependencies are installed")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
