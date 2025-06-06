import sys
import argparse
import traceback
from pathlib import Path
import pandas as pd

# Add the parent directory to the path so we can import our project modules
sys.path.append(str(Path(__file__).parent.parent))

# Import necessary components from your project
from video_analysis.utils.data_loader import load_video_data
from video_analysis.core.emotion_detector import EmotionDetector

def main():
    """Main function to run the analysis."""
    parser = argparse.ArgumentParser(description="Run emotion analysis on video data.")
    
    # This is the new argument that the job array will use
    parser.add_argument(
        '--video-index',
        type=int,
        help='The index of a single video to process (for job array execution).'
    )
    
    args = parser.parse_args()

    # --- SINGLE VIDEO MODE (for the Job Array) ---
    if args.video_index is not None:
        print(f"--- Running in single-video mode for index: {args.video_index} ---")
        try:
            video_df, _ = load_video_data()
            if args.video_index >= len(video_df):
                print(f"Error: Index {args.video_index} is out of bounds for {len(video_df)} videos.")
                sys.exit(1)
            
            video_to_process = video_df.iloc[args.video_index]
            
            print(f"Initializing detector for video: {video_to_process['filename']}")
            detector = EmotionDetector() # This will now detect and use the GPU
            
            features, _ = detector.analyze_single_video(
                video_path=video_to_process['file_path'],
                target_emotion=video_to_process['target_emotion'],
                participant=video_to_process['participant'],
                voice_type=video_to_process['voice_type']
            )
            
            output_dir = Path("video_analysis/results/individual_results")
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = output_dir / f"result_{args.video_index}.csv"
            pd.DataFrame([features]).to_csv(output_path, index=False)

            print(f"✓ Successfully processed and saved result to {output_path}")

        except Exception:
            print(f"✗ FAILED to process video index {args.video_index}.")
            traceback.print_exc()
            sys.exit(1) # Exit with an error code so the cluster knows the job failed

    # --- BATCH MODE (if --video-index is not given) ---
    else:
        print("--- Running in full batch mode (sequentially) for local testing. ---")
        try:
            video_df, _ = load_video_data()
            detector = EmotionDetector()
            all_results = []
            for i, video_to_process in video_df.iterrows():
                print(f"\nProcessing video {i+1}/{len(video_df)}: {video_to_process['filename']}")
                features, _ = detector.analyze_single_video(
                    video_path=video_to_process['file_path'],
                    target_emotion=video_to_process['target_emotion'],
                    participant=video_to_process['participant'],
                    voice_type=video_to_process['voice_type']
                )
                if features:
                    all_results.append(features)
            
            final_df = pd.DataFrame(all_results)
            results_dir = Path("video_analysis/results")
            results_dir.mkdir(exist_ok=True)
            final_df.to_csv(results_dir / "full_batch_results.csv", index=False)
            print("\n✓ Batch analysis complete. Results saved to video_analysis/results/full_batch_results.csv")

        except Exception:
            print("\n✗ AN UNEXPECTED ERROR OCCURRED DURING BATCH ANALYSIS.")
            traceback.print_exc()
            sys.exit(1)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())