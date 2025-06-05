"""
Test script to verify the video analysis system components
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

def test_data_loading():
    """Test data loading functionality"""
    print("Testing data loading...")
    
    try:
        from video_analysis.utils.data_loader import load_video_data
        
        video_df, validation = load_video_data()
        
        print(f"✓ Successfully loaded {len(video_df)} videos")
        print(f"✓ Found {validation['total_participants']} participants")
        print(f"✓ Complete data for {validation['complete_participants']} participants")
        
        # Check data structure
        expected_columns = ['participant', 'voice_type', 'target_emotion', 'file_path']
        missing_columns = [col for col in expected_columns if col not in video_df.columns]
        
        if missing_columns:
            print(f"✗ Missing columns: {missing_columns}")
            return False
        else:
            print("✓ All required columns present")
        
        # Check voice types
        voice_types = video_df['voice_type'].unique()
        expected_voice_types = ['Emotional_OpenAI_TTS', 'Emotionless_Voice']
        
        if set(voice_types) == set(expected_voice_types):
            print("✓ Voice types correct")
        else:
            print(f"✗ Unexpected voice types: {voice_types}")
            return False
        
        # Check emotions
        emotions = video_df['target_emotion'].unique()
        expected_emotions = ['neutral', 'surprise', 'fear', 'disgust', 'angry', 'happy', 'sad']
        
        if set(emotions) == set(expected_emotions):
            print("✓ All emotions present")
        else:
            missing_emotions = set(expected_emotions) - set(emotions)
            if missing_emotions:
                print(f"⚠ Missing emotions: {missing_emotions}")
        
        return True
        
    except Exception as e:
        print(f"✗ Error in data loading: {str(e)}")
        return False

def test_statistical_analyzer():
    """Test statistical analyzer with mock data"""
    print("\nTesting statistical analyzer...")
    
    try:
        import pandas as pd
        import numpy as np
        from video_analysis.analysis.statistical_analyzer import StatisticalAnalyzer
        
        # Create mock data
        np.random.seed(42)
        n_participants = 7
        emotions = ['happy', 'sad', 'angry', 'fear', 'disgust', 'surprise', 'neutral']
        
        mock_data = []
        for participant in range(1, n_participants + 1):
            for emotion in emotions:
                for voice_condition in ['Emotional', 'Emotionless']:
                    # Simulate slightly higher intensity for emotional condition
                    base_intensity = np.random.normal(0.5, 0.2)
                    if voice_condition == 'Emotional':
                        intensity = base_intensity + np.random.normal(0.1, 0.05)
                    else:
                        intensity = base_intensity
                    
                    intensity = max(0, min(1, intensity))  # Clamp to [0, 1]
                    
                    mock_data.append({
                        'participant': f'participant{participant}',
                        'voice_condition': voice_condition,
                        'target_emotion': emotion,
                        'target_emotion_intensity': intensity,
                        'classification_correct': np.random.choice([True, False], p=[0.7, 0.3])
                    })
        
        mock_df = pd.DataFrame(mock_data)
        
        # Test statistical analyzer
        analyzer = StatisticalAnalyzer()
        
        # Test data preparation
        clean_df = analyzer.prepare_data_for_analysis(mock_df)
        print(f"✓ Data preparation successful: {len(clean_df)} rows")
        
        # Test descriptive statistics
        desc_stats = analyzer.calculate_descriptive_statistics(clean_df)
        print("✓ Descriptive statistics calculated")
        
        # Test pairwise comparisons
        pairwise_results = analyzer.perform_pairwise_comparisons(clean_df)
        print(f"✓ Pairwise comparisons completed for {len(pairwise_results)} emotions")
        
        # Test ANOVA
        anova_results = analyzer.perform_repeated_measures_anova(clean_df)
        if 'error' not in anova_results:
            print("✓ ANOVA analysis completed")
        else:
            print(f"⚠ ANOVA had issues: {anova_results['error']}")
        
        return True
        
    except Exception as e:
        print(f"✗ Error in statistical analyzer: {str(e)}")
        return False

def test_visualization():
    """Test visualization components with mock data"""
    print("\nTesting visualization...")
    
    try:
        import pandas as pd
        import numpy as np
        from video_analysis.visualization.emotion_plots import EmotionVisualizer
        
        # Create mock data
        np.random.seed(42)
        mock_data = []
        for participant in range(1, 4):  # Smaller dataset for testing
            for emotion in ['happy', 'sad', 'angry']:
                for voice_condition in ['Emotional', 'Emotionless']:
                    intensity = np.random.normal(0.5, 0.2)
                    intensity = max(0, min(1, intensity))
                    
                    mock_data.append({
                        'participant': f'participant{participant}',
                        'voice_condition': voice_condition,
                        'target_emotion': emotion,
                        'target_emotion_intensity': intensity,
                        'classification_correct': np.random.choice([True, False])
                    })
        
        mock_df = pd.DataFrame(mock_data)
        
        # Test visualizer initialization
        visualizer = EmotionVisualizer()
        print("✓ Visualizer initialized")
        
        # Test individual plot creation (without saving)
        fig1 = visualizer.plot_emotion_intensity_comparison(mock_df, save=False)
        print("✓ Intensity comparison plot created")
        
        fig2 = visualizer.plot_classification_accuracy(mock_df, save=False)
        print("✓ Classification accuracy plot created")
        
        # Close figures to free memory
        import matplotlib.pyplot as plt
        plt.close(fig1)
        plt.close(fig2)
        
        return True
        
    except Exception as e:
        print(f"✗ Error in visualization: {str(e)}")
        return False

def main():
    """Run all tests"""
    print("=" * 60)
    print("VIDEO ANALYSIS SYSTEM TEST")
    print("=" * 60)
    
    tests = [
        ("Data Loading", test_data_loading),
        ("Statistical Analyzer", test_statistical_analyzer),
        ("Visualization", test_visualization)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        print("-" * len(test_name))
        success = test_func()
        results.append((test_name, success))
    
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    all_passed = True
    for test_name, success in results:
        status = "PASS" if success else "FAIL"
        print(f"{test_name}: {status}")
        if not success:
            all_passed = False
    
    print(f"\nOverall: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")
    
    if all_passed:
        print("\n✓ System is ready for video analysis!")
        print("✓ To run full analysis: python run_analysis.py")
        print("✓ Note: py-feat is required for actual emotion detection")
    else:
        print("\n✗ Please fix the failing tests before proceeding")
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
