# Video Emotion Analysis System

A comprehensive video analysis system for studying emotional responses in human-robot interaction experiments. This system analyzes facial expressions in video recordings to measure emotion elicitation effectiveness between different voice conditions.

## Overview

This system is designed to analyze the research question: **"How does a robot's use of emotionally expressive text-to-speech (TTS) versus a neutral TTS affect the emotional intensity experienced by human participants during a cooperative storytelling task?"**

## Features

- **Automated Facial Emotion Detection**: Uses py-feat library for robust emotion recognition
- **Statistical Analysis**: Comprehensive ANOVA and pairwise comparisons with effect sizes
- **Visualization**: Multiple plot types including intensity comparisons, heatmaps, and dashboards
- **Batch Processing**: Processes all videos automatically with progress tracking
- **Multiple Output Formats**: CSV, JSON, Excel, and detailed reports

## Installation

### Prerequisites

1. **Python 3.8+** is required
2. **Install py-feat** (the core emotion detection library):
   ```bash
   pip install py-feat
   ```

3. **Install additional dependencies** (if not already installed):
   ```bash
   pip install pandas numpy matplotlib seaborn scipy statsmodels opencv-python
   ```

### Optional Dependencies

For Excel export support:
```bash
pip install openpyxl
```

## Quick Start

### 1. Test Data Loading
First, verify that your video data is properly organized:

```bash
cd video_analysis
python run_analysis.py --test-data
```

This will check if all video files are found and properly named.

### 2. Run Complete Analysis
To run the full analysis pipeline:

```bash
python run_analysis.py
```

For faster processing (skip individual video results):
```bash
python run_analysis.py --no-individual
```

## Data Structure

The system expects videos to be organized as follows:
```
logs/sessions/
├── 2025-06-04_17-17-41_participant1/
│   └── videos/
│       ├── participant1_Emotional_OpenAI_TTS_happy_exchange_0.mp4
│       ├── participant1_Emotional_OpenAI_TTS_sad_exchange_1.mp4
│       ├── participant1_Emotionless_Voice_happy_exchange_2.mp4
│       └── ...
└── 2025-06-04_17-32-51_participant2/
    └── videos/
        └── ...
```

### Video Naming Convention
Videos must follow this pattern:
`{participant}_{voice_type}_{emotion}_exchange_{number}.mp4`

Where:
- `voice_type`: Either `Emotional_OpenAI_TTS` or `Emotionless_Voice`
- `emotion`: One of `happy`, `sad`, `angry`, `fear`, `disgust`, `surprise`, `neutral`

## Output Structure

After running the analysis, results are saved to:

```
video_analysis/results/
├── raw_data/                          # Individual video analysis files
├── statistical_results/               # ANOVA and statistical test results
├── visualizations/                    # Generated plots and charts
├── reports/                          # Human-readable summary reports
├── emotion_analysis_results_TIMESTAMP.csv    # Main results file
├── emotion_analysis_results_TIMESTAMP.json   # JSON format results
└── complete_analysis_TIMESTAMP.pkl           # Complete analysis object
```

## Key Outputs

### 1. Statistical Analysis
- **Two-way repeated measures ANOVA**: Voice condition × Target emotion
- **Pairwise t-tests**: Emotional vs Emotionless TTS for each emotion
- **Effect sizes**: Cohen's d with interpretation (small/medium/large)
- **Assumption testing**: Normality and homogeneity of variance

### 2. Visualizations
- **Emotion intensity comparison**: Bar charts comparing voice conditions
- **Classification accuracy**: Success rates for emotion elicitation
- **Participant heatmap**: Individual response patterns
- **Effect size plot**: Cohen's d for each emotion
- **Distribution plots**: Violin plots showing response distributions
- **Summary dashboard**: Comprehensive overview with multiple plots

### 3. Reports
- **Summary report**: Human-readable text file with key findings
- **Statistical tables**: Detailed ANOVA and t-test results
- **Data validation**: Information about missing or incomplete data

## Programmatic Usage

You can also use the system programmatically:

```python
from video_analysis import run_complete_analysis
from video_analysis.utils.data_loader import load_video_data

# Load and validate data
video_df, validation = load_video_data()
print(f"Found {len(video_df)} videos")

# Run complete analysis
results = run_complete_analysis()

# Access specific results
statistical_results = results['statistical_analysis']
pairwise_comparisons = statistical_results['pairwise_comparisons']

# Print significant emotions
for emotion, stats in pairwise_comparisons.items():
    if stats['significant']:
        print(f"{emotion}: Cohen's d = {stats['cohens_d']:.3f}")
```

## Individual Components

### Emotion Detection
```python
from video_analysis.core.emotion_detector import EmotionDetector

detector = EmotionDetector()
features, raw_results = detector.analyze_single_video(
    video_path="path/to/video.mp4",
    target_emotion="happy",
    participant="participant1",
    voice_type="Emotional_OpenAI_TTS"
)
```

### Statistical Analysis
```python
from video_analysis.analysis.statistical_analyzer import run_statistical_analysis

# Assuming you have a DataFrame with results
statistical_summary = run_statistical_analysis(results_df)
```

### Visualizations
```python
from video_analysis.visualization.emotion_plots import create_all_visualizations

figures = create_all_visualizations(results_df, pairwise_results)
```

## Configuration

Modify `video_analysis/utils/config.py` to adjust:
- Emotion detection models
- Statistical significance levels
- Output formats
- File paths

## Troubleshooting

### Common Issues

1. **py-feat installation problems**:
   ```bash
   pip install --upgrade pip
   pip install py-feat
   ```

2. **Video file not found errors**:
   - Check that video files are in the correct directory structure
   - Verify file naming follows the expected pattern
   - Run `python run_analysis.py --test-data` to validate

3. **Memory issues with large videos**:
   - Use `--no-individual` flag to save memory
   - Consider processing videos in smaller batches

4. **Missing dependencies**:
   ```bash
   pip install pandas numpy matplotlib seaborn scipy statsmodels opencv-python
   ```

### Performance Tips

- **Parallel processing**: The system processes videos sequentially. For large datasets, consider modifying the batch processor to use multiprocessing
- **Video preprocessing**: Very long videos may slow down analysis. Consider trimming to relevant segments
- **Storage**: Individual video results can take significant disk space. Use `--no-individual` if not needed

## Research Applications

This system is designed for research in:
- Human-robot interaction
- Emotion elicitation studies
- Voice and speech perception
- Affective computing
- Social robotics

## Citation

If you use this system in your research, please cite:
```
[Your research paper citation here]
```

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Verify your data follows the expected structure
3. Test with the `--test-data` flag first

## License

[Add your license information here]
