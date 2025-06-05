"""
Visualization functions for emotion analysis results
"""

import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import warnings
warnings.filterwarnings('ignore')

from ..utils.config import EMOTIONS, RESULTS_PATH

# Set style
plt.style.use('default')
sns.set_palette("husl")


class EmotionVisualizer:
    """Create visualizations for emotion analysis results"""
    
    def __init__(self, save_path: Optional[Path] = None):
        self.save_path = save_path or (RESULTS_PATH / "visualizations")
        self.save_path.mkdir(parents=True, exist_ok=True)
        
        # Color schemes
        self.voice_colors = {
            'Emotional': '#2E86AB',
            'Emotionless': '#A23B72'
        }
        
        self.emotion_colors = {
            'happy': '#FFD23F',
            'sad': '#4A90E2',
            'angry': '#E94B3C',
            'fear': '#9013FE',
            'disgust': '#4CAF50',
            'surprise': '#FF9800',
            'neutral': '#9E9E9E'
        }
    
    def plot_emotion_intensity_comparison(self, df: pd.DataFrame, save: bool = True) -> plt.Figure:
        """
        Create bar plot comparing emotion intensities between voice conditions
        
        Args:
            df: DataFrame with emotion analysis results
            save: Whether to save the plot
            
        Returns:
            matplotlib Figure object
        """
        # Prepare data
        plot_data = df.groupby(['target_emotion', 'voice_condition'])['target_emotion_intensity'].agg([
            'mean', 'std', 'count'
        ]).reset_index()
        
        # Create figure
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Create grouped bar plot
        emotions = plot_data['target_emotion'].unique()
        x = np.arange(len(emotions))
        width = 0.35
        
        emotional_means = []
        emotionless_means = []
        emotional_stds = []
        emotionless_stds = []
        
        for emotion in emotions:
            emo_data = plot_data[(plot_data['target_emotion'] == emotion) & 
                               (plot_data['voice_condition'] == 'Emotional')]
            emo_less_data = plot_data[(plot_data['target_emotion'] == emotion) & 
                                    (plot_data['voice_condition'] == 'Emotionless')]
            
            emotional_means.append(emo_data['mean'].iloc[0] if len(emo_data) > 0 else 0)
            emotionless_means.append(emo_less_data['mean'].iloc[0] if len(emo_less_data) > 0 else 0)
            emotional_stds.append(emo_data['std'].iloc[0] if len(emo_data) > 0 else 0)
            emotionless_stds.append(emo_less_data['std'].iloc[0] if len(emo_less_data) > 0 else 0)
        
        # Create bars
        bars1 = ax.bar(x - width/2, emotional_means, width, 
                      yerr=emotional_stds, label='Emotional TTS',
                      color=self.voice_colors['Emotional'], alpha=0.8,
                      capsize=5)
        
        bars2 = ax.bar(x + width/2, emotionless_means, width,
                      yerr=emotionless_stds, label='Emotionless TTS',
                      color=self.voice_colors['Emotionless'], alpha=0.8,
                      capsize=5)
        
        # Customize plot
        ax.set_xlabel('Target Emotion', fontsize=12, fontweight='bold')
        ax.set_ylabel('Mean Emotion Intensity', fontsize=12, fontweight='bold')
        ax.set_title('Emotion Intensity by Voice Condition', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([emotion.capitalize() for emotion in emotions], rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar in bars1:
            height = bar.get_height()
            ax.annotate(f'{height:.2f}',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3),  # 3 points vertical offset
                       textcoords="offset points",
                       ha='center', va='bottom', fontsize=9)
        
        for bar in bars2:
            height = bar.get_height()
            ax.annotate(f'{height:.2f}',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3),  # 3 points vertical offset
                       textcoords="offset points",
                       ha='center', va='bottom', fontsize=9)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(self.save_path / "emotion_intensity_comparison.png", 
                       dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_classification_accuracy(self, df: pd.DataFrame, save: bool = True) -> plt.Figure:
        """
        Create plot showing classification accuracy by emotion and voice condition
        
        Args:
            df: DataFrame with emotion analysis results
            save: Whether to save the plot
            
        Returns:
            matplotlib Figure object
        """
        # Calculate accuracy by emotion and voice condition
        accuracy_data = df.groupby(['target_emotion', 'voice_condition'])['classification_correct'].agg([
            'mean', 'count'
        ]).reset_index()
        accuracy_data['accuracy_percent'] = accuracy_data['mean'] * 100
        
        # Create figure
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Create grouped bar plot
        emotions = accuracy_data['target_emotion'].unique()
        x = np.arange(len(emotions))
        width = 0.35
        
        emotional_acc = []
        emotionless_acc = []
        
        for emotion in emotions:
            emo_data = accuracy_data[(accuracy_data['target_emotion'] == emotion) & 
                                   (accuracy_data['voice_condition'] == 'Emotional')]
            emo_less_data = accuracy_data[(accuracy_data['target_emotion'] == emotion) & 
                                        (accuracy_data['voice_condition'] == 'Emotionless')]
            
            emotional_acc.append(emo_data['accuracy_percent'].iloc[0] if len(emo_data) > 0 else 0)
            emotionless_acc.append(emo_less_data['accuracy_percent'].iloc[0] if len(emo_less_data) > 0 else 0)
        
        # Create bars
        bars1 = ax.bar(x - width/2, emotional_acc, width, 
                      label='Emotional TTS',
                      color=self.voice_colors['Emotional'], alpha=0.8)
        
        bars2 = ax.bar(x + width/2, emotionless_acc, width,
                      label='Emotionless TTS',
                      color=self.voice_colors['Emotionless'], alpha=0.8)
        
        # Customize plot
        ax.set_xlabel('Target Emotion', fontsize=12, fontweight='bold')
        ax.set_ylabel('Classification Accuracy (%)', fontsize=12, fontweight='bold')
        ax.set_title('Emotion Classification Accuracy by Voice Condition', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([emotion.capitalize() for emotion in emotions], rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 100)
        
        # Add value labels on bars
        for bar in bars1:
            height = bar.get_height()
            ax.annotate(f'{height:.1f}%',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3),
                       textcoords="offset points",
                       ha='center', va='bottom', fontsize=9)
        
        for bar in bars2:
            height = bar.get_height()
            ax.annotate(f'{height:.1f}%',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3),
                       textcoords="offset points",
                       ha='center', va='bottom', fontsize=9)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(self.save_path / "classification_accuracy.png", 
                       dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_participant_heatmap(self, df: pd.DataFrame, save: bool = True) -> plt.Figure:
        """
        Create heatmap showing individual participant responses
        
        Args:
            df: DataFrame with emotion analysis results
            save: Whether to save the plot
            
        Returns:
            matplotlib Figure object
        """
        # Create pivot table for heatmap
        heatmap_data = df.pivot_table(
            values='target_emotion_intensity',
            index='participant',
            columns=['target_emotion', 'voice_condition'],
            aggfunc='mean'
        )
        
        # Create figure
        fig, ax = plt.subplots(figsize=(16, 10))
        
        # Create heatmap
        sns.heatmap(heatmap_data, annot=True, fmt='.2f', cmap='RdYlBu_r',
                   center=0.5, ax=ax, cbar_kws={'label': 'Emotion Intensity'})
        
        ax.set_title('Individual Participant Emotion Responses', fontsize=14, fontweight='bold')
        ax.set_xlabel('Emotion × Voice Condition', fontsize=12, fontweight='bold')
        ax.set_ylabel('Participant', fontsize=12, fontweight='bold')
        
        plt.xticks(rotation=45, ha='right')
        plt.tight_layout()
        
        if save:
            plt.savefig(self.save_path / "participant_heatmap.png", 
                       dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_effect_sizes(self, pairwise_results: Dict, save: bool = True) -> plt.Figure:
        """
        Create plot showing effect sizes (Cohen's d) for each emotion
        
        Args:
            pairwise_results: Results from pairwise comparisons
            save: Whether to save the plot
            
        Returns:
            matplotlib Figure object
        """
        # Extract effect sizes
        emotions = []
        effect_sizes = []
        significance = []
        
        for emotion, results in pairwise_results.items():
            emotions.append(emotion.capitalize())
            effect_sizes.append(results['cohens_d'])
            significance.append(results['significant'])
        
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Create bar plot with different colors for significant/non-significant
        colors = ['#2E86AB' if sig else '#CCCCCC' for sig in significance]
        bars = ax.bar(emotions, effect_sizes, color=colors, alpha=0.8)
        
        # Add horizontal lines for effect size thresholds
        ax.axhline(y=0.2, color='green', linestyle='--', alpha=0.7, label='Small effect (0.2)')
        ax.axhline(y=0.5, color='orange', linestyle='--', alpha=0.7, label='Medium effect (0.5)')
        ax.axhline(y=0.8, color='red', linestyle='--', alpha=0.7, label='Large effect (0.8)')
        ax.axhline(y=0, color='black', linestyle='-', alpha=0.5)
        
        # Customize plot
        ax.set_xlabel('Target Emotion', fontsize=12, fontweight='bold')
        ax.set_ylabel("Cohen's d (Effect Size)", fontsize=12, fontweight='bold')
        ax.set_title('Effect Sizes: Emotional vs Emotionless TTS', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, effect_size, sig in zip(bars, effect_sizes, significance):
            height = bar.get_height()
            label = f'{effect_size:.2f}{"*" if sig else ""}'
            ax.annotate(label,
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3 if height >= 0 else -15),
                       textcoords="offset points",
                       ha='center', va='bottom' if height >= 0 else 'top',
                       fontsize=10, fontweight='bold' if sig else 'normal')
        
        plt.xticks(rotation=45)
        plt.tight_layout()
        
        if save:
            plt.savefig(self.save_path / "effect_sizes.png", 
                       dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_emotion_distributions(self, df: pd.DataFrame, save: bool = True) -> plt.Figure:
        """
        Create violin plots showing emotion intensity distributions
        
        Args:
            df: DataFrame with emotion analysis results
            save: Whether to save the plot
            
        Returns:
            matplotlib Figure object
        """
        # Create figure
        fig, ax = plt.subplots(figsize=(14, 8))
        
        # Create violin plot
        sns.violinplot(data=df, x='target_emotion', y='target_emotion_intensity',
                      hue='voice_condition', split=True, ax=ax,
                      palette=[self.voice_colors['Emotional'], self.voice_colors['Emotionless']])
        
        # Customize plot
        ax.set_xlabel('Target Emotion', fontsize=12, fontweight='bold')
        ax.set_ylabel('Emotion Intensity', fontsize=12, fontweight='bold')
        ax.set_title('Distribution of Emotion Intensities by Voice Condition', fontsize=14, fontweight='bold')
        ax.legend(title='Voice Condition', title_fontsize=12)
        ax.grid(True, alpha=0.3)
        
        plt.xticks(rotation=45)
        plt.tight_layout()
        
        if save:
            plt.savefig(self.save_path / "emotion_distributions.png", 
                       dpi=300, bbox_inches='tight')
        
        return fig
    
    def create_summary_dashboard(self, df: pd.DataFrame, pairwise_results: Dict, 
                               save: bool = True) -> plt.Figure:
        """
        Create a comprehensive dashboard with multiple plots
        
        Args:
            df: DataFrame with emotion analysis results
            pairwise_results: Results from pairwise comparisons
            save: Whether to save the plot
            
        Returns:
            matplotlib Figure object
        """
        # Create figure with subplots
        fig = plt.figure(figsize=(20, 16))
        
        # Plot 1: Emotion intensity comparison
        ax1 = plt.subplot(2, 3, 1)
        self._create_intensity_subplot(df, ax1)
        
        # Plot 2: Classification accuracy
        ax2 = plt.subplot(2, 3, 2)
        self._create_accuracy_subplot(df, ax2)
        
        # Plot 3: Effect sizes
        ax3 = plt.subplot(2, 3, 3)
        self._create_effect_size_subplot(pairwise_results, ax3)
        
        # Plot 4: Overall voice condition comparison
        ax4 = plt.subplot(2, 3, 4)
        self._create_overall_comparison_subplot(df, ax4)
        
        # Plot 5: Participant variability
        ax5 = plt.subplot(2, 3, 5)
        self._create_participant_variability_subplot(df, ax5)
        
        # Plot 6: Summary statistics table
        ax6 = plt.subplot(2, 3, 6)
        self._create_summary_table_subplot(df, pairwise_results, ax6)
        
        plt.suptitle('Emotion Elicitation Analysis Dashboard', fontsize=20, fontweight='bold')
        plt.tight_layout()
        
        if save:
            plt.savefig(self.save_path / "analysis_dashboard.png", 
                       dpi=300, bbox_inches='tight')
        
        return fig
    
    def _create_intensity_subplot(self, df: pd.DataFrame, ax):
        """Helper method for intensity comparison subplot"""
        plot_data = df.groupby(['target_emotion', 'voice_condition'])['target_emotion_intensity'].mean().unstack()
        plot_data.plot(kind='bar', ax=ax, color=[self.voice_colors['Emotional'], self.voice_colors['Emotionless']])
        ax.set_title('Mean Emotion Intensity', fontweight='bold')
        ax.set_ylabel('Intensity')
        ax.legend(['Emotional TTS', 'Emotionless TTS'])
        ax.tick_params(axis='x', rotation=45)
    
    def _create_accuracy_subplot(self, df: pd.DataFrame, ax):
        """Helper method for accuracy subplot"""
        accuracy_data = df.groupby(['target_emotion', 'voice_condition'])['classification_correct'].mean().unstack() * 100
        accuracy_data.plot(kind='bar', ax=ax, color=[self.voice_colors['Emotional'], self.voice_colors['Emotionless']])
        ax.set_title('Classification Accuracy', fontweight='bold')
        ax.set_ylabel('Accuracy (%)')
        ax.legend(['Emotional TTS', 'Emotionless TTS'])
        ax.tick_params(axis='x', rotation=45)
    
    def _create_effect_size_subplot(self, pairwise_results: Dict, ax):
        """Helper method for effect size subplot"""
        emotions = list(pairwise_results.keys())
        effect_sizes = [pairwise_results[emotion]['cohens_d'] for emotion in emotions]
        significance = [pairwise_results[emotion]['significant'] for emotion in emotions]
        
        colors = ['#2E86AB' if sig else '#CCCCCC' for sig in significance]
        ax.bar(emotions, effect_sizes, color=colors)
        ax.set_title("Effect Sizes (Cohen's d)", fontweight='bold')
        ax.set_ylabel("Cohen's d")
        ax.tick_params(axis='x', rotation=45)
        ax.axhline(y=0.5, color='red', linestyle='--', alpha=0.7)
    
    def _create_overall_comparison_subplot(self, df: pd.DataFrame, ax):
        """Helper method for overall comparison subplot"""
        overall_stats = df.groupby('voice_condition')['target_emotion_intensity'].agg(['mean', 'std'])
        overall_stats['mean'].plot(kind='bar', yerr=overall_stats['std'], ax=ax, 
                                  color=[self.voice_colors['Emotional'], self.voice_colors['Emotionless']])
        ax.set_title('Overall Emotion Intensity', fontweight='bold')
        ax.set_ylabel('Mean Intensity')
        ax.tick_params(axis='x', rotation=0)
    
    def _create_participant_variability_subplot(self, df: pd.DataFrame, ax):
        """Helper method for participant variability subplot"""
        participant_means = df.groupby(['participant', 'voice_condition'])['target_emotion_intensity'].mean().unstack()
        participant_means.plot(kind='box', ax=ax, color=dict(boxes=[self.voice_colors['Emotional'], self.voice_colors['Emotionless']]))
        ax.set_title('Participant Variability', fontweight='bold')
        ax.set_ylabel('Mean Intensity')
    
    def _create_summary_table_subplot(self, df: pd.DataFrame, pairwise_results: Dict, ax):
        """Helper method for summary statistics table"""
        ax.axis('tight')
        ax.axis('off')
        
        # Create summary statistics
        summary_data = []
        for emotion in EMOTIONS:
            if emotion in pairwise_results:
                results = pairwise_results[emotion]
                summary_data.append([
                    emotion.capitalize(),
                    f"{results['emotional_mean']:.3f}",
                    f"{results['emotionless_mean']:.3f}",
                    f"{results['cohens_d']:.3f}",
                    "Yes" if results['significant'] else "No"
                ])
        
        table = ax.table(cellText=summary_data,
                        colLabels=['Emotion', 'Emotional\nMean', 'Emotionless\nMean', "Cohen's d", 'Significant'],
                        cellLoc='center',
                        loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.2, 1.5)
        ax.set_title('Summary Statistics', fontweight='bold')


def create_all_visualizations(df: pd.DataFrame, pairwise_results: Dict, 
                            save_path: Optional[Path] = None) -> Dict[str, plt.Figure]:
    """
    Create all visualizations for emotion analysis
    
    Args:
        df: DataFrame with emotion analysis results
        pairwise_results: Results from pairwise comparisons
        save_path: Path to save visualizations
        
    Returns:
        Dictionary of figure names and matplotlib Figure objects
    """
    visualizer = EmotionVisualizer(save_path)
    
    figures = {}
    
    print("Creating visualizations...")
    
    # Individual plots
    figures['intensity_comparison'] = visualizer.plot_emotion_intensity_comparison(df)
    figures['classification_accuracy'] = visualizer.plot_classification_accuracy(df)
    figures['participant_heatmap'] = visualizer.plot_participant_heatmap(df)
    figures['effect_sizes'] = visualizer.plot_effect_sizes(pairwise_results)
    figures['emotion_distributions'] = visualizer.plot_emotion_distributions(df)
    
    # Summary dashboard
    figures['dashboard'] = visualizer.create_summary_dashboard(df, pairwise_results)
    
    print(f"Visualizations saved to: {visualizer.save_path}")
    
    return figures


if __name__ == "__main__":
    print("Emotion visualization module loaded successfully")
