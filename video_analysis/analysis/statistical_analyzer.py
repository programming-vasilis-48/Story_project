"""
Statistical analysis for emotion elicitation experiment
"""

import numpy as np
import pandas as pd
from typing import Dict, List, Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

# Statistical libraries
from scipy import stats
from scipy.stats import ttest_rel, normaltest, levene
import statsmodels.api as sm
from statsmodels.stats.anova import anova_lm
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd

from ..utils.config import STATS_CONFIG, EMOTIONS


class StatisticalAnalyzer:
    """Statistical analysis for emotion elicitation experiment"""
    
    def __init__(self, alpha: float = None):
        self.alpha = alpha or STATS_CONFIG['alpha']
        self.bonferroni_correction = STATS_CONFIG['bonferroni_correction']
        self.effect_size_thresholds = STATS_CONFIG['effect_size_thresholds']
    
    def prepare_data_for_analysis(self, results_df: pd.DataFrame) -> pd.DataFrame:
        """
        Prepare data for statistical analysis
        
        Args:
            results_df: DataFrame with emotion analysis results
            
        Returns:
            Cleaned and prepared DataFrame
        """
        # Create a copy for analysis
        df = results_df.copy()
        
        # Ensure we have the required columns
        required_cols = ['participant', 'voice_condition', 'target_emotion', 'target_emotion_intensity']
        missing_cols = [col for col in required_cols if col not in df.columns]
        
        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")
        
        # Remove rows with missing data
        df = df.dropna(subset=['target_emotion_intensity'])
        
        # Ensure voice_condition is properly coded
        if 'voice_type' in df.columns and 'voice_condition' not in df.columns:
            df['voice_condition'] = df['voice_type'].map({
                'Emotional_OpenAI_TTS': 'Emotional',
                'Emotionless_Voice': 'Emotionless'
            })
        
        return df
    
    def perform_repeated_measures_anova(self, df: pd.DataFrame) -> Dict:
        """
        Perform two-way repeated measures ANOVA
        
        Args:
            df: Prepared data for analysis
            
        Returns:
            ANOVA results dictionary
        """
        try:
            # Fit the model using OLS with proper formula
            formula = 'target_emotion_intensity ~ C(voice_condition) * C(target_emotion) + C(participant)'
            model = ols(formula, data=df).fit()
            
            # Perform ANOVA
            anova_results = anova_lm(model, typ=2)
            
            # Extract key statistics
            results = {
                'anova_table': anova_results,
                'model_summary': model.summary(),
                'r_squared': model.rsquared,
                'adj_r_squared': model.rsquared_adj,
                'f_statistic': model.fvalue,
                'model_p_value': model.f_pvalue
            }
            
            # Extract specific effects
            effects = {}
            for index, row in anova_results.iterrows():
                effect_name = index.replace('C(', '').replace(')', '').replace('voice_condition', 'Voice').replace('target_emotion', 'Emotion')
                effects[effect_name] = {
                    'F': row['F'],
                    'p_value': row['PR(>F)'],
                    'df': row['df'],
                    'sum_sq': row['sum_sq'],
                    'mean_sq': row['mean_sq'],
                    'significant': row['PR(>F)'] < self.alpha
                }
            
            results['effects'] = effects
            
            return results
            
        except Exception as e:
            print(f"Error in ANOVA analysis: {str(e)}")
            return {'error': str(e)}
    
    def perform_pairwise_comparisons(self, df: pd.DataFrame) -> Dict:
        """
        Perform pairwise t-tests between voice conditions for each emotion
        
        Args:
            df: Prepared data for analysis
            
        Returns:
            Pairwise comparison results
        """
        results = {}
        
        for emotion in EMOTIONS:
            emotion_data = df[df['target_emotion'] == emotion]
            
            if len(emotion_data) < 4:  # Need at least 2 per condition
                continue
            
            # Get data for each voice condition
            emotional_data = emotion_data[emotion_data['voice_condition'] == 'Emotional']['target_emotion_intensity']
            emotionless_data = emotion_data[emotion_data['voice_condition'] == 'Emotionless']['target_emotion_intensity']
            
            if len(emotional_data) == 0 or len(emotionless_data) == 0:
                continue
            
            # Perform paired t-test (assuming same participants in both conditions)
            try:
                # Check if we have paired data
                emotional_participants = set(emotion_data[emotion_data['voice_condition'] == 'Emotional']['participant'])
                emotionless_participants = set(emotion_data[emotion_data['voice_condition'] == 'Emotionless']['participant'])
                common_participants = emotional_participants.intersection(emotionless_participants)
                
                if len(common_participants) > 1:
                    # Paired t-test
                    paired_emotional = []
                    paired_emotionless = []
                    
                    for participant in common_participants:
                        emo_val = emotion_data[(emotion_data['participant'] == participant) & 
                                             (emotion_data['voice_condition'] == 'Emotional')]['target_emotion_intensity']
                        emo_less_val = emotion_data[(emotion_data['participant'] == participant) & 
                                                  (emotion_data['voice_condition'] == 'Emotionless')]['target_emotion_intensity']
                        
                        if len(emo_val) > 0 and len(emo_less_val) > 0:
                            paired_emotional.append(emo_val.iloc[0])
                            paired_emotionless.append(emo_less_val.iloc[0])
                    
                    if len(paired_emotional) > 1:
                        t_stat, p_value = ttest_rel(paired_emotional, paired_emotionless)
                        test_type = 'paired'
                    else:
                        t_stat, p_value = stats.ttest_ind(emotional_data, emotionless_data)
                        test_type = 'independent'
                else:
                    # Independent t-test
                    t_stat, p_value = stats.ttest_ind(emotional_data, emotionless_data)
                    test_type = 'independent'
                
                # Calculate effect size (Cohen's d)
                pooled_std = np.sqrt(((len(emotional_data) - 1) * emotional_data.var() + 
                                    (len(emotionless_data) - 1) * emotionless_data.var()) / 
                                   (len(emotional_data) + len(emotionless_data) - 2))
                
                cohens_d = (emotional_data.mean() - emotionless_data.mean()) / pooled_std
                
                # Apply Bonferroni correction if specified
                corrected_p = p_value * len(EMOTIONS) if self.bonferroni_correction else p_value
                
                results[emotion] = {
                    't_statistic': t_stat,
                    'p_value': p_value,
                    'corrected_p_value': corrected_p,
                    'cohens_d': cohens_d,
                    'effect_size_interpretation': self._interpret_effect_size(abs(cohens_d)),
                    'significant': corrected_p < self.alpha,
                    'emotional_mean': emotional_data.mean(),
                    'emotionless_mean': emotionless_data.mean(),
                    'emotional_std': emotional_data.std(),
                    'emotionless_std': emotionless_data.std(),
                    'emotional_n': len(emotional_data),
                    'emotionless_n': len(emotionless_data),
                    'test_type': test_type
                }
                
            except Exception as e:
                print(f"Error in pairwise comparison for {emotion}: {str(e)}")
                continue
        
        return results
    
    def calculate_descriptive_statistics(self, df: pd.DataFrame) -> Dict:
        """
        Calculate descriptive statistics by condition
        
        Args:
            df: Prepared data for analysis
            
        Returns:
            Descriptive statistics
        """
        # Overall statistics by voice condition
        voice_stats = df.groupby('voice_condition')['target_emotion_intensity'].agg([
            'count', 'mean', 'std', 'median', 'min', 'max'
        ]).round(4)
        
        # Statistics by emotion and voice condition
        emotion_voice_stats = df.groupby(['target_emotion', 'voice_condition'])['target_emotion_intensity'].agg([
            'count', 'mean', 'std', 'median', 'min', 'max'
        ]).round(4)
        
        # Classification accuracy by condition
        accuracy_stats = df.groupby(['target_emotion', 'voice_condition'])['classification_correct'].agg([
            'count', 'sum', 'mean'
        ]).round(4)
        accuracy_stats['accuracy_percent'] = (accuracy_stats['mean'] * 100).round(2)
        
        return {
            'voice_condition_stats': voice_stats,
            'emotion_voice_stats': emotion_voice_stats,
            'classification_accuracy': accuracy_stats
        }
    
    def test_assumptions(self, df: pd.DataFrame) -> Dict:
        """
        Test statistical assumptions for ANOVA
        
        Args:
            df: Prepared data for analysis
            
        Returns:
            Assumption test results
        """
        results = {}
        
        # Test normality for each group
        normality_results = {}
        for emotion in EMOTIONS:
            for voice in ['Emotional', 'Emotionless']:
                group_data = df[(df['target_emotion'] == emotion) & 
                              (df['voice_condition'] == voice)]['target_emotion_intensity']
                
                if len(group_data) > 3:  # Need at least 4 observations
                    stat, p_value = normaltest(group_data)
                    normality_results[f'{emotion}_{voice}'] = {
                        'statistic': stat,
                        'p_value': p_value,
                        'normal': p_value > self.alpha
                    }
        
        results['normality'] = normality_results
        
        # Test homogeneity of variance (Levene's test)
        try:
            groups = []
            for emotion in EMOTIONS:
                emotional_group = df[(df['target_emotion'] == emotion) & 
                                   (df['voice_condition'] == 'Emotional')]['target_emotion_intensity']
                emotionless_group = df[(df['target_emotion'] == emotion) & 
                                     (df['voice_condition'] == 'Emotionless')]['target_emotion_intensity']
                
                if len(emotional_group) > 0 and len(emotionless_group) > 0:
                    groups.extend([emotional_group, emotionless_group])
            
            if len(groups) > 1:
                levene_stat, levene_p = levene(*groups)
                results['homogeneity'] = {
                    'levene_statistic': levene_stat,
                    'p_value': levene_p,
                    'homogeneous': levene_p > self.alpha
                }
        except Exception as e:
            results['homogeneity'] = {'error': str(e)}
        
        return results
    
    def _interpret_effect_size(self, cohens_d: float) -> str:
        """Interpret Cohen's d effect size"""
        if cohens_d < self.effect_size_thresholds['small']:
            return 'negligible'
        elif cohens_d < self.effect_size_thresholds['medium']:
            return 'small'
        elif cohens_d < self.effect_size_thresholds['large']:
            return 'medium'
        else:
            return 'large'
    
    def generate_statistical_summary(self, df: pd.DataFrame) -> Dict:
        """
        Generate complete statistical analysis summary
        
        Args:
            df: Prepared data for analysis
            
        Returns:
            Complete statistical analysis results
        """
        print("Performing statistical analysis...")
        
        # Prepare data
        clean_df = self.prepare_data_for_analysis(df)
        
        # Descriptive statistics
        descriptive_stats = self.calculate_descriptive_statistics(clean_df)
        
        # Test assumptions
        assumptions = self.test_assumptions(clean_df)
        
        # ANOVA
        anova_results = self.perform_repeated_measures_anova(clean_df)
        
        # Pairwise comparisons
        pairwise_results = self.perform_pairwise_comparisons(clean_df)
        
        summary = {
            'sample_size': len(clean_df),
            'participants': clean_df['participant'].nunique(),
            'descriptive_statistics': descriptive_stats,
            'assumption_tests': assumptions,
            'anova_results': anova_results,
            'pairwise_comparisons': pairwise_results,
            'analysis_parameters': {
                'alpha': self.alpha,
                'bonferroni_correction': self.bonferroni_correction,
                'effect_size_thresholds': self.effect_size_thresholds
            }
        }
        
        return summary


def run_statistical_analysis(results_df: pd.DataFrame) -> Dict:
    """
    Convenience function to run complete statistical analysis
    
    Args:
        results_df: DataFrame with emotion analysis results
        
    Returns:
        Complete statistical analysis results
    """
    analyzer = StatisticalAnalyzer()
    return analyzer.generate_statistical_summary(results_df)


if __name__ == "__main__":
    # Test with sample data
    print("Statistical analyzer module loaded successfully")
