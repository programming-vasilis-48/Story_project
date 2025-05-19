#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Feature Extractor Node for QTrobot Confusion Detection System

This node processes facial features to extract higher-level features
for confusion detection.
"""

import rospy
import numpy as np
from confusion_detection.msg import FaceFeatures

class FeatureExtractorNode:
    """ROS node for processing facial features."""

    def __init__(self):
        """Initialize the feature extractor node."""
        rospy.init_node('feature_extractor_node', anonymous=True)

        # Parameters
        self.processing_frequency = rospy.get_param('~processing_frequency', 10)  # Hz
        self.model_path = rospy.get_param('~model_path', 'models/face_detection/au_detection_model.onnx')

        # Load feature extraction model
        self.load_feature_extraction_model()

        # Publishers and subscribers
        self.processed_features_pub = rospy.Publisher('/vision/processed_features', FaceFeatures, queue_size=10)
        self.face_features_sub = rospy.Subscriber('/vision/face_features', FaceFeatures, self.features_callback)

        rospy.loginfo("Feature extractor node initialized")

    def load_feature_extraction_model(self):
        """Load the feature extraction model."""
        try:
            # This is a placeholder for the actual model loading code
            # In a real implementation, this would load a model for extracting
            # higher-level features from facial features
            rospy.loginfo(f"Loading feature extraction model from {self.model_path}")

            # Placeholder for model loading
            # self.feature_extractor = ...

            rospy.loginfo("Feature extraction model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load feature extraction model: {e}")

    def features_callback(self, data):
        """Process incoming facial features."""
        try:
            # Process the facial features
            processed_features = self.process_features(data)

            # Publish the processed features
            self.processed_features_pub.publish(processed_features)

        except Exception as e:
            rospy.logerr(f"Error processing features: {e}")

    def process_features(self, features):
        """Process facial features to extract higher-level features."""
        # In a real implementation, this would use the loaded model to extract
        # higher-level features from the facial features

        # For now, we'll just pass through the features with some minor adjustments
        processed_features = features

        # Adjust action units based on domain knowledge about confusion
        # For example, increase the weight of AU04 (Brow Lowerer) which is strongly
        # associated with confusion
        if len(processed_features.action_units) >= 3:
            processed_features.action_units[2] = min(1.0, processed_features.action_units[2] * 1.2)  # AU04

        return processed_features

    def run(self):
        """Run the feature extractor node."""
        rate = rospy.Rate(self.processing_frequency)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        extractor = FeatureExtractorNode()
        extractor.run()
    except rospy.ROSInterruptException:
        pass