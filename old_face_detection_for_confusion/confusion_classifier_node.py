#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Confusion Classifier Node for QTrobot Confusion Detection System

This node classifies whether a human is confused based on processed facial features.
"""

import rospy
import numpy as np
from collections import deque
from confusion_detection.msg import FaceFeatures, ConfusionState

class ConfusionClassifierNode:
    """ROS node for classifying confusion from facial features."""

    def __init__(self):
        """Initialize the confusion classifier node."""
        rospy.init_node('confusion_classifier_node', anonymous=True)

        # Parameters
        self.classification_frequency = rospy.get_param('~classification_frequency', 5)  # Hz
        self.confusion_threshold = rospy.get_param('~confusion_threshold', 0.6)
        self.min_confidence = rospy.get_param('~min_confidence', 0.7)
        self.model_path = rospy.get_param('~model_path', 'models/confusion_detection/confusion_classifier_model.onnx')
        self.temporal_window = rospy.get_param('~temporal_window', 3.0)  # seconds

        # Load confusion classification model
        self.load_confusion_classification_model()

        # State variables
        self.feature_history = deque(maxlen=int(self.temporal_window * self.classification_frequency))
        self.confusion_start_time = None
        self.is_confused = False

        # Publishers and subscribers
        self.confusion_state_pub = rospy.Publisher('/human/confusion_state', ConfusionState, queue_size=10)
        self.processed_features_sub = rospy.Subscriber('/vision/processed_features', FaceFeatures, self.features_callback)

        rospy.loginfo("Confusion classifier node initialized")

    def load_confusion_classification_model(self):
        """Load the confusion classification model."""
        try:
            # This is a placeholder for the actual model loading code
            # In a real implementation, this would load a model for classifying
            # confusion from facial features
            rospy.loginfo(f"Loading confusion classification model from {self.model_path}")

            # Placeholder for model loading
            # self.confusion_classifier = ...

            rospy.loginfo("Confusion classification model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load confusion classification model: {e}")

    def features_callback(self, data):
        """Process incoming processed facial features."""
        try:
            # Add the features to the history
            self.feature_history.append(data)

            # Classify confusion
            self.classify_confusion()

        except Exception as e:
            rospy.logerr(f"Error classifying confusion: {e}")

    def classify_confusion(self):
        """Classify whether the human is confused based on the feature history."""
        # In a real implementation, this would use the loaded model to classify
        # confusion from the feature history

        # For now, we'll use a simple rule-based approach
        if len(self.feature_history) == 0:
            return

        # Extract action units from the most recent features
        latest_features = self.feature_history[-1]
        action_units = latest_features.action_units

        # Check if the action units associated with confusion are high
        # AU04 (Brow Lowerer), AU07 (Lid Tightener), AU14 (Dimpler), AU24 (Lip Pressor)
        confusion_score = 0.0
        confidence = 0.0

        if len(action_units) >= 18:
            # Calculate confusion score based on action units
            confusion_score = (
                action_units[2] * 0.4 +  # AU04 (Brow Lowerer)
                action_units[5] * 0.2 +  # AU07 (Lid Tightener)
                action_units[9] * 0.2 +  # AU14 (Dimpler)
                action_units[14] * 0.2   # AU24 (Lip Pressor)
            )

            # Calculate confidence based on the consistency of the action units
            if len(self.feature_history) > 1:
                prev_action_units = self.feature_history[-2].action_units
                if len(prev_action_units) >= 18:
                    consistency = 1.0 - min(1.0, sum(abs(a - b) for a, b in zip(action_units, prev_action_units)) / len(action_units))
                    confidence = 0.7 + 0.3 * consistency
                else:
                    confidence = 0.7
            else:
                confidence = 0.7

        # Determine if the human is confused
        is_confused = confusion_score >= self.confusion_threshold and confidence >= self.min_confidence

        # Update confusion state
        current_time = rospy.get_time()
        if is_confused and not self.is_confused:
            # Confusion started
            self.confusion_start_time = current_time
            self.is_confused = True
        elif not is_confused and self.is_confused:
            # Confusion ended
            self.confusion_start_time = None
            self.is_confused = False

        # Calculate confusion duration
        confusion_duration = 0.0
        if self.is_confused and self.confusion_start_time is not None:
            confusion_duration = current_time - self.confusion_start_time

        # Publish confusion state
        self.publish_confusion_state(is_confused, confusion_score, confidence, confusion_duration, action_units)

    def publish_confusion_state(self, is_confused, confusion_score, confidence, confusion_duration, action_units):
        """Publish the confusion state."""
        # Create a ConfusionState message
        confusion_state = ConfusionState()

        # Set header
        confusion_state.header.stamp = rospy.Time.now()
        confusion_state.header.frame_id = "face"

        # Set confusion state
        confusion_state.is_confused = is_confused
        confusion_state.confusion_score = confusion_score
        confusion_state.confidence = confidence
        confusion_state.duration = confusion_duration
        confusion_state.action_units = action_units

        # Set head pose and eye gaze (placeholder)
        confusion_state.head_pose = [0.0, 0.0, 0.0]
        confusion_state.eye_gaze = [0.0, 0.0, 0.0]

        # Publish the message
        self.confusion_state_pub.publish(confusion_state)

    def run(self):
        """Run the confusion classifier node."""
        rate = rospy.Rate(self.classification_frequency)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        classifier = ConfusionClassifierNode()
        classifier.run()
    except rospy.ROSInterruptException:
        pass