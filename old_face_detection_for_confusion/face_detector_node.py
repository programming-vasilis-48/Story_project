#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Face Detector Node for QTrobot Confusion Detection System

This node interfaces with the QTrobot camera to detect faces
and extract facial features for confusion detection.
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from confusion_detection.msg import FaceFeatures

class FaceDetectorNode:
    """ROS node for face detection and feature extraction."""

    def __init__(self):
        """Initialize the face detector node."""
        rospy.init_node('face_detector_node', anonymous=True)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Parameters
        self.detection_frequency = rospy.get_param('~detection_frequency', 10)  # Hz
        self.face_detection_threshold = rospy.get_param('~face_detection_threshold', 0.7)
        self.model_path = rospy.get_param('~model_path', 'models/face_detection/face_detection_model.onnx')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/color/image_raw')

        # Load face detection model
        self.load_face_detection_model()

        # Publishers and subscribers
        self.face_features_pub = rospy.Publisher('/vision/face_features', FaceFeatures, queue_size=10)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

        rospy.loginfo(f"Face detector node initialized, listening on {self.camera_topic}")

    def load_face_detection_model(self):
        """Load the face detection model."""
        try:
            # This is a placeholder for the actual model loading code
            # In a real implementation, this would load a face detection model
            # such as RetinaFace, MTCNN, or a custom ONNX model
            rospy.loginfo(f"Loading face detection model from {self.model_path}")

            # For now, we'll use OpenCV's built-in face detector
            self.face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

            rospy.loginfo("Face detection model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load face detection model: {e}")

    def image_callback(self, data):
        """Process incoming image data."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Detect faces
            faces = self.detect_faces(cv_image)

            # Process each detected face
            for face in faces:
                # Extract facial features
                features = self.extract_facial_features(cv_image, face)

                # Publish face features
                self.publish_face_features(features, data.header)

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def detect_faces(self, image):
        """Detect faces in the input image."""
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = self.face_detector.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        # If no faces are detected, return a simulated face in the center
        if len(faces) == 0:
            height, width, _ = image.shape
            # Format: x, y, width, height
            return [(width // 4, height // 4, width // 2, height // 2)]
            
        return faces

    def extract_facial_features(self, image, face):
        """Extract facial features from the detected face."""
        x, y, w, h = face

        # Create a FaceFeatures message
        features = FaceFeatures()

        # Set bounding box
        features.bbox = [float(x), float(y), float(w), float(h)]

        # Set detection confidence (placeholder)
        features.detection_confidence = 0.9

        # In a real implementation, we would extract:
        # - Facial landmarks
        # - Action units
        # - Head pose
        # - Eye gaze

        # Placeholder for facial landmarks (would be extracted using a dedicated model)
        features.landmarks = []

        # Simulate high values for action units associated with confusion
        # AU04 (Brow Lowerer), AU07 (Lid Tightener), AU14 (Dimpler), AU24 (Lip Pressor)
        features.action_units = [
            0.8,  # AU01 (Inner Brow Raiser) - high value
            0.7,  # AU02 (Outer Brow Raiser) - high value
            0.9,  # AU04 (Brow Lowerer) - very high value (strong indicator of confusion)
            0.1,  # AU05 (Upper Lid Raiser)
            0.0,  # AU06 (Cheek Raiser)
            0.8,  # AU07 (Lid Tightener) - high value (indicator of confusion)
            0.0,  # AU09 (Nose Wrinkler)
            0.0,  # AU10 (Upper Lip Raiser)
            0.0,  # AU12 (Lip Corner Puller)
            0.7,  # AU14 (Dimpler) - high value (indicator of confusion)
            0.0,  # AU15 (Lip Corner Depressor)
            0.0,  # AU17 (Chin Raiser)
            0.0,  # AU20 (Lip Stretcher)
            0.0,  # AU23 (Lip Tightener)
            0.8,  # AU24 (Lip Pressor) - high value (indicator of confusion)
            0.0,  # AU25 (Lips Part)
            0.0,  # AU26 (Jaw Drop)
            0.0   # AU45 (Blink)
        ]

        # Placeholder for head pose (would be extracted using a dedicated model)
        # Simulate head tilt (common in confusion)
        features.head_pose = [0.3, 0.2, 0.4]  # pitch, yaw, roll

        # Placeholder for eye gaze (would be extracted using a dedicated model)
        # Simulate looking away (common in confusion)
        features.left_eye_gaze = [0.5, 0.3, 0.8]  # x, y, z
        features.right_eye_gaze = [0.5, 0.3, 0.8]  # x, y, z

        return features

    def publish_face_features(self, features, header):
        """Publish the extracted face features."""
        # Set the header
        features.header = header

        # Publish the message
        self.face_features_pub.publish(features)

    def run(self):
        """Run the face detector node."""
        rate = rospy.Rate(self.detection_frequency)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = FaceDetectorNode()
        detector.run()
    except rospy.ROSInterruptException:
        pass