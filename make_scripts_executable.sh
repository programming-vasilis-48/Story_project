#!/bin/bash

# Make all debugging scripts executable

echo "Making debugging scripts executable..."

chmod +x check_ros_topics.py
chmod +x monitor_respeaker.py
chmod +x debug_speech_recognition.py
chmod +x check_respeaker_service.sh
chmod +x test_microphone.py
chmod +x test_respeaker.py

echo "Done! All scripts are now executable."
