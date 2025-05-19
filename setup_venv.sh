#!/bin/bash

# This script sets up a Python virtual environment for the Story-Switch application

# Create a virtual environment
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate

# Install the required Python dependencies
pip install numpy scipy matplotlib pandas scikit-learn opencv-python pyfeat pyaudio webrtcvad SpeechRecognition pyttsx3

echo "Virtual environment setup complete. To activate the virtual environment, run:"
echo "source venv/bin/activate"
