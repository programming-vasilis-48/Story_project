#!/bin/bash

# Script to install dependencies for QTrobot tests

echo "Installing dependencies for QTrobot tests..."

# Update package lists
sudo apt-get update

# Install system dependencies
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    libportaudio2 \
    portaudio19-dev \
    libpulse-dev \
    libasound2-dev

# Install Python dependencies
pip3 install -r requirements.txt

# Make test scripts executable
chmod +x run_all_tests.py
chmod +x test_llm_api.py
chmod +x test_tts.py
chmod +x test_speech_recognition.py
chmod +x test_vision_au_pyfeat.py

echo "Installation complete!"
echo "To run all tests: ./run_all_tests.py"
echo "To run individual tests:"
echo "  - LLM API Test: ./test_llm_api.py"
echo "  - Text-to-Speech Test: ./test_tts.py"
echo "  - Speech Recognition Test: ./test_speech_recognition.py"
echo "  - Vision/AU Detection Test: ./test_vision_au_pyfeat.py"
