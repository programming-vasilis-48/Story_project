#!/bin/bash

# Script to install dependencies for direct speech recognition

echo "Installing dependencies for direct speech recognition..."

# Update package lists
sudo apt-get update

# Install system dependencies
sudo apt-get install -y \
    python3-pip \
    python3-pyaudio \
    portaudio19-dev \
    libpulse-dev \
    libasound2-dev

# Install Python dependencies
pip3 install SpeechRecognition

echo "Installation complete!"
