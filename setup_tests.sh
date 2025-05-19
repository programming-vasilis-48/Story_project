#!/bin/bash

# Setup script for QTrobot test scripts
# This script installs necessary dependencies and makes the test scripts executable

echo "Setting up QTrobot test scripts..."

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install python-dotenv requests

# Make test scripts executable
echo "Making test scripts executable..."
chmod +x test_llm_api.py
chmod +x test_tts.py
chmod +x test_speech_recognition.py
chmod +x test_vision_au.py
chmod +x test_vision_au_pyfeat.py
chmod +x run_all_tests.py

# Create .env file if it doesn't exist
if [ ! -f .env ]; then
    echo "Creating .env file template..."
    echo "OPENROUTER_API_KEY=your_api_key_here" > .env
    echo "MODEL=meta-llama/llama-3.3-8b-instruct:free" >> .env
    echo "Please edit the .env file and add your OpenRouter API key."
else
    echo ".env file already exists."
fi

echo "Setup complete!"
echo "To run all tests: ./run_all_tests.py"
echo "To run individual tests:"
echo "  - LLM API Test: ./test_llm_api.py"
echo "  - Text-to-Speech Test: ./test_tts.py"
echo "  - Speech Recognition Test: ./test_speech_recognition.py"
echo "  - Vision/AU Detection Test: ./test_vision_au.py"
