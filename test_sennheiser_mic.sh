#!/bin/bash

# Script to test recording from the Sennheiser microphone

echo "Testing Sennheiser microphone recording..."

# Create a directory for test recordings if it doesn't exist
mkdir -p ~/mic_tests

# Record 5 seconds of audio
echo "Recording 5 seconds of audio. Please speak into the microphone..."
arecord -d 5 -f cd -t wav ~/mic_tests/test_recording.wav

# Play back the recording
echo -e "\nPlaying back the recording..."
aplay ~/mic_tests/test_recording.wav

echo -e "\nRecording saved to ~/mic_tests/test_recording.wav"
echo "Done testing Sennheiser microphone."
