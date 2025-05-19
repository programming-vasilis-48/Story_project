#!/bin/bash

# Script to check if the Sennheiser microphone is recognized

echo "Checking Sennheiser microphone..."

# Check if the Sennheiser device is connected
if lsusb | grep -i "Sennheiser" > /dev/null; then
    echo "✅ Sennheiser microphone is connected."
else
    echo "❌ Sennheiser microphone is NOT detected."
fi

# List all audio devices
echo -e "\nListing all audio devices:"
arecord -l

# List all PulseAudio sources
echo -e "\nListing all PulseAudio sources:"
pactl list sources | grep -E 'Name:|Description:'

echo -e "\nDone checking Sennheiser microphone."
