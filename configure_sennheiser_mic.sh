#!/bin/bash

# Script to configure the QTrobot to use the Sennheiser microphone

echo "Configuring QTrobot to use the Sennheiser microphone..."

# Make sure the Sennheiser microphone is connected
if ! lsusb | grep -i "Sennheiser" > /dev/null; then
    echo "❌ Sennheiser microphone is NOT detected. Please connect it first."
    exit 1
fi

# Set the Sennheiser microphone as the default input device
echo "Setting Sennheiser microphone as the default input device..."

# Get the card and device number for the Sennheiser microphone
CARD_INFO=$(arecord -l | grep -i "Sennheiser")
if [ -z "$CARD_INFO" ]; then
    echo "❌ Could not find Sennheiser microphone in audio devices."
    echo "Trying to find any USB audio device..."
    CARD_INFO=$(arecord -l | grep -i "USB")
fi

if [ -z "$CARD_INFO" ]; then
    echo "❌ Could not find any USB audio device."
    echo "Please check if the microphone is properly connected."
    exit 1
fi

# Extract card number and device number
CARD_NUM=$(echo "$CARD_INFO" | sed -r 's/.*card ([0-9]+).*/\1/')
DEVICE_NUM=$(echo "$CARD_INFO" | sed -r 's/.*device ([0-9]+).*/\1/')

echo "Found Sennheiser microphone at card $CARD_NUM, device $DEVICE_NUM"

# Create or update .asoundrc file
echo "Creating .asoundrc file..."
cat > ~/.asoundrc << EOF
pcm.!default {
    type hw
    card $CARD_NUM
    device $DEVICE_NUM
}

ctl.!default {
    type hw
    card $CARD_NUM
}
EOF

echo "Created ~/.asoundrc file with the following content:"
cat ~/.asoundrc

# Set the default PulseAudio source
echo "Setting default PulseAudio source..."
PULSE_SOURCE=$(pactl list sources | grep -i "Sennheiser" -A 1 | grep "Name:" | cut -d: -f2 | tr -d ' ')

if [ -z "$PULSE_SOURCE" ]; then
    echo "❌ Could not find Sennheiser microphone in PulseAudio sources."
    echo "Trying to find any USB audio source..."
    PULSE_SOURCE=$(pactl list sources | grep -i "USB" -A 1 | grep "Name:" | cut -d: -f2 | tr -d ' ')
fi

if [ -z "$PULSE_SOURCE" ]; then
    echo "❌ Could not find any USB audio source in PulseAudio."
    echo "Will try to use the default source."
else
    echo "Setting $PULSE_SOURCE as default source..."
    pactl set-default-source "$PULSE_SOURCE"
    echo "Default source set to $PULSE_SOURCE"
fi

echo "Configuration complete!"
echo "Please restart any applications that use the microphone."
