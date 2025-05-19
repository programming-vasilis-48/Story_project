#!/bin/bash

# Script to check if the ReSpeaker service is running on QTrobot

echo "Checking ReSpeaker service status..."

# Check if the ReSpeaker ROS node is running
if pgrep -f "qt_respeaker_app" > /dev/null; then
    echo "✅ ReSpeaker ROS node is running."
else
    echo "❌ ReSpeaker ROS node is NOT running."
    echo "Try starting it with: roslaunch qt_respeaker_app qt_respeaker_app.launch"
fi

# Check if the ReSpeaker hardware is connected
echo -e "\nChecking ReSpeaker hardware..."
if lsusb | grep -i "ReSpeaker" > /dev/null; then
    echo "✅ ReSpeaker hardware is connected."
else
    echo "❌ ReSpeaker hardware is NOT detected."
    echo "Make sure the ReSpeaker microphone array is properly connected."
fi

# Check if the audio device is recognized
echo -e "\nChecking audio devices..."
arecord -l

# Check if the ReSpeaker topics are being published
echo -e "\nChecking ReSpeaker ROS topics..."
rostopic list | grep -i "respeaker"

# Check if the speech recognition service is available
echo -e "\nChecking speech recognition service..."
if rosservice list | grep "/qt_robot/speech/recognize" > /dev/null; then
    echo "✅ Speech recognition service is available."
else
    echo "❌ Speech recognition service is NOT available."
    echo "Try starting it with: roslaunch qt_robot_interface qt_robot_interface.launch"
fi

echo -e "\nDone checking ReSpeaker service."
