#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to monitor ReSpeaker topics on QTrobot.
This script helps debug if the ReSpeaker microphone is publishing data.
"""

import rospy
from std_msgs.msg import String
import time
import threading

class ReSpeakerMonitor:
    """Class to monitor ReSpeaker topics."""
    
    def __init__(self):
        """Initialize the monitor."""
        # Initialize ROS node
        rospy.init_node('respeaker_monitor', anonymous=True)
        
        # Variables to track messages
        self.sound_direction_received = False
        self.is_speaking_received = False
        self.sound_direction_count = 0
        self.is_speaking_count = 0
        self.last_sound_direction = None
        self.last_is_speaking = None
        
        # Subscribe to ReSpeaker topics
        rospy.Subscriber('/qt_respeaker_app/sound_direction', String, self.sound_direction_callback)
        rospy.Subscriber('/qt_respeaker_app/is_speaking', String, self.is_speaking_callback)
        
        print("ReSpeaker monitor initialized. Listening for messages...")
    
    def sound_direction_callback(self, msg):
        """Callback for sound direction messages."""
        self.sound_direction_received = True
        self.sound_direction_count += 1
        self.last_sound_direction = msg.data
        print(f"Sound direction message received: {msg.data} (Total: {self.sound_direction_count})")
    
    def is_speaking_callback(self, msg):
        """Callback for is_speaking messages."""
        self.is_speaking_received = True
        self.is_speaking_count += 1
        self.last_is_speaking = msg.data
        print(f"Is speaking message received: {msg.data} (Total: {self.is_speaking_count})")
    
    def run(self, duration=30):
        """Run the monitor for a specified duration.
        
        Args:
            duration (int): Duration to monitor in seconds.
        """
        print(f"Monitoring ReSpeaker topics for {duration} seconds...")
        print("Please speak during this time to test if the microphone is detecting sound.")
        
        # Monitor for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(1)
            
            # Print a dot every second to show the script is running
            sys.stdout.write(".")
            sys.stdout.flush()
        
        print("\n\nMonitoring complete!")
        
        # Print summary
        print("\nSummary:")
        print(f"  Sound direction messages: {self.sound_direction_count}")
        print(f"  Is speaking messages: {self.is_speaking_count}")
        
        if self.sound_direction_count == 0 and self.is_speaking_count == 0:
            print("\nNo messages received from ReSpeaker. Possible issues:")
            print("  1. ReSpeaker microphone is not connected or not working")
            print("  2. ReSpeaker ROS node is not running")
            print("  3. ReSpeaker topics have different names")
        elif self.sound_direction_count == 0:
            print("\nNo sound direction messages received. Possible issues:")
            print("  1. Sound direction detection is disabled")
            print("  2. Sound was not loud enough to trigger direction detection")
        elif self.is_speaking_count == 0:
            print("\nNo is_speaking messages received. Possible issues:")
            print("  1. Voice activity detection is disabled")
            print("  2. Sound was not recognized as speech")
        else:
            print("\nBoth topics are working correctly!")

if __name__ == "__main__":
    import sys
    
    # Default monitoring duration
    duration = 30
    
    # Check if duration is provided as command line argument
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print(f"Invalid duration: {sys.argv[1]}. Using default: {duration} seconds.")
    
    try:
        monitor = ReSpeakerMonitor()
        monitor.run(duration)
    except rospy.ROSInterruptException:
        print("Monitoring interrupted!")
    except Exception as e:
        print(f"Error during monitoring: {str(e)}")
