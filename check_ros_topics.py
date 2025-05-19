#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to check available ROS topics on QTrobot.
This script helps debug which topics are available for speech recognition.
"""

import rospy
import time
import sys

def check_topics():
    """Check available ROS topics."""
    print("Checking available ROS topics...")
    
    # Initialize ROS node
    rospy.init_node('topic_checker', anonymous=True)
    
    # Wait for ROS to initialize
    time.sleep(2)
    
    # Get list of topics
    topics = rospy.get_published_topics()
    
    # Print all topics
    print("\nAll available topics:")
    for topic_name, topic_type in sorted(topics):
        print(f"  {topic_name} [{topic_type}]")
    
    # Check for specific topics
    respeaker_topics = [
        '/qt_respeaker_app/sound_direction',
        '/qt_respeaker_app/is_speaking',
        '/qt_respeaker_app/audio'
    ]
    
    print("\nChecking for ReSpeaker topics:")
    for topic in respeaker_topics:
        found = any(topic == t[0] for t in topics)
        status = "✅ Available" if found else "❌ Not available"
        print(f"  {topic}: {status}")
    
    # Check for speech recognition service
    print("\nChecking for speech recognition service:")
    try:
        rospy.wait_for_service('/qt_robot/speech/recognize', timeout=2)
        print("  /qt_robot/speech/recognize: ✅ Available")
    except rospy.ROSException:
        print("  /qt_robot/speech/recognize: ❌ Not available")
    
    return True

if __name__ == "__main__":
    try:
        check_topics()
    except rospy.ROSInterruptException:
        print("Check interrupted!")
    except Exception as e:
        print(f"Error during check: {str(e)}")
