#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Main script to run all QTrobot tests in sequence, using direct speech recognition.
This script runs tests for LLM API, TTS, Direct Speech Recognition, and Vision/AU Detection.
"""

import os
import sys
import time
import rospy
from std_msgs.msg import String

# Import test modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import test_llm_api
import test_tts
import test_speech_recognition_direct  # Using direct speech recognition
import test_vision_au_simple  # Using simplified AU detection

def run_all_tests():
    """Run all QTrobot tests in sequence."""
    print("=" * 50)
    print("Starting QTrobot Component Tests (with Direct Speech Recognition)")
    print("=" * 50)

    # Initialize ROS node for announcements
    rospy.init_node('qtrobot_test_suite', anonymous=True)
    speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    time.sleep(1)  # Wait for publisher to connect

    # Announce start of tests
    speech_pub.publish("Starting QT Robot component tests with direct speech recognition.")
    time.sleep(3)

    # Track test results
    results = {}

    # Test 1: LLM API
    print("\n" + "=" * 50)
    print("Test 1: LLM API")
    print("=" * 50)
    speech_pub.publish("Testing LLM API connection.")
    time.sleep(2)

    try:
        llm_result = test_llm_api.test_llm_api()
        results["LLM API"] = llm_result

        if llm_result:
            speech_pub.publish("LLM API test successful.")
        else:
            speech_pub.publish("LLM API test failed.")
        time.sleep(2)
    except Exception as e:
        print(f"Error in LLM API test: {str(e)}")
        results["LLM API"] = False
        speech_pub.publish("LLM API test encountered an error.")
        time.sleep(2)

    # Test 2: Text-to-Speech
    print("\n" + "=" * 50)
    print("Test 2: Text-to-Speech")
    print("=" * 50)
    speech_pub.publish("Starting Text to Speech test.")
    time.sleep(2)

    try:
        # Pass existing_node=True to avoid ROS node initialization error
        tts_result = test_tts.test_tts(existing_node=True)
        results["Text-to-Speech"] = tts_result

        if tts_result:
            speech_pub.publish("Text to Speech test successful.")
        else:
            speech_pub.publish("Text to Speech test failed.")
        time.sleep(2)
    except Exception as e:
        print(f"Error in Text-to-Speech test: {str(e)}")
        results["Text-to-Speech"] = False
        speech_pub.publish("Text to Speech test encountered an error.")
        time.sleep(2)

    # Test 3: Direct Speech Recognition
    print("\n" + "=" * 50)
    print("Test 3: Direct Speech Recognition")
    print("=" * 50)
    speech_pub.publish("Starting Direct Speech Recognition test.")
    time.sleep(2)

    try:
        # Pass existing_node=True to avoid ROS node initialization error
        sr_test = test_speech_recognition_direct.DirectSpeechRecognitionTest(existing_node=True)
        sr_result = sr_test.run_test()
        results["Speech Recognition"] = sr_result

        if sr_result:
            speech_pub.publish("Direct Speech Recognition test successful.")
        else:
            speech_pub.publish("Direct Speech Recognition test failed.")
        time.sleep(2)
    except Exception as e:
        print(f"Error in Direct Speech Recognition test: {str(e)}")
        results["Speech Recognition"] = False
        speech_pub.publish("Direct Speech Recognition test encountered an error.")
        time.sleep(2)

    # Test 4: Vision/AU Detection with Simplified Method
    print("\n" + "=" * 50)
    print("Test 4: Vision and AU Detection (Simplified)")
    print("=" * 50)
    speech_pub.publish("Starting Vision and Action Unit detection test using OpenCV.")
    time.sleep(2)

    try:
        # Pass existing_node=True to avoid ROS node initialization error
        vision_test = test_vision_au_simple.SimpleVisionAUTest(existing_node=True)
        vision_result = vision_test.run_test()
        results["Vision/AU Detection"] = vision_result

        if vision_result:
            speech_pub.publish("Vision and Action Unit detection test successful.")
        else:
            speech_pub.publish("Vision and Action Unit detection test failed.")
        time.sleep(2)
    except Exception as e:
        print(f"Error in Vision/AU Detection test: {str(e)}")
        results["Vision/AU Detection"] = False
        speech_pub.publish("Vision and Action Unit detection test encountered an error.")
        time.sleep(2)

    # Print summary of results
    print("\n" + "=" * 50)
    print("Test Results Summary")
    print("=" * 50)

    all_passed = True
    for test_name, result in results.items():
        status = "PASSED ✅" if result else "FAILED ❌"
        print(f"{test_name}: {status}")
        if not result:
            all_passed = False

    # Announce final results
    if all_passed:
        speech_pub.publish("All tests have passed successfully.")
        print("\nAll tests passed successfully! ✅")
    else:
        speech_pub.publish("Some tests have failed. Please check the console for details.")
        print("\nSome tests failed. See details above. ❌")

    return all_passed

if __name__ == "__main__":
    try:
        run_all_tests()
    except rospy.ROSInterruptException:
        print("Tests interrupted!")
    except Exception as e:
        print(f"Error during tests: {str(e)}")
