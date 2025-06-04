#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for the logging and video recording functionality.

This script tests the SessionLogger and VideoRecorder modules
to ensure they work correctly before running the main game.
"""

import os
import sys
import time

# Add the modules directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from modules.logging import SessionLogger
from modules.video_recording import VideoRecorder


def test_session_logger():
    """Test the SessionLogger functionality."""
    print("=" * 60)
    print("TESTING SESSION LOGGER")
    print("=" * 60)

    try:
        # Initialize logger
        logger = SessionLogger("Test_User", "Test Voice Type")
        print("✓ SessionLogger initialized successfully")

        # Test logging robot speech
        logger.log_robot_speech("Hello, this is a test message.", "happy", 2.5)
        print("✓ Robot speech logged")

        # Test logging user speech
        logger.log_user_speech("This is a user response.", 1.8)
        print("✓ User speech logged")

        # Test video logging
        video_filename = logger.log_video_start("angry")
        print(f"✓ Video start logged: {video_filename}")

        time.sleep(1)  # Simulate some recording time

        logger.log_video_stop(video_filename)
        print("✓ Video stop logged")

        # Test session summary
        summary = logger.get_session_summary()
        print(f"✓ Session summary: {summary}")

        # Save logs
        logger.save_session_log()
        print("✓ Session logs saved")

        return True

    except Exception as e:
        print(f"✗ SessionLogger test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_video_recorder():
    """Test the VideoRecorder functionality."""
    print("\n" + "=" * 60)
    print("TESTING VIDEO RECORDER (ROS-based)")
    print("=" * 60)

    try:
        # Initialize video recorder
        recorder = VideoRecorder()
        print("✓ VideoRecorder initialized")

        # Wait for ROS subscription to establish
        print("Waiting for ROS camera subscription...")
        time.sleep(2.0)

        # Check camera availability
        if recorder.is_camera_available():
            print("✓ ROS camera is available")

            # Get camera info
            info = recorder.get_camera_info()
            print(f"✓ Camera info: {info}")

            # Test camera
            if recorder.test_camera():
                print("✓ Camera test successful")

                # Test short recording
                test_video_path = os.path.join("logs", "test_video.mp4")
                os.makedirs(os.path.dirname(test_video_path), exist_ok=True)

                print("Starting 3-second test recording...")
                if recorder.start_recording(test_video_path):
                    print("✓ Recording started")
                    time.sleep(3)  # Record for 3 seconds

                    if recorder.stop_recording():
                        print("✓ Recording stopped")

                        # Check if file was created
                        if os.path.exists(test_video_path):
                            file_size = os.path.getsize(test_video_path)
                            print(f"✓ Video file created: {file_size} bytes")

                            # Clean up test file
                            os.remove(test_video_path)
                            print("✓ Test file cleaned up")
                        else:
                            print("✗ Video file was not created")
                            return False
                    else:
                        print("✗ Failed to stop recording")
                        return False
                else:
                    print("✗ Failed to start recording")
                    return False
            else:
                print("✗ Camera test failed")
                return False
        else:
            print("⚠ ROS camera not available - skipping video tests")
            print("This is normal if QT Robot camera is not connected or ROS is not running")

        # Cleanup
        recorder.cleanup()
        print("✓ VideoRecorder cleaned up")

        return True

    except Exception as e:
        print(f"✗ VideoRecorder test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_integration():
    """Test integration between logger and video recorder."""
    print("\n" + "=" * 60)
    print("TESTING INTEGRATION")
    print("=" * 60)

    try:
        # Initialize both components
        logger = SessionLogger("Integration_Test", "Test Voice")
        recorder = VideoRecorder()

        print("✓ Both components initialized")

        # Wait for ROS subscription
        print("Waiting for ROS camera subscription...")
        time.sleep(2.0)

        if recorder.is_camera_available():
            # Simulate a complete exchange
            print("Simulating complete exchange...")

            # Start video recording
            video_filename = logger.log_video_start("surprise")
            video_path = logger.get_video_path(video_filename)

            if recorder.start_recording(video_path):
                print("✓ Video recording started")

                # Log robot speech
                logger.log_robot_speech("This is a test robot message.", "surprise", 2.0)

                time.sleep(2)  # Simulate robot speaking

                # Log user speech
                logger.log_user_speech("This is a test user response.", 1.5)

                time.sleep(1)  # Simulate user speaking

                # Stop recording
                if recorder.stop_recording():
                    logger.log_video_stop(video_filename)
                    print("✓ Video recording stopped and logged")

                    # Check file
                    if os.path.exists(video_path):
                        file_size = os.path.getsize(video_path)
                        print(f"✓ Integration test video: {file_size} bytes")

                        # Clean up
                        os.remove(video_path)
                    else:
                        print("✗ Integration test video file not found")
                        return False
                else:
                    print("✗ Failed to stop recording in integration test")
                    return False
            else:
                print("✗ Failed to start recording in integration test")
                return False
        else:
            print("⚠ ROS camera not available - skipping integration video test")

        # Save logs
        logger.save_session_log()
        print("✓ Integration test logs saved")

        # Cleanup
        recorder.cleanup()
        print("✓ Integration test cleanup complete")

        return True

    except Exception as e:
        print(f"✗ Integration test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("TESTING LOGGING AND VIDEO RECORDING FUNCTIONALITY")
    print("=" * 60)

    results = []

    # Test session logger
    results.append(test_session_logger())

    # Test video recorder
    results.append(test_video_recorder())

    # Test integration
    results.append(test_integration())

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(results)
    total = len(results)

    print(f"Tests passed: {passed}/{total}")

    if passed == total:
        print("✓ All tests passed! Logging and video recording are ready.")
        return 0
    else:
        print("✗ Some tests failed. Please check the errors above.")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
