#!/bin/bash

# Script to clean up the codebase by removing unwanted files

echo "Cleaning up the codebase..."

# Remove Docker-related files
echo "Removing Docker-related files..."
rm -f Dockerfile
rm -f docker-compose.yml
rm -f setup_venv.sh

# Remove unwanted test files
echo "Removing unwanted test files..."
rm -f test_vision_au.py
rm -f test_vision_au_pyfeat.py
rm -f test_vision_au_pyfeat_simple.py
rm -f test_speech_recognition.py
rm -f run_all_tests.py

# Remove unused test helper scripts
echo "Removing unused test helper scripts..."
rm -f check_respeaker_service.sh
rm -f check_ros_topics.py
rm -f monitor_respeaker.py
rm -f debug_speech_recognition.py
rm -f test_microphone.py
rm -f test_respeaker.py
rm -f make_scripts_executable.sh
rm -f make_au_scripts_executable.sh
rm -f make_direct_tests_executable.sh
rm -f make_direct_test_executable.sh
rm -f make_sennheiser_scripts_executable.sh
rm -f make_vision_only_executable.sh
rm -f make_all_executable.sh
rm -f QTROBOT_TESTS_README.md
rm -f TESTS_README.md

# Rename the main test runner
echo "Renaming the main test runner..."
mv run_all_tests_direct.py run_all_tests.py

# Update the renamed test runner to reflect the new name
sed -i 's/run_all_tests_direct/run_all_tests/g' run_all_tests.py

echo "Cleanup complete!"
echo "The following files have been kept:"
echo "- test_llm_api.py - LLM API test"
echo "- test_tts.py - Text-to-Speech test"
echo "- test_speech_recognition_direct.py - Direct Speech Recognition test"
echo "- test_vision_au_direct.py - Direct Vision/AU Detection test"
echo "- test_vision_only.py - Standalone Vision test"
echo "- run_all_tests.py (renamed from run_all_tests_direct.py) - Main test runner"
echo "- install_dependencies.sh - Script to install dependencies"
echo "- install_pyfeat_deps.sh - Script to install PyFeat dependencies"
echo "- install_speech_recognition_deps.sh - Script to install speech recognition dependencies"
echo "- configure_sennheiser_mic.sh - Script to configure the Sennheiser microphone"
echo "- check_sennheiser_mic.sh - Script to check the Sennheiser microphone"
echo "- test_sennheiser_mic.sh - Script to test the Sennheiser microphone"
