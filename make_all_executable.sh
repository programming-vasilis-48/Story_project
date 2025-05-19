#!/bin/bash

# Make all scripts executable

echo "Making all scripts executable..."

# Make test scripts executable
chmod +x test_llm_api.py
chmod +x test_tts.py
chmod +x test_speech_recognition.py
chmod +x test_vision_au_pyfeat.py
chmod +x test_vision_au_simple.py
chmod +x test_speech_recognition_direct.py

# Make run scripts executable
chmod +x run_all_tests.py
chmod +x run_all_tests_direct.py

# Make utility scripts executable
chmod +x check_ros_topics.py
chmod +x monitor_respeaker.py
chmod +x debug_speech_recognition.py
chmod +x check_respeaker_service.sh
chmod +x test_microphone.py
chmod +x test_respeaker.py
chmod +x check_sennheiser_mic.sh
chmod +x test_sennheiser_mic.sh
chmod +x configure_sennheiser_mic.sh

# Make installation scripts executable
chmod +x install_speech_recognition_deps.sh
chmod +x install_pyfeat_deps.sh
chmod +x install_dependencies.sh

echo "Done! All scripts are now executable."
