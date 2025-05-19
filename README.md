# Story-Switch Application for QTrobot

This repository contains the code for the Story-Switch application, a collaborative, turn-based storytelling application for the QTrobot RD-V2 i7 platform.

## Project Structure

```
.
├── README.md                           # This file
├── run_all_tests.py                    # Main test runner for all components
├── test_llm_api.py                     # Test for LLM API connection
├── test_tts.py                         # Test for Text-to-Speech functionality
├── test_speech_recognition_direct.py   # Test for Speech Recognition using Sennheiser mic
├── test_vision_au_direct.py            # Test for Vision/AU Detection using PyFeat
├── test_vision_only.py                 # Standalone Vision test (no speech)
├── install_dependencies.sh             # Script to install all dependencies
├── install_pyfeat_deps.sh              # Script to install PyFeat dependencies
├── install_speech_recognition_deps.sh  # Script to install speech recognition deps
├── configure_sennheiser_mic.sh         # Script to configure the Sennheiser mic
├── check_sennheiser_mic.sh             # Script to check the Sennheiser mic
├── test_sennheiser_mic.sh              # Script to test the Sennheiser mic
├── requirements.txt                    # Python dependencies
├── src/                                # ROS workspace source directory
│   └── story_switch/                   # ROS package for the Story-Switch application
│       ├── CMakeLists.txt              # CMake build configuration
│       ├── package.xml                 # ROS package metadata
│       ├── launch/                     # ROS launch files
│       │   └── story_switch.launch     # Main launch file
│       └── src/                        # Source code
│           ├── nodes/                  # ROS nodes
│           │   ├── story_manager_node.py  # Story Manager node
│           │   ├── speech_synth_node.py   # Speech Synthesis node
│           │   ├── speech_recog_node.py   # Speech Recognition node
│           │   └── vision_track_node.py   # Vision Tracking node
│           ├── test_llm_client.py      # Test for LLM client
│           └── utils/                  # Utility modules
│               └── llm_client.py       # LLM client implementation
├── old_face_detection_for_confusion/   # Reference implementation for face detection
└── story_switch_plan.md                # Project plan
```

## Development Environment Setup

### Using a Virtual Environment

1. Install ROS Noetic on your system.
2. Install the required dependencies:

```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

3. Install PyFeat dependencies:

```bash
chmod +x install_pyfeat_deps.sh
./install_pyfeat_deps.sh
```

4. Install speech recognition dependencies:

```bash
chmod +x install_speech_recognition_deps.sh
./install_speech_recognition_deps.sh
```

5. Configure the Sennheiser microphone (if available):

```bash
chmod +x configure_sennheiser_mic.sh
./configure_sennheiser_mic.sh
```

6. Build the ROS workspace:

```bash
cd src
catkin_make
source devel/setup.bash
```

7. Run the ROS nodes:

```bash
roslaunch story_switch story_switch.launch
```

## Testing

The repository includes several test scripts to verify the functionality of different components:

1. Test all components:

```bash
./run_all_tests.py
```

2. Test individual components:

```bash
./test_llm_api.py                   # Test LLM API
./test_tts.py                       # Test Text-to-Speech
./test_speech_recognition_direct.py # Test Speech Recognition
./test_vision_au_direct.py          # Test Vision/AU Detection
./test_vision_only.py               # Test Vision (no speech)
```

## Deployment to QTrobot

To deploy the application to the QTrobot:

1. Copy the ROS package to the QTrobot's ROS workspace:

```bash
cp -r src/story_switch ~/catkin_ws/src/
```

2. Build the workspace on the QTrobot:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Run the application using the launch file:

```bash
roslaunch story_switch story_switch.launch
```

## License

MIT
