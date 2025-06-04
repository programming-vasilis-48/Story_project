# Story-Switch Application for QTrobot

This repository contains the code for the Story-Switch application, a collaborative, turn-based storytelling application for the QTrobot RD-V2 i7 platform.

## Project Structure

```
.
├── README.md                           # This file
├── run_all_tests.py                    # Main test runner for all components
├── modules/                            # Main modules directory
│   ├── llm_api/                        # LLM API integration
│   │   ├── __init__.py
│   │   └── llm_client.py               # Client for LLM API calls
│   │
│   ├── mic_to_text/                    # Microphone input and speech recognition
│   │   ├── __init__.py
│   │   └── speech_recognizer.py        # Speech recognition functionality
│   │
│   ├── text_to_speech/                 # Text-to-speech conversion
│   │   ├── __init__.py
│   │   └── tts_engine.py               # TTS engine implementation
│   │
│   ├── visual_au_detection/            # Visual Action Unit and Emotion Detection
│   │   ├── __init__.py
│   │   └── live_vision_au_save.py      # Live vision with AU detection
│   │
│   └── __init__.py                     # Main modules package
│
├── images/                             # Test images for facial analysis
│   ├── black_man_smiling.jpg
│   └── small_smile_man.jpg
│
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

3. Run the Visual AU Detection module:

```bash
./run_visual_au_detection.py        # Run Visual AU Detection
```

## Modules

The project is organized into modular components:

### Visual AU Detection

The Visual AU Detection module provides functionality for detecting faces, facial action units (AUs), and emotions using computer vision techniques. It uses the PyFeat library for facial analysis.

To run the visual AU detection:

```bash
./run_visual_au_detection.py --output-dir /path/to/output
```

### LLM API

The LLM API module provides functionality for making API calls to Large Language Models.

### Text-to-Speech

The Text-to-Speech module provides functionality for converting text to speech.

### Microphone to Text

The Microphone to Text module provides functionality for capturing audio from a microphone and converting it to text using speech recognition.

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
