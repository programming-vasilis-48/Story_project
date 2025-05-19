# QTrobot Component Tests

This repository contains test scripts for testing various components of the QTrobot:

1. **LLM API Test**: Tests if the OpenRouter API works and can generate responses.
2. **Text-to-Speech Test**: Tests if the robot can speak text using the QTrobot TTS system.
3. **Speech Recognition Test**: Tests if the robot can recognize speech and transcribe it to text.
4. **Vision/AU Detection Test**: Tests if the robot can detect faces and facial expressions using PyFeat.

## Changes from Previous Version

- Fixed ROS node initialization issues when running all tests together
- Replaced Nuitrack with PyFeat for facial expression analysis
- Improved error handling and timeout mechanisms
- Added installation script for dependencies

## Setup

1. Clone this repository to your QTrobot:

```bash
git clone <repository-url>
cd qtrobot-tests
```

2. Run the setup script to install dependencies and make the test scripts executable:

```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

3. Edit the `.env` file and add your OpenRouter API key:

```bash
nano .env
```

Add your API key:

```
OPENROUTER_API_KEY=your_api_key_here
MODEL=meta-llama/llama-3.3-8b-instruct:free
```

## Running the Tests

### Run All Tests

To run all tests in sequence:

```bash
./run_all_tests.py
```

### Run Individual Tests

To run individual tests:

1. LLM API Test:
```bash
./test_llm_api.py
```

2. Text-to-Speech Test:
```bash
./test_tts.py
```

3. Speech Recognition Test:
```bash
./test_speech_recognition.py
```

4. Vision/AU Detection Test (using PyFeat):
```bash
./test_vision_au_pyfeat.py
```

## Test Descriptions

### LLM API Test

This test verifies that the QTrobot can connect to the OpenRouter API and receive responses from the LLM. It sends a simple prompt and checks if a valid response is received.

### Text-to-Speech Test

This test verifies that the QTrobot can speak text. It sends several test phrases to the robot's speech system and checks if they are spoken correctly.

### Speech Recognition Test

This test verifies that the QTrobot can recognize speech. It listens for sound using the ReSpeaker microphone array and attempts to transcribe the speech to text.

### Vision/AU Detection Test

This test verifies that the QTrobot can detect faces and facial expressions using PyFeat. It captures images from the robot's camera, detects faces, and extracts facial action units (AUs) and emotions.

## Troubleshooting

### PyFeat Installation Issues

If you encounter issues installing PyFeat, try the following:

```bash
pip3 install torch torchvision
pip3 install py-feat
```

### ROS Node Initialization Errors

If you encounter ROS node initialization errors when running the tests individually, make sure you don't have other ROS nodes running with the same name.

### Camera Access Issues

If the Vision/AU Detection test cannot access the camera, check that the camera topic is correct in the test script. The default is `/camera/color/image_raw`.

## Dependencies

- Python 3.6+
- ROS Noetic
- PyFeat
- OpenCV
- NumPy
- SpeechRecognition
- PyAudio
- WebRTC VAD
