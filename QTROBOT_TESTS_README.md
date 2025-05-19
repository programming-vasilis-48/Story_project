# QTrobot Component Tests

This repository contains test scripts for testing various components of the QTrobot:

1. **LLM API Test**: Tests if the OpenRouter API works and can generate responses.
2. **Text-to-Speech Test**: Tests if the robot can speak text using the QTrobot TTS system.
3. **Speech Recognition Test**: Tests if the robot can recognize speech and transcribe it to text.
4. **Vision/AU Detection Test**: Tests if the robot can detect faces and facial expressions.

## Setup

1. Clone this repository to your QTrobot:

```bash
git clone <repository-url>
cd qtrobot-tests
```

2. Run the setup script to install dependencies and make the test scripts executable:

```bash
./setup_tests.sh
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

4. Vision/AU Detection Test:
```bash
./test_vision_au.py
```

## Test Descriptions

### LLM API Test

This test verifies that the QTrobot can connect to the OpenRouter API and receive responses from the LLM. It sends a simple prompt and checks if a valid response is received.

### Text-to-Speech Test

This test verifies that the QTrobot can speak text. It sends several test phrases to the robot's speech system and checks if they are spoken correctly.

### Speech Recognition Test

This test verifies that the QTrobot can recognize speech. It prompts the user to speak, listens for audio input, and then attempts to transcribe the speech to text.

### Vision/AU Detection Test

This test verifies that the QTrobot can detect faces and facial expressions. It uses the robot's camera to detect faces and extract facial features, emotions, and Action Units (AUs).

## Troubleshooting

If you encounter issues with the tests:

1. **LLM API Test**:
   - Check that your API key is correct in the `.env` file
   - Verify that the robot has internet connectivity
   - Check if the OpenRouter API is available

2. **Text-to-Speech Test**:
   - Verify that the robot's speakers are working
   - Check that the ROS speech topic is correct

3. **Speech Recognition Test**:
   - Verify that the robot's microphones are working
   - Check that the speech recognition service is running
   - Speak clearly and in a quiet environment

4. **Vision/AU Detection Test**:
   - Verify that the robot's camera is working
   - Check that the face detection service is running
   - Ensure good lighting conditions
   - Position yourself directly in front of the robot
