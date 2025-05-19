# Story-Switch Application Plan

This document outlines the plan for implementing the collaborative, turn-based Story-Switch application on the QTrobot RD-V2 i7 platform.

## 1. Software Architecture (ROS Nodes)

The core of the application will be implemented as a set of interconnected ROS nodes.

*   **`story_manager` (Python)**
    *   **Role:** The central orchestrator. Manages the story state, turn-taking logic, and the 14-exchange, 28-sentence structure.
    *   **Functionality:**
        *   Maintains the current story context (list of sentences).
        *   Tracks the current turn (robot/user) and iteration number (up to 2-3 per emotion).
        *   Selects the target emotion for each iteration.
        *   Calls the Cloud-based LLM/text generator API to:
            *   Generate the robot's internal monologue based on the current story context and target emotion.
            *   Generate the robot's next sentence based on the story context, target emotion, and the user's previous response.
        *   Publishes the robot's internal monologue and sentence to the `speech_synth` node.
        *   Subscribes to the `speech_recog` node for the user's transcribed response.
        *   Subscribes to the `vision_track` node for user facial emotion data.
        *   Publishes data logs (timestamp, internal thought, spoken sentence, user response, emotion label) to ROS bag files.
        *   Publishes the robot's intended emotion label, timestamp, and Py-Feat extracted Action Units (based on the intended emotion) for each robot sentence.
        *   Handles iteration control, looping through the target emotions (happy, sad, angry, fear, surprise, disgust, neutral) for 2-3 iterations each.
*   **`speech_synth` (Python/C++)**
    *   **Role:** Converts text to speech and outputs audio.
    *   **Functionality:**
        *   Subscribes to the `story_manager` node for text (internal monologue and sentences).
        *   Uses a ROS-compatible TTS package (e.g., `ros-access-speech`) to synthesize speech.
        *   Publishes the synthesized audio to the robot's speaker.
        *   Adapts voice tone based on the target emotion received from `story_manager`.
*   **`speech_recog` (Python/C++)**
    *   **Role:** Captures user audio, performs VAD/DOA/beamforming, and transcribes speech.
    *   **Functionality:**
        *   Subscribes to the ReSpeaker Mic Array v2.0 audio stream.
        *   Implements Voice Activity Detection (VAD) to detect when the user is speaking after the robot's turn.
        *   Performs Direction of Arrival (DOA) and beamforming to focus on the user's voice.
        *   Uses a ROS-compatible ASR package (e.g., `ros-access-speech`) to transcribe the captured audio to text.
        *   Publishes the transcribed text to the `story_manager` node.
*   **`vision_track` (Python/C++)**
    *   **Role:** Captures video from the Intel RealSense™ D455 camera and analyzes user facial expressions.
    *   **Functionality:**
        *   Captures RGB video stream.
        *   Uses Py-Feat to analyze the user's facial expression after each robot sentence and participant response.
        *   Infers user emotion and extracts Action Units.
        *   Publishes the inferred user emotion, extracted AUs, and timestamp to the `story_manager` node for logging.
        *   (Optional future use: Capture skeleton data for nonverbal cues).

## ROS Node Architecture Diagram

```mermaid
graph TD
    story_manager --> speech_synth: Text (Monologue/Sentence)
    story_manager --> Data_Log: Log Data
    story_manager --> vision_track: Request User Emotion Analysis
    story_manager --> LLM_API: Text Generation Request
    speech_recog --> story_manager: Transcribed Text
    vision_track --> story_manager: User Emotion Data
    Mic_Array --> speech_recog: Audio Stream
    RealSense_Camera --> vision_track: Video Stream
    speech_synth --> Speaker: Audio Output
    LLM_API --> story_manager: Generated Text
```

## 2. Development Environment Setup

The development environment will be set up directly on the QTrobot or on a local machine with ROS Noetic installed.

*   **Dependencies Installation:**
    *   Install necessary ROS packages for speech recognition and synthesis.
    *   Install PyFeat and its dependencies for facial expression analysis.
    *   Install Python 3 dependencies for the Python nodes (`story_manager`, `vision_track`, `speech_synth` and `speech_recog`).
    *   Configure the Sennheiser microphone for speech recognition.
*   **Configuration:**
    *   Set up ROS environment variables.
    *   Build the ROS workspace containing the Story-Switch nodes and launch files.
*   **Startup Scripts:**
    *   Use the `story_switch.launch` file to launch all necessary ROS nodes.
    *   Configure the launch file to run nodes in simulation mode when testing without hardware.

## 3. Testing & Deployment

*   **Unit Tests:**
    *   Write unit tests for individual node functionalities using `rostest` (for C++ nodes) and `pytest` (for Python nodes).
    *   Focus on testing logic within each node (e.g., `story_manager`'s turn logic, `speech_recog`'s transcription handling, `vision_track`'s Py-Feat integration).
*   **Simulation Scripts:**
    *   Develop scripts to simulate inputs to the ROS nodes:
        *   Mock user speech inputs by publishing text messages to the topic `speech_recog` would normally publish to.
        *   Mock facial expression data by publishing synthetic emotion/AU data to the topic `vision_track` would normally publish to.
    *   Develop scripts to evaluate robot output by subscribing to topics where `story_manager` publishes logs, robot sentences, and emotion annotations.
*   **CI Pipeline (e.g., GitHub Actions):**
    *   Configure a CI pipeline to:
        *   Build the Docker image.
        *   Run the unit tests within the Docker container.
        *   Execute simulation scripts to test the integrated system in a simulated environment.
        *   Report test results.

## Implementation Tasks

Here is a list of tasks to be implemented based on the plan:

- [x] Create the ROS workspace and package structure.
- [x] Implement the `story_manager` ROS node (Python).
    - [x] Implement story state management and turn-taking logic.
    - [x] Integrate with the Cloud-based LLM/text generator API.
    - [x] Implement iteration control and emotion selection.
    - [ ] Implement data logging to ROS bag files.
    - [ ] Implement publishing robot's intended emotion and AUs.
- [x] Implement the `speech_synth` ROS node (Python/C++).
    - [ ] Integrate with the ROS-compatible TTS package.
    - [ ] Implement voice tone adaptation based on emotion.
- [x] Implement the `speech_recog` ROS node (Python/C++).
    - [ ] Integrate with the ReSpeaker Mic Array audio stream.
    - [ ] Implement VAD, DOA, and beamforming.
    - [ ] Integrate with the ROS-compatible ASR package.
- [x] Implement the `vision_track` ROS node (Python/C++).
    - [ ] Integrate with the Intel RealSense™ D455 camera.
    - [ ] Integrate with Py-Feat for user facial emotion analysis.
- [x] Create installation scripts for dependencies.
    - [x] Install all necessary dependencies (ROS packages, Py-Feat, speech recognition libraries, etc.).
    - [x] Configure environment variables.
- [x] Create the `story_switch.launch` file.
- [x] Write tests for each component (LLM API, TTS, Speech Recognition, Vision/AU Detection).
- [x] Develop simulation scripts for testing.