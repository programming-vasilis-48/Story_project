# Story-Switch Application for QTrobot

This repository contains the code for the Story-Switch application, a collaborative, turn-based storytelling application for the QTrobot RD-V2 i7 platform.

## Project Structure

```
.
├── Dockerfile              # Docker configuration for development environment
├── docker-compose.yml      # Docker Compose configuration
├── README.md               # This file
├── src/                    # ROS workspace source directory
│   └── story_switch/       # ROS package for the Story-Switch application
│       ├── CMakeLists.txt  # CMake build configuration
│       ├── package.xml     # ROS package metadata
│       ├── launch/         # ROS launch files
│       │   └── story_switch.launch  # Main launch file
│       └── src/            # Source code
│           └── nodes/      # ROS nodes
│               ├── story_manager_node.py  # Story Manager node
│               ├── speech_synth_node.py   # Speech Synthesis node
│               ├── speech_recog_node.py   # Speech Recognition node
│               └── vision_track_node.py   # Vision Tracking node
└── story_switch_plan.md    # Project plan
```

## Development Environment Setup

### Using Docker (Recommended)

1. Install Docker and Docker Compose on your system.
2. Build and run the Docker container:

```bash
docker-compose up --build
```

This will build the Docker image and start the ROS nodes in the container.

### Using a Virtual Environment (Alternative)

If you prefer not to use Docker, you can set up a virtual environment:

1. Install ROS Noetic on your system (via WSL2 on Windows).
2. Create a Python virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

3. Install the required Python dependencies:

```bash
pip install numpy scipy matplotlib pandas scikit-learn opencv-python pyfeat pyaudio webrtcvad SpeechRecognition pyttsx3
```

4. Build the ROS workspace:

```bash
cd src
catkin_make
source devel/setup.bash
```

5. Run the ROS nodes:

```bash
roslaunch story_switch story_switch.launch
```

## Testing

The current implementation includes simulation capabilities for testing without the actual QTrobot hardware:

- The Speech Recognition node simulates user input.
- The Vision Tracking node simulates user emotion data.

## Deployment to QTrobot

To deploy the application to the QTrobot:

1. Copy the ROS package to the QTrobot's ROS workspace.
2. Build the workspace on the QTrobot.
3. Run the application using the launch file.

## License

MIT
