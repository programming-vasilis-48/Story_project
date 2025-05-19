FROM ros:noetic-ros-core-focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    wget \
    curl \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-catkin \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# For simulation purposes, we'll skip the Intel RealSense SDK installation
# and instead just install the necessary ROS packages for testing
RUN apt-get update && apt-get install -y \
    software-properties-common \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    scipy \
    matplotlib \
    pandas \
    scikit-learn \
    opencv-python \
    # Use mock packages for simulation instead of actual hardware dependencies
    # pyfeat \
    # pyaudio \
    # webrtcvad \
    SpeechRecognition \
    pyttsx3 \
    # Add some testing utilities
    pytest \
    pytest-mock \
    # Add dependencies for LLM integration
    python-dotenv \
    requests

# Create a catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copy the ROS package into the workspace
COPY src/story_switch /catkin_ws/src/story_switch

# Initialize rosdep
RUN rosdep init && rosdep update

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc"

# Make the Python scripts executable
RUN chmod +x /catkin_ws/src/story_switch/src/nodes/*.py

# Create a startup script
RUN echo '#!/bin/bash\nsource /opt/ros/noetic/setup.bash\nsource /catkin_ws/devel/setup.bash\nroslaunch story_switch story_switch.launch' > /start.sh && \
    chmod +x /start.sh

# Expose ROS ports
EXPOSE 11311

# Set the entrypoint
ENTRYPOINT ["/start.sh"]
