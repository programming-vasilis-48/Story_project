cmake_minimum_required(VERSION 3.0.2)
project(story_switch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

# Uncomment these sections when you have message files
# Generate messages
# add_message_files(
#   FILES
#   # Add message files here as they are created
#   # Example: StoryState.msg
# )

# Generate services
# add_service_files(
#   FILES
#   # Add service files here as they are created
#   # Example: RecognizeSpeech.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add executables and targets as needed
# Example:
# add_executable(story_manager src/story_manager_node.cpp)
# target_link_libraries(story_manager ${catkin_LIBRARIES})
# add_dependencies(story_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
