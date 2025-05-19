#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Story Manager Node for the Story-Switch application.

This node serves as the central orchestrator for the Story-Switch application.
It manages the story state, turn-taking logic, and the 14-exchange, 28-sentence structure.
"""

import rospy
import os
import sys
from std_msgs.msg import String
from dotenv import load_dotenv
# Import custom message types as they are created

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the LLM client
from utils.llm_client import LLMClient

class StoryManager:
    """Story Manager class for the Story-Switch application."""

    def __init__(self):
        """Initialize the Story Manager node."""
        # Load environment variables
        load_dotenv()

        rospy.init_node('story_manager', anonymous=False)

        # Initialize publishers
        self.speech_pub = rospy.Publisher('/story_switch/speech/say', String, queue_size=10)

        # Initialize subscribers
        rospy.Subscriber('/story_switch/speech/recognized', String, self.speech_callback)
        rospy.Subscriber('/story_switch/vision/emotion', String, self.emotion_callback)

        # Initialize story state
        self.story_context = []
        self.current_turn = "robot"  # "robot" or "user"
        self.iteration = 0
        self.target_emotion = "neutral"
        self.user_emotion = "neutral"

        # Initialize LLM client
        try:
            self.llm_client = LLMClient()
            self.llm_available = True
            rospy.loginfo("LLM client initialized successfully")
        except Exception as e:
            self.llm_available = False
            rospy.logerr(f"Failed to initialize LLM client: {str(e)}")

        rospy.loginfo("Story Manager node initialized")

    def emotion_callback(self, msg):
        """Callback for emotion recognition messages."""
        self.user_emotion = msg.data
        rospy.loginfo(f"User emotion detected: {self.user_emotion}")

    def speech_callback(self, msg):
        """Callback for speech recognition messages."""
        if self.current_turn == "user":
            user_response = msg.data
            rospy.loginfo(f"User response: {user_response}")

            # Add user's sentence to story context with emotion
            user_entry = f"User ({self.user_emotion}): {user_response}"
            self.story_context.append(user_entry)

            # Switch turn to robot
            self.current_turn = "robot"

            # Generate robot's next sentence
            self.generate_robot_sentence()

    def generate_robot_sentence(self):
        """Generate the robot's next sentence based on the story context and target emotion."""
        # Create a prompt for the LLM
        if self.llm_available:
            # Create a system prompt
            system_prompt = (
                "You are a storytelling robot participating in a collaborative storytelling exercise with a human. "
                "You are taking turns adding sentences to build a story together. "
                "Your responses should be exactly ONE sentence that continues the story. "
                f"Your response should express the emotion: {self.target_emotion}. "
                "Do not use quotation marks or prefixes like 'Robot:'. Just provide the sentence."
            )

            # Create a user prompt with the story context
            story_so_far = "\n".join(self.story_context[-6:] if len(self.story_context) > 6 else self.story_context)
            user_prompt = (
                f"Here is our story so far:\n{story_so_far}\n\n"
                f"Continue the story with ONE sentence expressing {self.target_emotion}. "
                "Remember to maintain continuity with what has been said before."
            )

            # Generate text using the LLM
            try:
                robot_sentence = self.llm_client.generate_text(
                    prompt=user_prompt,
                    system_prompt=system_prompt,
                    max_tokens=100,
                    temperature=0.7
                )

                # Clean up the response if needed
                robot_sentence = robot_sentence.strip()

                # Log the generated sentence
                rospy.loginfo(f"LLM generated sentence: {robot_sentence}")
            except Exception as e:
                rospy.logerr(f"Error generating text with LLM: {str(e)}")
                robot_sentence = f"I'm feeling {self.target_emotion} as we continue our story."
        else:
            # Fallback if LLM is not available
            robot_sentence = f"This is the robot's sentence with emotion: {self.target_emotion}"
            rospy.logwarn("Using fallback sentence generation (LLM not available)")

        # Add robot's sentence to story context with emotion
        robot_entry = f"Robot ({self.target_emotion}): {robot_sentence}"
        self.story_context.append(robot_entry)

        # Publish robot's sentence for speech synthesis
        self.speech_pub.publish(robot_sentence)

        # Switch turn to user
        self.current_turn = "user"

        # Update iteration and emotion if needed
        # This is a simplified version of the emotion selection logic
        if len(self.story_context) % 4 == 0:
            self.iteration += 1
            emotions = ["happy", "sad", "angry", "fear", "surprise", "disgust", "neutral"]
            self.target_emotion = emotions[self.iteration % len(emotions)]
            rospy.loginfo(f"Switching to new target emotion: {self.target_emotion}")

    def run(self):
        """Run the Story Manager node."""
        # Start with robot's first sentence
        self.generate_robot_sentence()

        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        story_manager = StoryManager()
        story_manager.run()
    except rospy.ROSInterruptException:
        pass
