# -*- coding: utf-8 -*-

"""
Main file for the Story-Switch game.

This file connects all 4 modules together and implements the story-switch game
with LLM API, text-to-speech, and terminal input/output.
"""

import os
import sys
import random
import time
import rospy
from std_msgs.msg import String
from typing import List, Dict, Tuple
from dotenv import load_dotenv
import tempfile

# Add the modules directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the LLM client from the llm_api module
from modules.llm_api.llm_client import LLMClient

# Import the TTS engines from the text_to_speech module
from modules.text_to_speech import OpenAITTSEngine, QTRobotTTSEngine, EmotionlessTTSEngine

# Import the SpeechRecognizer and WhisperRecognizer from the mic_to_text module
from modules.mic_to_text import SpeechRecognizer
from modules.mic_to_text.whisper_recognizer import WhisperRecognizer

# Import the logging and video recording modules
from modules.logging import SessionLogger
from modules.video_recording import VideoRecorder

# List of emotions for the game
EMOTIONS = ["happy", "sad", "angry", "fear", "surprise", "disgust", "neutral"]

# Map our emotions to QT Robot emotion animations
QT_EMOTION_MAP = {
    "happy": "QT/happy",
    "sad": "QT/sad",
    "angry": "QT/angry",
    "fear": "QT/afraid",
    "surprise": "QT/surprise",
    "disgust": "QT/disgusted",
    "neutral": "QT/neutral"
}

# Class to manage QT Robot emotions
class QTRobotEmotionManager:
    def __init__(self):
        try:
            # Initialize ROS node if not already initialized
            if not rospy.get_node_uri():
                rospy.init_node('story_switch_emotion_manager', anonymous=True)
            
            # Create publisher for QT Robot emotion with latch=True to ensure message delivery
            self.emotion_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10, latch=True)
            rospy.sleep(1.0)  # Give more time for publisher to connect
            self.available = True
            
            # Test the emotion manager with a neutral face to confirm it works
            self.show_emotion("neutral")  # Start with neutral face
        except Exception as e:
            print(f"Error initializing QT Robot emotion manager: {str(e)}")
            self.available = False
    
    def test_emotions(self):
        """Test the emotion display functionality"""
        time.sleep(0.5)
        self.show_emotion("neutral")  # Start with neutral face
        time.sleep(2)
        self.show_talking()
        time.sleep(2)
    
    def show_emotion(self, emotion):
        """Show an emotion on QT Robot's display"""
        if not self.available:
            return
        
        # Map our emotion to QT Robot emotion name
        qt_emotion = QT_EMOTION_MAP.get(emotion, "QT/neutral")
        
        try:
            self.emotion_pub.publish(String(qt_emotion))
            # Give a small delay to ensure the message is processed
            rospy.sleep(0.2)
        except Exception as e:
            print(f"Error showing QT Robot emotion: {str(e)}")
    
    def show_talking(self):
        """Show talking animation on QT Robot's display"""
        if not self.available:
            return
        
        try:
            # Direct command to show talking animation
            self.emotion_pub.publish(String("QT/talking"))
            # Give a small delay to ensure the message is processed
            rospy.sleep(0.2)
        except Exception as e:
            print(f"Error showing QT Robot talking animation: {str(e)}")

# Alternative direct function to test ROS communication
def test_qt_robot_emotions():
    """Directly test the QT Robot emotions through ROS"""
    try:
        print("Initializing ROS node for direct emotion testing...")
        rospy.init_node('emotion_test_node', anonymous=True)
        pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10, latch=True)
        rospy.sleep(1.0)  # Give time for the publisher to connect
        
        print("Testing neutral expression")
        pub.publish(String("QT/neutral"))
        time.sleep(2)
        
        print("Testing talking animation")
        pub.publish(String("QT/talking"))
        time.sleep(2)
        
        print("Testing happy expression")
        pub.publish(String("QT/happy"))
        time.sleep(2)
        
        print("Testing sad expression") 
        pub.publish(String("QT/sad"))
        time.sleep(2)
        
        print("Direct emotion test completed")
        return True
    except Exception as e:
        print(f"Error in direct emotion test: {e}")
        return False

def get_person_name() -> str:
    """
    Get the person's name for the experiment (text input only).

    Returns:
        str: Person's name
    """
    print("Please enter your name for this experiment session:")

    while True:
        try:
            name = input("> ").strip()

            if name:
                # Clean the name (remove special characters, keep only letters, numbers, spaces)
                import re
                cleaned_name = re.sub(r'[^a-zA-Z0-9\s]', '', name)
                cleaned_name = ' '.join(cleaned_name.split())  # Normalize whitespace

                if cleaned_name:
                    return cleaned_name
                else:
                    print("Please enter a valid name (letters and numbers only).")
            else:
                print("Please enter your name.")

        except KeyboardInterrupt:
            print("\nExiting...")
            raise
        except Exception as e:
            print("Please try again.")

def select_voice_type() -> int:
    """
    Prompt user to select voice type for the experiment.

    Returns:
        int: 1 for emotional OpenAI TTS, 2 for emotionless voice
    """
    print("Select which voice to use first:")
    print("1 - Emotional voice")
    print("2 - Emotionless voice")

    while True:
        try:
            choice = input("> ").strip()

            if choice == "1":
                return 1
            elif choice == "2":
                return 3  # Note: We return 3 to maintain compatibility with existing code
            else:
                print("Please press 1 for emotional or 2 for emotionless voice.")

        except KeyboardInterrupt:
            print("\nExiting...")
            raise
        except Exception as e:
            print(f"Error reading input: {e}")
            print("Please try again.")
            continue

def select_random_emotion(used_emotions: Dict[str, int], last_emotions: List[str]) -> str:
    """
    Select a random emotion from the list of emotions.

    Args:
        used_emotions (Dict[str, int]): Dictionary tracking how many times each emotion has been used
        last_emotions (List[str]): List of the last emotions used (to avoid repetition)

    Returns:
        str: Randomly selected emotion
    """
    # Filter out emotions that have been used twice already
    available_emotions = [emotion for emotion in EMOTIONS if used_emotions.get(emotion, 0) < 2]

    # If all emotions have been used twice, return None to end the game
    if not available_emotions:
        return None

    # Filter out the last emotion if it's been used twice in a row
    if len(last_emotions) >= 2 and last_emotions[-1] == last_emotions[-2]:
        if last_emotions[-1] in available_emotions:
            available_emotions.remove(last_emotions[-1])

    # If we've filtered out all emotions, just use any available emotion
    if not available_emotions:
        available_emotions = [emotion for emotion in EMOTIONS if used_emotions.get(emotion, 0) < 2]

    # Select a random emotion from the available emotions
    selected_emotion = random.choice(available_emotions)

    return selected_emotion

def generate_story_start(llm_client: LLMClient, emotion: str) -> str:
    """
    Generate the first 4 sentences of the story.

    Args:
        llm_client (LLMClient): LLM client for making API calls
        emotion (str): Emotion to elicit in the story

    Returns:
        str: First 4 sentences of the story
    """
    system_prompt = (
        "You are participating in a collaborative storytelling game. "
        "You are starting a new story and need to provide the first 4 short sentences. "
        "Sentences should be short, to the point and resemble dialogue like a conversation, simple vocabulary, not a lot of adjectives."
        f"Your story's objective is to powerfully evoke the feeling of {emotion}, every sentence of your story should evoke {emotion}, but without naming the emotion or describing how the listeners feels."
        "Speak in second person singular "
    )

    user_prompt = (
        "Please start a new story with 4 sentences. "
        "The story should be about the listener in Groningen, Netherlands, specifically at Zernike campus. "
        "Start with the listener sitting in a chair taking part in a study. "
        "Do not state what the study is about, just say that they are taking part in a study. "
        f"Your story's objective is to powerfully evoke the feeling of {emotion}, every sentence of your story should evoke {emotion}, but without naming the emotion or describing how the listeners feels."
        "Sentences should be short, to the point and resemble dialogue like a conversation, simple vocabulary, not a lot of adjectives."
    )

    # Generate the story start
    story_start = llm_client.generate_text(
        prompt=user_prompt,
        system_prompt=system_prompt,
        max_tokens=200,
        temperature=0.7
    )

    return story_start

def continue_story(llm_client: LLMClient, story_context: List[str], emotion: str) -> str:
    """
    Continue the story based on the context and try to elicit the specified emotion.

    Args:
        llm_client (LLMClient): LLM client for making API calls
        story_context (List[str]): List of previous story exchanges
        emotion (str): Emotion to elicit in the story

    Returns:
        str: Next part of the story
    """
    # Create the system prompt
    system_prompt = (
        "You are participating in a collaborative storytelling game. "
        "You are taking turns adding to the story. "
        f"Your story's objective is to powerfully evoke the feeling of {emotion}, every sentence of your story should evoke {emotion}, but without naming the emotion or describing how the listeners feels."
        "Sentences should be short, to the point and resemble dialogue like a conversation, simple vocabulary, not a lot of adjectives."
        "Speak in second person singular "
        "Keep your response to 1-2 sentences. "
    )

    # Create the user prompt with the story context
    user_prompt = "Here is our story so far:\n\n"
    for exchange in story_context:
        user_prompt += f"{exchange}\n\n"

    user_prompt += (
        "Continue the story where it was felt last. "
        f"Your story's objective is to powerfully evoke the feeling of {emotion}, every sentence of your story should evoke {emotion}, but without naming the emotion or describing how the listeners feels."
    )

    # Generate the continuation
    continuation = llm_client.generate_text(
        prompt=user_prompt,
        system_prompt=system_prompt,
        max_tokens=150,
        temperature=0.7
    )

    return continuation

# Extend the TTS engines to handle QT Robot emotions
class EnhancedOpenAITTSEngine(OpenAITTSEngine):
    def __init__(self, api_key, model, use_qtrobot_audio=False, qt_emotion_manager=None):
        super().__init__(api_key, model, use_qtrobot_audio)
        self.qt_emotion_manager = qt_emotion_manager
        self.current_emotion = "neutral"
        # Ensure rate is never below OpenAI's minimum
        self.rate = max(0.25, self.rate)
    
    def adjust_for_emotion(self, emotion):
        self.current_emotion = emotion
        super().adjust_for_emotion(emotion)
        # Ensure rate is never below OpenAI's minimum after adjustment
        self.rate = max(0.25, self.rate)
    
    def speak(self, text):
        # Show talking animation BEFORE we start speech processing
        if self.qt_emotion_manager:
            self.qt_emotion_manager.show_talking()
            # Ensure talking animation is visible before speech starts
            time.sleep(0.5)
        
        try:
            # Create a temporary file for the audio to estimate duration
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_file:
                temp_file_path = temp_file.name

            # Generate speech with fixed speed parameter
            self._generate_speech(text, temp_file_path)
            
            # Calculate a better speech duration estimate
            # Count words and estimate duration (average spoken English is ~150 words per minute)
            word_count = len(text.split())
            word_duration = word_count / 150.0 * 60.0  # duration in seconds
            
            # Also use file size for a minimum estimate
            file_size = os.path.getsize(temp_file_path)
            file_size_duration = file_size / (1024 * 16) # ~16KB per second for MP3
            
            # Take the higher estimate to be safe
            estimated_duration = max(word_duration, file_size_duration, 2.0)
            
            # Start a thread to maintain the talking animation
            import threading
            stop_animation = threading.Event()
            
            def talking_animation_loop():
                """Loop the talking animation until the speech is done"""
                while not stop_animation.is_set():
                    self.qt_emotion_manager.show_talking()
                    # Each talking animation lasts about 2.2 seconds
                    time.sleep(2.0)  # Slightly shorter to ensure overlap
            
            # Start the animation loop if we have an emotion manager
            animation_thread = None
            if self.qt_emotion_manager:
                animation_thread = threading.Thread(target=talking_animation_loop)
                animation_thread.daemon = True
                animation_thread.start()
            
            # Play the audio
            if self.use_qtrobot_audio and self.ros_available:
                self._play_audio_qtrobot(temp_file_path)
            else:
                self._play_audio(temp_file_path)
            
            # Stop the animation thread
            if animation_thread:
                stop_animation.set()
                animation_thread.join(timeout=1.0)
            
            # Clean up
            os.unlink(temp_file_path)
            
        except Exception as e:
            print(f"Speech error: {str(e)}")
        finally:
            # Show appropriate emotion after speaking (or on error)
            if self.qt_emotion_manager:
                # Emotional face right after speech ends
                self.qt_emotion_manager.show_emotion(self.current_emotion)
    
    # Override the _generate_speech method to fix the speed parameter issue
    def _generate_speech(self, text, output_file):
        """Generate speech using OpenAI TTS with fixed speed parameter."""
        try:
            # Ensure rate is valid (never below 0.25)
            safe_rate = max(0.25, float(self.rate))
            
            # Generate speech using OpenAI API with properly typed parameters
            if self.instructions:
                response = self.client.audio.speech.create(
                    model=self.model,
                    voice=self.voice,
                    input=text,
                    speed=safe_rate,  # Use validated rate
                    instructions=self.instructions
                )
            else:
                response = self.client.audio.speech.create(
                    model=self.model,
                    voice=self.voice,
                    input=text,
                    speed=safe_rate  # Use validated rate
                )

            # Save the audio to file
            response.stream_to_file(output_file)

        except Exception as e:
            print(f"Error generating speech with OpenAI TTS: {str(e)}")
            raise

class EnhancedEmotionlessTTSEngine(EmotionlessTTSEngine):
    def __init__(self, api_key, model, use_qtrobot_audio=False, qt_emotion_manager=None):
        super().__init__(api_key, model, use_qtrobot_audio)
        self.qt_emotion_manager = qt_emotion_manager
        self.current_emotion = "neutral"
        # Ensure rate is never below OpenAI's minimum
        self.rate = max(0.25, self.rate)
    
    def adjust_for_emotion(self, emotion):
        self.current_emotion = emotion
        super().adjust_for_emotion(emotion)
        # Ensure rate is never below OpenAI's minimum after adjustment
        self.rate = max(0.25, self.rate)
    
    def speak(self, text):
        # Show talking animation BEFORE we start speech processing
        if self.qt_emotion_manager:
            self.qt_emotion_manager.show_talking()
            # Ensure talking animation is visible before speech starts
            time.sleep(0.5)
        
        try:
            # Create a temporary file for the audio to estimate duration
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_file:
                temp_file_path = temp_file.name

            # Generate speech with fixed speed parameter
            self._generate_speech(text, temp_file_path)
            
            # Calculate a better speech duration estimate
            # Count words and estimate duration (average spoken English is ~150 words per minute)
            word_count = len(text.split())
            word_duration = word_count / 150.0 * 60.0  # duration in seconds
            
            # Also use file size for a minimum estimate
            file_size = os.path.getsize(temp_file_path)
            file_size_duration = file_size / (1024 * 16) # ~16KB per second for MP3
            
            # Take the higher estimate to be safe
            estimated_duration = max(word_duration, file_size_duration, 2.0)
            
            # Start a thread to maintain the talking animation
            import threading
            stop_animation = threading.Event()
            
            def talking_animation_loop():
                """Loop the talking animation until the speech is done"""
                while not stop_animation.is_set():
                    self.qt_emotion_manager.show_talking()
                    # Each talking animation lasts about 2.2 seconds
                    time.sleep(2.0)  # Slightly shorter to ensure overlap
            
            # Start the animation loop if we have an emotion manager
            animation_thread = None
            if self.qt_emotion_manager:
                animation_thread = threading.Thread(target=talking_animation_loop)
                animation_thread.daemon = True
                animation_thread.start()
            
            # Play the audio
            if self.use_qtrobot_audio and self.ros_available:
                self._play_audio_qtrobot(temp_file_path)
            else:
                self._play_audio(temp_file_path)
            
            # Stop the animation thread
            if animation_thread:
                stop_animation.set()
                animation_thread.join(timeout=1.0)
            
            # Clean up
            os.unlink(temp_file_path)
            
        except Exception as e:
            print(f"Speech error: {str(e)}")
        finally:
            # Show appropriate emotion after speaking (or on error)
            if self.qt_emotion_manager:
                # Emotional face right after speech ends
                self.qt_emotion_manager.show_emotion(self.current_emotion)
    
    # Override the _generate_speech method to fix the speed parameter issue
    def _generate_speech(self, text, output_file):
        """Generate speech using OpenAI TTS with fixed speed parameter."""
        try:
            # Ensure rate is valid (never below 0.25)
            safe_rate = max(0.25, float(self.rate))
            
            # Generate speech using OpenAI API with emotionless instructions
            response = self.client.audio.speech.create(
                model=self.model,
                voice=self.voice,
                input=text,
                speed=safe_rate,  # Use validated rate
                instructions=self.instructions
            )

            # Save the audio to file
            response.stream_to_file(output_file)

        except Exception as e:
            print(f"Error generating speech with OpenAI TTS: {str(e)}")
            raise

def main():
    """Main function to run the story-switch game."""
    # Load environment variables
    load_dotenv()
    
    # Get API keys from environment variables or use defaults
    openai_api_key = os.environ.get("OPENAI_API_KEY", "")
    openrouter_api_key = os.environ.get("OPENROUTER_API_KEY", "")
    
    # Check if we have valid API keys
    if not openai_api_key:
        print("ERROR: No OpenAI API key found. Set the OPENAI_API_KEY environment variable.")
        print("Create a .env file in the project root with: OPENAI_API_KEY=your_api_key_here")
        return 1
        
    if not openrouter_api_key:
        print("ERROR: No OpenRouter API key found. Set the OPENROUTER_API_KEY environment variable.")
        print("Add to your .env file: OPENROUTER_API_KEY=your_openrouter_key_here")
        return 1
    
    print("API keys loaded successfully")

    try:
        # Initialize QT Robot emotion manager
        qt_emotion_manager = QTRobotEmotionManager()
        
        # Get participant name
        person_name = get_person_name()

        # Get voice choice from experimenter - which one to use first
        first_voice_choice = select_voice_type()
        
        # Determine second voice based on first choice
        second_voice_choice = 3 if first_voice_choice == 1 else 1

        # Initialize session logger with placeholder voice type
        session_logger = SessionLogger(person_name, "Experiment with both voice types")
        
        # Initialize video recorder
        video_recorder = VideoRecorder(existing_node=qt_emotion_manager.available)
        time.sleep(1.0)  # Allow time for ROS subscription

        # Initialize Whisper speech recognition
        speech_recognizer = WhisperRecognizer(api_key=openai_api_key)
        speech_available = hasattr(speech_recognizer, 'microphone') and speech_recognizer.microphone

        # Initialize LLM client
        llm_client = LLMClient(provider="openrouter", api_key=openrouter_api_key)

        print("System initialization complete")
        
        # Log the order of voices
        session_logger.log_event(f"Voice order: {first_voice_choice} (first) and {second_voice_choice} (second)")
        
        # Create two separate lists with the same emotions but in different random orders
        # Ensure we use all 7 standard emotions for both voice types
        first_voice_emotions = EMOTIONS.copy()  # Use all 7 emotions
        second_voice_emotions = EMOTIONS.copy()  # Use all 7 emotions
        
        # Shuffle the emotions for randomization
        random.shuffle(first_voice_emotions)
        random.shuffle(second_voice_emotions)
        
        session_logger.log_event(f"First voice emotion order: {first_voice_emotions}")
        session_logger.log_event(f"Second voice emotion order: {second_voice_emotions}")
        
        # Run experiment with first TTS type
        run_experiment_with_tts(first_voice_choice, person_name, qt_emotion_manager, session_logger, video_recorder, 
                              speech_available, speech_recognizer, llm_client, openai_api_key, is_first_story=True,
                              emotion_sequence=first_voice_emotions)
        
        # Create transition TTS engine with emotionless voice for transition message
        transition_tts = EnhancedEmotionlessTTSEngine(
            api_key=openai_api_key,
            model="gpt-4o-mini-tts",
            use_qtrobot_audio=False,
            qt_emotion_manager=qt_emotion_manager
        )
        
        # Use speech for participant communication, not print statements
        transition_message = "Now, we're going to create another story using a different voice. Let's begin."
        transition_tts.speak(transition_message)
        
        # Run experiment with second TTS type
        run_experiment_with_tts(second_voice_choice, person_name, qt_emotion_manager, session_logger, video_recorder, 
                              speech_available, speech_recognizer, llm_client, openai_api_key, is_first_story=False,
                              emotion_sequence=second_voice_emotions)

        # Final thank you
        final_message = "Thank you so much for participating in this experiment! Your contributions are greatly appreciated."
        transition_tts.speak(final_message)

        # Save session logs
        session_logger.save_session_log()
        
        print("Experiment complete")
        
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

def run_experiment_with_tts(voice_choice, person_name, qt_emotion_manager, session_logger, video_recorder, 
                           speech_available, speech_recognizer, llm_client, openai_api_key, is_first_story=False,
                           emotion_sequence=None):
    """Run a story-switch experiment with a specific TTS voice."""
    # Initialize the appropriate TTS engine based on voice choice
    tts_engine = None
    voice_type_name = ""

    if voice_choice == 1:
        tts_engine = EnhancedOpenAITTSEngine(
            api_key=openai_api_key,
            model="gpt-4o-mini-tts",
            use_qtrobot_audio=False,
            qt_emotion_manager=qt_emotion_manager
        )
        voice_type_name = "Emotional OpenAI TTS"
    else:
        tts_engine = EnhancedEmotionlessTTSEngine(
            api_key=openai_api_key,
            model="gpt-4o-mini-tts",
            use_qtrobot_audio=False,
            qt_emotion_manager=qt_emotion_manager
        )
        voice_type_name = "Emotionless Voice"
    
    # Log which TTS is being used (only to logs, not to terminal)
    session_logger.log_event(f"Starting experiment with {voice_type_name}")
    
    # Only do the introduction for the first story
    if is_first_story:
        # Prepare welcome message - combine all into one continuous speech
        if voice_choice == 1:
            greeting = "Hello! I'm QT, and I'm excited to create a story with you today! "
        else:
            greeting = "Hello! I'm QT. Let's create a story together. "
        
        # Combine all introduction content into a single message
        introduction = (
            greeting + 
            "We're going to create a story together, taking turns to add to it. I'll start the story, and then "
            "you'll continue it. We'll keep going back and forth, building our story piece by piece. "
            "When it's your turn to speak, please press and hold the V key on your keyboard while you're talking. "
            "Release the key when you're done speaking. Let's begin our story."
        )

        # Speak the entire introduction as one continuous speech
        tts_engine.speak(introduction)

    # Initialize game state
    story_context = []
    
    # If no emotion sequence was provided, create one
    if emotion_sequence is None:
        emotion_sequence = EMOTIONS.copy()
        random.shuffle(emotion_sequence)
    
    # Ensure we're using all 7 emotions
    print(f"Using all {len(emotion_sequence)} emotions for {voice_type_name}: {emotion_sequence}")
    session_logger.log_event(f"Using emotions for {voice_type_name}: {emotion_sequence}")
    
    # Start with the first emotion (index 0)
    emotion_index = 0
    
    # Loop through all emotions
    while emotion_index < len(emotion_sequence):
        # Get current emotion
        current_emotion = emotion_sequence[emotion_index]
        print(f"Emotion {emotion_index} of {len(emotion_sequence)-1}: {current_emotion}")
        
        # QT speaks with current emotion
        if emotion_index == 0:
            # Generate and speak the story start
            story_part = generate_story_start(llm_client, current_emotion)
        else:
            # Generate continuation based on previous exchanges
            story_part = continue_story(llm_client, story_context, current_emotion)

        # Start video recording
        video_filename = None
        if video_recorder and video_recorder.is_camera_available():
            video_filename = f"{person_name.lower().replace(' ', '_')}_{voice_type_name.replace(' ', '_')}_{current_emotion}_exchange_{emotion_index}.mp4"
            session_logger.log_video_start(current_emotion, video_filename, voice_type_name)
        video_path = session_logger.get_video_path(video_filename)
        video_recorder.start_recording(video_path)

        # QT speaks
        speech_start_time = time.time()
        tts_engine.adjust_for_emotion(current_emotion)
        tts_engine.speak(story_part)
        speech_duration = time.time() - speech_start_time

        # Log QT's speech
        session_logger.log_robot_speech(story_part, current_emotion, speech_duration, voice_type_name)
        story_context.append(f"AI ({current_emotion}): {story_part}")

        # User's turn to respond (for all emotions including the last one)
        user_speech_start_time = time.time()

        if speech_available:
            user_input = speech_recognizer.listen_once(push_to_talk=True, push_to_talk_key='v')
            if not user_input:
                print("Please type your response:")
                user_input = input("> ")
        else:
            user_input = input("> ")

        user_speech_duration = time.time() - user_speech_start_time
        session_logger.log_user_speech(user_input, user_speech_duration)

        # Stop video recording after user response
        if video_recorder and video_recorder.is_recording and video_filename:
            video_recorder.stop_recording()
            session_logger.log_video_stop(video_filename)

        # Add user's response to context
        story_context.append(f"User: {user_input}")

        # Move to next emotion
        emotion_index += 1
        
    # Story is complete - all emotions have been used and participant has responded to each
    # Say thank you only if this is the final story
    if not is_first_story:
        end_message = f"Thank you for creating this story with me."
        tts_engine.adjust_for_emotion("neutral")
        tts_engine.speak(end_message)

    session_logger.log_event(f"Completed story with {voice_type_name}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSession ended by user.")
    except Exception as e:
        print(f"\nError during session: {str(e)}")
    finally:
        if 'session_logger' in locals():
            session_logger.save_session_log()
        if 'video_recorder' in locals():
            video_recorder.cleanup()
