# Logging and Video Recording Guide

This guide explains how the logging and video recording functionality works in the Story-Switch game system, what data is recorded, and how to access and analyze the logs and videos after running an experiment.

## Overview

The Story-Switch game records multiple types of information during a session:

1. **Speech Events**: All spoken text by both the robot and the user, with timestamps, durations, and emotions.
2. **Video Recordings**: Video clips capturing the user's facial expressions and reactions during emotion elicitation.
3. **Session Metadata**: Information about the session, including participant name, voice type, and emotion usage.

## Data Storage Structure

All logs and videos are stored in the `logs/sessions/` directory, organized by session:

```
logs/
└── sessions/
    └── 2023-06-10_14-35-28_John_Doe/
        ├── session_log.json    # Complete session data in JSON format
        ├── metadata.json       # Session metadata
        ├── summary.txt         # Human-readable summary
        └── videos/             # Directory containing all video recordings
            ├── john_doe_happy_exchange_0.mp4
            ├── john_doe_sad_exchange_1.mp4
            └── ...
```

## Recorded Information

### Speech Events

For each speech event (both robot and user), the following information is logged:

- **Timestamp**: When the speech occurred
- **Speaker**: Either "robot" or "user"
- **Text**: What was said
- **Emotion Attempted**: (For robot speech only) The emotion the system attempted to elicit
- **Duration**: How long the speech lasted
- **Exchange Number**: The sequential number of the exchange

### Video Recordings

Each video recording includes:

- Video footage from when the robot begins speaking to elicit an emotion until the person answers
- Filename format: `<person_name>_<emotion>_exchange_<number>.mp4`
- Associated metadata including start time, duration, and emotion attempted

### Session Metadata

The session metadata includes:

- **Session ID**: Unique identifier for the session (timestamp + person name)
- **Person Name**: Name of the participant
- **Voice Type**: Type of voice used ("Emotional OpenAI TTS" or "QT Robot Built-in Voice")
- **Start/End Time**: When the session started and ended
- **Total Exchanges**: Number of conversation turns
- **Emotions Used**: Count of each emotion type used in the session

## Log Files

### session_log.json

This is the complete record of all session data in JSON format. It contains:

```json
{
  "metadata": {
    "session_id": "2023-06-10_14-35-28_John_Doe",
    "person_name": "John Doe",
    "voice_type": "Emotional OpenAI TTS",
    "start_time": 1686411328.456,
    "end_time": 1686411987.789,
    "total_exchanges": 7,
    "emotions_used": {
      "happy": 2,
      "sad": 1,
      "angry": 1,
      "fear": 0,
      "surprise": 1,
      "disgust": 1,
      "neutral": 1
    }
  },
  "speech_events": [
    {
      "timestamp": 1686411350.123,
      "speaker": "robot",
      "text": "You sit in a chair on the Zernike campus...",
      "emotion_attempted": "happy",
      "duration": 12.5,
      "exchange_number": 1
    },
    {
      "timestamp": 1686411380.456,
      "speaker": "user",
      "text": "I feel amazed by the beautiful surroundings...",
      "duration": 5.2,
      "exchange_number": 1
    },
    ...
  ],
  "video_events": [
    {
      "timestamp": 1686411348.789,
      "event_type": "start",
      "filename": "john_doe_happy_exchange_0.mp4",
      "exchange_number": 1,
      "emotion_attempted": "happy"
    },
    {
      "timestamp": 1686411386.123,
      "event_type": "stop",
      "filename": "john_doe_happy_exchange_0.mp4",
      "exchange_number": 1
    },
    ...
  ]
}
```

### metadata.json

This file contains just the metadata section from the session_log.json file for quick reference.

### summary.txt

A human-readable summary of the session including:
- Session information
- Emotion usage
- Chronological list of all speech events
- Details about each video recording

## Accessing and Analyzing the Data

### Viewing Video Recordings

The video recordings are standard MP4 files that can be viewed with any video player. The filenames indicate:
1. The participant's name
2. The emotion being elicited
3. The exchange number

For example, `john_doe_angry_exchange_5.mp4` shows the exchange where the system attempted to elicit "angry" emotion from John Doe in exchange #5.

### Analyzing Logs

You can analyze the logs in several ways:

1. **Manual Review**: Read the human-readable `summary.txt` file
2. **Automated Analysis**: Process the JSON data with your own scripts
3. **Included Analysis Tool**: Use the `analyze_logs.py` script in the project root

Example of using the analysis tool:

```bash
python analyze_logs.py --session 2023-06-10_14-35-28_John_Doe
```

This will generate statistics and visualizations for the specified session.

## Privacy and Data Management

- All data is stored locally on the machine running the experiment
- No data is transmitted to external servers
- Consider implementing a data retention policy (removing old sessions)
- Get informed consent from participants before recording

## Troubleshooting

If you encounter issues with logging or video recording:

1. **Missing Video Files**: Check if the camera was properly initialized
2. **Empty Speech Logs**: Verify microphone configuration and push-to-talk operation
3. **Corrupt JSON**: If a session was interrupted, try using the emergency recovery script:
   ```bash
   python recover_logs.py --session 2023-06-10_14-35-28_John_Doe
   ```

## Advanced Usage

You can customize the logging and video recording behavior by:

1. Modifying the `SessionLogger` class in `modules/logging/session_logger.py`
2. Adjusting video parameters in `modules/video_recording/video_recorder.py`
3. Creating custom analysis scripts that process the JSON log files
