# Audio Improvements Summary

## Overview
This document summarizes the audio improvements made for the voice comparison experiment.

## Changes Made

### 1. QT Robot Volume Control (68% Setting)

**File Modified:** `modules/text_to_speech/qtrobot_tts_engine.py`

**Changes:**
- Added ROS service import for volume control: `qt_robot_interface.srv.setting_setVolume`
- Implemented `_set_qtrobot_volume()` method to control QT Robot speaker volume
- Set default volume to 68% during initialization (optimized for experiment)
- Updated `set_volume()` method to actually control QT Robot volume

**Key Features:**
- Automatically sets QT Robot volume to 68% when TTS engine initializes
- Supports dynamic volume adjustment during runtime
- Graceful fallback when ROS services are not available

### 2. OpenAI TTS Audio Output

**File Status:** `modules/text_to_speech/openai_tts_engine.py`

**Current Configuration:**
- Uses standard pygame/system audio output (screen/external speakers)
- Maintains original audio routing through default system audio
- No modifications to audio routing - uses pygame as designed

**Key Features:**
- Standard pygame audio playback
- System fallback methods (mpg123, mplayer)
- Reliable audio output through default system audio device

### 3. Audio Output Configuration

**Result:**
- QT Robot TTS: Uses internal speaker at 68% volume
- OpenAI TTS: Uses standard system audio output (screen/external speakers)
- Different audio outputs for comparison experiment

## Technical Implementation

### QT Robot Volume Control
```python
# Service call to set volume
rospy.ServiceProxy('/qt_robot/setting/setVolume', setting_setVolume)
response = set_volume_service(volume=68)  # 68% volume
```

### OpenAI TTS Audio Output
```python
# Standard pygame audio playback
pygame.mixer.music.load(audio_file)
pygame.mixer.music.set_volume(self.volume)
pygame.mixer.music.play()
```

### Usage in Main Application
```python
# OpenAI TTS with standard audio output
tts_engine = OpenAITTSEngine(
    api_key=openai_api_key,
    model="gpt-4o-mini-tts"
)

# QT Robot TTS with 68% volume
qtrobot_tts = QTRobotTTSEngine()  # Automatically sets 68% volume
```

## Files Modified

1. **`modules/text_to_speech/qtrobot_tts_engine.py`**
   - Added volume control functionality
   - Set default volume to 68%

2. **`modules/text_to_speech/openai_tts_engine.py`**
   - Reverted to standard audio output
   - Removed QT Robot audio routing attempts

3. **`main.py`**
   - Reverted to standard OpenAI TTS initialization

4. **`test_tts.py`**
   - Reverted to standard OpenAI TTS usage

5. **`demo_voice_selection.py`**
   - Reverted to standard OpenAI TTS usage

## New Test Files

1. **`test_audio_routing.py`**
   - Comprehensive test suite for volume control and audio output
   - Tests QT Robot volume control (68% setting)
   - Tests OpenAI TTS standard audio output
   - Provides audio comparison between both systems

2. **`diagnose_audio.py`**
   - Audio diagnosis tool to identify audio devices and routing
   - Tests different ALSA and PulseAudio devices
   - Helps verify which speaker each TTS system uses
   - Interactive testing for audio verification

## Benefits for Experiment

### 1. Volume Control
- QT Robot voice set to 68% (optimized for experiment)
- Consistent volume level for QT Robot's internal speaker
- Proper volume control through ROS services

### 2. Audio Output Separation
- QT Robot TTS: Uses internal speaker (68% volume)
- OpenAI TTS: Uses standard system audio (screen/external speakers)
- Clear distinction between voice types and audio sources

### 3. Reliable Audio Playback
- QT Robot TTS: Reliable internal speaker routing
- OpenAI TTS: Standard pygame/system audio with fallbacks
- Both systems work independently without conflicts

## Testing

Run the audio test suite to verify functionality:

```bash
python3 test_audio_routing.py
```

This will test:
- QT Robot volume control (68% setting)
- OpenAI TTS standard audio output
- Side-by-side comparison of both systems

For audio diagnosis and troubleshooting:

```bash
python3 diagnose_audio.py
```

This will help:
- Identify available audio devices
- Test which speaker each TTS system uses
- Verify audio routing and device configuration

## Troubleshooting

### If QT Robot Audio Services Are Not Available:
- QT Robot TTS will run in simulation mode
- Check ROS environment and QT Robot services
- Verify QT Robot interface is running

### If Volume Control Doesn't Work:
- Verify `/qt_robot/setting/setVolume` service is available
- Check ROS service permissions
- Ensure QT Robot interface is running

### If OpenAI TTS Has No Audio:
- Check pygame installation and audio drivers
- Verify system audio configuration
- Test with system audio playback tools

## Experiment Usage

The voice selection system now provides:

1. **Option 1 (Emotional Voice):**
   - OpenAI TTS with emotional expression
   - Uses standard system audio output (screen/external speakers)
   - Emotional voice characteristics and expression

2. **Option 2 (Robotic Voice):**
   - QT Robot built-in TTS
   - Uses internal speaker at 68% volume
   - Non-emotional, robotic characteristics

The two options use different audio outputs, allowing for clear distinction between the emotional and robotic voice types in your experiment.
