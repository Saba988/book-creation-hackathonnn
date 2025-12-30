---
sidebar_position: 2
title: 'Voice-to-Action Pipeline using Whisper'
---

# Voice-to-Action Pipeline using Whisper

This chapter introduces the Voice-to-Action pipeline using Whisper, a state-of-the-art speech recognition system. You'll learn how to convert human language commands into actionable robot commands, forming the foundation of the Vision-Language-Action (VLA) system.

## Introduction to Voice-to-Action Systems

Voice-to-Action systems bridge the gap between human language and robot execution. The pipeline typically involves:

1. **Speech Recognition**: Converting audio input to text
2. **Natural Language Processing**: Understanding the intent and entities
3. **Action Mapping**: Translating commands into robot actions
4. **Execution**: Carrying out the mapped actions

### Key Components of Voice-to-Action Systems

- **Automatic Speech Recognition (ASR)**: Converts spoken language to text
- **Natural Language Understanding (NLU)**: Interprets the meaning of the text
- **Dialog Manager**: Manages the conversation flow
- **Action Generator**: Maps understood commands to robot actions
- **Execution Engine**: Executes the generated actions

## Whisper for Robotics Applications

Whisper, developed by OpenAI, provides robust speech recognition capabilities that are well-suited for robotics applications. Its key advantages include:

- **Multilingual Support**: Works with multiple languages
- **Robustness**: Handles various accents and background noise
- **Accuracy**: High transcription accuracy across different domains
- **Open Source**: Available for customization and integration

### Whisper Architecture

Whisper uses a transformer-based architecture with the following components:

```
Audio Input → Feature Extraction → Encoder → Decoder → Text Output
```

The model processes audio in 30-second chunks and can perform speech recognition, translation, and language identification simultaneously.

### Whisper in Robotics Context

For robotics applications, Whisper can be customized and optimized:

```python
import whisper
import torch

# Load the Whisper model
model = whisper.load_model("base")

def transcribe_speech(audio_file):
    # Load audio and pad/trim it to fit 30 seconds
    audio = whisper.load_audio(audio_file)
    audio = whisper.pad_or_trim(audio)

    # Make log-Mel spectrogram and move to the same device as the model
    mel = whisper.log_mel_spectrogram(audio).to(model.device)

    # Decode the audio
    options = whisper.DecodingOptions()
    result = whisper.decode(model, mel, options)

    return result.text

# Example usage
command = transcribe_speech("robot_command.wav")
print(f"Recognized command: {command}")
```

## Implementing Voice-to-Action Pipeline

### Architecture Overview

The complete Voice-to-Action pipeline for robotics includes:

```
Speech Input → Whisper ASR → NLU Processing → Action Mapping → Robot Execution
```

### Step 1: Speech Recognition with Whisper

```python
import whisper
import pyaudio
import wave
import numpy as np

class WhisperRobotController:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.record_seconds = 5

    def record_audio(self, filename):
        """Record audio from microphone"""
        p = pyaudio.PyAudio()

        stream = p.open(format=self.audio_format,
                       channels=self.channels,
                       rate=self.rate,
                       input=True,
                       frames_per_buffer=self.chunk)

        print("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Finished recording")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save the recorded data as a WAV file
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.audio_format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

    def transcribe_audio(self, audio_file):
        """Transcribe audio using Whisper"""
        result = self.model.transcribe(audio_file)
        return result["text"]

# Example usage
controller = WhisperRobotController()
controller.record_audio("command.wav")
command = controller.transcribe_audio("command.wav")
print(f"Transcribed command: {command}")
```

### Step 2: Natural Language Understanding

After transcription, we need to understand the intent and extract relevant information:

```python
import re
from dataclasses import dataclass
from typing import Optional

@dataclass
class RobotCommand:
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    parameters: dict = None

class CommandParser:
    def __init__(self):
        # Define action patterns
        self.action_patterns = {
            'move': [r'go to (.+)', r'move to (.+)', r'go (.+)', r'walk to (.+)'],
            'grasp': [r'pick up (.+)', r'grasp (.+)', r'get (.+)', r'take (.+)'],
            'place': [r'put (.+) on (.+)', r'place (.+) on (.+)'],
            'follow': [r'follow (.+)', r'come after (.+)'],
            'stop': [r'stop', r'wait', r'hold on'],
            'explore': [r'explore', r'look around', r'scan']
        }

    def parse_command(self, text: str) -> Optional[RobotCommand]:
        """Parse natural language command into structured format"""
        text = text.lower().strip()

        for action, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    if action in ['place']:
                        # Handle multi-group patterns
                        target = match.group(1) if len(match.groups()) > 0 else None
                        location = match.group(2) if len(match.groups()) > 1 else None
                        return RobotCommand(action=action, target=target, location=location)
                    else:
                        target = match.group(1) if len(match.groups()) > 0 else None
                        return RobotCommand(action=action, target=target)

        # If no pattern matches, return None or a default command
        return None

# Example usage
parser = CommandParser()
command_text = "Please go to the kitchen and pick up the red cup"
command = parser.parse_command(command_text)
print(f"Parsed command: {command}")
```

### Step 3: Action Mapping

Map the parsed command to specific robot actions:

```python
class ActionMapper:
    def __init__(self):
        self.location_map = {
            'kitchen': [1.5, 0.0, 0.0],
            'living room': [0.0, 2.0, 0.0],
            'bedroom': [-1.5, 0.0, 0.0],
            'office': [0.0, -2.0, 0.0],
            'dining room': [2.0, 1.0, 0.0]
        }

        self.object_map = {
            'red cup': 'cup_red_01',
            'blue bottle': 'bottle_blue_01',
            'green box': 'box_green_01',
            'apple': 'fruit_apple_01'
        }

    def map_command_to_action(self, command: RobotCommand):
        """Map parsed command to robot-specific action"""
        if command.action == 'move':
            if command.target in self.location_map:
                target_pos = self.location_map[command.target]
                return {
                    'action_type': 'navigation',
                    'target_position': target_pos,
                    'description': f'Moving to {command.target}'
                }
            else:
                # Try to interpret as coordinates or relative position
                return {
                    'action_type': 'navigation',
                    'target_description': command.target,
                    'description': f'Moving toward {command.target}'
                }

        elif command.action == 'grasp':
            if command.target in self.object_map:
                target_obj = self.object_map[command.target]
                return {
                    'action_type': 'manipulation',
                    'action': 'grasp',
                    'target_object': target_obj,
                    'description': f'Grasping {command.target}'
                }
            else:
                return {
                    'action_type': 'manipulation',
                    'action': 'grasp',
                    'target_description': command.target,
                    'description': f'Looking for and grasping {command.target}'
                }

        elif command.action == 'place':
            return {
                'action_type': 'manipulation',
                'action': 'place',
                'target_object': command.target,
                'target_location': command.location,
                'description': f'Placing {command.target} on {command.location}'
            }

        # Add more action mappings as needed
        return {
            'action_type': 'unknown',
            'original_command': command,
            'description': 'Command not recognized'
        }

# Example usage
mapper = ActionMapper()
command = RobotCommand(action='move', target='kitchen')
action = mapper.map_command_to_action(command)
print(f"Mapped action: {action}")
```

## Integration with ROS 2

For robotics applications, we typically integrate with ROS 2 for communication:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import AudioData
import json

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Initialize Whisper controller and command processing
        self.whisper_controller = WhisperRobotController()
        self.command_parser = CommandParser()
        self.action_mapper = ActionMapper()

        # Publishers and subscribers
        self.command_publisher = self.create_publisher(String, 'robot_commands', 10)
        self.audio_subscription = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Timer for periodic processing
        self.timer = self.create_timer(1.0, self.process_commands)

        self.pending_commands = []

    def audio_callback(self, msg):
        """Handle incoming audio data"""
        # Process audio data and add to pending commands
        # This is a simplified example - real implementation would be more complex
        pass

    def process_commands(self):
        """Process any pending voice commands"""
        if self.pending_commands:
            for command_data in self.pending_commands:
                # Process the command through the pipeline
                parsed_command = self.command_parser.parse_command(command_data['text'])
                if parsed_command:
                    mapped_action = self.action_mapper.map_command_to_action(parsed_command)

                    # Publish the action to the robot
                    action_msg = String()
                    action_msg.data = json.dumps(mapped_action)
                    self.command_publisher.publish(action_msg)

                    self.get_logger().info(f'Published command: {mapped_action}')

            self.pending_commands.clear()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceToActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Building a Voice-Controlled Robot

### Objective
Implement a complete voice-to-action pipeline that receives spoken commands and translates them into robot actions.

### Steps

1. **Setup Whisper Environment**
   - Install Whisper and required dependencies
   - Configure audio input (microphone or file)
   - Test basic transcription capabilities

2. **Implement Command Processing**
   - Create a command parser that understands robot commands
   - Map natural language to structured robot commands
   - Handle various command formats and variations

3. **Integrate with Robot Platform**
   - Connect to a robot platform (simulated or physical)
   - Implement action execution based on parsed commands
   - Test with various voice commands

### Example Implementation

```python
# Complete example combining all components
class CompleteVoiceToActionSystem:
    def __init__(self):
        self.whisper_controller = WhisperRobotController()
        self.command_parser = CommandParser()
        self.action_mapper = ActionMapper()

    def process_voice_command(self, audio_file=None):
        """Complete pipeline from voice to action"""
        # Option 1: Use provided audio file
        if audio_file:
            text = self.whisper_controller.transcribe_audio(audio_file)
        # Option 2: Record from microphone
        else:
            self.whisper_controller.record_audio("temp_command.wav")
            text = self.whisper_controller.transcribe_audio("temp_command.wav")

        print(f"Recognized text: {text}")

        # Parse the command
        command = self.command_parser.parse_command(text)
        if command:
            print(f"Parsed command: {command}")

            # Map to robot action
            action = self.action_mapper.map_command_to_action(command)
            print(f"Mapped action: {action}")

            return action
        else:
            print("Could not parse the command")
            return None

# Example usage
system = CompleteVoiceToActionSystem()

# Process a command from a file
# action = system.process_voice_command("command.wav")

# Or record and process a command from microphone
action = system.process_voice_command()  # This will record from microphone
```

### Expected Outcome

The voice-to-action system should:
- Accurately transcribe spoken commands
- Parse natural language into structured robot commands
- Map commands to appropriate robot actions
- Execute actions with measurable accuracy (target: 90% for simple commands)

## Performance Considerations

### Accuracy Optimization

- **Microphone Quality**: Use high-quality microphones for better audio input
- **Noise Reduction**: Implement noise reduction techniques for cleaner audio
- **Custom Training**: Fine-tune Whisper for domain-specific vocabulary
- **Context Awareness**: Use contextual information to improve understanding

### Real-time Processing

- **Latency**: Minimize processing delay for responsive interaction
- **Resource Management**: Optimize for computational efficiency
- **Streaming**: Implement streaming processing for continuous input

## Troubleshooting Common Issues

### Speech Recognition Issues

```python
# Handle poor audio quality
def robust_transcription(audio_file, model):
    try:
        result = model.transcribe(audio_file)
        confidence = result.get("confidence", 1.0)  # If available

        if confidence < 0.7:  # Low confidence threshold
            print("Low confidence transcription - request repeat")
            return None

        return result["text"]
    except Exception as e:
        print(f"Transcription error: {e}")
        return None
```

### Command Parsing Issues

- **Ambiguous Commands**: Implement disambiguation strategies
- **Partial Commands**: Handle incomplete or interrupted commands
- **Error Recovery**: Provide feedback and recovery options

## Summary

The Voice-to-Action pipeline using Whisper provides a robust foundation for translating human language into robot commands. By combining accurate speech recognition with intelligent command parsing and action mapping, we can create natural interfaces for robot control.

The key components include:
- Whisper-based speech recognition for accurate transcription
- Natural language understanding for command interpretation
- Action mapping for robot-specific command execution
- Integration with ROS 2 for real-world robot control

In the next chapter, we'll explore LLM-based cognitive planning for more sophisticated robot behavior.