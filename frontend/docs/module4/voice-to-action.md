---
sidebar_position: 2
---

# Voice-to-Action

**From Spoken Words to Robot Commands with OpenAI Whisper**

---

## Overview

A humanoid robot operating in a home, hospital, or warehouse needs to understand spoken commands from the humans around it. Typing instructions into a terminal is not an option when your hands are full, when you are across the room, or when the person giving commands is not a robotics engineer. In this chapter, you will deploy OpenAI Whisper as a ROS 2 node for real-time speech-to-text, build a wake word detection system, parse spoken commands into structured intents, and connect the full voice pipeline to your robot's action system.

## OpenAI Whisper

Whisper is a general-purpose speech recognition model trained on 680,000 hours of multilingual audio. It handles accents, background noise, and technical vocabulary with near-human accuracy.

### Key Capabilities

| Feature | Specification |
|---------|--------------|
| **Languages** | 99+ languages with automatic detection |
| **Accuracy** | 5-8% word error rate (English), comparable to human transcribers |
| **Noise robustness** | Trained on diverse audio including noisy environments |
| **Model sizes** | tiny (39M) to large-v3 (1.5B parameters) |
| **Latency** | 0.5-3s depending on model size and hardware |
| **Input** | 16kHz mono audio, any duration |

### Model Size Selection

| Model | Parameters | Speed (Jetson Orin) | Accuracy | Use Case |
|-------|-----------|-------------------|----------|----------|
| `tiny` | 39M | ~0.3s per utterance | Good | Wake word + simple commands |
| `base` | 74M | ~0.5s per utterance | Better | General voice control |
| `small` | 244M | ~1.2s per utterance | Great | Noisy environments |
| `medium` | 769M | ~2.5s per utterance | Excellent | Complex instructions |
| `large-v3` | 1.5B | ~5.0s per utterance | Best | Offline/batch processing |

For a humanoid robot, **`small`** offers the best balance of accuracy and latency for real-time interaction.

### Installation

```bash
pip install openai-whisper
pip install sounddevice numpy

# For GPU acceleration (recommended)
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cu121

# For faster inference with CTranslate2
pip install faster-whisper
```

## The Whisper ROS 2 Node

### Core Speech Recognition Node

```python
# humanoid_voice/whisper_node.py
"""ROS 2 node for real-time speech recognition using OpenAI Whisper."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import numpy as np
import threading
import queue

# Use faster-whisper for better performance
from faster_whisper import WhisperModel


class WhisperNode(Node):
    """Listens for speech and publishes transcriptions."""

    def __init__(self):
        super().__init__('whisper_node')

        # Parameters
        self.declare_parameter('model_size', 'small')
        self.declare_parameter('language', 'en')
        self.declare_parameter('device', 'cuda')        # 'cuda' or 'cpu'
        self.declare_parameter('compute_type', 'float16')  # 'float16' for GPU
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('silence_threshold', 0.01)
        self.declare_parameter('silence_duration', 1.5)  # seconds of silence to end utterance
        self.declare_parameter('max_duration', 15.0)     # max recording duration

        model_size = self.get_parameter('model_size').value
        device = self.get_parameter('device').value
        compute_type = self.get_parameter('compute_type').value

        self.get_logger().info(f'Loading Whisper model: {model_size} on {device}')
        self.model = WhisperModel(
            model_size,
            device=device,
            compute_type=compute_type,
        )
        self.get_logger().info('Whisper model loaded')

        # Publishers
        self.transcript_pub = self.create_publisher(String, '/humanoid/voice/transcript', 10)
        self.status_pub = self.create_publisher(String, '/humanoid/voice/status', 10)

        # Service to trigger listening
        self.listen_srv = self.create_service(Trigger, '/humanoid/voice/listen', self.handle_listen)

        # Audio configuration
        self.sample_rate = self.get_parameter('sample_rate').value
        self.silence_threshold = self.get_parameter('silence_threshold').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.max_duration = self.get_parameter('max_duration').value
        self.language = self.get_parameter('language').value

        # Audio queue for thread-safe recording
        self.audio_queue = queue.Queue()
        self.is_listening = False

        self.get_logger().info('Whisper node ready — call /humanoid/voice/listen to start')

    def handle_listen(self, request, response):
        """Service handler to trigger a single listen-and-transcribe cycle."""
        if self.is_listening:
            response.success = False
            response.message = 'Already listening'
            return response

        self.is_listening = True
        self.publish_status('LISTENING')

        # Record audio in a separate thread
        thread = threading.Thread(target=self.record_and_transcribe)
        thread.start()

        response.success = True
        response.message = 'Listening started'
        return response

    def record_and_transcribe(self):
        """Record audio until silence detected, then transcribe."""
        try:
            import sounddevice as sd

            audio_chunks = []
            silence_samples = 0
            silence_limit = int(self.silence_duration * self.sample_rate)
            max_samples = int(self.max_duration * self.sample_rate)
            total_samples = 0

            self.get_logger().info('Recording... speak now')

            def audio_callback(indata, frames, time_info, status):
                self.audio_queue.put(indata.copy())

            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32',
                callback=audio_callback,
                blocksize=int(self.sample_rate * 0.1),  # 100ms blocks
            ):
                while total_samples < max_samples:
                    try:
                        chunk = self.audio_queue.get(timeout=0.5)
                    except queue.Empty:
                        continue

                    audio_chunks.append(chunk)
                    total_samples += len(chunk)

                    # Check for silence
                    rms = np.sqrt(np.mean(chunk ** 2))
                    if rms < self.silence_threshold:
                        silence_samples += len(chunk)
                    else:
                        silence_samples = 0

                    if silence_samples >= silence_limit and total_samples > self.sample_rate:
                        break  # Enough silence after speech — stop recording

            if not audio_chunks:
                self.publish_status('NO_AUDIO')
                return

            # Concatenate and transcribe
            audio = np.concatenate(audio_chunks).flatten()
            self.get_logger().info(
                f'Recorded {len(audio) / self.sample_rate:.1f}s of audio, transcribing...'
            )

            self.publish_status('TRANSCRIBING')
            segments, info = self.model.transcribe(
                audio,
                language=self.language,
                beam_size=5,
                vad_filter=True,  # Filter out non-speech segments
            )

            transcript = ' '.join(seg.text.strip() for seg in segments)

            if transcript:
                self.get_logger().info(f'Transcript: "{transcript}"')
                msg = String()
                msg.data = transcript
                self.transcript_pub.publish(msg)
                self.publish_status('TRANSCRIPT_READY')
            else:
                self.get_logger().info('No speech detected')
                self.publish_status('NO_SPEECH')

        except Exception as e:
            self.get_logger().error(f'Recording/transcription error: {e}')
            self.publish_status('ERROR')
        finally:
            self.is_listening = False

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Continuous Listening with Wake Word Detection

In practice, the robot should not transcribe everything it hears. It should listen for a **wake word** (like "Hey Robot") and only process the command that follows.

### Wake Word Detector

```python
# humanoid_voice/wake_word_detector.py
"""Lightweight wake word detection to trigger Whisper transcription."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import numpy as np
import threading

from faster_whisper import WhisperModel


class WakeWordDetector(Node):
    """Continuously listens for a wake word, then triggers full transcription."""

    def __init__(self):
        super().__init__('wake_word_detector')

        self.declare_parameter('wake_words', ['hey robot', 'ok robot', 'robot'])
        self.declare_parameter('model_size', 'tiny')  # Fast model for wake word
        self.declare_parameter('check_interval', 2.0)   # Check every 2 seconds
        self.declare_parameter('device', 'cuda')

        self.wake_words = self.get_parameter('wake_words').value
        device = self.get_parameter('device').value

        # Tiny model for fast wake word detection
        self.get_logger().info('Loading tiny Whisper model for wake word detection')
        self.model = WhisperModel('tiny', device=device, compute_type='float16')

        # Publisher: detected wake word triggers
        self.wake_pub = self.create_publisher(Bool, '/humanoid/voice/wake_detected', 10)

        # Client: trigger the main Whisper node
        self.listen_client = self.create_client(Trigger, '/humanoid/voice/listen')

        self.is_active = True
        self.listen_thread = threading.Thread(target=self.continuous_listen, daemon=True)
        self.listen_thread.start()

        self.get_logger().info(
            f'Wake word detector active. Words: {self.wake_words}'
        )

    def continuous_listen(self):
        """Continuously capture short audio segments and check for wake words."""
        import sounddevice as sd

        sample_rate = 16000
        segment_duration = 2.0  # 2-second rolling segments
        segment_samples = int(sample_rate * segment_duration)

        while self.is_active:
            try:
                # Record a short segment
                audio = sd.rec(
                    segment_samples,
                    samplerate=sample_rate,
                    channels=1,
                    dtype='float32',
                )
                sd.wait()
                audio = audio.flatten()

                # Quick transcription with tiny model
                segments, _ = self.model.transcribe(
                    audio,
                    language='en',
                    beam_size=1,  # Fastest setting
                    vad_filter=True,
                )

                text = ' '.join(seg.text.strip().lower() for seg in segments)

                # Check for wake word
                for wake_word in self.wake_words:
                    if wake_word in text:
                        self.get_logger().info(f'Wake word detected: "{wake_word}"')

                        # Publish wake detection
                        wake_msg = Bool()
                        wake_msg.data = True
                        self.wake_pub.publish(wake_msg)

                        # Trigger the main Whisper node for full transcription
                        self.trigger_listen()
                        break

            except Exception as e:
                self.get_logger().error(f'Wake word detection error: {e}')

    def trigger_listen(self):
        """Call the Whisper node's listen service."""
        if not self.listen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Whisper listen service not available')
            return

        future = self.listen_client.call_async(Trigger.Request())
        # Don't block — we'll resume wake word detection after the transcription

    def destroy_node(self):
        self.is_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WakeWordDetector())
    rclpy.shutdown()
```

## Intent Parsing

Raw transcriptions need to be parsed into structured commands the robot can act on.

### Rule-Based Intent Parser

For well-defined command vocabularies, a rule-based parser is fast and reliable:

```python
# humanoid_voice/intent_parser.py
"""Parse voice transcripts into structured robot commands."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import json


class IntentParser(Node):
    """Converts natural language transcripts into structured intents."""

    # Navigation targets the robot knows about
    KNOWN_LOCATIONS = {
        'kitchen': (5.0, 2.0),
        'living room': (0.0, 0.0),
        'bedroom': (-3.0, 4.0),
        'front door': (8.0, 0.0),
        'table': (2.0, 1.0),
        'desk': (1.0, -2.0),
        'charging station': (-1.0, -1.0),
    }

    # Objects the robot can manipulate
    KNOWN_OBJECTS = [
        'cup', 'glass', 'bottle', 'plate', 'bowl',
        'book', 'remote', 'phone', 'keys', 'bag',
        'towel', 'pillow', 'box', 'pen', 'ball',
    ]

    # Intent patterns
    PATTERNS = [
        # Navigation: "go to the kitchen", "walk to the table"
        {
            'pattern': r'(?:go|walk|move|navigate|come)\s+to\s+(?:the\s+)?(.+)',
            'intent': 'navigate',
            'extract': 'location',
        },
        # Fetch: "bring me the cup", "get the book", "fetch the remote"
        {
            'pattern': r'(?:bring|get|fetch|grab|pick up)\s+(?:me\s+)?(?:the\s+|a\s+)?(.+)',
            'intent': 'fetch',
            'extract': 'object',
        },
        # Place: "put the cup on the table"
        {
            'pattern': r'(?:put|place|set|drop)\s+(?:the\s+|a\s+)?(.+?)\s+(?:on|at|in|near)\s+(?:the\s+)?(.+)',
            'intent': 'place',
            'extract': 'object_and_location',
        },
        # Look: "look at the door", "find the keys"
        {
            'pattern': r'(?:look\s+at|look\s+for|find|search\s+for|locate)\s+(?:the\s+|a\s+)?(.+)',
            'intent': 'search',
            'extract': 'object',
        },
        # Stop: "stop", "halt", "freeze"
        {
            'pattern': r'(?:stop|halt|freeze|cancel|abort)',
            'intent': 'stop',
            'extract': None,
        },
        # Status: "where are you", "what do you see"
        {
            'pattern': r'(?:where\s+are\s+you|what\s+do\s+you\s+see|status|report)',
            'intent': 'status',
            'extract': None,
        },
        # Say: "say hello", "tell them I'm coming"
        {
            'pattern': r'(?:say|tell|announce|speak)\s+(.+)',
            'intent': 'say',
            'extract': 'text',
        },
    ]

    def __init__(self):
        super().__init__('intent_parser')

        self.create_subscription(
            String, '/humanoid/voice/transcript', self.on_transcript, 10
        )

        self.intent_pub = self.create_publisher(
            String, '/humanoid/voice/intent', 10
        )

        self.get_logger().info('Intent parser ready')

    def on_transcript(self, msg: String):
        transcript = msg.data.strip().lower()
        self.get_logger().info(f'Parsing: "{transcript}"')

        intent = self.parse(transcript)

        if intent:
            intent_msg = String()
            intent_msg.data = json.dumps(intent)
            self.intent_pub.publish(intent_msg)
            self.get_logger().info(f'Intent: {json.dumps(intent, indent=2)}')
        else:
            self.get_logger().warn(f'Could not parse intent from: "{transcript}"')

    def parse(self, text: str) -> dict:
        """Parse text into a structured intent."""
        for pattern_def in self.PATTERNS:
            match = re.search(pattern_def['pattern'], text)
            if not match:
                continue

            intent = {
                'type': pattern_def['intent'],
                'raw_text': text,
                'confidence': 0.8,  # Rule-based confidence
            }

            extract = pattern_def['extract']

            if extract == 'location':
                location_text = match.group(1).strip()
                intent['location'] = self.resolve_location(location_text)
                intent['location_name'] = location_text

            elif extract == 'object':
                object_text = match.group(1).strip()
                intent['object'] = self.resolve_object(object_text)
                intent['object_name'] = object_text

            elif extract == 'object_and_location':
                intent['object'] = self.resolve_object(match.group(1).strip())
                intent['object_name'] = match.group(1).strip()
                intent['location'] = self.resolve_location(match.group(2).strip())
                intent['location_name'] = match.group(2).strip()

            elif extract == 'text':
                intent['text'] = match.group(1).strip()

            return intent

        # Fallback: send to LLM for complex parsing
        return {
            'type': 'complex',
            'raw_text': text,
            'confidence': 0.3,
            'note': 'Requires LLM parsing',
        }

    def resolve_location(self, text: str) -> dict:
        """Match text to a known location."""
        for name, coords in self.KNOWN_LOCATIONS.items():
            if name in text:
                return {'name': name, 'x': coords[0], 'y': coords[1], 'resolved': True}
        return {'name': text, 'resolved': False}

    def resolve_object(self, text: str) -> dict:
        """Match text to a known object class."""
        for obj in self.KNOWN_OBJECTS:
            if obj in text:
                return {'class': obj, 'resolved': True}

        # Check for color + object patterns
        color_match = re.match(r'(red|blue|green|yellow|white|black)\s+(.+)', text)
        if color_match:
            color, obj_text = color_match.groups()
            for obj in self.KNOWN_OBJECTS:
                if obj in obj_text:
                    return {'class': obj, 'color': color, 'resolved': True}

        return {'class': text, 'resolved': False}


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IntentParser())
    rclpy.shutdown()
```

### LLM-Enhanced Intent Parsing

For complex or ambiguous commands, fall back to an LLM:

```python
# humanoid_voice/llm_intent_parser.py
"""LLM-based intent parsing for complex voice commands."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from openai import OpenAI


class LLMIntentParser(Node):
    """Uses an LLM to parse complex voice commands into structured intents."""

    SYSTEM_PROMPT = """You are an intent parser for a humanoid robot. Given a voice command,
extract a structured intent in JSON format.

Available intent types:
- navigate: {type: "navigate", location: "kitchen", coordinates: [x, y]}
- fetch: {type: "fetch", object: "red cup", destination: "user"}
- place: {type: "place", object: "cup", location: "table"}
- search: {type: "search", object: "keys"}
- say: {type: "say", text: "Hello!"}
- stop: {type: "stop"}
- complex: {type: "complex", steps: [{...}, {...}]} — for multi-step tasks

Known locations: kitchen (5,2), living room (0,0), bedroom (-3,4), front door (8,0),
table (2,1), desk (1,-2), charging station (-1,-1).

Known objects: cup, glass, bottle, plate, bowl, book, remote, phone, keys, bag,
towel, pillow, box, pen, ball.

Rules:
1. Always return valid JSON
2. If the command is ambiguous, include a "clarification" field
3. For multi-step tasks, break into sequential steps
4. Never invent locations or objects not in the known lists — mark as unresolved"""

    def __init__(self):
        super().__init__('llm_intent_parser')

        self.declare_parameter('api_key_env', 'OPENAI_API_KEY')
        self.declare_parameter('model', 'gpt-4o-mini')

        import os
        api_key = os.environ.get(self.get_parameter('api_key_env').value, '')
        self.client = OpenAI(api_key=api_key) if api_key else None
        self.model_name = self.get_parameter('model').value

        self.create_subscription(
            String, '/humanoid/voice/intent', self.on_intent, 10
        )
        self.parsed_pub = self.create_publisher(
            String, '/humanoid/voice/parsed_intent', 10
        )

        if self.client:
            self.get_logger().info(f'LLM intent parser ready (model: {self.model_name})')
        else:
            self.get_logger().warn('No API key found — LLM parsing disabled')

    def on_intent(self, msg: String):
        intent = json.loads(msg.data)

        # Only process intents that need LLM parsing
        if intent.get('type') != 'complex' or not self.client:
            self.parsed_pub.publish(msg)  # Pass through
            return

        raw_text = intent.get('raw_text', '')
        self.get_logger().info(f'LLM parsing: "{raw_text}"')

        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": self.SYSTEM_PROMPT},
                    {"role": "user", "content": raw_text},
                ],
                response_format={"type": "json_object"},
                temperature=0.1,  # Low temperature for consistent parsing
                max_tokens=500,
            )

            parsed = json.loads(response.choices[0].message.content)
            parsed['source'] = 'llm'
            parsed['raw_text'] = raw_text

            result = String()
            result.data = json.dumps(parsed)
            self.parsed_pub.publish(result)
            self.get_logger().info(f'LLM parsed: {json.dumps(parsed, indent=2)}')

        except Exception as e:
            self.get_logger().error(f'LLM parsing failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LLMIntentParser())
    rclpy.shutdown()
```

## Voice Feedback: Text-to-Speech

A conversational robot must also speak. Use a TTS engine to provide voice feedback:

```python
# humanoid_voice/tts_node.py
"""Text-to-speech node for robot voice feedback."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading


class TTSNode(Node):
    """Converts text to speech for robot feedback."""

    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('engine', 'piper')  # 'piper', 'espeak', or 'gtts'
        self.declare_parameter('voice', 'en_US-lessac-medium')
        self.engine = self.get_parameter('engine').value

        self.create_subscription(String, '/humanoid/voice/say', self.on_say, 10)
        self.speaking = False

        self.get_logger().info(f'TTS node ready (engine: {self.engine})')

    def on_say(self, msg: String):
        if self.speaking:
            self.get_logger().warn('Already speaking, queuing...')
            return

        text = msg.data
        self.get_logger().info(f'Speaking: "{text}"')

        thread = threading.Thread(target=self.speak, args=(text,))
        thread.start()

    def speak(self, text: str):
        self.speaking = True
        try:
            if self.engine == 'piper':
                # Piper: fast, local, good quality
                subprocess.run(
                    f'echo "{text}" | piper --model en_US-lessac-medium --output-raw | '
                    f'aplay -r 22050 -f S16_LE -c 1',
                    shell=True, check=True,
                )
            elif self.engine == 'espeak':
                # espeak: always available, robotic voice
                subprocess.run(['espeak', '-s', '150', text], check=True)
        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')
        finally:
            self.speaking = False


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TTSNode())
    rclpy.shutdown()
```

## The Complete Voice Pipeline

### Architecture

```
┌───────────────────────────────────────────────────────────┐
│                  VOICE PIPELINE                            │
│                                                             │
│  Microphone ──► Wake Word Detector (tiny Whisper)          │
│                      │                                      │
│                      │ wake detected                        │
│                      ▼                                      │
│                 Whisper Node (small model)                  │
│                      │                                      │
│                      │ /humanoid/voice/transcript           │
│                      ▼                                      │
│                 Intent Parser (rule-based)                  │
│                      │                                      │
│                      ├──► Simple intents → Action Executor  │
│                      │                                      │
│                      └──► Complex intents                   │
│                              │                              │
│                              │ /humanoid/voice/intent       │
│                              ▼                              │
│                         LLM Intent Parser                  │
│                              │                              │
│                              │ /humanoid/voice/parsed_intent│
│                              ▼                              │
│                        Action Executor                     │
│                              │                              │
│                              │ ROS 2 actions               │
│                              ▼                              │
│                     Robot Executes Task                    │
│                              │                              │
│                              │ feedback                     │
│                              ▼                              │
│                         TTS Node                           │
│                     "Task completed"                       │
└───────────────────────────────────────────────────────────┘
```

### Launch File

```python
# launch/voice_pipeline.launch.py
"""Launch the complete voice command pipeline."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Wake word detector (always running)
        Node(
            package='humanoid_voice',
            executable='wake_word_detector',
            name='wake_word_detector',
            parameters=[{
                'wake_words': ['hey robot', 'ok robot'],
                'model_size': 'tiny',
                'device': 'cuda',
                'check_interval': 2.0,
            }],
            output='screen',
        ),

        # Main Whisper transcription (triggered by wake word)
        Node(
            package='humanoid_voice',
            executable='whisper_node',
            name='whisper_node',
            parameters=[{
                'model_size': 'small',
                'language': 'en',
                'device': 'cuda',
                'silence_duration': 1.5,
                'max_duration': 15.0,
            }],
            output='screen',
        ),

        # Intent parser
        Node(
            package='humanoid_voice',
            executable='intent_parser',
            name='intent_parser',
            output='screen',
        ),

        # LLM intent parser (for complex commands)
        Node(
            package='humanoid_voice',
            executable='llm_intent_parser',
            name='llm_intent_parser',
            parameters=[{
                'model': 'gpt-4o-mini',
            }],
            output='screen',
        ),

        # Text-to-speech feedback
        Node(
            package='humanoid_voice',
            executable='tts_node',
            name='tts_node',
            parameters=[{
                'engine': 'piper',
            }],
            output='screen',
        ),
    ])
```

### Testing the Pipeline

```bash
# Launch the voice pipeline
ros2 launch humanoid_voice voice_pipeline.launch.py

# Test without microphone — publish a transcript directly
ros2 topic pub --once /humanoid/voice/transcript std_msgs/String \
  '{data: "go to the kitchen"}'

# Check the parsed intent
ros2 topic echo /humanoid/voice/intent
# {"type": "navigate", "location": {"name": "kitchen", "x": 5.0, "y": 2.0, "resolved": true}, ...}

# Test a complex command
ros2 topic pub --once /humanoid/voice/transcript std_msgs/String \
  '{data: "go to the kitchen and bring me a glass of water"}'

# Check LLM-parsed intent
ros2 topic echo /humanoid/voice/parsed_intent
# {"type": "complex", "steps": [{"type": "navigate", "location": "kitchen"}, ...]}

# Test TTS
ros2 topic pub --once /humanoid/voice/say std_msgs/String \
  '{data: "I am heading to the kitchen now"}'
```

---

## Exercises

### Exercise 1: Whisper Accuracy Test
Deploy the Whisper node with three model sizes (tiny, small, medium). Record 10 voice commands of varying complexity and measure the word error rate for each model. Document the latency vs accuracy tradeoff.

### Exercise 2: Custom Intent Parser
Extend the intent parser with 3 new command patterns specific to your application domain (e.g., "clean the table," "open the door," "turn on the lights"). Test with 10 spoken commands and verify that at least 8 are parsed correctly.

### Exercise 3: Conversational Loop
Build a full conversational loop: the robot listens for a wake word, transcribes the command, parses the intent, speaks a confirmation ("I'll go to the kitchen now"), executes the action (Nav2 goal), and reports completion ("I've arrived at the kitchen"). Test the end-to-end loop 5 times and measure the total time from voice command to task completion.

---

:::tip Key Takeaway
Voice control transforms a humanoid robot from a programmed machine into a conversational assistant. The key architecture is a **two-stage pipeline**: a fast, lightweight wake word detector (tiny Whisper) that runs continuously, followed by a more accurate transcription model (small/medium Whisper) that runs only when triggered. Combined with structured intent parsing — rule-based for simple commands, LLM-based for complex ones — this pipeline lets you talk to your robot naturally.
:::
