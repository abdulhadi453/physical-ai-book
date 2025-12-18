# Lesson 1: Voice Command Processing with Whisper

## Overview

Welcome to Lesson 1 of the Vision-Language-Action (VLA) Systems module! In this lesson, you will implement the foundational component of the VLA system: voice command processing using OpenAI's Whisper for speech-to-text conversion. This component serves as the entry point for the entire VLA pipeline, converting natural language commands from users into structured text that can be processed by the cognitive planning system.

## Learning Objectives

By the end of this lesson, you will be able to:
1. Set up and configure Whisper for real-time speech-to-text processing
2. Implement voice input handling with audio preprocessing
3. Integrate Whisper with the VLA system architecture
4. Validate voice commands and handle confidence scoring
5. Process basic voice commands and convert them to text format

## Prerequisites

Before starting this lesson, ensure you have:
- Completed Module 1-3 (ROS 2, Digital Twin, NVIDIA Isaac)
- Installed Python 3.11+ with necessary packages
- Set up OpenAI API access (for Whisper model downloads)
- Verified microphone access and permissions
- Familiarized yourself with the VLA system architecture

## Voice Processing Architecture

The voice processing component follows this architecture:

```
Microphone → Audio Preprocessing → Whisper STT → Text Validation → Processed Command
```

Key considerations include:
- Real-time processing capabilities
- Noise reduction and audio quality enhancement
- Confidence scoring for reliability assessment
- Error handling for failed recognition attempts

## Implementation Steps

### Step 1: Set Up Whisper Environment

First, let's create the necessary directory structure and install dependencies:

```bash
# Navigate to the VLA source directory
cd physical-ai-book/src/vla

# Create speech processing directory
mkdir -p speech
```

Now let's create the Whisper processor module:

```python
# src/vla/speech/whisper_processor.py

import whisper
import torch
import numpy as np
from typing import Optional, Dict, Any
import logging

logger = logging.getLogger(__name__)

class WhisperProcessor:
    """
    Handles speech-to-text conversion using OpenAI Whisper model.
    """

    def __init__(self, model_size: str = "base"):
        """
        Initialize Whisper processor with specified model size.

        Args:
            model_size: Size of Whisper model ('tiny', 'base', 'small', 'medium', 'large')
        """
        self.model_size = model_size
        self.model = None
        self._load_model()

    def _load_model(self):
        """Load the Whisper model based on specified size."""
        try:
            # Check if CUDA is available for GPU acceleration
            device = "cuda" if torch.cuda.is_available() else "cpu"
            logger.info(f"Loading Whisper {self.model_size} model on {device}")

            self.model = whisper.load_model(self.model_size, device=device)
            logger.info("Whisper model loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load Whisper model: {e}")
            raise

    def transcribe_audio(self, audio_data: np.ndarray, language: str = "en") -> Dict[str, Any]:
        """
        Transcribe audio data to text using Whisper.

        Args:
            audio_data: Audio data as numpy array (expected 16kHz sample rate)
            language: Language code for transcription (default: 'en')

        Returns:
            Dictionary containing transcribed text and confidence metrics
        """
        try:
            # Ensure audio is in the right format (16kHz)
            if len(audio_data) == 0:
                return {
                    "text": "",
                    "confidence": 0.0,
                    "language": language,
                    "success": False,
                    "error": "Empty audio data"
                }

            # Run transcription
            result = self.model.transcribe(
                audio_data,
                language=language,
                temperature=0.0  # Deterministic output
            )

            # Extract confidence information (Whisper doesn't provide direct confidence)
            # We'll use alternative methods to estimate reliability
            confidence = self._estimate_confidence(result)

            return {
                "text": result["text"].strip(),
                "confidence": confidence,
                "language": language,
                "success": True,
                "segments": result.get("segments", []),
                "processing_time": result.get("processing_time", 0.0)
            }
        except Exception as e:
            logger.error(f"Transcription failed: {e}")
            return {
                "text": "",
                "confidence": 0.0,
                "language": language,
                "success": False,
                "error": str(e)
            }

    def _estimate_confidence(self, result) -> float:
        """
        Estimate confidence based on various factors since Whisper doesn't provide direct confidence.

        Args:
            result: Whisper transcription result

        Returns:
            Estimated confidence score (0.0 to 1.0)
        """
        # For now, return a simple confidence estimate
        # In a production system, you might use more sophisticated methods
        text = result.get("text", "")

        if not text or len(text.strip()) == 0:
            return 0.0

        # Simple heuristics for confidence estimation
        # - Longer text might indicate more confidence
        # - Presence of common command words
        text_length = len(text)
        confidence = min(0.3 + (text_length * 0.01), 1.0)  # Basic length-based estimate

        # Adjust based on common command patterns
        command_indicators = ["move", "go", "pick", "place", "grasp", "navigate", "turn", "stop"]
        if any(indicator in text.lower() for indicator in command_indicators):
            confidence = min(confidence + 0.2, 1.0)

        return confidence
```

### Step 2: Implement Voice Input Handler

Next, we'll create a module to handle audio capture from the microphone:

```python
# src/vla/speech/voice_input_handler.py

import pyaudio
import numpy as np
import threading
import queue
import time
from typing import Optional, Callable
import logging

logger = logging.getLogger(__name__)

class VoiceInputHandler:
    """
    Handles audio input from microphone for voice command processing.
    """

    def __init__(self,
                 sample_rate: int = 16000,
                 chunk_size: int = 1024,
                 max_buffer_size: int = 10):
        """
        Initialize voice input handler.

        Args:
            sample_rate: Audio sample rate (default: 16kHz for Whisper compatibility)
            chunk_size: Size of audio chunks to process
            max_buffer_size: Maximum number of chunks to buffer
        """
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.max_buffer_size = max_buffer_size

        # Audio stream parameters
        self.format = pyaudio.paInt16
        self.channels = 1

        # Audio processing
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.is_recording = False
        self.audio_queue = queue.Queue(maxsize=max_buffer_size)

        # Callback for processed audio
        self.process_callback: Optional[Callable] = None

    def start_recording(self, callback: Optional[Callable] = None):
        """
        Start recording audio from microphone.

        Args:
            callback: Function to call when audio is ready for processing
        """
        if self.is_recording:
            logger.warning("Recording already in progress")
            return

        self.process_callback = callback
        self.is_recording = True

        # Open audio stream
        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            logger.info("Started recording from microphone")

            # Start recording thread
            self.recording_thread = threading.Thread(target=self._record_audio)
            self.recording_thread.daemon = True
            self.recording_thread.start()

        except Exception as e:
            logger.error(f"Failed to start recording: {e}")
            self.is_recording = False
            raise

    def stop_recording(self):
        """Stop recording audio."""
        if not self.is_recording:
            return

        self.is_recording = False

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()

        logger.info("Stopped recording")

    def _record_audio(self):
        """Internal method to record audio in a separate thread."""
        try:
            while self.is_recording:
                # Read audio data
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Convert to numpy array
                audio_array = np.frombuffer(data, dtype=np.int16)
                audio_array = audio_array.astype(np.float32) / 32768.0  # Normalize to [-1, 1]

                # Add to processing queue
                try:
                    self.audio_queue.put_nowait(audio_array)
                except queue.Full:
                    # Drop oldest audio if queue is full
                    try:
                        self.audio_queue.get_nowait()
                        self.audio_queue.put_nowait(audio_array)
                    except queue.Empty:
                        pass

                # Process accumulated audio if callback is set
                if self.process_callback:
                    self._process_accumulated_audio()

        except Exception as e:
            logger.error(f"Error in audio recording thread: {e}")
        finally:
            self.is_recording = False

    def _process_accumulated_audio(self):
        """Process accumulated audio chunks."""
        if self.process_callback and not self.audio_queue.empty():
            # Collect all available audio data
            audio_data = []
            try:
                while True:
                    chunk = self.audio_queue.get_nowait()
                    audio_data.append(chunk)
            except queue.Empty:
                pass

            if audio_data:
                # Concatenate all chunks
                full_audio = np.concatenate(audio_data)

                # Call the processing callback
                self.process_callback(full_audio)

    def get_audio_buffer(self) -> Optional[np.ndarray]:
        """
        Get accumulated audio data from the buffer.

        Returns:
            Concatenated audio data as numpy array, or None if no data available
        """
        if self.audio_queue.empty():
            return None

        audio_data = []
        try:
            while True:
                chunk = self.audio_queue.get_nowait()
                audio_data.append(chunk)
        except queue.Empty:
            pass

        if audio_data:
            return np.concatenate(audio_data)

        return None

    def __del__(self):
        """Cleanup audio resources."""
        if hasattr(self, 'audio') and self.audio:
            self.audio.terminate()
```

### Step 3: Create Voice Command Validator

Now let's create a validation module to ensure voice commands are properly formatted:

```python
# src/vla/utils/validators.py

import re
from typing import Dict, Any, List
import logging

logger = logging.getLogger(__name__)

class VoiceCommandValidator:
    """
    Validates voice commands to ensure they meet VLA system requirements.
    """

    def __init__(self):
        # Define valid command patterns
        self.command_patterns = [
            # Navigation commands
            r"move\s+(forward|backward|left|right|up|down)",
            r"go\s+to\s+\w+",
            r"navigate\s+to\s+\w+",
            r"turn\s+(left|right|around)",

            # Manipulation commands
            r"pick\s+up\s+\w+",
            r"grasp\s+\w+",
            r"take\s+\w+",
            r"place\s+\w+\s+on\s+\w+",
            r"put\s+\w+\s+on\s+\w+",

            # Object interaction
            r"find\s+\w+",
            r"look\s+at\s+\w+",
            r"identify\s+\w+",

            # Complex commands
            r"pick\s+up\s+\w+\s+and\s+place\s+it\s+on\s+\w+",
            r"go\s+to\s+\w+\s+and\s+pick\s+up\s+\w+"
        ]

    def validate_command(self, command_text: str, confidence: float) -> Dict[str, Any]:
        """
        Validate a voice command.

        Args:
            command_text: The transcribed command text
            confidence: Confidence score from speech recognition

        Returns:
            Dictionary with validation results
        """
        result = {
            "is_valid": False,
            "errors": [],
            "warnings": [],
            "processed_command": None,
            "command_type": None
        }

        # Check if text is empty
        if not command_text or len(command_text.strip()) == 0:
            result["errors"].append("Command text is empty")
            return result

        # Check confidence threshold
        if confidence < 0.5:
            result["warnings"].append(f"Low confidence score: {confidence:.2f}")

        # Check for basic command structure
        if len(command_text.split()) < 2:
            result["errors"].append("Command too short - minimum 2 words required")

        # Check against valid patterns
        command_lower = command_text.lower()
        matched_pattern = False

        for pattern in self.command_patterns:
            if re.search(pattern, command_lower):
                matched_pattern = True
                # Determine command type based on pattern
                if any(word in command_lower for word in ["move", "go", "navigate", "turn"]):
                    result["command_type"] = "navigation"
                elif any(word in command_lower for word in ["pick", "grasp", "take", "place", "put"]):
                    result["command_type"] = "manipulation"
                elif any(word in command_lower for word in ["find", "look", "identify"]):
                    result["command_type"] = "inspection"
                else:
                    result["command_type"] = "general"
                break

        if not matched_pattern:
            result["warnings"].append("Command pattern not recognized, may not be supported")

        # If we have warnings but no errors, consider it valid with warnings
        if not result["errors"]:
            result["is_valid"] = True
            result["processed_command"] = command_text.strip()

        return result
```

### Step 4: Integrate Components

Now let's create a main module that integrates all voice processing components:

```python
# src/vla/speech/vla_voice_processor.py

import numpy as np
import time
from typing import Dict, Any, Optional, Callable
import logging

from src.vla.speech.whisper_processor import WhisperProcessor
from src.vla.speech.voice_input_handler import VoiceInputHandler
from src.vla.utils.validators import VoiceCommandValidator

logger = logging.getLogger(__name__)

class VLAVoiceProcessor:
    """
    Main voice processing module for the VLA system.
    Integrates Whisper, voice input, and validation components.
    """

    def __init__(self, model_size: str = "base"):
        """
        Initialize the VLA voice processor.

        Args:
            model_size: Whisper model size to use
        """
        self.whisper_processor = WhisperProcessor(model_size=model_size)
        self.voice_handler = VoiceInputHandler()
        self.validator = VoiceCommandValidator()

        self.is_active = False
        self.command_callback: Optional[Callable] = None

    def start_listening(self, command_callback: Callable[[Dict[str, Any]], None]):
        """
        Start listening for voice commands.

        Args:
            command_callback: Function to call when a command is processed
        """
        self.command_callback = command_callback
        self.is_active = True

        # Start voice input with processing callback
        self.voice_handler.start_recording(self._process_audio_chunk)

    def stop_listening(self):
        """Stop listening for voice commands."""
        self.is_active = False
        self.voice_handler.stop_recording()

    def _process_audio_chunk(self, audio_data: np.ndarray):
        """
        Process an audio chunk through the voice pipeline.

        Args:
            audio_data: Audio data to process
        """
        if not self.is_active:
            return

        try:
            # Transcribe audio to text
            transcription_result = self.whisper_processor.transcribe_audio(audio_data)

            if transcription_result["success"] and transcription_result["text"]:
                # Validate the command
                validation_result = self.validator.validate_command(
                    transcription_result["text"],
                    transcription_result["confidence"]
                )

                # Create command result
                command_result = {
                    "text": transcription_result["text"],
                    "confidence": transcription_result["confidence"],
                    "is_valid": validation_result["is_valid"],
                    "validation_errors": validation_result["errors"],
                    "validation_warnings": validation_result["warnings"],
                    "command_type": validation_result["command_type"],
                    "timestamp": time.time(),
                    "processing_details": {
                        "transcription_time": transcription_result.get("processing_time", 0.0),
                        "segments": transcription_result.get("segments", [])
                    }
                }

                # Call the command callback if available
                if self.command_callback:
                    self.command_callback(command_result)

        except Exception as e:
            logger.error(f"Error processing audio chunk: {e}")

    def process_voice_command(self, audio_data: np.ndarray) -> Dict[str, Any]:
        """
        Process a single voice command synchronously.

        Args:
            audio_data: Audio data to process

        Returns:
            Dictionary with command processing results
        """
        # Transcribe audio
        transcription_result = self.whisper_processor.transcribe_audio(audio_data)

        if not transcription_result["success"]:
            return {
                "success": False,
                "error": transcription_result.get("error", "Transcription failed"),
                "confidence": 0.0
            }

        # Validate command
        validation_result = self.validator.validate_command(
            transcription_result["text"],
            transcription_result["confidence"]
        )

        return {
            "success": True,
            "text": transcription_result["text"],
            "confidence": transcription_result["confidence"],
            "is_valid": validation_result["is_valid"],
            "validation_errors": validation_result["errors"],
            "validation_warnings": validation_result["warnings"],
            "command_type": validation_result["command_type"],
            "timestamp": time.time()
        }
```

## Practical Exercise

### Exercise 1.1: Basic Voice Command Processing

1. **Setup**: Create a simple test script that initializes the Whisper processor and voice input handler.

2. **Implementation**:
   ```python
   # test_voice_processing.py
   import time
   import numpy as np
   from src.vla.speech.vla_voice_processor import VLAVoiceProcessor

   def main():
       # Initialize the VLA voice processor
       voice_processor = VLAVoiceProcessor(model_size="base")

       print("VLA Voice Processing system initialized")
       print("Say a command like 'move forward' or 'pick up the cube'")

       # Callback function for processed commands
       def command_callback(command_result):
           print(f"\n--- New Command Processed ---")
           print(f"Text: '{command_result['text']}'")
           print(f"Confidence: {command_result['confidence']:.2f}")
           print(f"Valid: {command_result['is_valid']}")
           print(f"Type: {command_result['command_type']}")

           if command_result['validation_warnings']:
               print(f"Warnings: {command_result['validation_warnings']}")
           if command_result['validation_errors']:
               print(f"Errors: {command_result['validation_errors']}")

       # Start listening
       voice_processor.start_listening(command_callback)

       try:
           # Keep the program running for 30 seconds
           print("\nListening for voice commands (30 seconds)...")
           time.sleep(30)
       except KeyboardInterrupt:
           print("\nStopping...")
       finally:
           voice_processor.stop_listening()
           print("Voice processing stopped")

   if __name__ == "__main__":
       main()
   ```

3. **Testing**: Run the test script and speak simple commands like "move forward" or "pick up the red cube". Observe the transcription, confidence scores, and validation results.

## Key Concepts

### Whisper Model Selection
- **Tiny/Small**: Faster but less accurate, good for real-time applications
- **Base/Medium**: Balance of speed and accuracy
- **Large**: Most accurate but slower, best for high-quality requirements

### Confidence Scoring
Whisper doesn't provide direct confidence scores, so we estimate reliability using:
- Text length and structure
- Presence of common command patterns
- Audio quality indicators

### Real-time Processing
The voice input handler uses threading to capture audio continuously while allowing other system components to operate concurrently.

## Common Challenges and Solutions

### Challenge 1: Audio Quality
**Problem**: Background noise affects transcription quality.
**Solution**: Implement noise reduction algorithms or use higher-quality microphones.

### Challenge 2: Command Ambiguity
**Problem**: Natural language can be ambiguous ("that thing over there").
**Solution**: Use visual context to disambiguate references (covered in Lesson 3).

### Challenge 3: Real-time Performance
**Problem**: Processing delays make the system feel unresponsive.
**Solution**: Optimize model size, use GPU acceleration, implement audio buffering.

## Assessment Questions

1. What is the primary role of the Whisper processor in the VLA system?
2. How does the system estimate confidence when Whisper doesn't provide direct confidence scores?
3. Why is it important to validate voice commands before passing them to the planning system?
4. What are the advantages of using threading for audio capture?

## Summary

In this lesson, you have implemented the foundational voice command processing system for the VLA system. You learned how to:
- Set up and configure Whisper for speech-to-text conversion
- Handle real-time audio input from a microphone
- Validate voice commands for proper structure and content
- Integrate all components into a cohesive voice processing pipeline

This voice processing system serves as the entry point for the entire VLA pipeline, converting natural language commands into structured text that can be processed by subsequent components. In the next lesson, you will integrate this voice processing system with LLM-based cognitive planning to interpret commands and generate action sequences.