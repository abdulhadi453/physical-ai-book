# VLA Practical Exercises

## Overview

This document contains hands-on exercises for Module 4: Vision-Language-Action (VLA) Systems. These exercises are designed to reinforce the concepts learned in each lesson and provide practical experience with implementing and using the VLA system components.

## Exercise 1: Voice Command Processing

### Exercise 1.1: Basic Whisper Integration
**Objective**: Set up and test the Whisper speech-to-text system.

**Steps**:
1. Initialize the Whisper processor with the 'base' model
2. Create a simple audio recording script that captures voice input
3. Process the recorded audio through Whisper
4. Display the transcribed text and confidence score

**Code Template**:
```python
from src.vla.speech.whisper_processor import WhisperProcessor
from src.vla.speech.voice_input_handler import VoiceInputHandler
import time

# Initialize Whisper processor
whisper_proc = WhisperProcessor(model_size="base")

# Record and transcribe
def record_and_transcribe():
    print("Recording for 5 seconds...")
    # Use voice_input_handler to record audio
    # Process with whisper_proc.transcribe_audio()
    # Print results
```

**Expected Output**: Transcribed text with confidence score above 0.5

**Assessment**: Successfully transcribe at least 3 different voice commands with >0.6 confidence

### Exercise 1.2: Voice Command Validation
**Objective**: Implement and test voice command validation.

**Steps**:
1. Create a voice command validation system
2. Test various command formats and invalid inputs
3. Verify that valid commands pass validation
4. Ensure invalid commands are properly rejected

**Code Template**:
```python
from src.vla.utils.validators import VoiceCommandValidator

validator = VoiceCommandValidator()

test_commands = [
    "move forward",  # Valid
    "pick up the red cube",  # Valid
    "a",  # Invalid - too short
    "xyz abc def ghi jkl"  # Valid length but invalid pattern
]

for cmd in test_commands:
    result = validator.validate_command(cmd, confidence=0.8)
    print(f"Command: '{cmd}' -> Valid: {result['is_valid']}")
```

**Expected Output**: Valid commands marked as True, invalid as False

**Assessment**: Correctly validate at least 80% of test cases

## Exercise 2: LLM Cognitive Planning

### Exercise 2.1: Simple Command Planning
**Objective**: Plan actions for simple navigation commands using the LLM.

**Steps**:
1. Initialize the LLM client and cognitive planner
2. Create a simple navigation command (e.g., "Go to the table")
3. Generate an action sequence using the planner
4. Verify the action sequence is appropriate for the command

**Code Template**:
```python
from src.vla.llm.llm_client import LLMClient
from src.vla.llm.cognitive_planner import CognitivePlanner

# Initialize components
llm_client = LLMClient()  # Requires API key
planner = CognitivePlanner(llm_client)

# Plan a simple command
command = "Go to the table"
intent = planner.plan_command(command)

print(f"Command: {command}")
print(f"Intent type: {intent.intent_type}")
print(f"Action sequence:")
for i, action in enumerate(intent.action_sequence):
    print(f"  {i+1}. {action.action_type.value}: {action.parameters}")
```

**Expected Output**: Action sequence with NAVIGATE_TO action and appropriate parameters

**Assessment**: Generate correct action sequences for 5 different navigation commands

### Exercise 2.2: Complex Task Decomposition
**Objective**: Decompose complex multi-step commands into action sequences.

**Steps**:
1. Create a complex command (e.g., "Go to kitchen, find red cup, bring to table")
2. Plan the command using the cognitive planner
3. Verify the action sequence has multiple steps in logical order
4. Test with different complex commands

**Code Template**:
```python
# Test complex command
complex_command = "Go to the kitchen, find a red cup, pick it up, and bring it to the table"

# Plan with mock objects for context
from src.vla.models.detected_object import DetectedObject, Point3D

mock_objects = [
    DetectedObject(
        id="cup_1",
        class_name="cup",
        confidence=0.85,
        bbox=None,
        position_3d=Point3D(x=1.0, y=2.0, z=0.0),
        dimensions=None,
        color="red",
        is_graspable=True
    )
]

intent = planner.plan_command(complex_command, context_objects=mock_objects)
print(f"Complex command planned with {len(intent.action_sequence)} actions")
```

**Expected Output**: Multi-step action sequence with navigation, manipulation, and navigation again

**Assessment**: Successfully plan 3 different complex commands with appropriate action sequences

## Exercise 3: Visual Perception Integration

### Exercise 3.1: Object Detection Test
**Objective**: Test object detection with sample images.

**Steps**:
1. Initialize the object detector
2. Create or use sample images with known objects
3. Run detection on the images
4. Verify detection accuracy and confidence scores

**Code Template**:
```python
import cv2
from src.vla.vision.object_detector import ObjectDetector

detector = ObjectDetector(confidence_threshold=0.5)

# Load a test image
test_image = cv2.imread("path/to/test/image.jpg")

# Detect objects
detected_objects = detector.detect_objects(test_image)

print(f"Detected {len(detected_objects)} objects:")
for obj in detected_objects:
    print(f"  - {obj.class_name} ({obj.color}) at {obj.bbox.x_min}, {obj.bbox.y_min}")
    print(f"    Confidence: {obj.confidence:.2f}, Graspable: {obj.is_graspable}")
```

**Expected Output**: List of detected objects with class names, positions, and properties

**Assessment**: Detect at least 5 different object types with >0.6 confidence

### Exercise 3.2: 3D Position Estimation
**Objective**: Test 3D position estimation from 2D images.

**Steps**:
1. Initialize the perception pipeline
2. Process images with known object positions
3. Verify 3D position estimates are reasonable
4. Test with different object distances

**Code Template**:
```python
from src.vla.vision.perception_pipeline import PerceptionPipeline
from src.vla.vision.object_detector import ObjectDetector

detector = ObjectDetector()
pipeline = PerceptionPipeline(detector)

# Process a frame (using mock robot pose)
robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
perception_data = pipeline.process_frame(test_image, robot_pose)

print("Objects with 3D positions:")
for obj in perception_data.objects:
    pos = obj.position_3d
    print(f"  {obj.class_name}: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
```

**Expected Output**: Objects with estimated 3D coordinates relative to robot

**Assessment**: Estimate positions within 0.2m accuracy for test objects

## Exercise 4: Action Execution

### Exercise 4.1: Single Action Execution
**Objective**: Execute a single action in simulation.

**Steps**:
1. Initialize the action executor
2. Create a simple action (e.g., navigation to specific coordinates)
3. Execute the action
4. Monitor execution status and completion

**Code Template**:
```python
from src.vla.ros2.action_executor import ActionExecutor
from src.vla.models.action_step import ActionStep, ActionStepType

executor = ActionExecutor()

# Create a simple navigation action
nav_action = ActionStep(
    id="nav_test",
    action_type=ActionStepType.NAVIGATE_TO,
    parameters={"target_position": {"x": 1.0, "y": 0.5, "z": 0.0}},
    timeout=10,
    required_objects=[],
    preconditions=[],
    expected_outcomes=[]
)

# Execute the action
def status_callback(state):
    print(f"Execution status: {state.status.value}, Progress: {state.progress:.2f}")

execution_id = executor.execute_action_sequence([nav_action], callback=status_callback)
print(f"Started execution: {execution_id}")
```

**Expected Output**: Action execution with status updates and completion

**Assessment**: Successfully execute 5 different action types with >80% success rate

### Exercise 4.2: Action Sequence Execution
**Objective**: Execute a sequence of actions and monitor progress.

**Steps**:
1. Create an action sequence (e.g., navigate → grasp → navigate → place)
2. Execute the sequence
3. Monitor progress and state changes
4. Handle any failures gracefully

**Code Template**:
```python
from src.vla.models.action_step import ActionStep, ActionStepType

# Create multi-step sequence
action_sequence = [
    ActionStep(
        id="nav_to_object",
        action_type=ActionStepType.NAVIGATE_TO,
        parameters={"target_position": {"x": 1.0, "y": 1.0, "z": 0.0}},
        timeout=10,
        required_objects=[],
        preconditions=[],
        expected_outcomes=[]
    ),
    ActionStep(
        id="grasp_object",
        action_type=ActionStepType.GRASP_OBJECT,
        parameters={"object_id": "test_object"},
        timeout=8,
        required_objects=["test_object"],
        preconditions=[],
        expected_outcomes=[]
    ),
    ActionStep(
        id="nav_to_destination",
        action_type=ActionStepType.NAVIGATE_TO,
        parameters={"target_position": {"x": 2.0, "y": 0.0, "z": 0.0}},
        timeout=10,
        required_objects=[],
        preconditions=[],
        expected_outcomes=[]
    ),
    ActionStep(
        id="place_object",
        action_type=ActionStepType.RELEASE_OBJECT,
        parameters={},
        timeout=5,
        required_objects=[],
        preconditions=[],
        expected_outcomes=[]
    )
]

# Execute sequence
execution_id = executor.execute_action_sequence(action_sequence, callback=status_callback)
print(f"Multi-step execution started: {execution_id}")
```

**Expected Output**: Sequential execution with progress tracking

**Assessment**: Execute 3 different multi-step sequences with >75% success rate

## Exercise 5: Complete VLA Integration

### Exercise 5.1: End-to-End Command Processing
**Objective**: Process a complete voice command through the entire VLA pipeline.

**Steps**:
1. Set up the complete VLA system with all components
2. Provide a natural language command (or use direct text input for testing)
3. Track the command through speech processing → planning → perception → execution
4. Verify the final outcome matches the command intent

**Code Template**:
```python
from src.vla.integration.vla_system import VLASystem
from src.vla.integration.feedback_processor import FeedbackProcessor
from src.vla.speech.whisper_processor import WhisperProcessor
from src.vla.llm.llm_client import LLMClient
from src.vla.vision.object_detector import ObjectDetector
from src.vla.ros2.action_executor import ActionExecutor

# Initialize all components
whisper_proc = WhisperProcessor(model_size="base")
llm_client = LLMClient()  # Requires API key
object_detector = ObjectDetector()
action_executor = ActionExecutor()

vla_system = VLASystem(whisper_proc, llm_client, object_detector, action_executor)
feedback_processor = FeedbackProcessor()

# Process a command end-to-end
command_text = "Go to the table and pick up the red cube"
execution_id = vla_system.process_command_direct(command_text)

print(f"End-to-end command processed: {command_text}")
print(f"Execution ID: {execution_id}")
```

**Expected Output**: Complete pipeline execution from command to action

**Assessment**: Successfully process 5 different commands through the full pipeline

### Exercise 5.2: Capstone Task Simulation
**Objective**: Simulate the capstone autonomous task with simplified components.

**Steps**:
1. Set up a simplified version of the capstone task
2. Use mock components where real hardware/simulation isn't available
3. Execute a multi-step task similar to the capstone project
4. Validate each step and the overall task completion

**Code Template**:
```python
from src.vla.capstone.autonomous_task import AutonomousHumanoidTask

# Create simplified capstone task
capstone_task = AutonomousHumanoidTask(vla_system, feedback_processor)

# Define capstone-like sequence
capstone_commands = [
    "Navigate to the kitchen area",
    "Look for a red object",
    "Grasp the red object",
    "Move to the drop-off location",
    "Place the object"
]

print("Starting simplified capstone task...")
for cmd in capstone_commands:
    print(f"Executing: {cmd}")
    vla_system.process_command_direct(cmd)
    # Add appropriate delays between commands
    import time
    time.sleep(3)

print("Simplified capstone task completed")
```

**Expected Output**: Multi-step task execution with feedback at each stage

**Assessment**: Complete the simplified capstone task with at least 80% step success rate

## Advanced Exercises

### Exercise A1: Performance Optimization
**Objective**: Optimize the VLA system for better performance.

**Steps**:
1. Measure current performance of each component
2. Identify bottlenecks in the pipeline
3. Implement optimizations (model size, threading, etc.)
4. Re-measure performance and calculate improvements

### Exercise A2: Error Handling Enhancement
**Objective**: Improve error handling and recovery in the VLA system.

**Steps**:
1. Introduce various error conditions in the pipeline
2. Implement comprehensive error detection
3. Add recovery mechanisms for common failures
4. Test system resilience under error conditions

### Exercise A3: Custom Object Training
**Objective**: Train a custom object detector for specific objects.

**Steps**:
1. Collect images of custom objects
2. Label the images for training
3. Train a custom YOLO model
4. Integrate the custom model into the VLA system

## Assessment Criteria

### Technical Implementation (40%)
- Correct implementation of all components
- Proper integration between modules
- Error handling and validation
- Code quality and documentation

### Functionality (35%)
- Successful execution of planned actions
- Accurate voice command processing
- Proper object detection and manipulation
- Multi-step task completion

### Performance (15%)
- Response time efficiency
- Accuracy of processing
- Resource utilization
- System stability

### Documentation (10%)
- Clear code comments
- Proper function documentation
- Test results and validation
- Troubleshooting notes

## Submission Requirements

For each exercise, submit:
1. **Code files**: Complete implementation files
2. **Test results**: Output and validation results
3. **Performance metrics**: Timing and accuracy measurements
4. **Documentation**: Brief explanation of implementation approach
5. **Reflection**: What was learned and challenges faced

## Resources and References

- VLA System Architecture Diagram
- ROS 2 Action Interface Documentation
- Whisper API Documentation
- YOLO Object Detection Guide
- NVIDIA Isaac Sim Integration Guide

## Troubleshooting Tips

- If audio input fails, check microphone permissions and configuration
- If LLM calls fail, verify API key and network connectivity
- If ROS 2 nodes don't communicate, ensure proper environment sourcing
- If Isaac Sim connection fails, check server status and network ports
- For object detection issues, verify image format and model compatibility

Complete these exercises in order to build a comprehensive understanding of the VLA system. Each exercise builds upon the previous ones, so ensure you have working implementations before proceeding to the next exercise.