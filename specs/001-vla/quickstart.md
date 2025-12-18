# Quickstart Guide: Vision-Language-Action (VLA) Systems

**Module 4 Setup Guide**
**Date**: 2025-12-17

## Prerequisites

Before starting with Module 4, ensure you have completed:

- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- NVIDIA Isaac Sim installed and running
- ROS 2 Humble Hawksbill installed
- Python 3.11+ with pip
- OpenAI API key (for LLM integration) or local LLM setup
- Microphone for voice input

## System Requirements

- Ubuntu 22.04 LTS (or equivalent ROS 2 environment)
- NVIDIA GPU with CUDA support (for Isaac Sim)
- 16GB+ RAM recommended
- Internet connection for LLM access

## Installation Steps

### 1. Clone and Setup Repository

```bash
git clone [repository-url]
cd hackathon-book/physical-ai-book
```

### 2. Install Python Dependencies

```bash
pip install openai-whisper
pip install openai
pip install opencv-python
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install rclpy
pip install numpy
pip install matplotlib
```

### 3. Configure Environment Variables

Create a `.env` file in the project root:

```bash
OPENAI_API_KEY=your_openai_api_key_here
ROS_DOMAIN_ID=42
ISAAC_SIM_PATH=/path/to/isaac/sim
```

### 4. Initialize VLA System

```bash
cd src/vla
python setup.py initialize
```

This will:
- Download required models (Whisper, vision models)
- Verify ROS 2 connectivity
- Test Isaac Sim connection
- Validate microphone access

## Basic Usage

### 1. Start the VLA System

```bash
python -m vla.main
```

### 2. Test Voice Command

Once the system is running, try a simple command:
- Speak clearly: "Move forward 1 meter"
- Observe the system processing your command
- Watch the robot execute the action in simulation

### 3. Verify System Components

The system will display status for each component:
- Voice input: ✓ Active
- Speech recognition: ✓ Ready
- LLM planning: ✓ Connected
- Vision processing: ✓ Active
- ROS 2 execution: ✓ Connected

## Running the First Exercise

Navigate to the exercises directory:

```bash
cd ../../docs/chapter-4/exercises
```

Follow the `exercise-1-basic-voice-control.md` guide to:

1. Launch Isaac Sim with a simple environment
2. Start the VLA system
3. Execute basic voice commands
4. Observe the vision-language-action pipeline in action

## Troubleshooting

### Common Issues:

**Issue**: "Microphone not detected"
- Solution: Check audio permissions and ensure microphone is selected as default input device

**Issue**: "LLM connection failed"
- Solution: Verify your OpenAI API key is correctly set in environment variables

**Issue**: "ROS 2 connection failed"
- Solution: Ensure ROS 2 Humble is sourced: `source /opt/ros/humble/setup.bash`

**Issue**: "Isaac Sim not responding"
- Solution: Verify Isaac Sim is running and the connection parameters are correct

## Next Steps

After completing the quickstart:

1. Proceed to Lesson 1: Voice Command Processing with Whisper
2. Complete the setup verification exercises
3. Move to Lesson 2: LLM-based Cognitive Planning
4. Continue through all lessons to build the complete VLA system

## Verification

To verify your setup is working correctly, run:

```bash
python -c "from vla.tests import test_setup; test_setup.run_all_tests()"
```

Expected output: All tests should pass with 100% success rate.

---

**Note**: This quickstart guide provides the minimum setup required to begin working with the VLA system. For detailed configuration options and advanced features, refer to the full documentation in `docs/chapter-4/setup.md`.