# VLA System Setup Guide

## Overview

This guide provides detailed instructions for setting up the Vision-Language-Action (VLA) system. The VLA system integrates speech processing, cognitive planning, visual perception, and robotic action execution to enable natural language control of humanoid robots in simulation.

## System Requirements

### Hardware Requirements
- **CPU**: Multi-core processor (Intel i7 or equivalent recommended)
- **GPU**: NVIDIA GPU with CUDA support (RTX 3060 or better recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB free space for Isaac Sim and dependencies
- **Microphone**: USB or built-in microphone for voice input
- **Internet**: Required for LLM access and package downloads

### Software Requirements
- **OS**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **ROS 2**: Humble Hawksbill distribution
- **Python**: 3.11 or higher
- **CUDA**: 11.8 or higher (for GPU acceleration)
- **Docker**: For Isaac Sim container deployment (optional but recommended)

## Prerequisites Installation

### 1. ROS 2 Humble Installation

For Ubuntu 22.04:
```bash
# Set locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. NVIDIA Isaac Sim Installation

Choose one of the following installation methods:

#### Method A: Docker Installation (Recommended)
```bash
# Install Docker if not already installed
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker $USER  # Log out and back in for this to take effect

# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:4.0.0

# Create a script to run Isaac Sim
cat << 'EOF' > ~/run_isaac_sim.sh
#!/bin/bash
xhost +local:docker
docker run --gpus all -it --rm \
  --network=host \
  --env NVIDIA_DISABLE_REQUIRE=1 \
  --env PYTHON_ROOT_DIR=/isaac-sim/python.sh \
  --env IsaacSim_SERVER_PORT=50051 \
  --volume $HOME/isaac-sim-assets:/isaac-sim/assets \
  --volume $HOME/isaac-sim-examples:/isaac-sim/examples \
  --volume $HOME/isaac-sim-workspace:/isaac-sim/workspace \
  --volume $HOME/isaac-sim-python:/isaac-sim/python \
  --volume $HOME/isaac-sim-config:/isaac-sim/config \
  --volume $HOME/isaac-sim-logs:/isaac-sim/logs \
  --volume $HOME/isaac-sim-data:/isaac-sim/data \
  --privileged \
  --pid=host \
  nvcr.io/nvidia/isaac-sim:4.0.0
EOF

chmod +x ~/run_isaac_sim.sh
```

#### Method B: Local Installation
```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow the official installation guide for your platform
# https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html
```

### 3. Python Environment Setup

```bash
# Create virtual environment
python3 -m venv vla_env
source vla_env/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install core dependencies
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install openai-whisper
pip install openai
pip install ultralytics  # For YOLO object detection
pip install opencv-python
pip install rclpy  # ROS 2 Python client library
pip install numpy pandas matplotlib
pip install tenacity  # For retry logic
pip install pyaudio  # For audio input
```

## VLA System Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-organization/physical-ai-book.git
cd physical-ai-book
```

### 2. Install VLA-Specific Dependencies

```bash
cd physical-ai-book
source vla_env/bin/activate

# Install additional dependencies
pip install -e .  # If setup.py exists
# OR install individual packages
pip install transformers  # For additional NLP capabilities
pip install sentence-transformers  # For semantic similarity
pip install faiss-cpu  # For vector similarity search
```

### 3. Configure Environment Variables

Create a `.env` file in the project root:

```bash
# Create .env file
cat << 'EOF' > .env
# OpenAI API Configuration
OPENAI_API_KEY=your_openai_api_key_here

# ROS 2 Configuration
ROS_DOMAIN_ID=42

# Isaac Sim Configuration
ISAAC_SIM_PATH=/path/to/isaac/sim  # Only if installed locally
ISAAC_SIM_SERVER_PORT=50051

# Whisper Model Configuration
WHISPER_MODEL_SIZE=base  # tiny, base, small, medium, large

# System Configuration
VLA_WORKSPACE_PATH=/path/to/vla/workspace
VLA_LOG_LEVEL=INFO
EOF
```

## Component Configuration

### 1. Whisper Speech Processing Setup

The Whisper component is configured automatically, but you can customize the model size:

```python
# In src/vla/speech/whisper_processor.py, you can adjust:
model_size = "base"  # Options: tiny, base, small, medium, large
# Larger models are more accurate but slower
```

### 2. LLM Client Configuration

Set up your OpenAI API key in the environment variables:

```bash
export OPENAI_API_KEY="your-api-key-here"
```

Or use the `.env` file created earlier.

### 3. Vision System Configuration

The vision system uses YOLOv8 by default. You can configure it as follows:

```python
# In src/vla/vision/object_detector.py:
confidence_threshold = 0.5  # Adjust based on your needs
# Lower values detect more objects but may include false positives
```

### 4. ROS 2 Integration Setup

Ensure ROS 2 is properly sourced:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # If you have a colcon workspace
```

## Environment Configuration

### 1. Isaac Sim Connection

If using Isaac Sim Docker, ensure it's running:

```bash
# Run Isaac Sim in one terminal
./run_isaac_sim.sh

# In another terminal, verify connection
python3 -c "import omni; print('Isaac Sim connection test')"
```

### 2. Audio Input Configuration

Test your microphone:

```bash
# Install and test audio input
pip install pyaudio
python3 -c "
import pyaudio
p = pyaudio.PyAudio()
print('Available audio devices:')
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f'{i}: {info[\"name\"]} - {info[\"maxInputChannels\"]} input channels')
"
```

### 3. Network Configuration

Ensure proper network connectivity between components:

```bash
# Check if ROS 2 nodes can communicate
source /opt/ros/humble/setup.bash
ros2 topic list

# Check Isaac Sim connection (if using remote API)
netstat -tuln | grep 50051
```

## Verification Steps

### 1. Test Individual Components

```bash
# Test Whisper processing
source vla_env/bin/activate
cd physical-ai-book
python3 -c "
from src.vla.speech.whisper_processor import WhisperProcessor
processor = WhisperProcessor(model_size='base')
print('Whisper processor initialized successfully')
"

# Test LLM client
python3 -c "
import os
from src.vla.llm.llm_client import LLMClient
# This will fail if API key is not set, which is expected
try:
    client = LLMClient()
    print('LLM client initialized successfully')
except ValueError as e:
    print(f'LLM client setup issue: {e}')
"

# Test vision components
python3 -c "
from src.vla.vision.object_detector import ObjectDetector
detector = ObjectDetector()
print('Vision components initialized successfully')
"
```

### 2. Test ROS 2 Integration

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Run a simple ROS 2 test
python3 -c "
import rclpy
rclpy.init()
node = rclpy.create_node('test_node')
print('ROS 2 connection successful')
node.destroy_node()
rclpy.shutdown()
"
```

### 3. Run Complete System Test

```bash
# In the project root
source vla_env/bin/activate
source /opt/ros/humble/setup.bash

python3 -c "
from src.vla.integration.vla_system import VLASystem
from src.vla.speech.whisper_processor import WhisperProcessor
from src.vla.llm.llm_client import LLMClient
from src.vla.vision.object_detector import ObjectDetector
from src.vla.ros2.action_executor import ActionExecutor

# Initialize components (with mock API key for testing)
import os
os.environ['OPENAI_API_KEY'] = 'test-key'  # Only for initialization test

try:
    whisper = WhisperProcessor(model_size='base')
    llm_client = LLMClient(api_key='test-key')
    detector = ObjectDetector()
    executor = ActionExecutor()

    vla_system = VLASystem(whisper, llm_client, detector, executor)
    print('VLA System initialized successfully')

    # Test direct command processing
    result = vla_system.process_command_direct('Move forward 1 meter')
    print(f'Direct command test completed: {result}')

except Exception as e:
    print(f'Error in system initialization: {e}')
"
```

## Troubleshooting

### Common Issues and Solutions

**Issue 1**: CUDA not found
- **Symptoms**: PyTorch errors, Whisper processing fails
- **Solution**: Ensure CUDA drivers and toolkit are properly installed
```bash
nvidia-smi  # Check if GPU drivers are working
nvcc --version  # Check CUDA toolkit
```

**Issue 2**: ROS 2 packages not found
- **Symptoms**: ImportError when importing rclpy
- **Solution**: Ensure ROS 2 environment is properly sourced
```bash
source /opt/ros/humble/setup.bash
echo $PYTHONPATH  # Should include ROS 2 paths
```

**Issue 3**: Isaac Sim connection fails
- **Symptoms**: Isaac Sim client cannot connect to server
- **Solution**: Verify Isaac Sim is running and ports are accessible
```bash
# Check if Isaac Sim server is running
netstat -tuln | grep 50051
docker ps  # If using Docker
```

**Issue 4**: Audio input not working
- **Symptoms**: Voice commands not being captured
- **Solution**: Check microphone permissions and configuration
```bash
# Test audio recording
arecord -D hw:0,0 -f cd test.wav  # Replace hw:0,0 with your device
# Play back to verify
aplay test.wav
```

**Issue 5**: OpenAI API errors
- **Symptoms**: LLM planning fails with authentication errors
- **Solution**: Verify API key is correctly set in environment
```bash
echo $OPENAI_API_KEY  # Should show your API key
# Or check .env file
```

## Performance Optimization

### 1. GPU Acceleration
Ensure all components utilize GPU when available:
- Whisper processing with GPU
- PyTorch models with CUDA
- Vision processing acceleration

### 2. Model Selection
Choose appropriate model sizes for your hardware:
- Whisper: Use 'base' or 'small' for real-time performance
- YOLO: Use 'n' (nano) or 's' (small) for faster inference

### 3. Resource Management
Monitor system resources during operation:
```bash
# Monitor GPU usage
nvidia-smi -l 1

# Monitor CPU and memory
htop

# Monitor ROS 2 topics
source /opt/ros/humble/setup.bash
ros2 topic hz /your_topic_name
```

## Next Steps

After completing the setup:

1. **Run the VLA system**: Start with simple commands to verify functionality
2. **Follow the lessons**: Complete Module 4 lessons in order
3. **Test with Isaac Sim**: Connect to simulation environment
4. **Execute capstone project**: Implement the autonomous humanoid task

## Support and Resources

- **Documentation**: Refer to individual component documentation
- **Issues**: Report setup problems in the project repository
- **Community**: Join the Physical AI & Humanoid Robotics community
- **Troubleshooting**: Check the troubleshooting section in each lesson

Your VLA system is now ready for use. Proceed to Lesson 1 to begin implementing the voice processing component.