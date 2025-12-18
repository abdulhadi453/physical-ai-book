# Module 3 Setup: NVIDIA Isaac Platform Configuration

## Overview

This document provides detailed instructions for setting up the NVIDIA Isaac platform environment required for Module 3. The setup includes Docker container configuration, Isaac Sim installation, Isaac ROS bridge setup, and integration with ROS 2 Humble Hawksbill.

## Prerequisites

### Hardware Requirements
- **GPU**: NVIDIA GPU with compute capability 6.0 or higher (recommended: RTX 30xx or higher)
- **VRAM**: Minimum 8GB (recommended: 11GB or higher)
- **CPU**: Multi-core processor (Intel i7 or AMD Ryzen 7 or better)
- **RAM**: Minimum 16GB (recommended: 32GB)
- **Storage**: Minimum 50GB free space for Docker images and simulation environments
- **Display**: Monitor capable of running Isaac Sim with appropriate drivers

### Software Requirements
- **OS**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- **Docker**: Version 20.10 or higher
- **NVIDIA Container Toolkit**: Latest version compatible with Docker
- **NVIDIA GPU Drivers**: Version 520 or higher
- **ROS 2**: Humble Hawksbill distribution
- **Python**: Version 3.8 or higher

## System Preparation

### 1. Update System Packages
```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install Required Dependencies
```bash
sudo apt install -y build-essential cmake pkg-config
sudo apt install -y libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y libv4l-dev libxvidcore-dev libx264-dev
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev
sudo apt install -y gfortran openblas libopenblas-dev libatlas-base-dev
```

## NVIDIA GPU Driver Installation

### 1. Check Current Driver
```bash
nvidia-smi
```

### 2. Install NVIDIA Drivers
```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install latest recommended driver
sudo ubuntu-drivers autoinstall

# Or install specific version
sudo apt install nvidia-driver-535  # Replace with latest version

# Reboot system
sudo reboot
```

### 3. Verify GPU Recognition
```bash
# After reboot, check GPU status
nvidia-smi
```

## Docker Installation

### 1. Remove Old Docker Versions
```bash
sudo apt remove docker docker-engine docker.io containerd runc
```

### 2. Install Docker Repository
```bash
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update package index
sudo apt update

# Install Docker Engine
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin
```

### 3. Configure Docker Group
```bash
# Add current user to docker group
sudo groupadd docker  # If group doesn't exist
sudo usermod -aG docker $USER

# Activate changes to groups
newgrp docker

# Verify Docker installation
docker run hello-world
```

## NVIDIA Container Toolkit Installation

### 1. Configure Repository
```bash
# Add the nvidia-container-toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add the repository
echo "deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/ubuntu20.04/$(dpkg --print-architecture) /" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package list
sudo apt update
```

### 2. Install and Configure Toolkit
```bash
# Install nvidia-container-toolkit
sudo apt install -y nvidia-container-toolkit

# Configure Docker to use nvidia runtime
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker daemon
sudo systemctl restart docker
```

### 3. Verify Installation
```bash
# Test GPU access in Docker
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi
```

## ROS 2 Humble Hawksbill Installation

### 1. Set Locale
```bash
locale  # Check for UTF-8
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Add ROS 2 Repository
```bash
# Add the repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to apt sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS 2 Packages
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

### 4. Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

### 5. Set Up Environment
```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## NVIDIA Isaac Sim Installation

### 1. Pull Isaac Sim Docker Image
```bash
# Pull the latest Isaac Sim image
docker pull nvcr.io/nvidia/isaac-sim:latest

# Or pull a specific version
# docker pull nvcr.io/nvidia/isaac-sim:4.0.0
```

### 2. Verify Image Download
```bash
# List Docker images to verify Isaac Sim is present
docker images | grep isaac-sim
```

### 3. Basic Isaac Sim Test
```bash
# Test Isaac Sim with basic launch
xhost +local:docker
docker run --gpus all -e "ACCEPT_EULA=Y" --rm -it \
  -p 5000:5000 \
  --env "NVIDIA_VISIBLE_DEVICES=0" \
  --env "NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute" \
  --env "PRIVILEGED_TRAINING_MODE=0" \
  --network=host \
  --shm-size="1g" \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  nvcr.io/nvidia/isaac-sim:latest
```

## Isaac ROS Bridge Installation

### 1. Create ROS 2 Workspace
```bash
# Create workspace directory
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### 2. Install Isaac ROS Packages
```bash
# Clone Isaac ROS common repository
cd ~/isaac_ros_ws/src
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Clone Isaac ROS navigation packages
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_navigation.git

# Clone Isaac ROS perception packages
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_perception.git

# Clone Isaac ROS manipulation packages
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_manipulation.git
```

### 3. Install Dependencies
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Workspace
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 5. Source Workspace
```bash
# Add to ~/.bashrc
echo "source ~/isaac_ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Nav2 Installation

### 1. Install Nav2 Packages
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization
```

### 2. Verify Installation
```bash
# Check if Nav2 packages are available
ros2 pkg list | grep nav2
```

## Environment Configuration

### 1. Create Isaac Sim Environment File
```bash
# Create environment file
cat > ~/isaac_sim_env.sh << 'EOF'
#!/bin/bash

# NVIDIA Isaac Sim Environment Configuration

# Isaac Sim Docker Image
export ISAAC_SIM_IMAGE="nvcr.io/nvidia/isaac-sim:latest"

# Isaac Sim Container Name
export ISAAC_SIM_CONTAINER="isaac-sim-module3"

# Shared Directory for Isaac Sim
export ISAAC_SIM_SHARED_DIR="$HOME/isaac_sim_shared"

# Create shared directory if it doesn't exist
mkdir -p $ISAAC_SIM_SHARED_DIR

# Isaac ROS Workspace
export ISAAC_ROS_WS="$HOME/isaac_ros_ws"

# ROS 2 Humble
export ROS_DISTRO="humble"
export ROS_DOMAIN_ID=1

echo "Isaac Sim Environment configured"
echo "Image: $ISAAC_SIM_IMAGE"
echo "Shared Dir: $ISAAC_SIM_SHARED_DIR"
echo "ROS Workspace: $ISAAC_ROS_WS"
EOF

# Make executable
chmod +x ~/isaac_sim_env.sh
```

### 2. Create Isaac Sim Launch Script
```bash
# Create launch script
cat > ~/launch_isaac_sim.sh << 'EOF'
#!/bin/bash

# Source environment
source ~/isaac_sim_env.sh

# Check if display is available
if [ -z "$DISPLAY" ]; then
    echo "No display available. Please run from a system with GUI support."
    exit 1
fi

# Enable X11 forwarding
xhost +local:docker

# Launch Isaac Sim with proper GPU access
docker run --gpus all \
  --rm \
  -it \
  --name $ISAAC_SIM_CONTAINER \
  -p 5000:5000 \
  -p 5555-5558:5555-5558 \
  --env "NVIDIA_VISIBLE_DEVICES=0" \
  --env "NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute" \
  --env "ACCEPT_EULA=Y" \
  --env "PRIVILEGED_TRAINING_MODE=0" \
  --network=host \
  --shm-size="1g" \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  -e "DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $ISAAC_SIM_SHARED_DIR:/workspace/shared_dir \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  $ISAAC_SIM_IMAGE

# Clean up X11 permissions
xhost -local:docker
EOF

# Make executable
chmod +x ~/launch_isaac_sim.sh
```

### 3. Create Isaac ROS Integration Script
```bash
# Create ROS integration script
cat > ~/setup_isaac_ros.sh << 'EOF'
#!/bin/bash

# Setup script for Isaac ROS integration

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source Isaac ROS workspace
source $HOME/isaac_ros_ws/install/setup.bash

# Set ROS domain ID for Isaac
export ROS_DOMAIN_ID=1

# Set Isaac Sim path (if running Isaac Sim in container, this is for host communication)
export ISAAC_SIM_HOST=localhost
export ISAAC_SIM_PORT=5000

# Set Isaac ROS parameters
export ISAAC_ROS_WS=$HOME/isaac_ros_ws

echo "Isaac ROS integration environment configured"
echo "ROS Domain ID: $ROS_DOMAIN_ID"
echo "Isaac ROS Workspace: $ISAAC_ROS_WS"
EOF

# Make executable
chmod +x ~/setup_isaac_ros.sh
```

## Testing the Setup

### 1. Test Isaac Sim Launch
```bash
# Run the launch script
~/launch_isaac_sim.sh
```

### 2. Test ROS Communication
```bash
# In a new terminal, source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check ROS nodes (should see Isaac Sim nodes if connected)
ros2 node list
```

### 3. Test Isaac ROS Packages
```bash
# Check available Isaac ROS packages
ros2 pkg list | grep isaac
```

### 4. Test Nav2 Installation
```bash
# Check Nav2 nodes
ros2 node list | grep nav
```

## Troubleshooting

### Common Issues and Solutions

#### Issue: "nvidia-smi" command not found
**Solution**: NVIDIA drivers are not installed properly
```bash
sudo apt install nvidia-driver-535
sudo reboot
```

#### Issue: Docker GPU access denied
**Solution**: NVIDIA Container Toolkit not properly configured
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

#### Issue: Isaac Sim fails to launch with GUI
**Solution**: X11 forwarding not enabled
```bash
xhost +local:docker
```

#### Issue: ROS nodes not communicating
**Solution**: Check ROS_DOMAIN_ID and network configuration
```bash
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=1
```

#### Issue: Isaac Sim Docker container exits immediately
**Solution**: Check GPU access and Docker permissions
```bash
# Test GPU access in Docker
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi
```

## Verification Checklist

- [ ] NVIDIA GPU drivers installed and recognized
- [ ] Docker installed and running
- [ ] NVIDIA Container Toolkit installed and configured
- [ ] ROS 2 Humble installed and sourced
- [ ] Isaac Sim Docker image pulled successfully
- [ ] Isaac ROS workspace built without errors
- [ ] Nav2 packages installed
- [ ] Isaac Sim can be launched with GUI
- [ ] ROS communication established between host and Isaac Sim
- [ ] Isaac ROS packages available and accessible

## Next Steps

After successful setup, proceed to:
1. **Lesson 1**: NVIDIA Isaac Simulation Environment and ROS Integration
2. **Exercise 1**: Basic Isaac Sim Setup
3. **Exercise 2**: ROS Bridge Communication

The system is now ready for Module 3 implementation. All necessary components for AI-robot brain development using NVIDIA Isaac platform are installed and configured.