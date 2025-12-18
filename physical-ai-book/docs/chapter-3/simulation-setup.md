# Isaac Sim Simulation Environment Configuration

## Overview

This document provides comprehensive instructions for configuring the NVIDIA Isaac Sim simulation environment for Module 3. The simulation environment serves as the foundation for testing AI-robot systems, perception pipelines, and navigation algorithms in a controlled, repeatable environment.

## Introduction to Isaac Sim Environment

### What is Isaac Sim?
NVIDIA Isaac Sim is a robotics simulator built on NVIDIA Omniverse that provides high-fidelity physics simulation, sensor simulation, and AI training capabilities. It enables realistic testing of AI-robot systems before deployment on physical hardware.

### Key Features of Isaac Sim
1. **High-Fidelity Physics**: Accurate physics simulation with PhysX engine
2. **Sensor Simulation**: Realistic camera, LiDAR, IMU, and other sensor models
3. **AI Training Support**: Integration with reinforcement learning frameworks
4. **ROS/ROS2 Bridge**: Seamless integration with ROS/ROS2 ecosystem
5. **Extensible Architecture**: Custom extensions and USD-based scenes

## Prerequisites

Before configuring the simulation environment, ensure you have:
- Completed Isaac Sim installation and basic setup
- Docker with NVIDIA Container Toolkit configured
- Isaac ROS bridge installed and configured
- Basic understanding of USD (Universal Scene Description) format

## Isaac Sim Docker Configuration

### 1. Isaac Sim Docker Environment Setup

```bash
# Create Isaac Sim environment configuration
cat > ~/isaac_sim_env_config.sh << 'EOF'
#!/bin/bash

# Isaac Sim Environment Configuration

# Isaac Sim Docker Image
export ISAAC_SIM_IMAGE="nvcr.io/nvidia/isaac-sim:latest"

# Isaac Sim Container Configuration
export ISAAC_SIM_CONTAINER_NAME="isaac-sim-module3"
export ISAAC_SIM_SHARED_DIR="$HOME/isaac_sim_shared"
export ISAAC_SIM_VOLUME_DIR="$HOME/isaac_sim_volumes"

# Create necessary directories
mkdir -p $ISAAC_SIM_SHARED_DIR
mkdir -p $ISAAC_SIM_VOLUME_DIR
mkdir -p $ISAAC_SIM_SHARED_DIR/maps
mkdir -p $ISAAC_SIM_SHARED_DIR/models
mkdir -p $ISAAC_SIM_SHARED_DIR/scenes
mkdir -p $ISAAC_SIM_SHARED_DIR/logs

# Isaac Sim Port Configuration
export ISAAC_SIM_WEB_PORT=5000
export ISAAC_SIM_RPC_PORTS="5555-5558"

# Isaac Sim Graphics Configuration
export NVIDIA_VISIBLE_DEVICES=0
export NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Isaac Sim Environment Variables
export ACCEPT_EULA=Y
export PRIVILEGED_TRAINING_MODE=0

# Isaac ROS Integration
export ISAAC_ROS_WS="$HOME/isaac_ros_ws"
export ISAAC_ROS_DOMAIN_ID=1

echo "Isaac Sim environment configured"
echo "Shared Directory: $ISAAC_SIM_SHARED_DIR"
echo "Container Name: $ISAAC_SIM_CONTAINER_NAME"
echo "ROS Workspace: $ISAAC_ROS_WS"
EOF

# Make executable
chmod +x ~/isaac_sim_env_config.sh
source ~/isaac_sim_env_config.sh
```

### 2. Isaac Sim Docker Launch Script

```bash
cat > ~/launch_isaac_sim_configured.sh << 'EOF'
#!/bin/bash

# Source environment configuration
source ~/isaac_sim_env_config.sh

# Enable X11 forwarding for GUI
if [ -z "$DISPLAY" ]; then
    echo "No display available. Please run from a system with GUI support."
    exit 1
fi

xhost +local:docker

# Launch Isaac Sim with custom configuration
docker run --gpus all \
  --rm \
  -it \
  --name $ISAAC_SIM_CONTAINER_NAME \
  -p $ISAAC_SIM_WEB_PORT:$ISAAC_SIM_WEB_PORT \
  -p $ISAAC_SIM_RPC_PORTS:$ISAAC_SIM_RPC_PORTS \
  --env "NVIDIA_VISIBLE_DEVICES=$NVIDIA_VISIBLE_DEVICES" \
  --env "NVIDIA_DRIVER_CAPABILITIES=$NVIDIA_DRIVER_CAPABILITIES" \
  --env "ACCEPT_EULA=$ACCEPT_EULA" \
  --env "PRIVILEGED_TRAINING_MODE=$PRIVILEGED_TRAINING_MODE" \
  --env "ISAAC_ROS_DOMAIN_ID=$ISAAC_ROS_DOMAIN_ID" \
  --network=host \
  --shm-size="1g" \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  -e "DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $ISAAC_SIM_SHARED_DIR:/workspace/shared_dir \
  -v $ISAAC_SIM_VOLUME_DIR:/workspace/volumes \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -v /etc/localtime:/etc/localtime:ro \
  $ISAAC_SIM_IMAGE

# Clean up X11 permissions
xhost -local:docker
EOF

# Make executable
chmod +x ~/launch_isaac_sim_configured.sh
```

## Isaac Sim Scene Configuration

### 1. Create Basic Scene Configuration

```bash
mkdir -p ~/isaac_sim_shared/scenes
```

### 2. Create Indoor Navigation Scene

```bash
cat > ~/isaac_sim_shared/scenes/basic_navigation.usd << 'EOF'
#usda 1.0
(
    doc = """Basic indoor navigation scene for Isaac Sim Module 3"""
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "GroundPlane"
    {
        def Mesh "Plane"
        {
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
            float3[] points = [(-100, 0, -100), (100, 0, -100), (100, 0, 100), (-100, 0, 100), (-100, 1, -100), (100, 1, -100), (100, 1, 100), (-100, 1, 100), (-100, -1, -100), (100, -1, -100), (100, -1, 100), (-100, -1, 100), (-100, 0, -101), (100, 0, -101), (100, 0, 101), (-100, 0, 101), (-101, 0, -100), (-99, 0, -100), (-99, 0, 100), (-101, 0, 100), (100, 0, -100), (100, 0, -99), (100, 0, 99), (100, 0, 100)]
            def Material "VisualMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </World/GroundPlane/Plane/Material/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.1
                    float inputs:metallic = 0.0
                    color3f inputs:diffuse_color = (0.2, 0.2, 0.2)
                }
            }
        }
    }

    def Xform "Walls"
    {
        def Cube "Wall1"
        {
            float3 xformOp:translate = (0, 50, 0)
            float3 xformOp:scale = (200, 100, 2)
            rel material:binding = </World/GroundPlane/Plane/Material>
        }

        def Cube "Wall2"
        {
            float3 xformOp:translate = (0, -50, 0)
            float3 xformOp:scale = (200, 100, 2)
            rel material:binding = </World/GroundPlane/Plane/Material>
        }

        def Cube "Wall3"
        {
            float3 xformOp:translate = (100, 0, 0)
            float3 xformOp:scale = (2, 100, 200)
            rel material:binding = </World/GroundPlane/Plane/Material>
        }

        def Cube "Wall4"
        {
            float3 xformOp:translate = (-100, 0, 0)
            float3 xformOp:scale = (2, 100, 200)
            rel material:binding = </World/GroundPlane/Plane/Material>
        }
    }

    def Xform "Obstacles"
    {
        def Cube "Obstacle1"
        {
            float3 xformOp:translate = (20, 20, 0)
            float3 xformOp:scale = (5, 5, 10)
            rel material:binding = </World/GroundPlane/Plane/Material>
        }

        def Cube "Obstacle2"
        {
            float3 xformOp:translate = (-30, -15, 0)
            float3 xformOp:scale = (8, 8, 12)
            rel material:binding = </World/GroundPlane/Plane/Material>
        }

        def Cylinder "Pillar"
        {
            float3 xformOp:translate = (0, 0, 0)
            float radius = 5
            float height = 20
            rel material:binding = </World/GroundPlane/Plane/Material>
        }
    }

    def Xform "Robot"
    {
        def Xform "Carter"
        {
            float3 xformOp:translate = (-50, -40, 0)
            float3 xformOp:orient = (1, 0, 0, 0)  # Identity quaternion (no rotation)

            # Robot body
            def Cube "Base"
            {
                float3 xformOp:translate = (0, 0, 0.1)
                float3 xformOp:scale = (0.5, 0.3, 0.2)
            }

            # Wheels
            def Cylinder "FrontLeftWheel"
            {
                float3 xformOp:translate = (0.15, 0.15, 0)
                float radius = 0.05
                float height = 0.04
            }

            def Cylinder "FrontRightWheel"
            {
                float3 xformOp:translate = (0.15, -0.15, 0)
                float radius = 0.05
                float height = 0.04
            }

            def Cylinder "BackLeftWheel"
            {
                float3 xformOp:translate = (-0.15, 0.15, 0)
                float radius = 0.05
                float height = 0.04
            }

            def Cylinder "BackRightWheel"
            {
                float3 xformOp:translate = (-0.15, -0.15, 0)
                float radius = 0.05
                float height = 0.04
            }
        }
    }

    def Xform "Lighting"
    {
        def DistantLight "KeyLight"
        {
            float3 color = (1, 1, 1)
            float intensity = 3000
            float3 xformOp:rotateXYZ = (60, 45, 0)
        }

        def DistantLight "FillLight"
        {
            float3 color = (0.5, 0.5, 0.5)
            float intensity = 1500
            float3 xformOp:rotateXYZ = (-60, 135, 0)
        }
    }
}
EOF
```

### 3. Create Perception Test Scene

```bash
cat > ~/isaac_sim_shared/scenes/perception_test.usd << 'EOF'
#usda 1.0
(
    doc = """Perception test scene for Isaac Sim Module 3"""
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "GroundPlane"
    {
        def Mesh "Plane"
        {
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            float3[] points = [(-50, 0, -50), (50, 0, -50), (50, 0, 50), (-50, 0, 50)]

            def Material "FloorMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </World/GroundPlane/Plane/FloorMaterial/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.8
                    float inputs:metallic = 0.0
                    color3f inputs:diffuse_color = (0.7, 0.7, 0.7)
                }
            }
        }
    }

    def Xform "Objects"
    {
        def Cube "RedBox"
        {
            float3 xformOp:translate = (10, 0, 10)
            float3 xformOp:scale = (2, 2, 2)
            def Material "RedMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </World/Objects/RedBox/RedMaterial/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.5
                    float inputs:metallic = 0.0
                    color3f inputs:diffuse_color = (1.0, 0.0, 0.0)
                }
            }
        }

        def Sphere "BlueSphere"
        {
            float3 xformOp:translate = (-10, 0, 10)
            float radius = 2
            def Material "BlueMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </World/Objects/BlueSphere/BlueMaterial/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.3
                    float inputs:metallic = 0.2
                    color3f inputs:diffuse_color = (0.0, 0.0, 1.0)
                }
            }
        }

        def Cylinder "GreenCylinder"
        {
            float3 xformOp:translate = (0, 0, -10)
            float radius = 1.5
            float height = 4
            def Material "GreenMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </World/Objects/GreenCylinder/GreenMaterial/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.4
                    float inputs:metallic = 0.1
                    color3f inputs:diffuse_color = (0.0, 1.0, 0.0)
                }
            }
        }

        def Cone "YellowCone"
        {
            float3 xformOp:translate = (15, 0, -15)
            float radius = 2
            float height = 4
            def Material "YellowMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </World/Objects/YellowCone/YellowMaterial/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.6
                    float inputs:metallic = 0.0
                    color3f inputs:diffuse_color = (1.0, 1.0, 0.0)
                }
            }
        }
    }

    def Xform "Robot"
    {
        def Xform "PerceptionRobot"
        {
            float3 xformOp:translate = (0, 0, 0)
            float3 xformOp:orient = (1, 0, 0, 0)

            # Robot body
            def Cube "Base"
            {
                float3 xformOp:translate = (0, 0, 0.1)
                float3 xformOp:scale = (0.5, 0.3, 0.2)
            }

            # Camera
            def Camera "RGB_Camera"
            {
                float3 xformOp:translate = (0.2, 0, 0.15)
                float3 clippingRange = (0.1, 100)
                float focalLength = 24
                float horizontalAperture = 36
                float verticalAperture = 24
            }

            # Depth sensor
            def Camera "Depth_Camera"
            {
                float3 xformOp:translate = (0.2, 0, 0.15)
                float3 clampingRange = (0.1, 10)
                float focalLength = 24
            }
        }
    }

    def Xform "Lighting"
    {
        def DistantLight "MainLight"
        {
            float3 color = (1, 1, 1)
            float intensity = 5000
            float3 xformOp:rotateXYZ = (45, 30, 0)
        }

        def DomeLight "EnvironmentLight"
        {
            float3 color = (0.2, 0.2, 0.2)
            float intensity = 1
        }
    }
}
EOF
```

## Isaac Sim Extensions Configuration

### 1. Create Isaac Sim Extensions Directory

```bash
mkdir -p ~/isaac_sim_shared/extensions
```

### 2. Create Isaac ROS Bridge Extension

```bash
mkdir -p ~/isaac_sim_shared/extensions/omni.isaac.ros2_bridge
cat > ~/isaac_sim_shared/extensions/omni.isaac.ros2_bridge/config/extension.toml << 'EOF'
[package]
name = "omni.isaac.ros2_bridge"
title = "Isaac ROS 2 Bridge"
version = "1.0.0"
category = "ROS"
description = "Extension to bridge Isaac Sim with ROS 2"
author = "NVIDIA"

[dependencies]
"omni.kit.ui_app" = {}
"omni.isaac.core" = {}

[python]
requires = "3.7"

[settings.ros2_bridge]
enabled = true
domain_id = 1
namespace = ""
use_sim_time = true
enable_compression = false
compression_level = 1
publish_tf = true
enable_statistics = false
EOF
```

### 3. Create Isaac Sim Perception Extension

```bash
mkdir -p ~/isaac_sim_shared/extensions/omni.isaac.perception
cat > ~/isaac_sim_shared/extensions/omni.isaac.perception/config/extension.toml << 'EOF'
[package]
name = "omni.isaac.perception"
title = "Isaac Perception"
version = "1.0.0"
category = "Perception"
description = "Extension for perception simulation in Isaac Sim"
author = "NVIDIA"

[dependencies]
"omni.isaac.core" = {}
"omni.isaac.sensor" = {}

[python]
requires = "3.7"

[settings.perception]
enabled = true
enable_object_detection = true
enable_segmentation = true
enable_depth_estimation = true
enable_point_cloud = true
detection_model = "yolov5"
segmentation_model = "fcn_resnet101"
depth_model = "midas"
detection_confidence_threshold = 0.5
enable_visualization = true
EOF
```

## Isaac Sim Robot Configuration

### 1. Create Carter Robot USD File

```bash
cat > ~/isaac_sim_shared/robots/carter_robot.usd << 'EOF'
#usda 1.0
(
    doc = """Carter robot configuration for Isaac Sim"""
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "Carter"
{
    def Xform "Chassis"
    {
        def Cube "Base"
        {
            float3 xformOp:translate = (0, 0, 0.1)
            float3 xformOp:scale = (0.5, 0.3, 0.2)

            def Material "ChassisMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </Carter/Chassis/Base/ChassisMaterial/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.7
                    float inputs:metallic = 0.5
                    color3f inputs:diffuse_color = (0.8, 0.8, 0.8)
                }
            }
        }
    }

    def Xform "Wheels"
    {
        def Cylinder "FrontLeftWheel"
        {
            float3 xformOp:translate = (0.15, 0.15, 0)
            float radius = 0.05
            float height = 0.04
            float3 xformOp:rotateXYZ = (90, 0, 0)

            def Material "WheelMaterial"
            {
                def Shader "PreviewSurface"
                {
                    uniform token inputs:surface = </Carter/Wheels/FrontLeftWheel/WheelMaterial/PreviewSurface.outputs:surface>
                    float inputs:roughness = 0.9
                    float inputs:metallic = 0.1
                    color3f inputs:diffuse_color = (0.2, 0.2, 0.2)
                }
            }
        }

        def Cylinder "FrontRightWheel"
        {
            float3 xformOp:translate = (0.15, -0.15, 0)
            float radius = 0.05
            float height = 0.04
            float3 xformOp:rotateXYZ = (90, 0, 0)
            rel material:binding = </Carter/Wheels/FrontLeftWheel/WheelMaterial>
        }

        def Cylinder "BackLeftWheel"
        {
            float3 xformOp:translate = (-0.15, 0.15, 0)
            float radius = 0.05
            float height = 0.04
            float3 xformOp:rotateXYZ = (90, 0, 0)
            rel material:binding = </Carter/Wheels/FrontLeftWheel/WheelMaterial>
        }

        def Cylinder "BackRightWheel"
        {
            float3 xformOp:translate = (-0.15, -0.15, 0)
            float radius = 0.05
            float height = 0.04
            float3 xformOp:rotateXYZ = (90, 0, 0)
            rel material:binding = </Carter/Wheels/FrontLeftWheel/WheelMaterial>
        }
    }

    def Xform "Sensors"
    {
        def Camera "RGB_Camera"
        {
            float3 xformOp:translate = (0.2, 0, 0.15)
            float3 clampingRange = (0.1, 100)
            float focalLength = 24
            float horizontalAperture = 36
            float verticalAperture = 24
        }

        def Camera "Depth_Camera"
        {
            float3 xformOp:translate = (0.2, 0, 0.15)
            float3 clampingRange = (0.1, 10)
            float focalLength = 24
        }

        def RotatingLidar "Lidar"
        {
            float3 xformOp:translate = (0.18, 0, 0.19)
            float3 linearVelocity = (0, 0, 0)
            float3 angularVelocity = (0, 0, 0)
        }

        def Imu "IMU"
        {
            float3 xformOp:translate = (0, 0, 0.1)
        }
    }

    def Xform "Physics"
    {
        # Collision meshes and rigid body properties
        def Mesh "CollisionMesh"
        {
            # Simplified collision representation
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
            float3[] points = [(-0.25, -0.15, -0.1), (0.25, -0.15, -0.1), (0.25, 0.15, -0.1), (-0.25, 0.15, -0.1), (-0.25, -0.15, 0.1), (0.25, -0.15, 0.1), (0.25, 0.15, 0.1), (-0.25, 0.15, 0.1), (-0.25, -0.15, -0.1), (0.25, -0.15, -0.1), (0.25, -0.15, 0.1), (-0.25, -0.15, 0.1), (-0.25, 0.15, -0.1), (0.25, 0.15, -0.1), (0.25, 0.15, 0.1), (-0.25, 0.15, 0.1), (-0.25, -0.15, -0.1), (-0.25, 0.15, -0.1), (-0.25, 0.15, 0.1), (-0.25, -0.15, 0.1), (0.25, -0.15, -0.1), (0.25, 0.15, -0.1), (0.25, 0.15, 0.1), (0.25, -0.15, 0.1)]
        }
    }
}
EOF
```

## Isaac Sim Physics Configuration

### 1. Create Physics Scene Configuration

```bash
cat > ~/isaac_sim_shared/configs/physics_config.json << 'EOF'
{
  "physics": {
    "gravity": [0, -9.81, 0],
    "solverType": "TGS",
    "numPositionIterations": 4,
    "numVelocityIterations": 1,
    "maxDepenetrationVelocity": 1000.0,
    "maxAngularSpeed": 50.0,
    "sleepThreshold": 0.005,
    "stabilizationThreshold": 0.001,
    "cacheKinematicBodies": true,
    "enableEnhancedDeterminism": false,
    "useGroupSpheres": false,
    "enableSceneQuerySupport": true
  },
  "rigidBodies": {
    "defaultSleepThreshold": 0.005,
    "defaultStabilizationThreshold": 0.001,
    "solverPositionIterationCount": 4,
    "solverVelocityIterationCount": 1
  },
  "materials": {
    "defaultStaticFriction": 0.5,
    "defaultDynamicFriction": 0.5,
    "defaultRestitution": 0.1
  }
}
EOF
```

### 2. Create Sensor Configuration

```bash
cat > ~/isaac_sim_shared/configs/sensor_config.json << 'EOF'
{
  "cameras": {
    "default_resolution": [640, 480],
    "default_framerate": 30,
    "default_fov": 60,
    "enable_distortion": false,
    "color_correct": true
  },
  "lidar": {
    "default_samples": 360,
    "default_range": [0.1, 25.0],
    "default_fov": 360,
    "default_fps": 10,
    "enable_noise": true,
    "noise_params": {
      "mean": 0.0,
      "stddev": 0.01
    }
  },
  "imu": {
    "default_rate": 100,
    "enable_noise": true,
    "noise_params": {
      "accelerometer_noise_density": 0.001,
      "gyroscope_noise_density": 0.0001
    }
  },
  "depth": {
    "default_resolution": [640, 480],
    "default_framerate": 30,
    "enable_noise": true,
    "noise_params": {
      "mean": 0.0,
      "stddev": 0.001
    }
  }
}
EOF
```

## Isaac Sim Simulation Scripts

### 1. Create Simulation Control Script

```bash
cat > ~/isaac_sim_shared/scripts/simulation_control.py << 'EOF'
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.sensor import AcquisitionSensor
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import carb
import numpy as np
import asyncio

# Enable ROS bridge extension
omni.kit.app.get_app().extension_manager.set_enabled("omni.isaac.ros2_bridge", True)

# Enable perception extension
omni.kit.app.get_app().extension_manager.set_enabled("omni.isaac.perception", True)

async def setup_simulation():
    """Set up the Isaac Sim environment for Module 3"""

    # Create world instance
    world = World(stage_units_in_meters=1.0)

    # Get assets root path
    assets_root_path = get_assets_root_path()

    if assets_root_path is not None:
        # Add Carter robot from assets library
        robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_instanceable.usd"
        add_reference_to_stage(
            usd_path=robot_path,
            prim_path="/World/Carter"
        )

        # Set initial position and orientation
        world.scene.add_default_ground_plane()

        # Set up camera view
        set_camera_view(eye=[5, 5, 5], target=[0, 0, 0])

        # Initialize the world
        await world.initialize_async()

        # Reset the world
        world.reset()

        print("Isaac Sim environment initialized for Module 3")

        # Run simulation for a few steps
        for i in range(100):
            await world.step_async(render=True)
            if i % 20 == 0:
                print(f"Simulation step: {i}")

        # Clean up
        world.clear()
        carb.log_info("Isaac Sim setup completed.")

        return world
    else:
        carb.log_error("Could not find assets root path")
        return None

def run_basic_simulation():
    """Run a basic simulation to test the environment"""
    try:
        world = asyncio.run(setup_simulation())
        if world:
            print("Basic simulation completed successfully")
        else:
            print("Failed to set up simulation")
    except Exception as e:
        print(f"Error running simulation: {e}")

if __name__ == "__main__":
    run_basic_simulation()
EOF
```

### 2. Create Perception Simulation Script

```bash
cat > ~/isaac_sim_shared/scripts/perception_simulation.py << 'EOF'
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import carb

def setup_perception_simulation():
    """Set up Isaac Sim for perception testing"""

    # Enable required extensions
    ext_manager = omni.kit.app.get_app().extension_manager
    ext_manager.set_enabled("omni.isaac.ros2_bridge", True)
    ext_manager.set_enabled("omni.isaac.perception", True)

    # Create world instance
    world = World(stage_units_in_meters=1.0)

    # Get assets root path
    assets_root_path = get_assets_root_path()

    if assets_root_path is not None:
        # Add a simple robot with sensors
        robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_instanceable.usd"
        add_reference_to_stage(
            usd_path=robot_path,
            prim_path="/World/Carter"
        )

        # Add perception test objects
        add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Props/Blocks/block_instanceable.usd",
            prim_path="/World/RedBlock"
        )

        # Set up ground plane
        world.scene.add_default_ground_plane()

        # Set up camera view
        set_camera_view(eye=[3, 3, 3], target=[0, 0, 0])

        # Initialize the world
        world.initialize()

        # Get robot prim for positioning
        robot_prim = get_prim_at_path("/World/Carter")
        if robot_prim:
            # Move robot to starting position
            robot_prim.GetAttribute("xformOp:translate").Set(carb.Float3(0, 0, 0.2))

        # Reset the world
        world.reset()

        print("Perception simulation environment set up")

        # Run simulation for perception testing
        for i in range(200):
            world.step(render=True)

            if i % 50 == 0:
                print(f"Perception simulation step: {i}")

        # Clean up
        world.clear()
        carb.log_info("Perception simulation completed.")

    else:
        carb.log_error("Could not find assets root path")

if __name__ == "__main__":
    setup_perception_simulation()
EOF
```

## Isaac Sim Configuration for Module 3

### 1. Create Module 3 Specific Configuration

```bash
mkdir -p ~/isaac_sim_shared/configs/module3
```

### 2. Create Module 3 Scene Configuration

```bash
cat > ~/isaac_sim_shared/configs/module3/module3_config.json << 'EOF'
{
  "module3": {
    "name": "AI-Robot Brain (NVIDIA Isaac™)",
    "version": "1.0.0",
    "description": "Configuration for Module 3: The AI-Robot Brain",
    "scenes": {
      "navigation": "/workspace/shared_dir/scenes/basic_navigation.usd",
      "perception": "/workspace/shared_dir/scenes/perception_test.usd",
      "integration": "/workspace/shared_dir/scenes/integration_test.usd"
    },
    "robots": {
      "default": "/workspace/shared_dir/robots/carter_robot.usd"
    },
    "sensors": {
      "camera": {
        "resolution": [640, 480],
        "framerate": 30,
        "fov": 60
      },
      "lidar": {
        "samples": 360,
        "range": [0.1, 25.0],
        "fps": 10
      },
      "imu": {
        "rate": 100
      }
    },
    "simulation": {
      "gravity": [0, -9.81, 0],
      "max_step_size": 0.01,
      "solver_iterations": 4,
      "enable_collision": true
    },
    "ros_integration": {
      "domain_id": 1,
      "namespace": "",
      "use_sim_time": true,
      "bridge_enabled": true
    },
    "perception": {
      "object_detection": {
        "model": "yolov5",
        "confidence_threshold": 0.5,
        "enable_visualization": true
      },
      "segmentation": {
        "model": "fcn_resnet101",
        "enable_visualization": true
      },
      "depth_estimation": {
        "model": "midas",
        "enable_visualization": true
      }
    },
    "navigation": {
      "planner": "navfn",
      "controller": "dwb",
      "costmap_resolution": 0.05,
      "robot_radius": 0.22
    }
  }
}
EOF
```

### 3. Create Isaac Sim Startup Script for Module 3

```bash
cat > ~/isaac_sim_shared/scripts/module3_startup.py << 'EOF'
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.sensor import Camera, RotatingLidar, Imu
import carb
import json
import asyncio

# Load Module 3 configuration
CONFIG_PATH = "/workspace/shared_dir/configs/module3/module3_config.json"

def load_config():
    """Load Module 3 configuration"""
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return config["module3"]
    except Exception as e:
        carb.log_error(f"Could not load config: {e}")
        # Return default config
        return {
            "scenes": {
                "navigation": "/workspace/shared_dir/scenes/basic_navigation.usd"
            },
            "robots": {
                "default": "/workspace/shared_dir/robots/carter_robot.usd"
            }
        }

def setup_module3_environment():
    """Set up the complete Module 3 environment"""

    # Enable required extensions
    ext_manager = omni.kit.app.get_app().extension_manager
    ext_manager.set_enabled("omni.isaac.ros2_bridge", True)
    ext_manager.set_enabled("omni.isaac.perception", True)

    # Load configuration
    config = load_config()

    # Create world instance
    world = World(stage_units_in_meters=1.0)

    # Get assets root path
    assets_root_path = get_assets_root_path()

    if assets_root_path is not None:
        # Add Carter robot
        robot_path = config["robots"]["default"]
        add_reference_to_stage(
            usd_path=robot_path,
            prim_path="/World/Carter"
        )

        # Set up ground plane
        world.scene.add_default_ground_plane()

        # Set up camera view
        set_camera_view(eye=[5, 5, 5], target=[0, 0, 0])

        # Initialize the world
        world.initialize()

        # Reset the world
        world.reset()

        print("Module 3 environment set up successfully")
        print(f"Using scene: {config['scenes']['navigation']}")
        print(f"Using robot: {config['robots']['default']}")

        # Run initial simulation steps
        for i in range(50):
            world.step(render=True)
            if i % 10 == 0:
                print(f"Environment initialization step: {i}")

        # Clean up
        world.clear()
        carb.log_info("Module 3 environment ready for exercises.")

    else:
        carb.log_error("Could not find assets root path")

def run_module3_simulation():
    """Run the Module 3 simulation setup"""
    try:
        setup_module3_environment()
        print("Module 3 simulation environment is ready for student exercises")
    except Exception as e:
        print(f"Error setting up Module 3 environment: {e}")

if __name__ == "__main__":
    run_module3_simulation()
EOF
```

## Isaac Sim Performance Optimization

### 1. Create Performance Configuration

```bash
cat > ~/isaac_sim_shared/configs/performance_config.json << 'EOF'
{
  "rendering": {
    "max_texture_resolution": 2048,
    "max_mesh_complexity": 100000,
    "enable_msaa": true,
    "msaa_samples": 4,
    "enable_motion_blur": false,
    "enable_dof": false
  },
  "physics": {
    "max_substeps": 1,
    "min_step_size": 0.001,
    "max_step_size": 0.01,
    "solver_type": "TGS",
    "position_iterations": 4,
    "velocity_iterations": 1
  },
  "simulation": {
    "target_framerate": 60,
    "max_update_rate": 500,
    "enable_multithreading": true,
    "worker_thread_count": 4
  },
  "gpu": {
    "max_memory_allocation": "80%",
    "enable_gpu_physics": true,
    "gpu_physics_solver": "PhysX",
    "cuda_stream_count": 2
  },
  "caching": {
    "enable_texture_caching": true,
    "texture_cache_size": "512MB",
    "enable_mesh_caching": true,
    "mesh_cache_size": "256MB",
    "enable_material_caching": true
  }
}
EOF
```

## Testing and Validation

### 1. Create Isaac Sim Test Script

```bash
cat > ~/test_isaac_sim_setup.sh << 'EOF'
#!/bin/bash

# Test script for Isaac Sim environment setup

echo "Testing Isaac Sim environment setup..."

# Check if Isaac Sim shared directory exists
if [ -d "$HOME/isaac_sim_shared" ]; then
    echo "✓ Isaac Sim shared directory exists"
else
    echo "✗ Isaac Sim shared directory does not exist"
    exit 1
fi

# Check for scene files
if [ -f "$HOME/isaac_sim_shared/scenes/basic_navigation.usd" ]; then
    echo "✓ Navigation scene file exists"
else
    echo "✗ Navigation scene file does not exist"
    exit 1
fi

if [ -f "$HOME/isaac_sim_shared/scenes/perception_test.usd" ]; then
    echo "✓ Perception test scene file exists"
else
    echo "✗ Perception test scene file does not exist"
    exit 1
fi

# Check for robot files
if [ -f "$HOME/isaac_sim_shared/robots/carter_robot.usd" ]; then
    echo "✓ Carter robot file exists"
else
    echo "✗ Carter robot file does not exist"
    exit 1
fi

# Check for configuration files
if [ -f "$HOME/isaac_sim_shared/configs/module3/module3_config.json" ]; then
    echo "✓ Module 3 configuration file exists"
else
    echo "✗ Module 3 configuration file does not exist"
    exit 1
fi

# Check for scripts
if [ -f "$HOME/isaac_sim_shared/scripts/module3_startup.py" ]; then
    echo "✓ Module 3 startup script exists"
else
    echo "✗ Module 3 startup script does not exist"
    exit 1
fi

echo "Isaac Sim environment setup test completed successfully."
echo "All required files and configurations are in place."
EOF

# Make executable
chmod +x ~/test_isaac_sim_setup.sh
```

### 2. Run Isaac Sim Setup Test

```bash
~/test_isaac_sim_setup.sh
```

## Troubleshooting Isaac Sim Configuration

### Common Issues and Solutions

#### Issue: "Isaac Sim fails to start with graphics error"
**Solution**: Check GPU access and Docker configuration
```bash
# Verify GPU access in Docker
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi

# Check NVIDIA Container Toolkit
nvidia-ctk runtime configure --runtime=docker --debug
```

#### Issue: "USD scenes not loading properly"
**Solution**: Check file permissions and paths
```bash
# Check permissions
ls -la ~/isaac_sim_shared/scenes/

# Verify USD syntax
# This would be done inside Isaac Sim
```

#### Issue: "ROS bridge not connecting"
**Solution**: Verify ROS domain and network settings
```bash
# Check ROS domain
echo $ROS_DOMAIN_ID

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash
```

## Verification Checklist

- [ ] Isaac Sim Docker environment configured
- [ ] Navigation and perception scenes created
- [ ] Robot USD files created and configured
- [ ] Extensions configured for Isaac Sim
- [ ] Physics and sensor configurations created
- [ ] Simulation control scripts created
- [ ] Module 3 specific configuration created
- [ ] Performance optimization configuration created
- [ ] Test script created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After configuring the simulation environment:

1. **Test the complete setup** with Isaac Sim
2. **Run simulation exercises** from Lessons 1-3
3. **Integrate with perception and navigation systems**
4. **Create simulation-based assessments**

The Isaac Sim simulation environment is now configured and ready for Module 3, providing students with a comprehensive platform for AI-robot system development and testing.