# Isaac ROS Bridge Setup and Configuration

## Overview

This document provides comprehensive instructions for setting up and configuring the Isaac ROS bridge, which enables communication between NVIDIA Isaac Sim and ROS 2. The Isaac ROS bridge is essential for integrating AI perception and navigation capabilities with robotic platforms.

## Introduction to Isaac ROS

### What is Isaac ROS?
Isaac ROS is a collection of GPU-accelerated perception and navigation packages designed to run on NVIDIA platforms. It provides:

- High-performance perception algorithms optimized for NVIDIA GPUs
- ROS 2 interfaces for seamless integration with the ROS ecosystem
- Real-time processing capabilities for robotics applications
- Integration with Isaac Sim for simulation and testing

### Key Components
1. **Isaac ROS Common**: Core utilities and interfaces
2. **Isaac ROS Perception**: GPU-accelerated perception algorithms
3. **Isaac ROS Navigation**: GPU-accelerated navigation packages
4. **Isaac ROS Manipulation**: Manipulation-specific packages
5. **Isaac ROS Visual SLAM**: Simultaneous localization and mapping

## Prerequisites

Before setting up Isaac ROS bridge, ensure you have:
- Completed the Module 3 setup (Docker, NVIDIA drivers, ROS 2 Humble)
- Isaac Sim Docker image pulled and tested
- Isaac ROS workspace built successfully
- NVIDIA GPU with CUDA support

## Isaac ROS Package Installation

### 1. Verify Isaac ROS Workspace
```bash
# Check if Isaac ROS workspace exists and is built
ls -la ~/isaac_ros_ws/
ls -la ~/isaac_ros_ws/install/
```

### 2. Install Additional Isaac ROS Dependencies
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash

# Install additional dependencies
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions
sudo apt install -y ros-humble-vision-msgs ros-humble-geometry-msgs
sudo apt install -y ros-humble-sensor-msgs ros-humble-nav-msgs
sudo apt install -y ros-humble-std-msgs ros-humble-message-filters
```

### 3. Build Isaac ROS Workspace Again (if needed)
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  isaac_ros_common \
  isaac_ros_image_pipeline \
  isaac_ros_visual_slam \
  isaac_ros_bi3d \
  isaac_ros_apriltag
```

## Isaac ROS Bridge Configuration

### 1. Create Isaac ROS Bridge Configuration Directory
```bash
mkdir -p ~/isaac_ros_ws/src/isaac_ros_bridge/config
```

### 2. Basic ROS Bridge Configuration
Create a basic configuration file for the ROS bridge:

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_bridge/config/basic_bridge_config.yaml << 'EOF'
# Basic Isaac ROS Bridge Configuration

# Camera configuration
camera:
  image_topic: "/camera/color/image_raw"
  camera_info_topic: "/camera/color/camera_info"
  width: 640
  height: 480
  fps: 30

# LiDAR configuration
lidar:
  scan_topic: "/scan"
  pointcloud_topic: "/pointcloud"
  range_min: 0.1
  range_max: 25.0

# IMU configuration
imu:
  topic: "/imu"
  frame_id: "imu_link"

# Robot state publisher
robot_state_publisher:
  joint_state_topic: "/joint_states"
  tf_prefix: ""

# ROS bridge parameters
ros_bridge:
  publish_tf: true
  enable_compression: false
  compression_level: 1
EOF
```

### 3. Isaac ROS Perception Pipeline Configuration
```bash
cat > ~/isaac_ros_ws/src/isaac_ros_bridge/config/perception_pipeline.yaml << 'EOF'
# Isaac ROS Perception Pipeline Configuration

# Object detection configuration
object_detection:
  model_type: "yolov5"
  model_path: "/workspace/models/yolov5s.pt"
  confidence_threshold: 0.5
  nms_threshold: 0.4
  image_input_topic: "/camera/color/image_raw"
  detection_output_topic: "/object_detections"
  enable_visualization: true

# Depth estimation configuration
depth_estimation:
  model_type: "midas"
  model_path: "/workspace/models/midas.pt"
  input_topic: "/camera/depth/image_rect_raw"
  output_topic: "/estimated_depth"
  enable_visualization: true

# Semantic segmentation configuration
semantic_segmentation:
  model_type: "fcn_resnet101"
  model_path: "/workspace/models/fcn_resnet101.pt"
  input_topic: "/camera/color/image_raw"
  output_topic: "/segmentation_mask"
  enable_visualization: true

# Feature detection configuration
feature_detection:
  detector_type: "orb"
  descriptor_type: "orb"
  max_features: 1000
  matching_threshold: 0.8
  input_topic: "/camera/color/image_raw"
  output_topic: "/features"
EOF
```

## Isaac Sim ROS Bridge Setup

### 1. Create Isaac Sim Extension for ROS Bridge
```bash
mkdir -p ~/isaac_sim_shared/extensions/isaac_ros_bridge
```

### 2. Isaac Sim ROS Bridge Extension Configuration
```bash
cat > ~/isaac_sim_shared/extensions/isaac_ros_bridge/extension.toml << 'EOF'
[package]
name = "omni.isaac.ros_bridge"
title = "Isaac ROS Bridge"
version = "1.0.0"
category = "ROS"
description = "Extension to bridge Isaac Sim with ROS 2"
author = "NVIDIA"

[dependencies]
"omni.isaac.ros2_bridge" = {}

[python]
requires = "3.7"

[settings.ros_bridge]
enabled = true
domain_id = 1
namespace = ""
use_sim_time = true
EOF
```

### 3. ROS Bridge Launch Configuration
```bash
cat > ~/isaac_ros_ws/src/isaac_ros_bridge/launch/isaac_ros_bridge.launch.py << 'EOF'
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch file for Isaac ROS Bridge."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    # Isaac ROS Image Pipeline nodes
    debayer_node = Node(
        package='isaac_ros_image_proc',
        executable='isaac_ros_debayer_exe',
        name='isaac_ros_debayer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_encoding': 'bayer_rggb8',
            'output_encoding': 'rgb8'
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('image', '/camera/color/image_debayered')
        ]
    )

    # Isaac ROS AprilTag node for fiducial detection
    apriltag_node = Node(
        package='isaac_ros_apriltag',
        executable='isaac_ros_apriltag_exe',
        name='isaac_ros_apriltag',
        parameters=[{
            'use_sim_time': use_sim_time,
            'family': 'tag36h11',
            'max_tags': 1,
            'tag_size': 0.166
        }],
        remappings=[
            ('image', '/camera/color/image_debayered'),
            ('camera_info', '/camera/color/camera_info'),
            ('detections', '/apriltag_detections')
        ]
    )

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='isaac_ros_visual_slam',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_rectified_pose': True,
            'rectified_pose_frame_id': 'camera_link'
        }],
        remappings=[
            ('/visual_slam/camera0/extrinsic_parameter', '/camera/color/camera_info'),
            ('/visual_slam/camera0/pose', '/camera/color/image_raw'),
            ('/visual_slam/camera0/linear_acceleration', '/accel'),
            ('/visual_slam/camera0/angular_velocity', '/gyro')
        ]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'),
        debayer_node,
        apriltag_node,
        visual_slam_node
    ])
EOF
```

## Isaac ROS Perception Setup

### 1. Install Isaac ROS Perception Dependencies
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash

# Install perception-specific dependencies
sudo apt install -y ros-humble-vision-msgs
sudo apt install -y ros-humble-image-transport
sudo apt install -y ros-humble-compressed-image-transport
sudo apt install -y ros-humble-compressed-depth-image-transport
```

### 2. Create Isaac ROS Perception Launch File
```bash
cat > ~/isaac_ros_ws/src/isaac_ros_bridge/launch/isaac_perception.launch.py << 'EOF'
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch file for Isaac ROS Perception Pipeline."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    # Isaac ROS Detection NITROS node
    detection_node = Node(
        package='isaac_ros_detection_based_onnx',
        executable='isaac_ros_detection_based_onnx_exe',
        name='isaac_ros_detection',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': '/workspace/models/yolov5s.onnx',
            'input_tensor_layout': 'NCHW',
            'input_binding_name': 'images',
            'output_binding_names': ['output'],
            'model_input_width': 640,
            'model_input_height': 640,
            'confidence_threshold': 0.5,
            'max_batch_size': 1,
            'input_tensor_names': ['images'],
            'output_tensor_names': ['output']
        }],
        remappings=[
            ('image', '/camera/color/image_rect_color'),
            ('detections', '/detections')
        ]
    )

    # Isaac ROS Stereo DNN node for depth estimation
    stereo_dnn_node = Node(
        package='isaac_ros_stereo_dnn',
        executable='isaac_ros_stereo_dnn_exe',
        name='isaac_ros_stereo_dnn',
        parameters=[{
            'use_sim_time': use_sim_time,
            'network_selection': 0,  # 0 for DispNet, 1 for DYN dispnet
            'input_layer_names': ['left_tensor', 'right_tensor'],
            'output_layer_names': ['output_disp_tensor'],
            'tensor_padding': [0.0, 0.0, 0.0, 0.0],
            'max_disparity_values': [64],
            'confidence_threshold': 0.5
        }],
        remappings=[
            ('left_image', '/camera/depth/image_raw'),
            ('right_image', '/camera/depth/image_raw'),  # In simulation, using same image for both
            ('disparity', '/disparity_map')
        ]
    )

    # Isaac ROS Image Format Converter for preprocessing
    format_converter_node = Node(
        package='isaac_ros_image_proc',
        executable='isaac_ros_image_format_converter_exe',
        name='isaac_ros_image_format_converter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_encoding': 'rgb8',
            'output_encoding': 'rgb8'
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('image', '/camera/color/image_converted')
        ]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'),
        format_converter_node,
        detection_node,
        stereo_dnn_node
    ])
EOF
```

## Isaac ROS Navigation Setup

### 1. Install Isaac ROS Navigation Dependencies
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash

# Install navigation-specific dependencies
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-slam-toolbox
```

### 2. Create Isaac ROS Navigation Configuration
```bash
cat > ~/isaac_ros_ws/src/isaac_ros_bridge/config/navigation_config.yaml << 'EOF'
# Isaac ROS Navigation Configuration

amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.2
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: False
    default_nav_through_poses_bt_xml: "navigate_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_consistent_localization_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_on_amcl_reset_condition_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # DWB parameters
    FollowPath:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      speed_limit: 1.0
      sim_time: 1.7
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_frequency: 20.0
      angular_smooth_weight: 1.0
      time_smooth_weight: 0.1
      path_distance_bias: 32.0
      goal_distance_bias: 24.0
      occdist_scale: 0.02
      forward_point_distance: 0.325
      stop_time_buffer: 0.2
      scaling_speed: 0.25
      max_scaling_factor: 0.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        enabled: True
        map_subscribe_transient_local: True
      inflation_layer:
        enabled: True
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: True
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_queue_size: 10
EOF
```

## Isaac ROS Bridge Testing

### 1. Create Isaac ROS Bridge Test Script
```bash
cat > ~/test_isaac_ros_bridge.sh << 'EOF'
#!/bin/bash

# Test script for Isaac ROS Bridge setup

echo "Testing Isaac ROS Bridge setup..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if Isaac ROS packages are available
echo "Checking Isaac ROS packages..."
ros2 pkg list | grep isaac

if [ $? -eq 0 ]; then
    echo "✓ Isaac ROS packages found"
else
    echo "✗ Isaac ROS packages not found"
    exit 1
fi

# Check for Isaac ROS specific nodes
echo "Checking Isaac ROS nodes..."
ros2 node list | grep -i isaac

# Test Isaac ROS bridge launch
echo "Testing Isaac ROS bridge launch..."
timeout 10s ros2 launch isaac_ros_bridge isaac_ros_bridge.launch.py use_sim_time:=true &

# Wait a moment for launch
sleep 5

# Check if nodes are running
echo "Checking running nodes..."
ros2 node list

# Kill the test launch
pkill -f "isaac_ros_bridge.launch.py"

echo "Isaac ROS Bridge setup test completed."
EOF

# Make executable
chmod +x ~/test_isaac_ros_bridge.sh
```

### 2. Run Isaac ROS Bridge Test
```bash
~/test_isaac_ros_bridge.sh
```

## Isaac ROS Bridge Integration with Isaac Sim

### 1. Isaac Sim ROS Bridge Extension Setup
```bash
# Create Isaac Sim extension for ROS bridge
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
EOF
```

### 2. Isaac Sim Python Script for ROS Bridge
```bash
cat > ~/isaac_sim_shared/ros_bridge_script.py << 'EOF'
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.sensor import AcquisitionSensor
import carb

# Enable ROS bridge extension
omni.kit.app.get_app().extension_manager.set_enabled("omni.isaac.ros2_bridge", True)

# Create world instance
world = World(stage_units_in_meters=1.0)

# Add a simple robot to the stage
assets_root_path = get_assets_root_path()
if assets_root_path is not None:
    # Add a simple robot from the assets library
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Carter/carter.usd",
        prim_path="/World/Carter"
    )

# Set up camera view
set_camera_view(eye=[5, 5, 5], target=[0, 0, 0])

# Initialize the world
world.reset()

# Simulation loop
for i in range(500):
    # Step the world
    world.step(render=True)

    # Print progress every 100 steps
    if i % 100 == 0:
        print(f"Simulation step: {i}")

# Clean up
world.clear()
carb.log_info("Isaac Sim with ROS Bridge script completed.")
EOF
```

## Isaac ROS Bridge Performance Optimization

### 1. GPU Optimization Configuration
```bash
cat > ~/isaac_ros_ws/src/isaac_ros_bridge/config/gpu_optimization.yaml << 'EOF'
# Isaac ROS GPU Optimization Configuration

# GPU memory management
gpu_memory:
  max_memory_allocation: 80%  # Percentage of GPU memory to use
  memory_pool_size: 512MB     # Size of memory pool for allocations
  enable_memory_pool: true    # Enable memory pooling for efficiency

# TensorRT optimization parameters
tensorrt:
  enable_tensorrt: true
  precision: "fp16"           # Precision mode: fp32, fp16, int8
  max_batch_size: 1          # Maximum batch size for inference
  dynamic_shapes: true       # Enable dynamic shape optimization

# CUDA stream configuration
cuda_streams:
  num_streams: 2             # Number of CUDA streams for parallel processing
  enable_events: true        # Enable CUDA events for synchronization

# Pipeline optimization
pipeline:
  enable_async_processing: true  # Enable asynchronous processing
  max_queue_size: 10             # Maximum size of processing queues
  enable_batching: false         # Enable batch processing for multiple inputs
EOF
```

### 2. Isaac ROS Bridge Monitoring Setup
```bash
cat > ~/isaac_ros_ws/src/isaac_ros_bridge/launch/monitoring.launch.py << 'EOF'
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch file for Isaac ROS Bridge monitoring."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    # System monitor node
    system_monitor = Node(
        package='isaac_ros_system_monitor',
        executable='isaac_ros_system_monitor_exe',
        name='isaac_ros_system_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitor_cpu': True,
            'monitor_gpu': True,
            'monitor_memory': True,
            'monitor_temperature': True,
            'publish_frequency': 1.0
        }]
    )

    # Performance monitor node
    performance_monitor = Node(
        package='isaac_ros_performance_monitor',
        executable='isaac_ros_performance_monitor_exe',
        name='isaac_ros_performance_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitor_perception_pipeline': True,
            'monitor_navigation_pipeline': True,
            'monitor_communication': True,
            'publish_frequency': 10.0
        }]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'),
        system_monitor,
        performance_monitor
    ])
EOF
```

## Troubleshooting Isaac ROS Bridge

### Common Issues and Solutions

#### Issue: "Isaac ROS packages not found"
**Solution**: Check if workspace is properly sourced
```bash
source ~/isaac_ros_ws/install/setup.bash
ros2 pkg list | grep isaac
```

#### Issue: "CUDA error: device-side assert triggered"
**Solution**: Check GPU memory and reduce batch sizes
```bash
# Check GPU memory usage
nvidia-smi

# Reduce model batch size in configuration
```

#### Issue: "ROS bridge not connecting to Isaac Sim"
**Solution**: Verify ROS domain ID and network settings
```bash
# Check ROS domain ID
echo $ROS_DOMAIN_ID

# Set to same domain as Isaac Sim
export ROS_DOMAIN_ID=1
```

#### Issue: "High latency in perception pipeline"
**Solution**: Optimize for GPU performance
```bash
# Check GPU utilization
nvidia-smi dmon -s u -d 1

# Adjust pipeline parameters for performance
```

## Verification Checklist

- [ ] Isaac ROS workspace built successfully
- [ ] Isaac ROS packages available in ROS environment
- [ ] Isaac ROS bridge launch files created
- [ ] Perception pipeline configuration complete
- [ ] Navigation configuration complete
- [ ] Isaac Sim ROS bridge extension configured
- [ ] Test script created and functional
- [ ] Performance optimization parameters set
- [ ] Monitoring setup configured
- [ ] Troubleshooting guide reviewed

## Next Steps

After completing Isaac ROS bridge setup:

1. **Test the complete pipeline** with Isaac Sim
2. **Run perception exercises** from Lesson 1
3. **Integrate with navigation system** for Lesson 2
4. **Optimize performance** based on monitoring results

The Isaac ROS bridge is now configured and ready for Module 3 implementation, enabling high-performance AI-robot integration using NVIDIA Isaac platform.