# Isaac ROS Perception Pipeline Implementation

## Overview

This document provides detailed instructions for implementing a basic perception pipeline using Isaac ROS. The perception pipeline is crucial for enabling robots to understand and interact with their environment through AI-powered perception capabilities.

## Introduction to Perception Pipelines

### What is a Perception Pipeline?
A perception pipeline in robotics processes sensor data to extract meaningful information about the environment. In the context of Isaac ROS, perception pipelines leverage GPU acceleration to achieve real-time performance for complex AI tasks.

### Components of a Perception Pipeline
1. **Sensor Interface**: Receives raw sensor data from cameras, LiDAR, IMU, etc.
2. **Preprocessing**: Normalizes and formats data for AI models
3. **AI Inference**: Runs deep learning models on the data
4. **Postprocessing**: Converts model outputs to actionable information
5. **Decision Interface**: Provides information to decision-making systems

## Prerequisites

Before implementing the perception pipeline, ensure you have:
- Completed Isaac ROS bridge setup
- Isaac Sim running with appropriate sensors
- ROS 2 Humble environment sourced
- Isaac ROS workspace built successfully

## Perception Pipeline Architecture

### Data Flow in Perception Systems

```
[Sensors] -> [Preprocessing] -> [AI Model] -> [Postprocessing] -> [Decision Making]
```

### Isaac ROS Perception Pipeline Components

1. **Isaac ROS Image Pipeline**: For camera image processing
2. **Isaac ROS Stereo DNN**: For depth estimation
3. **Isaac ROS Detection NITROS**: For object detection
4. **Isaac ROS AprilTag**: For fiducial marker detection
5. **Isaac ROS Bi3D**: For 3D segmentation

## Implementing Basic Perception Pipeline

### 1. Create Perception Pipeline Package

```bash
cd ~/isaac_ros_ws/src
source /opt/ros/humble/setup.bash

# Create a new package for perception pipeline
ros2 pkg create --build-type ament_python isaac_ros_perception_pipeline --dependencies rclpy sensor_msgs vision_msgs cv_bridge geometry_msgs
```

### 2. Create Perception Pipeline Node

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/isaac_ros_perception_pipeline/perception_pipeline.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from std_msgs.msg import String


class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to camera info
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        # Publisher for processed images
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/processed_image',
            10
        )

        # Publisher for perception status
        self.status_publisher = self.create_publisher(
            String,
            '/perception_status',
            10
        )

        # Initialize YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()

        # Transformation for input images
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((640, 640)),
            transforms.ToTensor()
        ])

        # Store camera info
        self.camera_info = None
        self.image_width = 640
        self.image_height = 480

        # Confidence threshold for detections
        self.confidence_threshold = 0.5

        self.get_logger().info('Perception Pipeline initialized')

    def camera_info_callback(self, msg):
        """Callback for camera info"""
        self.camera_info = msg
        self.image_width = msg.width
        self.image_height = msg.height

    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Run perception pipeline
            detections, processed_image = self.run_perception_pipeline(cv_image)

            # Publish detections
            if detections is not None:
                self.detection_publisher.publish(detections)

            # Publish processed image
            processed_img_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
            processed_img_msg.header = image_msg.header
            self.processed_image_publisher.publish(processed_img_msg)

            # Publish status
            status_msg = String()
            status_msg.data = f"Processed {len(detections.detections) if detections else 0} objects"
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

    def run_perception_pipeline(self, cv_image):
        """Run the complete perception pipeline"""
        try:
            # Preprocess image
            input_tensor = self.preprocess_image(cv_image)

            # Run AI inference
            results = self.run_ai_inference(input_tensor)

            # Postprocess results
            detections = self.postprocess_results(results, cv_image.shape[0], cv_image.shape[1])

            # Draw detections on image
            processed_image = self.draw_detections(cv_image, detections)

            return detections, processed_image

        except Exception as e:
            self.get_logger().error(f'Error in perception pipeline: {e}')
            return None, cv_image

    def preprocess_image(self, cv_image):
        """Preprocess image for AI model"""
        # Convert to RGB if needed
        if cv_image.shape[2] == 3:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        else:
            rgb_image = cv_image

        # Convert to tensor
        input_tensor = self.transform(rgb_image)
        input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

        return input_batch

    def run_ai_inference(self, input_batch):
        """Run AI model inference"""
        with torch.no_grad():
            results = self.model(input_batch)
        return results

    def postprocess_results(self, results, img_height, img_width):
        """Postprocess AI model results"""
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, ObjectHypothesis, BoundingBox2D

        detections_msg = Detection2DArray()
        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.header.frame_id = "camera_frame"

        # Extract bounding boxes and labels
        for detection in results.xyxy[0]:  # x1, y1, x2, y2, confidence, class
            x1, y1, x2, y2, conf, cls = detection

            if conf > self.confidence_threshold:  # Confidence threshold
                detection_2d = Detection2D()

                # Set bounding box
                bbox = BoundingBox2D()
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)
                bbox.center.x = float(x1 + (x2 - x1) / 2)
                bbox.center.y = float(y1 + (y2 - y1) / 2)
                detection_2d.bbox = bbox

                # Set confidence and class
                detection_2d.results.append(
                    ObjectHypothesisWithPose(
                        hypothesis=ObjectHypothesis(
                            id=int(cls),
                            score=float(conf)
                        )
                    )
                )

                detections_msg.detections.append(detection_2d)

        return detections_msg

    def draw_detections(self, image, detections):
        """Draw detection results on image"""
        if detections is None:
            return image

        output_image = image.copy()

        for detection in detections.detections:
            # Get bounding box coordinates
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)
            size_x = int(detection.bbox.size_x)
            size_y = int(detection.bbox.size_y)

            # Calculate top-left and bottom-right corners
            x1 = center_x - size_x // 2
            y1 = center_y - size_y // 2
            x2 = center_x + size_x // 2
            y2 = center_y + size_y // 2

            # Draw bounding box
            cv2.rectangle(output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Add confidence text
            if detection.results:
                confidence = detection.results[0].hypothesis.score
                cv2.putText(output_image, f'{confidence:.2f}',
                           (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                           0.5, (0, 255, 0), 1)

        return output_image


def main(args=None):
    rclpy.init(args=args)
    perception_pipeline = PerceptionPipeline()

    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### 3. Create Setup File for the Package

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/setup.py << 'EOF'
from setuptools import setup

package_name = 'isaac_ros_perception_pipeline'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Basic perception pipeline using Isaac ROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_pipeline = isaac_ros_perception_pipeline.perception_pipeline:main',
        ],
    },
)
EOF
```

### 4. Create Package XML

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_ros_perception_pipeline</name>
  <version>0.0.1</version>
  <description>Basic perception pipeline using Isaac ROS</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF
```

## Advanced Perception Pipeline Components

### 1. Semantic Segmentation Pipeline

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/isaac_ros_perception_pipeline/semantic_segmentation.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
import numpy as np
import cv2


class SemanticSegmentation(Node):
    def __init__(self):
        super().__init__('semantic_segmentation')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publisher for segmentation masks
        self.mask_publisher = self.create_publisher(
            Image,
            '/segmentation_mask',
            10
        )

        # Publisher for segmented image overlay
        self.overlay_publisher = self.create_publisher(
            Image,
            '/segmentation_overlay',
            10
        )

        # Load segmentation model (using torchvision's FCN-ResNet101)
        self.model = torch.hub.load('pytorch/vision:v0.10.0', 'fcn_resnet101', pretrained=True)
        self.model.eval()

        # Define class names for visualization
        self.class_names = [
            'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle',
            'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog',
            'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa',
            'train', 'tvmonitor'
        ]

        # Color palette for segmentation visualization
        self.color_palette = np.random.randint(0, 255, size=(len(self.class_names), 3))

        self.get_logger().info('Semantic Segmentation node initialized')

    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Run segmentation
            segmentation, overlay = self.run_segmentation(cv_image)

            # Publish segmentation mask
            if segmentation is not None:
                mask_msg = self.bridge.cv2_to_imgmsg(segmentation, encoding="mono8")
                mask_msg.header = image_msg.header
                self.mask_publisher.publish(mask_msg)

            # Publish overlay image
            if overlay is not None:
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_msg.header = image_msg.header
                self.overlay_publisher.publish(overlay_msg)

        except Exception as e:
            self.get_logger().error(f'Error in segmentation: {e}')

    def run_segmentation(self, cv_image):
        """Run semantic segmentation on the image"""
        try:
            # Preprocess image
            input_tensor = transforms.functional.to_tensor(cv_image)
            input_tensor = input_tensor.unsqueeze(0)  # Add batch dimension

            # Run segmentation
            with torch.no_grad():
                output = self.model(input_tensor)['out'][0]
                segmentation = output.argmax(0).cpu().numpy()

            # Create colored segmentation mask
            colored_mask = self.color_palette[segmentation].astype(np.uint8)

            # Create overlay image
            overlay = cv2.addWeighted(cv_image, 0.7, colored_mask, 0.3, 0)

            return segmentation, overlay

        except Exception as e:
            self.get_logger().error(f'Error in segmentation pipeline: {e}')
            return None, cv_image


def main(args=None):
    rclpy.init(args=args)
    segmentation_node = SemanticSegmentation()

    try:
        rclpy.spin(segmentation_node)
    except KeyboardInterrupt:
        pass
    finally:
        segmentation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### 2. Depth Estimation Pipeline

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/isaac_ros_perception_pipeline/depth_estimation.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
import numpy as np
import cv2


class DepthEstimation(Node):
    def __init__(self):
        super().__init__('depth_estimation')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to depth images
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Publisher for processed depth maps
        self.depth_publisher = self.create_publisher(
            Image,
            '/estimated_depth',
            10
        )

        # Publisher for point cloud (as image for visualization)
        self.pointcloud_publisher = self.create_publisher(
            Image,
            '/pointcloud_visualization',
            10
        )

        # Load depth estimation model (using MiDaS)
        try:
            self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS", pretrained=True)
            self.model.eval()
        except Exception as e:
            self.get_logger().warn(f'Could not load MiDaS model: {e}')
            self.model = None

        # Transformation for depth estimation
        self.transform = transforms.Compose([
            transforms.Resize((384, 384)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        self.get_logger().info('Depth Estimation node initialized')

    def depth_callback(self, image_msg):
        """Callback for depth images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "passthrough")

            if self.model is not None:
                # Run depth estimation
                depth_map = self.run_depth_estimation(cv_image)

                # Publish depth map
                if depth_map is not None:
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="mono16")
                    depth_msg.header = image_msg.header
                    self.depth_publisher.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f'Error in depth estimation: {e}')

    def run_depth_estimation(self, cv_image):
        """Run depth estimation on the image"""
        try:
            # Convert to RGB if needed
            if len(cv_image.shape) == 2:  # Grayscale
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            else:
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Preprocess image
            input_tensor = self.transform(rgb_image)
            input_batch = input_tensor.unsqueeze(0)

            # Run depth estimation
            with torch.no_grad():
                prediction = self.model(input_batch)

            # Normalize depth prediction
            depth_map = prediction.squeeze().cpu().numpy()
            depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
            depth_map = (depth_map * 65535).astype(np.uint16)  # Convert to 16-bit

            return depth_map

        except Exception as e:
            self.get_logger().error(f'Error in depth estimation pipeline: {e}')
            return cv_image


def main(args=None):
    rclpy.init(args=args)
    depth_node = DepthEstimation()

    try:
        rclpy.spin(depth_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Perception Pipeline Launch Files

### 1. Create Launch Directory

```bash
mkdir -p ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/launch
```

### 2. Create Basic Perception Pipeline Launch File

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/launch/basic_perception_pipeline.launch.py << 'EOF'
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch file for basic perception pipeline."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    # Perception pipeline node
    perception_pipeline_node = Node(
        package='isaac_ros_perception_pipeline',
        executable='perception_pipeline',
        name='perception_pipeline',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/object_detections', '/object_detections'),
        ]
    )

    # Semantic segmentation node
    semantic_segmentation_node = Node(
        package='isaac_ros_perception_pipeline',
        executable='semantic_segmentation',
        name='semantic_segmentation',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/segmentation_mask', '/segmentation_mask'),
        ]
    )

    # Depth estimation node
    depth_estimation_node = Node(
        package='isaac_ros_perception_pipeline',
        executable='depth_estimation',
        name='depth_estimation',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
            ('/estimated_depth', '/estimated_depth'),
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
        perception_pipeline_node,
        semantic_segmentation_node,
        depth_estimation_node
    ])
EOF
```

## Isaac ROS Perception Pipeline Optimization

### 1. GPU Optimization Configuration

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/config/perception_optimization.yaml << 'EOF'
# Isaac ROS Perception Pipeline Optimization Configuration

perception_pipeline:
  # GPU memory management
  gpu_memory:
    max_memory_allocation: 80%
    memory_pool_size: 512MB
    enable_memory_pool: true

  # Model optimization
  model_optimization:
    enable_tensorrt: true
    precision: "fp16"
    max_batch_size: 1
    dynamic_shapes: true

  # Pipeline optimization
  pipeline:
    enable_async_processing: true
    max_queue_size: 10
    enable_batching: false
    input_buffer_size: 2
    output_buffer_size: 2

  # Performance parameters
  performance:
    target_fps: 30
    max_latency_ms: 100
    enable_profiling: true
    profile_output_file: "/tmp/perception_profile.json"

semantic_segmentation:
  # GPU memory management
  gpu_memory:
    max_memory_allocation: 60%
    memory_pool_size: 256MB
    enable_memory_pool: true

  # Model optimization
  model_optimization:
    enable_tensorrt: true
    precision: "fp16"
    max_batch_size: 1
    dynamic_shapes: false

depth_estimation:
  # GPU memory management
  gpu_memory:
    max_memory_allocation: 40%
    memory_pool_size: 256MB
    enable_memory_pool: true

  # Model optimization
  model_optimization:
    enable_tensorrt: true
    precision: "fp16"
    max_batch_size: 1
    dynamic_shapes: true
EOF
```

## Building and Testing the Perception Pipeline

### 1. Build the Perception Pipeline Package

```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select isaac_ros_perception_pipeline
source install/setup.bash
```

### 2. Create Test Script for Perception Pipeline

```bash
cat > ~/test_perception_pipeline.sh << 'EOF'
#!/bin/bash

# Test script for Isaac ROS Perception Pipeline

echo "Testing Isaac ROS Perception Pipeline..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if perception pipeline package is available
echo "Checking perception pipeline package..."
ros2 pkg list | grep perception_pipeline

if [ $? -eq 0 ]; then
    echo "✓ Perception pipeline package found"
else
    echo "✗ Perception pipeline package not found"
    exit 1
fi

# Check if Isaac ROS perception packages are available
echo "Checking Isaac ROS perception packages..."
ros2 pkg list | grep isaac_ros

if [ $? -eq 0 ]; then
    echo "✓ Isaac ROS packages found"
else
    echo "✗ Isaac ROS packages not found"
    exit 1
fi

# Test perception pipeline launch
echo "Testing perception pipeline launch..."
timeout 15s ros2 launch isaac_ros_perception_pipeline basic_perception_pipeline.launch.py use_sim_time:=true &

# Wait a moment for launch
sleep 10

# Check if perception nodes are running
echo "Checking running perception nodes..."
ros2 node list | grep -i perception

# Check perception topics
echo "Checking perception topics..."
ros2 topic list | grep -i perception

# Kill the test launch
pkill -f "basic_perception_pipeline.launch.py"

echo "Isaac ROS Perception Pipeline test completed."
EOF

# Make executable
chmod +x ~/test_perception_pipeline.sh
```

### 3. Run Perception Pipeline Test

```bash
~/test_perception_pipeline.sh
```

## Performance Monitoring and Evaluation

### 1. Create Performance Monitoring Script

```bash
cat > ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/perception_performance_monitor.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
import time
from collections import deque


class PerceptionPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('perception_performance_monitor')

        # Subscribe to perception pipeline topics
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publishers for performance metrics
        self.fps_publisher = self.create_publisher(Float32, '/perception_fps', 10)
        self.latency_publisher = self.create_publisher(Float32, '/perception_latency', 10)
        self.detection_rate_publisher = self.create_publisher(Float32, '/detection_rate', 10)

        # Performance tracking
        self.frame_times = deque(maxlen=30)  # Last 30 frames for FPS calculation
        self.detection_times = deque(maxlen=30)  # Last 30 detections for latency
        self.last_image_time = None
        self.last_detection_time = None
        self.detection_count = 0
        self.image_count = 0

        # Setup timer for periodic metrics publishing
        self.timer = self.create_timer(1.0, self.publish_metrics)

        self.get_logger().info('Perception Performance Monitor initialized')

    def image_callback(self, msg):
        """Track image reception for FPS calculation"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.image_count += 1

        if self.last_image_time is not None:
            frame_time = current_time - self.last_image_time
            self.frame_times.append(frame_time)

        self.last_image_time = current_time

    def detection_callback(self, msg):
        """Track detection for latency and rate calculation"""
        detection_time = self.get_clock().now().nanoseconds / 1e9
        self.detection_count += 1

        # Calculate latency if we have corresponding image time
        if self.last_image_time is not None:
            latency = detection_time - self.last_image_time
            self.detection_times.append(latency)

        self.last_detection_time = detection_time

    def publish_metrics(self):
        """Publish performance metrics"""
        # Calculate FPS
        if len(self.frame_times) > 0:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0
            fps_msg = Float32()
            fps_msg.data = fps
            self.fps_publisher.publish(fps_msg)

        # Calculate average latency
        if len(self.detection_times) > 0:
            avg_latency = sum(self.detection_times) / len(self.detection_times)
            latency_msg = Float32()
            latency_msg.data = avg_latency
            self.latency_publisher.publish(latency_msg)

        # Calculate detection rate
        detection_rate_msg = Float32()
        detection_rate_msg.data = self.detection_count
        self.detection_rate_publisher.publish(detection_rate_msg)

        # Reset counters for next period
        self.detection_count = 0

    def get_performance_summary(self):
        """Get a summary of performance metrics"""
        summary = {
            'fps': 0.0,
            'avg_latency': 0.0,
            'detection_rate': 0.0
        }

        if len(self.frame_times) > 0:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            summary['fps'] = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

        if len(self.detection_times) > 0:
            summary['avg_latency'] = sum(self.detection_times) / len(self.detection_times)

        summary['detection_rate'] = self.detection_count

        return summary


def main(args=None):
    rclpy.init(args=args)
    monitor = PerceptionPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        summary = monitor.get_performance_summary()
        print(f"Performance Summary: {summary}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### 2. Add Performance Monitor to Setup Files

```bash
# Add to setup.py entry points
sed -i '/entry_points=/a\        "perception_performance_monitor = isaac_ros_perception_pipeline.perception_performance_monitor:main,"' ~/isaac_ros_ws/src/isaac_ros_perception_pipeline/setup.py
```

## Troubleshooting Perception Pipeline

### Common Issues and Solutions

#### Issue: "CUDA out of memory error"
**Solution**: Reduce model batch size or input resolution
```bash
# Check GPU memory usage
nvidia-smi

# Reduce input size in perception pipeline
# Modify the transform in the perception pipeline to use smaller images
```

#### Issue: "Perception pipeline not receiving images"
**Solution**: Check topic remapping and Isaac Sim camera configuration
```bash
# Check available topics
ros2 topic list | grep camera

# Check if Isaac Sim is publishing images
ros2 topic echo /camera/color/image_raw --field data --field header
```

#### Issue: "High latency in perception"
**Solution**: Optimize model and pipeline settings
```bash
# Check current performance
ros2 topic echo /perception_fps

# Adjust optimization parameters in config file
```

## Verification Checklist

- [ ] Perception pipeline package created and built successfully
- [ ] Basic object detection pipeline implemented
- [ ] Semantic segmentation pipeline implemented
- [ ] Depth estimation pipeline implemented
- [ ] Launch files created for pipeline
- [ ] Optimization configuration applied
- [ ] Performance monitoring implemented
- [ ] Test script created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After implementing the basic perception pipeline:

1. **Test the pipeline** with Isaac Sim
2. **Optimize performance** based on monitoring results
3. **Integrate with navigation system** for Lesson 2
4. **Create perception exercises** for student practice

The Isaac ROS perception pipeline is now implemented and ready for Module 3, providing students with a foundation for AI-powered robot perception using NVIDIA Isaac platform.