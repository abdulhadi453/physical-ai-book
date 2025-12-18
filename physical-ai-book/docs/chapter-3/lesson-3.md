# Lesson 3: Perception and AI Decision Making

## Overview

This lesson focuses on implementing perception systems using NVIDIA Isaac's AI capabilities and creating intelligent decision-making algorithms for robots. You'll learn how to process sensor data using AI models and make intelligent decisions based on perception results.

## Learning Objectives

After completing this lesson, you will be able to:

- Implement perception systems using Isaac's AI capabilities
- Deploy machine learning models for robot perception
- Create AI decision-making algorithms
- Integrate perception with action planning
- Optimize perception systems for real-time performance

## Introduction to AI Perception in Isaac

AI perception in NVIDIA Isaac involves using deep learning models to process sensor data and extract meaningful information about the environment. This includes:

- Object detection and classification
- Semantic segmentation
- Depth estimation
- Pose estimation
- Activity recognition

### Key Isaac AI Perception Components

1. **Isaac ROS Manipulator Perception**: For object detection in manipulation tasks
2. **Isaac ROS Stereo DNN**: For depth estimation using stereo cameras
3. **Isaac ROS Detection NITROS**: For optimized object detection
4. **Isaac ROS AprilTag**: For fiducial marker detection
5. **Isaac ROS Stereo Image Rectification**: For stereo image processing

## Agent Interaction Points

### AI Assistant Request: Isaac AI Perception Architecture
**Context**: Understanding Isaac AI perception components
**Request**: "Explain the architecture of Isaac's AI perception system and how different components work together"
**Expected Output**: Detailed explanation of Isaac AI perception architecture and component interactions

### AI Assistant Request: Troubleshoot Perception Pipeline
**Context**: Isaac perception pipeline setup
**Request**: "I'm having issues with my Isaac perception pipeline. The object detection isn't working properly. What should I check?"
**Expected Output**: Troubleshooting guide for Isaac perception pipeline issues

### AI Assistant Request: Optimize AI Performance
**Context**: Performance optimization for AI perception
**Request**: "How can I optimize my AI perception system for real-time performance on Isaac?"
**Expected Output**: Guide to optimizing AI perception performance on Isaac platform

## Perception Pipeline Architecture

### Data Flow in Perception Systems

```
[Sensors] -> [Preprocessing] -> [AI Model] -> [Postprocessing] -> [Decision Making]
```

### Components of an AI Perception Pipeline

1. **Sensor Interface**: Receives raw sensor data
2. **Data Preprocessing**: Normalizes and formats data for AI models
3. **AI Inference**: Runs deep learning models on the data
4. **Result Postprocessing**: Converts model outputs to actionable information
5. **Decision Interface**: Provides information to decision-making systems

## Object Detection with Isaac

### Isaac Detection Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms

class IsaacObjectDetection(Node):
    def __init__(self):
        super().__init__('isaac_object_detection')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publisher for detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        # Load pre-trained model (example with TorchVision)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()

        # Transformation for input images
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((640, 640)),
            transforms.ToTensor()
        ])

    def image_callback(self, image_msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Preprocess image
        input_tensor = self.transform(cv_image)
        input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

        # Run inference
        with torch.no_grad():
            results = self.model(input_batch)

        # Process results
        detections_msg = self.process_detections(results, image_msg.height, image_msg.width)

        # Publish detections
        self.detection_publisher.publish(detections_msg)

    def process_detections(self, results, img_height, img_width):
        # Process YOLOv5 results
        detections_msg = Detection2DArray()

        # Extract bounding boxes and labels
        for detection in results.xyxy[0]:  # x1, y1, x2, y2, confidence, class
            x1, y1, x2, y2, conf, cls = detection
            if conf > 0.5:  # Confidence threshold
                detection_2d = Detection2D()

                # Set bounding box
                bbox = BoundingBox2D()
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)
                bbox.center.x = float(x1 + (x2 - x1) / 2)
                bbox.center.y = float(y1 + (y2 - y1) / 2)
                detection_2d.bbox = bbox

                # Set confidence
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

def main(args=None):
    rclpy.init(args=args)
    detector = IsaacObjectDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Semantic Segmentation

Semantic segmentation assigns a class label to each pixel in an image, enabling detailed scene understanding.

### Isaac Semantic Segmentation Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
import numpy as np

class IsaacSemanticSegmentation(Node):
    def __init__(self):
        super().__init__('isaac_semantic_segmentation')

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

        # Load segmentation model
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

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Preprocess image
        input_tensor = self.transform(cv_image)
        input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

        # Run segmentation
        with torch.no_grad():
            output = self.model(input_batch)['out'][0]
            segmentation = output.argmax(0).cpu().numpy()

        # Create colored segmentation mask
        colored_mask = self.color_palette[segmentation].astype(np.uint8)

        # Convert back to ROS Image and publish
        mask_msg = self.bridge.cv2_to_imgmsg(colored_mask, encoding="rgb8")
        mask_msg.header = msg.header
        self.mask_publisher.publish(mask_msg)

def main(args=None):
    rclpy.init(args=args)
    segmenter = IsaacSemanticSegmentation()
    rclpy.spin(segmenter)
    segmenter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Estimation

Depth estimation provides 3D information from 2D images, crucial for navigation and manipulation.

### Isaac Depth Estimation Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
import numpy as np

class IsaacDepthEstimation(Node):
    def __init__(self):
        super().__init__('isaac_depth_estimation')

        self.bridge = CvBridge()

        # Subscribe to stereo images
        self.left_image_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Publisher for depth maps
        self.depth_publisher = self.create_publisher(
            Image,
            '/estimated_depth',
            10
        )

        # Load depth estimation model
        # Using MiDaS for monocular depth estimation
        self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS", pretrained=True)
        self.model.eval()

        # Transformation for depth estimation
        self.transform = transforms.Compose([
            transforms.Resize((384, 384)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def depth_callback(self, image_msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Preprocess image
        input_tensor = self.transform(cv_image)
        input_batch = input_tensor.unsqueeze(0)

        # Run depth estimation
        with torch.no_grad():
            prediction = self.model(input_batch)

        # Normalize depth prediction
        depth_map = prediction.squeeze().cpu().numpy()
        depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
        depth_map = (depth_map * 255).astype(np.uint8)

        # Convert to ROS Image and publish
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="mono8")
        depth_msg.header = image_msg.header
        self.depth_publisher.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    depth_estimator = IsaacDepthEstimation()
    rclpy.spin(depth_estimator)
    depth_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## AI Decision Making Systems

### Behavior Trees for AI Decision Making

Behavior trees provide a structured approach to AI decision making:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
import math

class BehaviorTreeAI(Node):
    def __init__(self):
        super().__init__('behavior_tree_ai')

        # Publishers and subscribers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.detections = []
        self.obstacle_distances = []
        self.target_found = False
        self.target_position = None

    def detection_callback(self, msg):
        self.detections = msg.detections

    def scan_callback(self, msg):
        self.obstacle_distances = msg.ranges

    def run_behavior_tree(self):
        """Execute the behavior tree"""
        # Check if target is detected
        if self.check_target_detected():
            return self.move_to_target()
        else:
            # Explore environment
            return self.explore_environment()

    def check_target_detected(self):
        """Check if target object is detected"""
        for detection in self.detections:
            # Assuming target is a person (class ID 15 in COCO dataset)
            if detection.results[0].hypothesis.id == 15 and detection.results[0].hypothesis.score > 0.7:
                self.target_position = detection.bbox.center
                return True
        return False

    def move_to_target(self):
        """Move robot towards detected target"""
        twist = Twist()

        # Simple proportional controller
        if self.target_position.x > 320:  # Target is to the right
            twist.angular.z = -0.3
        elif self.target_position.x < 320:  # Target is to the left
            twist.angular.z = 0.3
        else:
            twist.linear.x = 0.2  # Move forward

        return twist

    def explore_environment(self):
        """Explore environment looking for targets"""
        twist = Twist()

        # Check for obstacles
        min_distance = min(self.obstacle_distances) if self.obstacle_distances else float('inf')

        if min_distance < 0.5:  # Obstacle too close
            twist.angular.z = 0.5  # Turn away
        else:
            twist.linear.x = 0.2  # Move forward

        return twist

    def timer_callback(self):
        """Main AI loop"""
        command = self.run_behavior_tree()
        self.cmd_publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    ai = BehaviorTreeAI()

    # Create timer for main loop
    timer = ai.create_timer(0.1, ai.timer_callback)

    rclpy.spin(ai)
    ai.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Fusion

Sensor fusion combines data from multiple sensors to improve perception accuracy.

### Multi-Sensor Fusion Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.spatial.transform import Rotation as R

class IsaacSensorFusion(Node):
    def __init__(self):
        super().__init__('isaac_sensor_fusion')

        # Initialize TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to multiple sensors
        self.camera_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.camera_callback, 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        # Publisher for fused perception
        self.fused_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/fused_pose', 10)

        # Store sensor data
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None

    def camera_callback(self, msg):
        self.camera_data = msg

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def imu_callback(self, msg):
        self.imu_data = msg

    def fuse_sensors(self):
        """Fusion algorithm combining multiple sensors"""
        if not all([self.camera_data, self.lidar_data, self.imu_data]):
            return None

        # Example fusion: combine visual detection with LiDAR and IMU
        # This is a simplified example - real fusion would use more sophisticated methods
        fused_pose = PoseWithCovarianceStamped()

        # Extract orientation from IMU
        orientation = self.imu_data.orientation

        # Use LiDAR for position (simplified)
        min_range_idx = np.argmin(self.lidar_data.ranges)
        range_distance = self.lidar_data.ranges[min_range_idx]

        # Convert to position (simplified - assumes robot facing forward)
        x = range_distance * math.cos(self.lidar_data.angle_min + min_range_idx * self.lidar_data.angle_increment)
        y = range_distance * math.sin(self.lidar_data.angle_min + min_range_idx * self.lidar_data.angle_increment)

        fused_pose.pose.pose.position.x = x
        fused_pose.pose.pose.position.y = y
        fused_pose.pose.pose.orientation = orientation

        # Set covariance based on sensor uncertainties
        covariance = np.zeros(36)
        covariance[0] = 0.1  # x uncertainty
        covariance[7] = 0.1  # y uncertainty
        covariance[35] = 0.05  # yaw uncertainty
        fused_pose.pose.covariance = covariance

        return fused_pose

def main(args=None):
    rclpy.init(args=args)
    fusion = IsaacSensorFusion()
    rclpy.spin(fusion)
    fusion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 5: Basic Object Detection

### Objective
Implement an object detection system using Isaac's AI capabilities.

### Steps
1. Set up the Isaac object detection pipeline
2. Configure the AI model for detection
3. Process camera images for object detection
4. Visualize detection results
5. Test with different objects

### Expected Outcome
System successfully detects and labels objects in camera images.

## Exercise 6: AI Decision Making Integration

### Objective
Integrate AI perception with decision-making algorithms.

### Steps
1. Create a perception system that detects specific objects
2. Implement a decision-making algorithm based on perception
3. Connect perception output to action commands
4. Test the integrated system in simulation
5. Evaluate decision-making performance

### Expected Outcome
Robot makes intelligent decisions based on perceived environment.

## Performance Optimization

### GPU Acceleration

```python
import torch

# Check if CUDA is available
if torch.cuda.is_available():
    device = torch.device("cuda")
    print(f"Using GPU: {torch.cuda.get_device_name(0)}")
else:
    device = torch.device("cpu")
    print("Using CPU")

# Move model to GPU
model = model.to(device)

# Move input data to GPU
input_tensor = input_tensor.to(device)
```

### TensorRT Optimization

```python
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

def optimize_with_tensorrt(model):
    """Optimize model with TensorRT"""
    # Create TensorRT builder
    builder = trt.Builder(trt.Logger(trt.Logger.WARNING))
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()

    # Build engine
    serialized_engine = builder.build_serialized_network(network, config)

    return serialized_engine
```

## Assessment Questions

1. What are the main components of an AI perception pipeline?
2. How does semantic segmentation differ from object detection?
3. What is sensor fusion and why is it important in robotics?
4. How can behavior trees be used for AI decision making?
5. What techniques can be used to optimize AI perception performance?

## Summary

This lesson covered AI perception and decision making in the NVIDIA Isaac platform. You learned about object detection, semantic segmentation, depth estimation, AI decision making systems, and sensor fusion. The exercises provided hands-on experience with implementing AI perception systems and integrating them with decision-making algorithms.

## Agent Interaction Points for Review

### AI Assistant Request: Lesson Review
**Context**: Review of Lesson 3 content
**Request**: "Summarize the key AI perception concepts from Lesson 3 and suggest optimization strategies"
**Expected Output**: Concise summary of AI perception concepts and recommended optimization strategies

### AI Assistant Request: Troubleshooting Summary
**Context**: Common issues in Lesson 3
**Request**: "What are the most common AI perception issues students face in Lesson 3 and how to solve them?"
**Expected Output**: Compilation of common AI perception issues and their solutions