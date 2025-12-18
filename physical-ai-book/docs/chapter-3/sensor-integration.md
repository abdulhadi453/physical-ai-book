# Multimodal Sensor Integration in Isaac Sim

## Overview

This document provides comprehensive instructions for setting up multimodal sensor integration in the NVIDIA Isaac Sim environment for Module 3. The focus is on combining data from multiple sensors (camera, LiDAR, IMU, etc.) to create a comprehensive perception system for AI-robot applications.

## Introduction to Multimodal Sensor Integration

### What is Multimodal Sensor Integration?
Multimodal sensor integration combines data from multiple sensors to create a more complete and accurate understanding of the environment. In the context of Isaac Sim and ROS 2, this involves:

- Synchronizing data from different sensor types
- Fusing sensor data for enhanced perception
- Managing sensor calibration and transformations
- Implementing sensor fusion algorithms

### Key Sensor Types in Isaac Sim
1. **Camera**: Visual information for object detection and recognition
2. **LiDAR**: 3D spatial information for mapping and obstacle detection
3. **IMU**: Inertial measurements for orientation and motion
4. **Depth Camera**: Depth information for 3D scene understanding
5. **GPS**: Position information for outdoor navigation
6. **Encoders**: Wheel odometry for motion tracking

## Prerequisites

Before setting up multimodal sensor integration, ensure you have:
- Completed Isaac Sim and Isaac ROS setup
- Basic understanding of ROS 2 topics and messages
- Familiarity with sensor message types (sensor_msgs)
- TF (Transform) knowledge for coordinate frame management

## Isaac Sim Sensor Configuration

### 1. Creating Multimodal Robot Configuration

```bash
cat > ~/isaac_sim_shared/robots/multimodal_robot.usd << 'EOF'
#usda 1.0
(
    doc = """Multimodal sensor robot configuration for Isaac Sim"""
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "MultimodalRobot"
{
    def Xform "Chassis"
    {
        def Cube "Base"
        {
            float3 xformOp:translate = (0, 0, 0.1)
            float3 xformOp:scale = (0.5, 0.3, 0.2)
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
        }

        def Cylinder "FrontRightWheel"
        {
            float3 xformOp:translate = (0.15, -0.15, 0)
            float radius = 0.05
            float height = 0.04
            float3 xformOp:rotateXYZ = (90, 0, 0)
        }

        def Cylinder "BackLeftWheel"
        {
            float3 xformOp:translate = (-0.15, 0.15, 0)
            float radius = 0.05
            float height = 0.04
            float3 xformOp:rotateXYZ = (90, 0, 0)
        }

        def Cylinder "BackRightWheel"
        {
            float3 xformOp:translate = (-0.15, -0.15, 0)
            float radius = 0.05
            float height = 0.04
            float3 xformOp:rotateXYZ = (90, 0, 0)
        }
    }

    def Xform "Sensors"
    {
        # RGB Camera
        def Camera "RGB_Camera"
        {
            float3 xformOp:translate = (0.2, 0, 0.15)
            float3 clampingRange = (0.1, 100)
            float focalLength = 24
            float horizontalAperture = 36
            float verticalAperture = 24
        }

        # Depth Camera
        def Camera "Depth_Camera"
        {
            float3 xformOp:translate = (0.2, 0, 0.15)
            float3 clampingRange = (0.1, 10)
            float focalLength = 24
        }

        # 360-degree LiDAR
        def RotatingLidar "360Lidar"
        {
            float3 xformOp:translate = (0.18, 0, 0.19)
        }

        # IMU
        def Imu "IMU"
        {
            float3 xformOp:translate = (0, 0, 0.1)
        }

        # GPS (simulated)
        def UsdGeomGps "GPS"
        {
            float3 xformOp:translate = (0, 0, 0.2)
        }

        # Wheel encoders (simulated)
        def UsdGeomWheelEncoder "WheelEncoders"
        {
            float3 xformOp:translate = (0, 0, 0.05)
        }
    }

    def Xform "Links"
    {
        # Define coordinate frames for TF tree
        def Xform "base_link"
        {
            # Robot base coordinate frame
        }

        def Xform "camera_link"
        {
            float3 xformOp:translate = (0.2, 0, 0.15)
        }

        def Xform "lidar_link"
        {
            float3 xformOp:translate = (0.18, 0, 0.19)
        }

        def Xform "imu_link"
        {
            float3 xformOp:translate = (0, 0, 0.1)
        }

        def Xform "gps_link"
        {
            float3 xformOp:translate = (0, 0, 0.2)
        }

        def Xform "odom"
        {
            # Odometry frame
        }

        def Xform "map"
        {
            # Map frame
        }
    }
}
EOF
```

### 2. Creating Sensor Configuration File

```bash
cat > ~/isaac_sim_shared/configs/sensor_config.yaml << 'EOF'
# Multimodal Sensor Configuration for Isaac Sim

# Camera Configuration
camera:
  rgb:
    topic: "/camera/color/image_raw"
    info_topic: "/camera/color/camera_info"
    width: 640
    height: 480
    fps: 30
    fov: 60
    frame_id: "camera_link"
    enable_distortion: false
    color_correct: true
  depth:
    topic: "/camera/depth/image_rect_raw"
    info_topic: "/camera/depth/camera_info"
    width: 640
    height: 480
    fps: 30
    fov: 60
    frame_id: "camera_link"
    enable_noise: true
    noise_params:
      mean: 0.0
      stddev: 0.001

# LiDAR Configuration
lidar:
  topic: "/scan"
  samples: 360
  range_min: 0.1
  range_max: 25.0
  fov: 360
  fps: 10
  frame_id: "lidar_link"
  enable_noise: true
  noise_params:
    mean: 0.0
    stddev: 0.01

# IMU Configuration
imu:
  topic: "/imu"
  frame_id: "imu_link"
  rate: 100
  enable_noise: true
  noise_params:
    accelerometer_noise_density: 0.001
    gyroscope_noise_density: 0.0001

# GPS Configuration
gps:
  topic: "/gps/fix"
  frame_id: "gps_link"
  rate: 1
  enable_noise: true
  noise_params:
    horizontal_accuracy: 2.0
    vertical_accuracy: 4.0

# Wheel Encoder Configuration
wheel_encoders:
  left_topic: "/left_wheel/encoder"
  right_topic: "/right_wheel/encoder"
  frame_id: "base_link"
  rate: 50
  ticks_per_revolution: 1000
  wheel_radius: 0.05
  wheel_separation: 0.3

# TF Configuration
tf:
  publish_tf: true
  tf_prefix: ""
  odom_frame: "odom"
  base_frame: "base_link"
  map_frame: "map"
  publish_frequency: 50.0

# Sensor Synchronization
synchronization:
  enable_sync: true
  sync_method: "approximate_time"  # or "exact_time"
  sync_queue_size: 10
  max_interval_seconds: 0.1
EOF
```

## Sensor Data Synchronization

### 1. Creating Sensor Synchronization Node

```bash
cat > ~/isaac_ros_ws/src/isaac_sensor_fusion/sensor_fusion/sensor_synchronizer.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, NavSatFix
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

class SensorSynchronizer(Node):
    def __init__(self):
        super().__init__('sensor_synchronizer')

        # Initialize data storage
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None
        self.gps_data = None

        # Initialize TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscribers for each sensor
        self.camera_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.lidar_sub = Subscriber(self, LaserScan, '/scan')
        self.imu_sub = Subscriber(self, Imu, '/imu')
        self.gps_sub = Subscriber(self, NavSatFix, '/gps/fix')

        # Synchronize sensor data with approximate time synchronization
        self.sync = ApproximateTimeSynchronizer(
            [self.camera_sub, self.lidar_sub, self.imu_sub, self.gps_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Publisher for fused sensor data
        self.fused_publisher = self.create_publisher(
            Twist,  # Using Twist as a placeholder; in practice, you'd create a custom message
            '/fused_sensor_data',
            10
        )

        # Publisher for sensor status
        self.status_publisher = self.create_publisher(
            Twist,  # Using Twist as a placeholder
            '/sensor_status',
            10
        )

        # Sensor status tracking
        self.sensor_status = {
            'camera': False,
            'lidar': False,
            'imu': False,
            'gps': False
        }

        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Sensor Synchronizer initialized')

    def synchronized_callback(self, camera_msg, lidar_msg, imu_msg, gps_msg):
        """Callback for synchronized sensor data"""
        try:
            # Update sensor status
            self.sensor_status['camera'] = True
            self.sensor_status['lidar'] = True
            self.sensor_status['imu'] = True
            self.sensor_status['gps'] = True

            # Process synchronized data
            self.camera_data = camera_msg
            self.lidar_data = lidar_msg
            self.imu_data = imu_msg
            self.gps_data = gps_msg

            # Perform basic fusion (placeholder implementation)
            fused_data = self.perform_basic_fusion()

            # Publish fused data
            self.fused_publisher.publish(fused_data)

            self.get_logger().info(f'Synchronized data: Camera {camera_msg.header.stamp.sec}, '
                                 f'Lidar {lidar_msg.header.stamp.sec}, '
                                 f'IMU {imu_msg.header.stamp.sec}, '
                                 f'GPS {gps_msg.header.stamp.sec}')

        except Exception as e:
            self.get_logger().error(f'Error in synchronized callback: {e}')

    def perform_basic_fusion(self):
        """Perform basic sensor fusion"""
        # This is a placeholder implementation
        # In practice, you would implement more sophisticated fusion algorithms
        fused_msg = Twist()

        # Example: combine IMU orientation with GPS position for pose estimation
        if self.imu_data and self.gps_data:
            # Extract orientation from IMU
            imu_orientation = [
                self.imu_data.orientation.x,
                self.imu_data.orientation.y,
                self.imu_data.orientation.z,
                self.imu_data.orientation.w
            ]

            # Extract position from GPS
            gps_position = [
                self.gps_data.latitude,
                self.gps_data.longitude,
                self.gps_data.altitude
            ]

            # Combine in a meaningful way (simplified example)
            fused_msg.angular.x = imu_orientation[0]
            fused_msg.angular.y = imu_orientation[1]
            fused_msg.angular.z = imu_orientation[2]
            fused_msg.linear.x = gps_position[0]
            fused_msg.linear.y = gps_position[1]
            fused_msg.linear.z = gps_position[2]

        return fused_msg

    def publish_status(self):
        """Publish sensor status"""
        status_msg = Twist()
        status_msg.linear.x = 1.0 if self.sensor_status['camera'] else 0.0
        status_msg.linear.y = 1.0 if self.sensor_status['lidar'] else 0.0
        status_msg.linear.z = 1.0 if self.sensor_status['imu'] else 0.0
        status_msg.angular.x = 1.0 if self.sensor_status['gps'] else 0.0

        self.status_publisher.publish(status_msg)

        # Reset status for next cycle
        self.sensor_status = {k: False for k in self.sensor_status.keys()}


def main(args=None):
    rclpy.init(args=args)
    synchronizer = SensorSynchronizer()

    try:
        rclpy.spin(synchronizer)
    except KeyboardInterrupt:
        pass
    finally:
        synchronizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Sensor Fusion Implementation

### 1. Creating Sensor Fusion Package

```bash
cd ~/isaac_ros_ws/src
source /opt/ros/humble/setup.bash

# Create sensor fusion package
ros2 pkg create --build-type ament_python isaac_sensor_fusion --dependencies rclpy sensor_msgs geometry_msgs message_filters tf2_ros tf2_geometry_msgs std_msgs
```

### 2. Creating Advanced Sensor Fusion Node

```bash
cat > ~/isaac_ros_ws/src/isaac_sensor_fusion/sensor_fusion/advanced_sensor_fusion.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
import threading
from typing import Dict, List, Tuple

class AdvancedSensorFusion(Node):
    def __init__(self):
        super().__init__('advanced_sensor_fusion')

        # Initialize data buffers
        self.camera_buffer = deque(maxlen=10)
        self.lidar_buffer = deque(maxlen=10)
        self.imu_buffer = deque(maxlen=50)  # Higher frequency
        self.gps_buffer = deque(maxlen=10)

        # Initialize TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscribers
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        # Publishers
        self.fused_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/fused_pose',
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/fused_odom',
            10
        )

        # Initialize state estimation
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = R.from_quat([0, 0, 0, 1])  # Identity quaternion
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # Covariance matrices (placeholder values)
        self.position_covariance = np.eye(3) * 0.1
        self.orientation_covariance = np.eye(3) * 0.05

        # Update timer
        self.update_timer = self.create_timer(0.05, self.update_fusion)  # 20 Hz

        self.get_logger().info('Advanced Sensor Fusion node initialized')

    def camera_callback(self, msg):
        """Handle camera data"""
        self.camera_buffer.append(msg)

    def lidar_callback(self, msg):
        """Handle LiDAR data"""
        self.lidar_buffer.append(msg)

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_buffer.append(msg)

    def gps_callback(self, msg):
        """Handle GPS data"""
        self.gps_buffer.append(msg)

    def update_fusion(self):
        """Update fused state estimate"""
        try:
            # Get latest sensor data
            latest_imu = self.imu_buffer[-1] if self.imu_buffer else None
            latest_gps = self.gps_buffer[-1] if self.gps_buffer else None

            if latest_imu:
                # Update orientation from IMU
                imu_quat = np.array([
                    latest_imu.orientation.x,
                    latest_imu.orientation.y,
                    latest_imu.orientation.z,
                    latest_imu.orientation.w
                ])

                # Normalize quaternion
                imu_quat = imu_quat / np.linalg.norm(imu_quat)

                # Update orientation (simple approach, in practice use more sophisticated fusion)
                self.orientation = R.from_quat(imu_quat)

            if latest_gps:
                # Convert GPS to local coordinates (simplified)
                # In practice, you'd use a proper coordinate transformation
                lat_diff = latest_gps.latitude - 0.0  # Reference latitude
                lon_diff = latest_gps.longitude - 0.0  # Reference longitude

                # Convert to meters (approximate)
                lat_to_meters = 111320.0  # meters per degree at equator
                x = lon_diff * lat_to_meters * np.cos(np.radians(latest_gps.latitude))
                y = lat_diff * lat_to_meters
                z = latest_gps.altitude

                # Update position
                self.position = np.array([x, y, z])

            # Create and publish fused pose
            self.publish_fused_pose()

        except Exception as e:
            self.get_logger().error(f'Error in fusion update: {e}')

    def publish_fused_pose(self):
        """Publish the fused pose estimate"""
        try:
            # Create PoseWithCovarianceStamped message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"

            # Set position
            pose_msg.pose.pose.position.x = float(self.position[0])
            pose_msg.pose.pose.position.y = float(self.position[1])
            pose_msg.pose.pose.position.z = float(self.position[2])

            # Set orientation
            quat = self.orientation.as_quat()
            pose_msg.pose.pose.orientation.x = float(quat[0])
            pose_msg.pose.pose.orientation.y = float(quat[1])
            pose_msg.pose.pose.orientation.z = float(quat[2])
            pose_msg.pose.pose.orientation.w = float(quat[3])

            # Set covariance
            covariance = np.zeros(36)
            covariance[0] = self.position_covariance[0, 0]  # x
            covariance[7] = self.position_covariance[1, 1]  # y
            covariance[14] = self.position_covariance[2, 2]  # z
            covariance[21] = self.orientation_covariance[0, 0]  # rx
            covariance[28] = self.orientation_covariance[1, 1]  # ry
            covariance[35] = self.orientation_covariance[2, 2]  # rz

            pose_msg.pose.covariance = covariance.tolist()

            self.fused_pose_publisher.publish(pose_msg)

            # Also publish as odometry
            odom_msg = Odometry()
            odom_msg.header = pose_msg.header
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose = pose_msg.pose

            self.odom_publisher.publish(odom_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing fused pose: {e}')

    def get_fusion_status(self) -> Dict:
        """Get status of sensor fusion"""
        return {
            'camera_buffer_size': len(self.camera_buffer),
            'lidar_buffer_size': len(self.lidar_buffer),
            'imu_buffer_size': len(self.imu_buffer),
            'gps_buffer_size': len(self.gps_buffer),
            'position': self.position.tolist(),
            'orientation': self.orientation.as_quat().tolist()
        }


def main(args=None):
    rclpy.init(args=args)
    fusion_node = AdvancedSensorFusion()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        status = fusion_node.get_fusion_status()
        print(f"Final fusion status: {status}")
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## TF (Transform) Configuration

### 1. Creating Static Transform Publisher

```bash
cat > ~/isaac_ros_ws/src/isaac_sensor_fusion/sensor_fusion/static_transform_publisher.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        # Create static transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms
        self.publish_static_transforms()

        self.get_logger().info('Static Transform Publisher initialized')

    def publish_static_transforms(self):
        """Publish static transforms between sensor frames"""
        transforms = []

        # Transform from base_link to camera_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.15
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # Transform from base_link to lidar_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = 0.18
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.19
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # Transform from base_link to imu_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # Transform from base_link to gps_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'gps_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # Send all transforms
        for transform in transforms:
            self.tf_static_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    publisher = StaticTransformPublisher()

    try:
        # Keep the node alive
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Sensor Integration Launch Files

### 1. Creating Launch Directory and Files

```bash
mkdir -p ~/isaac_ros_ws/src/isaac_sensor_fusion/launch
```

### 2. Creating Sensor Integration Launch File

```bash
cat > ~/isaac_ros_ws/src/isaac_sensor_fusion/launch/sensor_integration.launch.py << 'EOF
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Static transform publisher node
    static_transform_publisher = Node(
        package='isaac_sensor_fusion',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Sensor synchronizer node
    sensor_synchronizer = Node(
        package='isaac_sensor_fusion',
        executable='sensor_synchronizer',
        name='sensor_synchronizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Advanced sensor fusion node
    advanced_fusion = Node(
        package='isaac_sensor_fusion',
        executable='advanced_sensor_fusion',
        name='advanced_sensor_fusion',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'))

    # Add nodes
    ld.add_action(static_transform_publisher)
    ld.add_action(TimerAction(
        period=2.0,
        actions=[sensor_synchronizer]
    ))
    ld.add_action(TimerAction(
        period=3.0,
        actions=[advanced_fusion]
    ))

    return ld
EOF
```

## Sensor Calibration and Validation

### 1. Creating Sensor Calibration Tools

```bash
cat > ~/isaac_sim_shared/scripts/sensor_calibration.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import yaml

class SensorCalibrator(Node):
    def __init__(self):
        super().__init__('sensor_calibrator')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to sensors
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Data storage for calibration
        self.camera_info = None
        self.calibration_data = []
        self.calibration_samples = 0
        self.max_samples = 100

        self.get_logger().info('Sensor Calibrator initialized')

    def camera_info_callback(self, msg):
        """Store camera info for calibration"""
        self.camera_info = msg

    def camera_callback(self, msg):
        """Process camera data for calibration"""
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # For calibration, we typically look for calibration patterns
            # In this example, we'll look for checkerboard patterns
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Look for checkerboard pattern (9x6 corners)
            ret, corners = cv2.findChessboardCorners(
                gray, (9, 6),
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            if ret and self.calibration_samples < self.max_samples:
                # Refine corner positions
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )

                # Store calibration data
                self.calibration_data.append({
                    'image': cv_image,
                    'corners': corners_refined,
                    'timestamp': msg.header.stamp
                })

                self.calibration_samples += 1
                self.get_logger().info(f'Calibration sample {self.calibration_samples}/{self.max_samples}')

                if self.calibration_samples >= self.max_samples:
                    self.perform_calibration()

        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        # Store LiDAR data for fusion validation
        pass

    def imu_callback(self, msg):
        """Process IMU data"""
        # Store IMU data for fusion validation
        pass

    def perform_calibration(self):
        """Perform camera calibration"""
        if len(self.calibration_data) < 10:
            self.get_logger().warn('Not enough calibration samples collected')
            return

        # Prepare object points (3D points in real world space)
        objp = np.zeros((9*6, 3), np.float32)
        objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

        # Arrays to store object points and image points from all images
        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane

        for data in self.calibration_data:
            # Only use data where corners were detected
            if data['corners'] is not None:
                objpoints.append(objp)
                imgpoints.append(data['corners'])

        if len(objpoints) < 10:
            self.get_logger().warn('Not enough valid calibration samples')
            return

        # Perform calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints,
            (self.camera_info.width, self.camera_info.height),
            None, None
        )

        if ret:
            # Save calibration results
            calibration_data = {
                'camera_matrix': mtx.tolist(),
                'distortion_coefficients': dist.tolist(),
                'image_width': self.camera_info.width,
                'image_height': self.camera_info.height,
                'samples_used': len(objpoints)
            }

            # Save to file
            with open('/workspace/shared_dir/calibration/camera_calibration.yaml', 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)

            self.get_logger().info(f'Camera calibration completed. Reprojection error: {ret}')
            self.get_logger().info(f'Calibration saved to /workspace/shared_dir/calibration/camera_calibration.yaml')
        else:
            self.get_logger().error('Camera calibration failed')

    def validate_sensor_fusion(self):
        """Validate sensor fusion quality"""
        # This would implement validation of sensor fusion accuracy
        # by comparing fused estimates to ground truth in simulation
        pass


def main(args=None):
    rclpy.init(args=args)
    calibrator = SensorCalibrator()

    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Sensor Data Visualization

### 1. Creating Sensor Data Visualization Script

```bash
cat > ~/isaac_sim_shared/scripts/sensor_visualization.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, PointCloud2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Data storage for visualization
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None
        self.fused_pose = None

        # Subscribe to sensors
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/fused_pose',
            self.pose_callback,
            10
        )

        # Visualization setup
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 10))
        self.lidar_line, = self.axs[0, 0].plot([], [], 'b.')
        self.axs[0, 0].set_title('LiDAR Scan')
        self.axs[0, 0].set_xlim(-10, 10)
        self.axs[0, 0].set_ylim(-10, 10)
        self.axs[0, 0].grid(True)

        self.imu_ax = self.axs[0, 1]
        self.imu_ax.set_title('IMU Data')
        self.imu_ax.set_ylim(-10, 10)

        self.pose_ax = self.axs[1, 0]
        self.pose_ax.set_title('Robot Pose')
        self.pose_ax.set_xlim(-5, 5)
        self.pose_ax.set_ylim(-5, 5)
        self.pose_ax.grid(True)

        self.camera_ax = self.axs[1, 1]
        self.camera_ax.set_title('Camera Image')

        # Start visualization thread
        self.viz_thread = threading.Thread(target=self.run_visualization)
        self.viz_thread.daemon = True
        self.viz_thread.start()

        self.get_logger().info('Sensor Visualizer initialized')

    def camera_callback(self, msg):
        """Store camera data"""
        try:
            self.camera_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting camera image: {e}')

    def lidar_callback(self, msg):
        """Store LiDAR data"""
        try:
            # Convert LiDAR scan to x,y coordinates
            angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
            ranges = np.array(msg.ranges)

            # Filter out invalid ranges
            valid_indices = (ranges >= msg.range_min) & (ranges <= msg.range_max)
            valid_angles = angles[valid_indices]
            valid_ranges = ranges[valid_indices]

            # Convert to Cartesian coordinates
            x = valid_ranges * np.cos(valid_angles)
            y = valid_ranges * np.sin(valid_angles)

            self.lidar_data = (x, y)
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')

    def imu_callback(self, msg):
        """Store IMU data"""
        try:
            self.imu_data = {
                'linear_acceleration': [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ],
                'angular_velocity': [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ],
                'orientation': [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ]
            }
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')

    def pose_callback(self, msg):
        """Store fused pose data"""
        try:
            self.fused_pose = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'qx': msg.pose.orientation.x,
                'qy': msg.pose.orientation.y,
                'qz': msg.pose.orientation.z,
                'qw': msg.pose.orientation.w
            }
        except Exception as e:
            self.get_logger().error(f'Error processing pose data: {e}')

    def update_visualization(self, frame):
        """Update visualization plots"""
        # Update LiDAR plot
        if self.lidar_data:
            x, y = self.lidar_data
            self.lidar_line.set_data(x, y)

        # Update IMU plot
        if self.imu_data:
            acc = self.imu_data['linear_acceleration']
            self.imu_ax.clear()
            self.imu_ax.plot(acc, 'b-', label='Linear Acc')
            self.imu_ax.set_title('IMU Data')
            self.imu_ax.legend()
            self.imu_ax.grid(True)

        # Update pose plot
        if self.fused_pose:
            x = self.fused_pose['x']
            y = self.fused_pose['y']
            self.pose_ax.plot(x, y, 'ro', markersize=10)
            self.pose_ax.set_title(f'Robot Pose: ({x:.2f}, {y:.2f})')

        # Update camera plot
        if self.camera_data is not None:
            self.camera_ax.clear()
            self.camera_ax.imshow(self.camera_data)
            self.camera_ax.set_title('Camera Image')
            self.camera_ax.axis('off')

        return self.lidar_line,

    def run_visualization(self):
        """Run the visualization in a separate thread"""
        try:
            ani = FuncAnimation(self.fig, self.update_visualization, interval=100, blit=False)
            plt.tight_layout()
            plt.show()
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {e}')

def main(args=None):
    rclpy.init(args=args)
    visualizer = SensorVisualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        plt.close('all')
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Testing and Validation

### 1. Creating Sensor Integration Test Script

```bash
cat > ~/test_sensor_integration.sh << 'EOF'
#!/bin/bash

# Test script for multimodal sensor integration

echo "Testing Multimodal Sensor Integration..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if sensor fusion package is available
echo "Checking sensor fusion package..."
ros2 pkg list | grep sensor_fusion

if [ $? -eq 0 ]; then
    echo "✓ Sensor fusion package found"
else
    echo "✗ Sensor fusion package not found"
    exit 1
fi

# Check for sensor configuration files
if [ -f "/workspace/shared_dir/configs/sensor_config.yaml" ]; then
    echo "✓ Sensor configuration file found"
else
    echo "✗ Sensor configuration file not found"
    exit 1
fi

# Check for multimodal robot configuration
if [ -f "/workspace/shared_dir/robots/multimodal_robot.usd" ]; then
    echo "✓ Multimodal robot configuration found"
else
    echo "✗ Multimodal robot configuration not found"
    exit 1
fi

# Build the sensor fusion package
echo "Building sensor fusion package..."
cd ~/isaac_ros_ws
colcon build --packages-select isaac_sensor_fusion
source install/setup.bash

# Check if launch files exist
if [ -f "/workspace/shared_dir/src/isaac_sensor_fusion/launch/sensor_integration.launch.py" ]; then
    echo "✓ Sensor integration launch file found"
else
    echo "✗ Sensor integration launch file not found"
    exit 1
fi

# Test sensor topics are available (when Isaac Sim is running)
echo "Sensor integration test completed."
echo "To fully test, run Isaac Sim and launch the sensor integration system:"
echo "ros2 launch isaac_sensor_fusion sensor_integration.launch.py"
EOF

# Make executable
chmod +x ~/test_sensor_integration.sh
```

### 2. Running Sensor Integration Test

```bash
~/test_sensor_integration.sh
```

## Troubleshooting Sensor Integration

### Common Issues and Solutions

#### Issue: "Sensor data not synchronizing properly"
**Solution**: Check timing and buffer sizes
```bash
# Verify sensor topics are publishing
ros2 topic echo /camera/color/image_raw --field header.stamp --field height --field width &
ros2 topic echo /scan --field header.stamp --field ranges.size &
ros2 topic echo /imu --field header.stamp --field orientation
```

#### Issue: "TF transforms not available"
**Solution**: Verify static transform publisher and frame names
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Echo transforms
ros2 run tf2_ros tf2_echo base_link camera_link
```

#### Issue: "High CPU/GPU usage with multiple sensors"
**Solution**: Optimize sensor frequencies and processing
```bash
# Check current sensor frequencies
ros2 topic hz /camera/color/image_raw
ros2 topic hz /scan
ros2 topic hz /imu
```

## Verification Checklist

- [ ] Multimodal robot configuration created
- [ ] Sensor configuration files created
- [ ] Sensor synchronization implemented
- [ ] Advanced sensor fusion node created
- [ ] TF configuration and static transforms set up
- [ ] Launch files created for sensor integration
- [ ] Calibration tools implemented
- [ ] Visualization tools created
- [ ] Test script created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After implementing multimodal sensor integration:

1. **Test sensor fusion** with Isaac Sim
2. **Validate sensor data** synchronization and accuracy
3. **Optimize performance** based on monitoring results
4. **Create sensor integration exercises** for students

The multimodal sensor integration framework is now configured and ready for Module 3, providing students with tools to combine data from multiple sensors for enhanced AI-robot perception.