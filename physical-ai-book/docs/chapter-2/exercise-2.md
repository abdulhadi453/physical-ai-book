---
sidebar_position: 15
---

# Exercise 2: Sensor Integration

## Overview
This exercise focuses on integrating multiple sensors (LiDAR, depth camera, and IMU) in a digital twin environment to create a comprehensive perception system. You will learn how to combine data from different sensors to improve the robot's understanding of its environment and motion state.

## Learning Objectives
By the end of this exercise, you will be able to:
- Integrate LiDAR, depth camera, and IMU data streams
- Implement basic sensor fusion algorithms
- Validate sensor integration performance
- Apply fused sensor data for navigation tasks
- Analyze the benefits of multi-sensor integration

## Prerequisites
- Completion of Exercise 1: Basic Environment Creation
- Understanding of individual sensor simulation (LiDAR, depth camera, IMU)
- Basic knowledge of coordinate systems and transformations
- Familiarity with the simulation environment (Gazebo or Unity)

## Exercise Setup

### Environment Configuration
1. Create a test environment with:
   - A robot equipped with LiDAR, depth camera, and IMU
   - Multiple obstacles of different shapes and sizes
   - Varying lighting conditions (for depth camera testing)
   - Inclined surfaces (for IMU testing)

### Robot Configuration
Configure your robot with the following sensors:
- **LiDAR**: 360° horizontal field of view, 16 vertical beams, 30m range
- **Depth Camera**: 640x480 resolution, 60° field of view, 0.1-10m range
- **IMU**: 100Hz sampling rate, realistic noise and bias parameters

## Exercise Steps

### Step 1: Individual Sensor Validation
1. Test each sensor independently in the environment
2. Verify that LiDAR detects obstacles correctly
3. Confirm that depth camera provides accurate distance measurements
4. Validate that IMU provides reasonable orientation and acceleration data

### Step 2: Coordinate System Alignment
1. Establish a common coordinate system for all sensors
2. Define transformations between sensor frames and robot base frame
3. Implement coordinate transformation functions
4. Test transformations with known geometric shapes

### Step 3: Data Synchronization
1. Implement timestamp synchronization between sensors
2. Handle different sampling rates (LiDAR: 10Hz, Camera: 30Hz, IMU: 100Hz)
3. Implement interpolation for sensor data alignment
4. Validate temporal synchronization accuracy

### Step 4: Basic Sensor Fusion
1. **Environment Mapping**: Combine LiDAR and depth camera data to create a comprehensive 3D map
2. **Motion Compensation**: Use IMU data to correct for robot motion when processing LiDAR/depth data
3. **Data Validation**: Implement cross-validation between sensors

### Step 5: Advanced Fusion Implementation
1. **Kalman Filter**: Implement a simple Kalman filter to combine position estimates
2. **Particle Filter**: Implement a particle filter for robust tracking
3. **Multi-Sensor SLAM**: Basic implementation of simultaneous localization and mapping

## Implementation Tasks

### Task 1: Sensor Data Structure
Create a unified data structure to hold synchronized sensor data:

```python
import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class SensorData:
    """Unified sensor data structure"""
    timestamp: float
    lidar_points: Optional[np.ndarray]  # Shape: (N, 3) - x, y, z points
    depth_image: Optional[np.ndarray]   # Shape: (H, W) - depth values
    imu_data: Optional[dict]           # Keys: 'gyro', 'accel', 'orientation'
    camera_pose: Optional[np.ndarray]   # 4x4 transformation matrix

class SensorFusion:
    def __init__(self):
        self.sensor_buffer = []  # Buffer for synchronization
        self.max_buffer_time = 0.1  # 100ms max delay

    def add_lidar_data(self, timestamp: float, points: np.ndarray):
        """Add LiDAR data to buffer"""
        sensor_data = SensorData(
            timestamp=timestamp,
            lidar_points=points,
            depth_image=None,
            imu_data=None,
            camera_pose=None
        )
        self._add_to_buffer(sensor_data)

    def add_depth_data(self, timestamp: float, depth_image: np.ndarray):
        """Add depth camera data to buffer"""
        sensor_data = SensorData(
            timestamp=timestamp,
            lidar_points=None,
            depth_image=depth_image,
            imu_data=None,
            camera_pose=None
        )
        self._add_to_buffer(sensor_data)

    def add_imu_data(self, timestamp: float, gyro: np.ndarray,
                     accel: np.ndarray, orientation: np.ndarray):
        """Add IMU data to buffer"""
        imu_dict = {
            'gyro': gyro,
            'accel': accel,
            'orientation': orientation
        }
        sensor_data = SensorData(
            timestamp=timestamp,
            lidar_points=None,
            depth_image=None,
            imu_data=imu_dict,
            camera_pose=None
        )
        self._add_to_buffer(sensor_data)

    def _add_to_buffer(self, sensor_data: SensorData):
        """Add data to buffer and maintain synchronization"""
        self.sensor_buffer.append(sensor_data)

        # Remove old data beyond max buffer time
        current_time = sensor_data.timestamp
        self.sensor_buffer = [
            data for data in self.sensor_buffer
            if current_time - data.timestamp <= self.max_buffer_time
        ]

    def get_synchronized_data(self) -> Optional[SensorData]:
        """Get synchronized sensor data if available"""
        if len(self.sensor_buffer) < 3:  # Need all three sensor types
            return None

        # Find the most recent common timestamp
        lidar_data = [d for d in self.sensor_buffer if d.lidar_points is not None]
        depth_data = [d for d in self.sensor_buffer if d.depth_image is not None]
        imu_data = [d for d in self.sensor_buffer if d.imu_data is not None]

        if not (lidar_data and depth_data and imu_data):
            return None

        # Get latest of each type
        latest_lidar = max(lidar_data, key=lambda x: x.timestamp)
        latest_depth = max(depth_data, key=lambda x: x.timestamp)
        latest_imu = max(imu_data, key=lambda x: x.timestamp)

        # Check if they're close enough in time
        max_time_diff = 0.05  # 50ms tolerance
        if (abs(latest_lidar.timestamp - latest_depth.timestamp) > max_time_diff or
            abs(latest_lidar.timestamp - latest_imu.timestamp) > max_time_diff or
            abs(latest_depth.timestamp - latest_imu.timestamp) > max_time_diff):
            return None

        # Create combined data structure
        combined_data = SensorData(
            timestamp=max(latest_lidar.timestamp, latest_depth.timestamp, latest_imu.timestamp),
            lidar_points=latest_lidar.lidar_points,
            depth_image=latest_depth.depth_image,
            imu_data=latest_imu.imu_data,
            camera_pose=None  # To be computed
        )

        return combined_data
```

### Task 2: Coordinate Transformation
Implement coordinate transformations between sensor frames:

```python
import numpy as np

class CoordinateTransformer:
    def __init__(self):
        # Define static transforms between sensors and robot base
        self.transforms = {
            'lidar_to_base': self._create_transform(
                position=[0.1, 0.0, 0.3],  # 10cm forward, 30cm up
                rotation=[0, 0, 0]  # No rotation
            ),
            'camera_to_base': self._create_transform(
                position=[0.1, 0.0, 0.2],  # 10cm forward, 20cm up
                rotation=[0, 0, 0]  # No rotation
            ),
            'imu_to_base': self._create_transform(
                position=[0.0, 0.0, 0.1],  # At robot center, 10cm up
                rotation=[0, 0, 0]  # No rotation
            )
        }

    def _create_transform(self, position, rotation):
        """Create a 4x4 transformation matrix"""
        x, y, z = position
        rx, ry, rz = rotation

        # Simplified for this example (no rotation)
        transform = np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

        return transform

    def transform_point(self, point, transform_matrix):
        """Transform a 3D point using a 4x4 transformation matrix"""
        # Convert to homogeneous coordinates
        homogeneous_point = np.array([point[0], point[1], point[2], 1])

        # Apply transformation
        transformed_point = transform_matrix @ homogeneous_point

        # Convert back to 3D
        return transformed_point[:3]

    def transform_lidar_to_base(self, lidar_points):
        """Transform LiDAR points to robot base frame"""
        transform = self.transforms['lidar_to_base']
        transformed_points = []

        for point in lidar_points:
            transformed_point = self.transform_point(point, transform)
            transformed_points.append(transformed_point)

        return np.array(transformed_points)

    def transform_camera_to_base(self, depth_image, camera_intrinsics):
        """Transform depth image to 3D points in base frame"""
        h, w = depth_image.shape
        points_3d = []

        for v in range(h):
            for u in range(w):
                depth = depth_image[v, u]
                if depth > 0:  # Valid depth
                    # Convert pixel coordinates to 3D camera coordinates
                    x = (u - camera_intrinsics['cx']) * depth / camera_intrinsics['fx']
                    y = (v - camera_intrinsics['cy']) * depth / camera_intrinsics['fy']
                    z = depth

                    point_camera = np.array([x, y, z])

                    # Transform to base frame
                    transform = self.transforms['camera_to_base']
                    point_base = self.transform_point(point_camera, transform)
                    points_3d.append(point_base)

        return np.array(points_3d)
```

### Task 3: Basic Fusion Algorithm
Implement a basic fusion algorithm that combines sensor data:

```python
class BasicFusion:
    def __init__(self):
        self.transformer = CoordinateTransformer()
        self.map = {}  # Simple occupancy map
        self.robot_pose = np.array([0, 0, 0, 0, 0, 0])  # x, y, z, roll, pitch, yaw

    def update_map(self, lidar_points, depth_points, imu_data):
        """Update occupancy map with fused sensor data"""
        # Transform points to common coordinate system
        lidar_base = self.transformer.transform_lidar_to_base(lidar_points)
        depth_base = self.transformer.transform_camera_to_base(depth_points,
                                                              {'fx': 525, 'fy': 525, 'cx': 320, 'cy': 240})

        # Combine all points
        all_points = np.vstack([lidar_base, depth_base])

        # Update occupancy map
        for point in all_points:
            # Simple grid-based mapping
            grid_x = int(point[0] / 0.1)  # 10cm resolution
            grid_y = int(point[1] / 0.1)

            key = (grid_x, grid_y)
            if key not in self.map:
                self.map[key] = 0
            self.map[key] += 1  # Increment occupancy count

    def estimate_robot_motion(self, imu_data, dt):
        """Estimate robot motion using IMU data"""
        gyro = imu_data['gyro']
        accel = imu_data['accel']

        # Simple integration for motion estimation
        # In practice, use more sophisticated methods
        self.robot_pose[3:6] += gyro * dt  # Update orientation
        self.robot_pose[0:3] += accel * dt * dt * 0.5  # Update position (simplified)

    def get_environment_map(self):
        """Return the current environment map"""
        return self.map

    def get_robot_pose(self):
        """Return current robot pose estimate"""
        return self.robot_pose
```

### Task 4: Validation and Testing
Implement validation functions to test sensor fusion:

```python
def validate_sensor_fusion(fusion_system, ground_truth):
    """Validate sensor fusion performance"""
    # Get current map and pose estimates
    estimated_map = fusion_system.get_environment_map()
    estimated_pose = fusion_system.get_robot_pose()

    # Compare with ground truth
    map_accuracy = calculate_map_accuracy(estimated_map, ground_truth['map'])
    pose_accuracy = calculate_pose_accuracy(estimated_pose, ground_truth['pose'])

    return {
        'map_accuracy': map_accuracy,
        'pose_accuracy': pose_accuracy,
        'overall_performance': (map_accuracy + pose_accuracy) / 2
    }

def calculate_map_accuracy(estimated_map, ground_truth_map):
    """Calculate accuracy of occupancy map"""
    # Simple accuracy calculation
    correct_cells = 0
    total_cells = 0

    for key in ground_truth_map:
        if key in estimated_map:
            # Check if occupancy classification is correct
            est_occupied = estimated_map[key] > 0
            gt_occupied = ground_truth_map[key] > 0
            if est_occupied == gt_occupied:
                correct_cells += 1
        total_cells += 1

    return correct_cells / total_cells if total_cells > 0 else 0

def calculate_pose_accuracy(estimated_pose, ground_truth_pose):
    """Calculate accuracy of pose estimation"""
    position_error = np.linalg.norm(estimated_pose[:3] - ground_truth_pose[:3])
    orientation_error = np.linalg.norm(estimated_pose[3:] - ground_truth_pose[3:])

    # Normalize to 0-1 scale
    position_accuracy = max(0, 1 - position_error / 1.0)  # 1m tolerance
    orientation_accuracy = max(0, 1 - orientation_error / 0.1)  # 0.1rad tolerance

    return (position_accuracy + orientation_accuracy) / 2
```

## Expected Outcomes

### Technical Outcomes
- Successfully synchronize data from multiple sensors
- Implement coordinate transformations between sensor frames
- Create a unified representation of the environment
- Demonstrate improved accuracy through sensor fusion
- Validate the system with quantitative metrics

### Learning Outcomes
- Understand the importance of sensor fusion in robotics
- Learn practical techniques for handling different sensor data rates
- Gain experience with coordinate system management
- Develop skills in data validation and quality assessment

## Assessment Questions

1. **Analysis**: How does combining LiDAR and depth camera data improve environment mapping compared to using each sensor individually?

2. **Problem-Solving**: What challenges arise when synchronizing sensors with different sampling rates, and how did you address them?

3. **Application**: In what scenarios would IMU data be most valuable for improving the accuracy of other sensors?

4. **Evaluation**: How did sensor fusion impact the accuracy of your environment map and robot pose estimation?

## Advanced Challenges

1. **Robust Fusion**: Implement a weighted fusion algorithm that considers sensor reliability based on environmental conditions

2. **Dynamic Objects**: Extend your fusion algorithm to detect and track moving objects in the environment

3. **Multi-Robot Fusion**: Design a system that fuses sensor data from multiple robots exploring the same environment

4. **Real-time Performance**: Optimize your fusion algorithm for real-time performance with minimal computational overhead

## Resources
- [ROS Sensor Fusion Tutorials](http://wiki.ros.org/sensor_fusion)
- [Kalman Filter Implementation Guide](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
- [Multi-Sensor Integration in Robotics](https://arxiv.org/abs/2001.07449)

## Next Steps
After completing this exercise, proceed to Exercise 3: Advanced Navigation to apply your sensor fusion skills in a complete navigation task.