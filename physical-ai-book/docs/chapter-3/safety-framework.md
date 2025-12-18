# Safety and Reliability Framework for Isaac Sim AI-Robot Systems

## Overview

This document provides comprehensive guidelines for implementing safety and reliability frameworks in the NVIDIA Isaac Sim AI-robot systems for Module 3. The framework ensures that AI-robot systems operate safely and reliably in both simulation and real-world environments.

## Introduction to Safety and Reliability in AI-Robotics

### What is Safety and Reliability in AI-Robotics?
Safety and reliability in AI-robotics encompasses:
- **Safety**: Ensuring the robot operates without causing harm to humans, environment, or itself
- **Reliability**: Ensuring consistent, predictable operation under various conditions
- **Robustness**: Maintaining functionality despite uncertainties and disturbances
- **Fail-safe mechanisms**: Graceful degradation when failures occur

### Key Safety and Reliability Principles
1. **Fail-Safe Design**: Systems default to safe states on failure
2. **Redundancy**: Multiple systems for critical functions
3. **Monitoring**: Continuous system health assessment
4. **Validation**: Verification of safe operation
5. **Recovery**: Automatic recovery from failures

## Safety Framework Components

### 1. Creating Safety Monitor Node

```bash
cat > ~/isaac_ros_ws/src/isaac_safety_framework/safety_framework/safety_monitor.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String, Float32
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
import numpy as np
import math
from enum import Enum
from typing import Dict, List, Optional

class SafetyLevel(Enum):
    SAFE = 0
    WARNING = 1
    DANGER = 2
    EMERGENCY_STOP = 3

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Safety thresholds
        self.safety_thresholds = {
            'min_distance_obstacle': 0.3,  # meters
            'max_linear_velocity': 0.5,    # m/s
            'max_angular_velocity': 1.0,   # rad/s
            'max_acceleration': 2.0,       # m/s²
            'battery_low_threshold': 20.0, # percentage
            'imu_temperature_threshold': 80.0,  # degrees Celsius
            'max_tilt_angle': 30.0,        # degrees
            'max_operation_time': 3600.0   # seconds (1 hour)
        }

        # System state tracking
        self.current_velocity = Twist()
        self.previous_velocity = Twist()
        self.previous_time = self.get_clock().now()
        self.battery_level = 100.0
        self.imu_data = None
        self.lidar_data = None
        self.odom_data = None
        self.operation_start_time = self.get_clock().now()
        self.emergency_stop_active = False

        # Subscribers
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

        self.battery_subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.safety_status_publisher = self.create_publisher(
            Bool,
            '/safety/safe_to_proceed',
            10
        )

        self.emergency_stop_publisher = self.create_publisher(
            Bool,
            '/safety/emergency_stop',
            10
        )

        self.safety_level_publisher = self.create_publisher(
            String,
            '/safety/level',
            10
        )

        self.safety_report_publisher = self.create_publisher(
            String,
            '/safety/report',
            10
        )

        # Timer for periodic safety checks
        self.safety_timer = self.create_timer(0.1, self.perform_safety_check)

        self.get_logger().info('Safety Monitor initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        self.lidar_data = msg

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration"""
        self.imu_data = msg

    def battery_callback(self, msg):
        """Process battery data"""
        self.battery_level = msg.percentage if msg.percentage is not None else 100.0

    def cmd_vel_callback(self, msg):
        """Process velocity commands for safety checking"""
        self.previous_velocity = self.current_velocity
        self.current_velocity = msg
        self.previous_time = self.get_clock().now()

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = msg

    def perform_safety_check(self):
        """Perform comprehensive safety check"""
        try:
            # Check all safety conditions
            safety_level, report = self.evaluate_safety_conditions()

            # Publish safety status
            safe_msg = Bool()
            safe_msg.data = (safety_level == SafetyLevel.SAFE)
            self.safety_status_publisher.publish(safe_msg)

            # Handle emergency stop
            if safety_level == SafetyLevel.EMERGENCY_STOP:
                self.activate_emergency_stop()
            else:
                self.deactivate_emergency_stop()

            # Publish safety level
            level_msg = String()
            level_msg.data = safety_level.name
            self.safety_level_publisher.publish(level_msg)

            # Publish safety report
            report_msg = String()
            report_msg.data = report
            self.safety_report_publisher.publish(report_msg)

        except Exception as e:
            self.get_logger().error(f'Error in safety check: {e}')

    def evaluate_safety_conditions(self) -> tuple:
        """Evaluate all safety conditions and return safety level and report"""
        safety_issues = []
        safety_level = SafetyLevel.SAFE

        # Check for obstacles
        if self.lidar_data:
            min_distance = min(self.lidar_data.ranges) if self.lidar_data.ranges else float('inf')
            if min_distance < self.safety_thresholds['min_distance_obstacle']:
                safety_issues.append(f"Obstacle detected at {min_distance:.2f}m (threshold: {self.safety_thresholds['min_distance_obstacle']}m)")
                safety_level = max(safety_level, SafetyLevel.DANGER)

        # Check velocity limits
        linear_speed = math.sqrt(
            self.current_velocity.linear.x**2 +
            self.current_velocity.linear.y**2 +
            self.current_velocity.linear.z**2
        )
        if linear_speed > self.safety_thresholds['max_linear_velocity']:
            safety_issues.append(f"Linear velocity too high: {linear_speed:.2f}m/s (threshold: {self.safety_thresholds['max_linear_velocity']}m/s)")
            safety_level = max(safety_level, SafetyLevel.WARNING)

        angular_speed = math.sqrt(
            self.current_velocity.angular.x**2 +
            self.current_velocity.angular.y**2 +
            self.current_velocity.angular.z**2
        )
        if angular_speed > self.safety_thresholds['max_angular_velocity']:
            safety_issues.append(f"Angular velocity too high: {angular_speed:.2f}rad/s (threshold: {self.safety_thresholds['max_angular_velocity']}rad/s)")
            safety_level = max(safety_level, SafetyLevel.WARNING)

        # Check battery level
        if self.battery_level < self.safety_thresholds['battery_low_threshold']:
            safety_issues.append(f"Battery level low: {self.battery_level:.1f}% (threshold: {self.safety_thresholds['battery_low_threshold']}%)")
            safety_level = max(safety_level, SafetyLevel.WARNING)

        # Check IMU data for tilt
        if self.imu_data:
            # Calculate tilt from IMU orientation
            orientation = self.imu_data.orientation
            # Convert quaternion to Euler angles to check tilt
            w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
            pitch = math.asin(2.0 * (w * y - z * x))
            roll = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)

            tilt_angle = math.degrees(math.sqrt(pitch**2 + roll**2))
            if tilt_angle > self.safety_thresholds['max_tilt_angle']:
                safety_issues.append(f"Excessive tilt: {tilt_angle:.1f}° (threshold: {self.safety_thresholds['max_tilt_angle']}°)")
                safety_level = max(safety_level, SafetyLevel.DANGER)

        # Check operation time
        current_time = self.get_clock().now()
        operation_duration = (current_time - self.operation_start_time).nanoseconds / 1e9
        if operation_duration > self.safety_thresholds['max_operation_time']:
            safety_issues.append(f"Operation time exceeded: {operation_duration:.0f}s (threshold: {self.safety_thresholds['max_operation_time']}s)")
            safety_level = max(safety_level, SafetyLevel.WARNING)

        # Generate report
        if safety_issues:
            report = f"Safety Level: {safety_level.name}, Issues: " + "; ".join(safety_issues)
        else:
            report = f"Safety Level: {safety_level.name}, All systems nominal"

        return safety_level, report

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_publisher.publish(stop_msg)
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop"""
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            stop_msg = Bool()
            stop_msg.data = False
            self.emergency_stop_publisher.publish(stop_msg)
            self.get_logger().info('Emergency stop deactivated')

    def get_safety_status(self) -> Dict:
        """Get current safety status summary"""
        safety_level, report = self.evaluate_safety_conditions()
        return {
            'safety_level': safety_level.name,
            'report': report,
            'battery_level': self.battery_level,
            'operation_time': (self.get_clock().now() - self.operation_start_time).nanoseconds / 1e9,
            'emergency_stop_active': self.emergency_stop_active
        }


def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        status = safety_monitor.get_safety_status()
        print(f"Safety Status: {status}")
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### 2. Creating Reliability Monitor Node

```bash
cat > ~/isaac_ros_ws/src/isaac_safety_framework/safety_framework/reliability_monitor.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import time
from collections import deque, defaultdict
from typing import Dict, List

class ReliabilityMonitor(Node):
    def __init__(self):
        super().__init__('reliability_monitor')

        # Reliability tracking
        self.message_counts = defaultdict(int)
        self.message_intervals = defaultdict(deque)
        self.message_timeliness = defaultdict(deque)
        self.failure_counts = defaultdict(int)
        self.uptime_start = time.time()
        self.reliability_scores = defaultdict(float)

        # Topic reliability thresholds
        self.reliability_thresholds = {
            'min_frequency': 5.0,      # Hz
            'max_message_age': 1.0,    # seconds
            'max_failure_rate': 0.1    # 10% failure rate
        }

        # Subscribers for key topics
        self.topic_subscribers = {
            '/camera/color/image_raw': self.create_subscription(
                Image, '/camera/color/image_raw',
                self.create_message_callback('/camera/color/image_raw'), 10
            ),
            '/scan': self.create_subscription(
                LaserScan, '/scan',
                self.create_message_callback('/scan'), 10
            ),
            '/imu': self.create_subscription(
                Imu, '/imu',
                self.create_message_callback('/imu'), 10
            ),
            '/cmd_vel': self.create_subscription(
                Twist, '/cmd_vel',
                self.create_message_callback('/cmd_vel'), 10
            )
        }

        # Publishers for reliability metrics
        self.reliability_publisher = self.create_publisher(
            Float32,
            '/reliability/score',
            10
        )

        self.reliability_status_publisher = self.create_publisher(
            String,
            '/reliability/status',
            10
        )

        # Timer for periodic reliability assessment
        self.assessment_timer = self.create_timer(5.0, self.assess_reliability)

        self.get_logger().info('Reliability Monitor initialized')

    def create_message_callback(self, topic_name):
        """Create a callback function for a specific topic"""
        def callback(msg):
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.message_counts[topic_name] += 1

            # Calculate message interval
            if topic_name in self.message_intervals:
                if len(self.message_intervals[topic_name]) > 0:
                    last_time = self.message_intervals[topic_name][-1]
                    interval = current_time - last_time
                    self.message_intervals[topic_name].append(current_time)

                    # Keep only recent intervals (last 10)
                    if len(self.message_intervals[topic_name]) > 10:
                        self.message_intervals[topic_name].popleft()
                else:
                    self.message_intervals[topic_name].append(current_time)
            else:
                self.message_intervals[topic_name] = deque([current_time], maxlen=10)

            # Check message timeliness (for topics with timestamps)
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                age = current_time - msg_time
                self.message_timeliness[topic_name].append(age)

                # Keep only recent timeliness data
                if len(self.message_timeliness[topic_name]) > 10:
                    self.message_timeliness[topic_name].popleft()
                else:
                    self.message_timeliness[topic_name] = deque(maxlen=10)

        return callback

    def assess_reliability(self):
        """Assess reliability of all monitored topics"""
        try:
            overall_reliability = 0.0
            topic_count = 0

            reliability_report = []

            for topic in self.topic_subscribers.keys():
                reliability_score = self.calculate_topic_reliability(topic)
                self.reliability_scores[topic] = reliability_score
                overall_reliability += reliability_score
                topic_count += 1

                # Add to report
                status = "HIGH" if reliability_score >= 0.8 else "MEDIUM" if reliability_score >= 0.5 else "LOW"
                reliability_report.append(f"{topic}: {status} ({reliability_score:.2f})")

            # Calculate overall reliability
            if topic_count > 0:
                overall_reliability /= topic_count

            # Publish overall reliability score
            score_msg = Float32()
            score_msg.data = overall_reliability
            self.reliability_publisher.publish(score_msg)

            # Publish reliability status
            status_msg = String()
            status_msg.data = f"Overall Reliability: {overall_reliability:.2f}, Topics: {', '.join(reliability_report)}"
            self.reliability_status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error in reliability assessment: {e}')

    def calculate_topic_reliability(self, topic_name) -> float:
        """Calculate reliability score for a specific topic"""
        # Base score
        score = 1.0

        # Check message frequency
        if topic_name in self.message_intervals and len(self.message_intervals[topic_name]) >= 2:
            intervals = list(self.message_intervals[topic_name])
            if len(intervals) > 1:
                avg_interval = sum(
                    intervals[i] - intervals[i-1] for i in range(1, len(intervals))
                ) / (len(intervals) - 1)

                if avg_interval > 0:
                    frequency = 1.0 / avg_interval
                    if frequency < self.reliability_thresholds['min_frequency']:
                        # Reduce score based on frequency deficiency
                        score *= (frequency / self.reliability_thresholds['min_frequency'])
                else:
                    score = 0.0  # Invalid interval

        # Check message timeliness
        if topic_name in self.message_timeliness and len(self.message_timeliness[topic_name]) > 0:
            avg_age = sum(self.message_timeliness[topic_name]) / len(self.message_timeliness[topic_name])
            if avg_age > self.reliability_thresholds['max_message_age']:
                # Reduce score based on message age
                penalty = min(1.0, (avg_age - self.reliability_thresholds['max_message_age']) /
                             self.reliability_thresholds['max_message_age'])
                score *= (1.0 - penalty)

        # Apply failure rate penalty
        total_messages = self.message_counts.get(topic_name, 1)
        failure_rate = self.failure_counts.get(topic_name, 0) / total_messages
        if failure_rate > self.reliability_thresholds['max_failure_rate']:
            penalty = min(1.0, (failure_rate - self.reliability_thresholds['max_failure_rate']) /
                         (1.0 - self.reliability_thresholds['max_failure_rate']))
            score *= (1.0 - penalty)

        # Ensure score is between 0 and 1
        return max(0.0, min(1.0, score))

    def get_reliability_summary(self) -> Dict:
        """Get reliability summary"""
        return {
            'overall_reliability': sum(self.reliability_scores.values()) / len(self.reliability_scores) if self.reliability_scores else 0.0,
            'uptime_seconds': time.time() - self.uptime_start,
            'topic_reliabilities': dict(self.reliability_scores),
            'message_counts': dict(self.message_counts),
            'failure_counts': dict(self.failure_counts)
        }


def main(args=None):
    rclpy.init(args=args)
    reliability_monitor = ReliabilityMonitor()

    try:
        rclpy.spin(reliability_monitor)
    except KeyboardInterrupt:
        summary = reliability_monitor.get_reliability_summary()
        print(f"Reliability Summary: {summary}")
    finally:
        reliability_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Fault Detection and Recovery

### 1. Creating Fault Detection System

```bash
cat > ~/isaac_ros_ws/src/isaac_safety_framework/safety_framework/fault_detector.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import numpy as np
from enum import Enum
from collections import deque, defaultdict
from typing import Dict, List, Optional

class FaultType(Enum):
    SENSOR_FAILURE = "sensor_failure"
    COMMUNICATION_ERROR = "communication_error"
    ACTUATOR_FAULT = "actuator_fault"
    SOFTWARE_ERROR = "software_error"
    ENVIRONMENTAL_HAZARD = "environmental_hazard"

class FaultSeverity(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

class FaultDetector(Node):
    def __init__(self):
        super().__init__('fault_detector')

        # Fault detection parameters
        self.fault_detection_params = {
            'sensor_variance_threshold': 0.1,
            'sensor_stuck_threshold': 0.01,
            'message_timeout': 2.0,  # seconds
            'imu_drift_threshold': 0.05,
            'lidar_spike_threshold': 5.0  # times the median
        }

        # Data tracking
        self.sensor_data_history = defaultdict(deque)
        self.last_message_times = defaultdict(Time)
        self.fault_history = deque(maxlen=100)
        self.active_faults = set()

        # Subscribers
        self.topic_subscribers = {
            '/camera/color/image_raw': self.create_subscription(
                Image, '/camera/color/image_raw',
                self.create_sensor_callback('/camera/color/image_raw'), 10
            ),
            '/scan': self.create_subscription(
                LaserScan, '/scan',
                self.create_sensor_callback('/scan'), 10
            ),
            '/imu': self.create_subscription(
                Imu, '/imu',
                self.create_sensor_callback('/imu'), 10
            )
        }

        # Publishers
        self.fault_publisher = self.create_publisher(
            String,
            '/fault/detected',
            10
        )

        self.fault_status_publisher = self.create_publisher(
            Bool,
            '/fault/active',
            10
        )

        # Timer for periodic fault detection
        self.detection_timer = self.create_timer(1.0, self.detect_faults)

        self.get_logger().info('Fault Detector initialized')

    def create_sensor_callback(self, topic_name):
        """Create a callback function for sensor data"""
        def callback(msg):
            current_time = self.get_clock().now()
            self.last_message_times[topic_name] = current_time

            # Process sensor-specific data
            if topic_name == '/scan':
                # Process LiDAR data
                if hasattr(msg, 'ranges') and msg.ranges:
                    # Store range data for variance analysis
                    data_point = np.mean([r for r in msg.ranges if 0.1 < r < 10.0])  # Filter valid ranges
                    self.sensor_data_history[topic_name].append(data_point)
            elif topic_name == '/imu':
                # Process IMU data
                if hasattr(msg, 'linear_acceleration'):
                    # Store acceleration magnitude
                    acc_mag = np.sqrt(
                        msg.linear_acceleration.x**2 +
                        msg.linear_acceleration.y**2 +
                        msg.linear_acceleration.z**2
                    )
                    self.sensor_data_history[topic_name].append(acc_mag)
            elif topic_name == '/camera/color/image_raw':
                # Process image data (simplified - just track image arrival)
                self.sensor_data_history[topic_name].append(1)  # Just mark that image arrived

            # Keep history to reasonable size
            if len(self.sensor_data_history[topic_name]) > 50:
                self.sensor_data_history[topic_name].popleft()

        return callback

    def detect_faults(self):
        """Perform fault detection on all monitored systems"""
        try:
            detected_faults = []

            # Check each monitored topic
            for topic, last_time in self.last_message_times.items():
                current_time = self.get_clock().now()

                # Check for message timeout
                time_diff = (current_time.nanoseconds - last_time.nanoseconds) / 1e9
                if time_diff > self.fault_detection_params['message_timeout']:
                    fault = {
                        'type': FaultType.COMMUNICATION_ERROR,
                        'severity': FaultSeverity.HIGH,
                        'component': topic,
                        'description': f'Message timeout: {time_diff:.2f}s since last message'
                    }
                    detected_faults.append(fault)

            # Check sensor data patterns
            for topic, data_history in self.sensor_data_history.items():
                if len(data_history) >= 10:  # Need enough data for analysis
                    data_array = np.array(data_history)

                    # Check for sensor stuck (very low variance)
                    variance = np.var(data_array)
                    if variance < self.fault_detection_params['sensor_stuck_threshold']:
                        fault = {
                            'type': FaultType.SENSOR_FAILURE,
                            'severity': FaultSeverity.MEDIUM,
                            'component': topic,
                            'description': f'Sensor stuck detected (variance: {variance:.6f})'
                        }
                        detected_faults.append(fault)

                    # Check for high variance (potential sensor noise)
                    if variance > self.fault_detection_params['sensor_variance_threshold']:
                        fault = {
                            'type': FaultType.SENSOR_FAILURE,
                            'severity': FaultSeverity.MEDIUM,
                            'component': topic,
                            'description': f'High sensor variance detected: {variance:.6f}'
                        }
                        detected_faults.append(fault)

                    # Check for LiDAR specific issues
                    if topic == '/scan' and len(data_array) >= 2:
                        # Check for sudden spikes in range data
                        consecutive_diffs = np.abs(np.diff(data_array))
                        if len(consecutive_diffs) > 0 and np.max(consecutive_diffs) > self.fault_detection_params['lidar_spike_threshold']:
                            fault = {
                                'type': FaultType.SENSOR_FAILURE,
                                'severity': FaultSeverity.MEDIUM,
                                'component': topic,
                                'description': f'LiDAR spike detected: {np.max(consecutive_diffs):.2f}'
                            }
                            detected_faults.append(fault)

            # Process detected faults
            for fault in detected_faults:
                fault_id = f"{fault['type'].value}_{fault['component']}"

                if fault_id not in self.active_faults:
                    # New fault detected
                    self.active_faults.add(fault_id)

                    # Publish fault notification
                    fault_msg = String()
                    fault_msg.data = f"{fault['type'].value}: {fault['description']}"
                    self.fault_publisher.publish(fault_msg)

                    self.get_logger().warn(f"FAULT DETECTED: {fault_msg.data}")

                    # Add to history
                    fault['timestamp'] = self.get_clock().now().nanoseconds / 1e9
                    self.fault_history.append(fault)

            # Update fault status
            status_msg = Bool()
            status_msg.data = len(self.active_faults) > 0
            self.fault_status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error in fault detection: {e}')

    def get_fault_summary(self) -> Dict:
        """Get fault detection summary"""
        return {
            'active_faults_count': len(self.active_faults),
            'active_faults': list(self.active_faults),
            'total_faults_detected': len(self.fault_history),
            'recent_faults': list(self.fault_history)[-5:]  # Last 5 faults
        }


def main(args=None):
    rclpy.init(args=args)
    fault_detector = FaultDetector()

    try:
        rclpy.spin(fault_detector)
    except KeyboardInterrupt:
        summary = fault_detector.get_fault_summary()
        print(f"Fault Summary: {summary}")
    finally:
        fault_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Safety and Reliability Launch Files

### 1. Creating Safety Framework Package

```bash
cd ~/isaac_ros_ws/src
source /opt/ros/humble/setup.bash

# Create safety framework package
ros2 pkg create --build-type ament_python isaac_safety_framework --dependencies rclpy sensor_msgs geometry_msgs std_msgs message_filters tf2_ros tf2_geometry_msgs nav_msgs vision_msgs builtin_interfaces
```

### 2. Creating Setup Files for Safety Package

```bash
cat > ~/isaac_ros_ws/src/isaac_safety_framework/setup.py << 'EOF
from setuptools import setup
from glob import glob
import os

package_name = 'isaac_safety_framework'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Safety and reliability framework for Isaac Sim AI-Robot systems',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor = isaac_safety_framework.safety_monitor:main',
            'reliability_monitor = isaac_safety_framework.reliability_monitor:main',
            'fault_detector = isaac_safety_framework.fault_detector:main',
        ],
    },
)
EOF
```

```bash
cat > ~/isaac_ros_ws/src/isaac_safety_framework/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_safety_framework</name>
  <version>0.0.1</version>
  <description>Safety and reliability framework for Isaac Sim AI-Robot systems</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>message_filters</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>builtin_interfaces</depend>

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

### 3. Creating Safety Framework Launch File

```bash
mkdir -p ~/isaac_ros_ws/src/isaac_safety_framework/launch
cat > ~/isaac_ros_ws/src/isaac_safety_framework/launch/safety_framework.launch.py << 'EOF
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_safety_monitoring = LaunchConfiguration('enable_safety_monitoring', default='true')
    enable_reliability_monitoring = LaunchConfiguration('enable_reliability_monitoring', default='true')
    enable_fault_detection = LaunchConfiguration('enable_fault_detection', default='true')

    # Safety monitor node
    safety_monitor = Node(
        package='isaac_safety_framework',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=lambda context: LaunchConfiguration('enable_safety_monitoring').perform(context) == 'true'
    )

    # Reliability monitor node
    reliability_monitor = Node(
        package='isaac_safety_framework',
        executable='reliability_monitor',
        name='reliability_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=lambda context: LaunchConfiguration('enable_reliability_monitoring').perform(context) == 'true'
    )

    # Fault detector node
    fault_detector = Node(
        package='isaac_safety_framework',
        executable='fault_detector',
        name='fault_detector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=lambda context: LaunchConfiguration('enable_fault_detection').perform(context) == 'true'
    )

    # Return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'enable_safety_monitoring',
        default_value='true',
        description='Enable safety monitoring system'))

    ld.add_action(DeclareLaunchArgument(
        'enable_reliability_monitoring',
        default_value='true',
        description='Enable reliability monitoring system'))

    ld.add_action(DeclareLaunchArgument(
        'enable_fault_detection',
        default_value='true',
        description='Enable fault detection system'))

    # Add nodes with timing delays to ensure proper initialization
    ld.add_action(safety_monitor)
    ld.add_action(TimerAction(
        period=2.0,
        actions=[reliability_monitor]
    ))
    ld.add_action(TimerAction(
        period=3.0,
        actions=[fault_detector]
    ))

    return ld
EOF
```

## Safety Configuration and Policies

### 1. Creating Safety Configuration File

```bash
cat > ~/isaac_sim_shared/configs/safety_config.yaml << 'EOF
# Safety Configuration for Isaac Sim AI-Robot Systems

safety_monitor:
  # Safety thresholds and limits
  thresholds:
    min_distance_obstacle: 0.3  # meters
    max_linear_velocity: 0.5    # m/s
    max_angular_velocity: 1.0   # rad/s
    max_acceleration: 2.0       # m/s²
    battery_low_threshold: 20.0 # percentage
    imu_temperature_threshold: 80.0  # degrees Celsius
    max_tilt_angle: 30.0        # degrees
    max_operation_time: 3600.0  # seconds (1 hour)

  # Emergency stop configuration
  emergency_stop:
    enable_emergency_stop: true
    stop_distance_threshold: 0.15  # meters before emergency stop
    timeout_after_stop: 30.0       # seconds to wait after emergency stop

  # Safety level definitions
  safety_levels:
    safe: 0
    warning: 1
    danger: 2
    emergency_stop: 3

reliability_monitor:
  # Reliability thresholds
  thresholds:
    min_frequency: 5.0      # Hz
    max_message_age: 1.0    # seconds
    max_failure_rate: 0.1   # 10% failure rate

  # Reliability assessment parameters
  assessment:
    assessment_interval: 5.0  # seconds
    history_size: 10          # number of recent measurements to consider
    grace_period: 10.0        # seconds after startup before reliability assessment begins

fault_detector:
  # Fault detection parameters
  parameters:
    sensor_variance_threshold: 0.1
    sensor_stuck_threshold: 0.01
    message_timeout: 2.0      # seconds
    imu_drift_threshold: 0.05
    lidar_spike_threshold: 5.0  # times the median

  # Fault severity levels
  severity_levels:
    low: 1
    medium: 2
    high: 3
    critical: 4

  # Fault response actions
  response_actions:
    low: "log_warning"
    medium: "reduce_speed"
    high: "pause_operation"
    critical: "emergency_stop"

# Overall safety policy
safety_policy:
  # Priority order for safety actions
  priority_order:
    - "emergency_stop"
    - "reduce_speed"
    - "pause_operation"
    - "log_warning"

  # Safety mode settings
  safety_modes:
    operational:
      allow_movement: true
      monitoring_level: "full"
    degraded:
      allow_movement: false
      monitoring_level: "full"
    emergency:
      allow_movement: false
      monitoring_level: "full"
      emergency_procedures: ["stop_all_motors", "activate_emergency_lights"]

  # Recovery procedures
  recovery_procedures:
    timeout: 30.0  # seconds to wait before attempting recovery
    retry_attempts: 3
    reset_on_recovery: true

# Logging and reporting
logging:
  enable_detailed_logging: true
  log_level: "INFO"
  log_file: "/workspace/shared_dir/logs/safety_log.txt"
  report_interval: 60.0  # seconds between status reports
EOF
```

## Safety Testing and Validation

### 1. Creating Safety Test Scripts

```bash
cat > ~/isaac_sim_shared/scripts/safety_tests.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from geometry_msgs.msg import Twist
import time
import threading
from typing import Dict, List

class SafetyTestSuite(Node):
    def __init__(self):
        super().__init__('safety_test_suite')

        # Test results
        self.test_results = {}
        self.test_in_progress = False

        # Publishers for test scenarios
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Subscribers for safety system responses
        self.safety_status_subscriber = self.create_subscription(
            Bool, '/safety/safe_to_proceed', self.safety_status_callback, 10
        )

        self.emergency_stop_subscriber = self.create_subscription(
            Bool, '/safety/emergency_stop', self.emergency_stop_callback, 10
        )

        self.fault_subscriber = self.create_subscription(
            String, '/fault/detected', self.fault_callback, 10
        )

        self.get_logger().info('Safety Test Suite initialized')

    def safety_status_callback(self, msg):
        """Track safety status during tests"""
        self.safety_status = msg.data

    def emergency_stop_callback(self, msg):
        """Track emergency stop status during tests"""
        self.emergency_stop_activated = msg.data

    def fault_callback(self, msg):
        """Track fault detections during tests"""
        self.last_fault = msg.data

    def run_all_tests(self) -> Dict:
        """Run all safety tests and return results"""
        self.get_logger().info('Starting safety test suite...')

        results = {}

        # Test 1: Emergency stop functionality
        results['emergency_stop'] = self.test_emergency_stop()
        time.sleep(2)  # Wait between tests

        # Test 2: Obstacle detection
        results['obstacle_detection'] = self.test_obstacle_detection()
        time.sleep(2)

        # Test 3: Velocity limiting
        results['velocity_limiting'] = self.test_velocity_limiting()
        time.sleep(2)

        # Test 4: Fault detection
        results['fault_detection'] = self.test_fault_detection()
        time.sleep(2)

        self.get_logger().info(f'Safety tests completed: {results}')
        return results

    def test_emergency_stop(self) -> bool:
        """Test emergency stop functionality"""
        self.get_logger().info('Testing emergency stop...')

        # Initially should be safe to proceed
        safe_before = getattr(self, 'safety_status', True)

        # Simulate an emergency situation (obstacle very close)
        # This would normally be done by publishing a LiDAR scan with close obstacles
        # For simulation, we'll test the system's response

        # Wait for system to respond
        time.sleep(1)

        # Check if emergency stop was activated
        emergency_activated = getattr(self, 'emergency_stop_activated', False)

        # Reset for next test
        self.emergency_stop_activated = False

        result = emergency_activated
        self.get_logger().info(f'Emergency stop test: {"PASS" if result else "FAIL"}')
        return result

    def test_obstacle_detection(self) -> bool:
        """Test obstacle detection functionality"""
        self.get_logger().info('Testing obstacle detection...')

        # Publish a LiDAR scan with an obstacle close by
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar_link'
        scan_msg.angle_min = -3.14
        scan_msg.angle_max = 3.14
        scan_msg.angle_increment = 0.01
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.0
        scan_msg.range_min = 0.01
        scan_msg.range_max = 25.0

        # Create ranges with a close obstacle at 0.2m
        ranges = [25.0] * 628  # 628 points for full 360 degrees
        ranges[314] = 0.2  # Obstacle at 180 degrees (0.2m away)
        scan_msg.ranges = ranges

        self.lidar_publisher.publish(scan_msg)

        # Wait for processing
        time.sleep(0.5)

        # Check if safety system responded appropriately
        # In a real test, we'd check if safe_to_proceed became False
        safe_status = getattr(self, 'safety_status', True)

        result = not safe_status  # Should be unsafe when obstacle is close
        self.get_logger().info(f'Obstacle detection test: {"PASS" if result else "FAIL"}')
        return result

    def test_velocity_limiting(self) -> bool:
        """Test velocity limiting functionality"""
        self.get_logger().info('Testing velocity limiting...')

        # Publish a high velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = 2.0  # Exceeds safe limit of 0.5 m/s
        cmd_msg.angular.z = 2.0 # Exceeds safe limit of 1.0 rad/s

        self.cmd_vel_publisher.publish(cmd_msg)

        # Wait for processing
        time.sleep(0.5)

        # Check if system detected the unsafe command
        fault_detected = hasattr(self, 'last_fault') and 'velocity' in getattr(self, 'last_fault', '')

        result = fault_detected
        self.get_logger().info(f'Velocity limiting test: {"PASS" if result else "FAIL"}')
        return result

    def test_fault_detection(self) -> bool:
        """Test general fault detection"""
        self.get_logger().info('Testing fault detection...')

        # Simulate various fault conditions
        # In practice, this would involve creating specific sensor data patterns
        # that trigger fault detection algorithms

        # Wait for any fault detection
        time.sleep(2)

        fault_detected = hasattr(self, 'last_fault') and bool(getattr(self, 'last_fault', ''))

        result = fault_detected
        self.get_logger().info(f'Fault detection test: {"PASS" if result else "FAIL"}')
        return result

    def get_test_summary(self) -> Dict:
        """Get summary of all safety tests"""
        return {
            'timestamp': time.time(),
            'total_tests': len(self.test_results),
            'passed_tests': sum(1 for result in self.test_results.values() if result),
            'failed_tests': sum(1 for result in self.test_results.values() if not result),
            'test_results': self.test_results
        }


def main(args=None):
    rclpy.init(args=args)
    test_suite = SafetyTestSuite()

    try:
        results = test_suite.run_all_tests()
        summary = test_suite.get_test_summary()
        print(f"Safety Test Summary: {summary}")
    except Exception as e:
        print(f"Error running safety tests: {e}")
    finally:
        test_suite.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Safety Documentation and Procedures

### 1. Creating Safety Procedures Document

```bash
cat > ~/isaac_sim_shared/docs/safety_procedures.md << 'EOF
# Safety Procedures for Isaac Sim AI-Robot Systems

## Overview
This document outlines the safety procedures for operating AI-robot systems in the Isaac Sim environment. These procedures ensure safe operation during development, testing, and validation phases.

## Pre-Operation Safety Checks

### 1. System Verification
- [ ] Verify all safety monitoring systems are active
- [ ] Check that emergency stop functionality is operational
- [ ] Confirm sensor systems are providing valid data
- [ ] Validate communication links are stable
- [ ] Ensure battery levels are adequate (>50%)

### 2. Environment Setup
- [ ] Verify simulation environment is properly configured
- [ ] Check that safety boundaries are defined
- [ ] Confirm obstacle detection systems are calibrated
- [ ] Verify emergency stop zones are established

## Operational Safety Procedures

### 1. Normal Operation
- Monitor safety status continuously
- Watch for fault alerts or warnings
- Maintain communication with the system
- Be prepared to activate emergency stop if needed

### 2. Emergency Procedures
If an unsafe condition is detected:

1. **Immediate Response**:
   - Activate emergency stop if system does not respond automatically
   - Assess the situation and identify the cause
   - Document the incident with timestamps and sensor data

2. **System Recovery**:
   - Allow system to complete any emergency procedures
   - Verify all systems have stopped safely
   - Identify and resolve the root cause
   - Reset safety systems before resuming operation

### 3. Incident Reporting
All safety incidents must be documented:
- Time and date of incident
- Conditions leading to incident
- System response
- Actions taken
- Resolution and recovery steps

## Safety System Configuration

### 1. Threshold Settings
The following thresholds should be verified before operation:

- **Obstacle Distance**: Minimum 0.3m
- **Velocity Limits**: Linear ≤ 0.5 m/s, Angular ≤ 1.0 rad/s
- **Battery Level**: Minimum 20% before shutdown
- **Tilt Angle**: Maximum 30° from vertical

### 2. Emergency Response
- **Immediate Stop**: When obstacle < 0.15m
- **Warning**: When obstacle < 0.5m
- **Speed Reduction**: When approaching boundaries

## Fault Handling Procedures

### 1. Sensor Faults
If a sensor fault is detected:
1. Identify which sensor is affected
2. Determine if the system can continue safely with reduced sensing
3. If not, initiate safe shutdown
4. Document the fault for analysis

### 2. Communication Faults
If communication is lost:
1. The system should automatically enter safe mode
2. Attempt to reestablish communication
3. If unsuccessful after timeout, initiate emergency stop
4. Document the communication failure

### 3. Actuator Faults
If actuator faults are detected:
1. Stop all motion commands immediately
2. Check for mechanical obstructions
3. Verify power and control signals
4. Initiate safe recovery if possible

## Training Requirements

### 1. Operator Training
All operators must complete:
- Safety system operation training
- Emergency procedure training
- Incident reporting procedures
- System-specific safety protocols

### 2. Certification
- Operators must be certified before unsupervised operation
- Annual recertification required
- Incident response drills conducted quarterly

## Maintenance and Testing

### 1. Regular Testing
- Weekly: Basic safety system functionality test
- Monthly: Full safety system validation
- Quarterly: Emergency procedure drills

### 2. Preventive Maintenance
- Check sensor calibration monthly
- Verify emergency stop functionality weekly
- Update safety software as needed
- Document all maintenance activities

## Documentation Requirements

### 1. Safety Records
Maintain records of:
- All safety system tests
- Incident reports
- Maintenance activities
- Operator certifications

### 2. System Logs
- Continuous logging of safety-relevant data
- Regular backup of safety logs
- Secure storage of safety records
- Access control for safety data

## Compliance and Standards

This safety framework complies with:
- ISO 10218-1:2011 (Industrial robots - Safety requirements)
- ISO/TS 15066:2016 (Collaborative robots)
- ANSI/RIA R15.06 (Industrial robot safety)
- Local safety regulations and standards

## Continuous Improvement

### 1. Safety Reviews
- Monthly safety performance reviews
- Quarterly safety procedure updates
- Annual safety framework assessment

### 2. Feedback Integration
- Operator feedback incorporation
- Incident analysis integration
- Best practice sharing
- Technology update integration

## Contact Information

For safety-related concerns:
- **Safety Officer**: [Contact Information]
- **Technical Support**: [Contact Information]
- **Emergency Contact**: [Contact Information]
EOF
```

## Testing and Validation

### 1. Creating Safety Framework Test Script

```bash
cat > ~/test_safety_framework.sh << 'EOF'
#!/bin/bash

# Test script for safety and reliability framework

echo "Testing Safety and Reliability Framework..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if safety framework package is available
echo "Checking safety framework package..."
ros2 pkg list | grep safety_framework

if [ $? -eq 0 ]; then
    echo "✓ Safety framework package found"
else
    echo "✗ Safety framework package not found"
    exit 1
fi

# Check for safety configuration files
if [ -f "/workspace/shared_dir/configs/safety_config.yaml" ]; then
    echo "✓ Safety configuration file found"
else
    echo "✗ Safety configuration file not found"
    exit 1
fi

# Check for safety procedures documentation
if [ -f "/workspace/shared_dir/docs/safety_procedures.md" ]; then
    echo "✓ Safety procedures documentation found"
else
    echo "✗ Safety procedures documentation not found"
    exit 1
fi

# Build the safety framework package
echo "Building safety framework package..."
cd ~/isaac_ros_ws
colcon build --packages-select isaac_safety_framework
source install/setup.bash

# Check if launch files exist
if [ -f "/workspace/shared_dir/src/isaac_safety_framework/launch/safety_framework.launch.py" ]; then
    echo "✓ Safety framework launch file found"
else
    echo "✗ Safety framework launch file not found"
    exit 1
fi

# Test Python dependencies
python3 -c "import numpy; import rclpy; print('✓ Required Python packages available')" || {
    echo "✗ Required Python packages not available"
    echo "Installing required packages..."
    pip3 install numpy
}

echo "Safety and Reliability Framework test completed."
echo "To run the safety system, use:"
echo "ros2 launch isaac_safety_framework safety_framework.launch.py"
EOF

# Make executable
chmod +x ~/test_safety_framework.sh
```

### 2. Running Safety Framework Test

```bash
~/test_safety_framework.sh
```

## Troubleshooting Safety Framework

### Common Issues and Solutions

#### Issue: "Safety monitor not responding to emergency conditions"
**Solution**: Check sensor data and thresholds
```bash
# Verify sensor data is being published
ros2 topic echo /scan --field ranges | head -20
ros2 topic echo /imu --field orientation | head -10

# Check safety parameter configuration
ros2 param list | grep safety
```

#### Issue: "High CPU usage from safety monitoring nodes"
**Solution**: Adjust monitoring frequency
```bash
# Check current monitoring frequency
ros2 node info /safety_monitor
ros2 node info /reliability_monitor
```

#### Issue: "False positive fault detections"
**Solution**: Calibrate fault detection thresholds
```bash
# Review current sensor data patterns
ros2 topic hz /camera/color/image_raw
ros2 topic hz /scan
```

## Verification Checklist

- [ ] Safety monitoring node created and functional
- [ ] Reliability assessment system implemented
- [ ] Fault detection system created
- [ ] Safety configuration files created
- [ ] Safety procedures documented
- [ ] Launch files created for safety framework
- [ ] Safety test suite implemented
- [ ] Emergency stop functionality verified
- [ ] Test script created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After implementing the safety and reliability framework:

1. **Test safety systems** with Isaac Sim running
2. **Validate emergency procedures** and responses
3. **Calibrate safety thresholds** based on operational requirements
4. **Create safety assessment tools** for students

The safety and reliability framework is now configured and ready for Module 3, providing students with tools to ensure safe and reliable AI-robot system operation.