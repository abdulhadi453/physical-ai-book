# Performance Monitoring Tools for Isaac Sim AI-Robot Systems

## Overview

This document provides comprehensive instructions for setting up and using performance monitoring tools for the NVIDIA Isaac Sim AI-robot systems in Module 3. The focus is on monitoring system performance, resource utilization, and AI model efficiency to ensure optimal operation of the AI-robot brain system.

## Introduction to Performance Monitoring

### What is Performance Monitoring in Isaac Sim?
Performance monitoring in Isaac Sim involves tracking and analyzing various metrics to ensure that the AI-robot system operates efficiently and reliably. This includes:

- System resource utilization (CPU, GPU, memory)
- AI model performance (inference time, accuracy, throughput)
- Sensor data processing (latency, frequency, quality)
- Navigation and control performance (path efficiency, success rate)
- Communication efficiency (ROS topic latency, bandwidth)

### Key Performance Metrics
1. **Computational Performance**: CPU/GPU utilization, memory usage
2. **AI Model Performance**: Inference time, frames per second (FPS)
3. **Sensor Performance**: Data frequency, latency, accuracy
4. **Navigation Performance**: Path efficiency, success rate, safety
5. **System Integration**: End-to-end latency, resource sharing

## Prerequisites

Before setting up performance monitoring, ensure you have:
- Completed Isaac Sim and Isaac ROS setup
- Basic understanding of ROS 2 topics and services
- Familiarity with system monitoring tools
- NVIDIA GPU with proper monitoring capabilities

## System Resource Monitoring

### 1. Creating System Resource Monitor Node

```bash
cat > ~/isaac_ros_ws/src/isaac_performance_monitor/performance_monitor/system_monitor.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import BatteryState
import psutil
import GPUtil
import os
import time
from collections import deque

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')

        # Publishers for system metrics
        self.cpu_usage_publisher = self.create_publisher(Float32, '/system/cpu_usage', 10)
        self.memory_usage_publisher = self.create_publisher(Float32, '/system/memory_usage', 10)
        self.gpu_usage_publisher = self.create_publisher(Float32, '/system/gpu_usage', 10)
        self.gpu_memory_publisher = self.create_publisher(Float32, '/system/gpu_memory_usage', 10)
        self.disk_usage_publisher = self.create_publisher(Float32, '/system/disk_usage', 10)
        self.temperature_publisher = self.create_publisher(Float32, '/system/cpu_temperature', 10)
        self.status_publisher = self.create_publisher(String, '/system/status', 10)

        # Performance tracking
        self.cpu_history = deque(maxlen=100)
        self.memory_history = deque(maxlen=100)
        self.gpu_history = deque(maxlen=100)

        # Setup timer for periodic monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_system)

        # Setup timer for status publishing
        self.status_timer = self.create_timer(5.0, self.publish_status)

        self.get_logger().info('System Monitor initialized')

    def monitor_system(self):
        """Monitor system resources and publish metrics"""
        try:
            # CPU usage
            cpu_percent = psutil.cpu_percent()
            cpu_msg = Float32()
            cpu_msg.data = float(cpu_percent)
            self.cpu_usage_publisher.publish(cpu_msg)

            # Memory usage
            memory = psutil.virtual_memory()
            memory_msg = Float32()
            memory_msg.data = float(memory.percent)
            self.memory_usage_publisher.publish(memory_msg)

            # GPU usage (if available)
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu = gpus[0]  # Assuming single GPU
                gpu_msg = Float32()
                gpu_msg.data = float(gpu.load * 100)
                self.gpu_usage_publisher.publish(gpu_msg)

                gpu_memory_msg = Float32()
                gpu_memory_msg.data = float(gpu.memoryUtil * 100)
                self.gpu_memory_publisher.publish(gpu_memory_msg)

            # Disk usage
            disk_usage = psutil.disk_usage('/')
            disk_msg = Float32()
            disk_msg.data = float((disk_usage.used / disk_usage.total) * 100)
            self.disk_usage_publisher.publish(disk_msg)

            # Store history for trend analysis
            self.cpu_history.append(cpu_percent)
            self.memory_history.append(memory.percent)
            if gpus:
                self.gpu_history.append(gpu.load * 100)

        except Exception as e:
            self.get_logger().error(f'Error monitoring system: {e}')

    def publish_status(self):
        """Publish overall system status"""
        try:
            status_msg = String()

            # Determine status based on resource usage
            cpu_avg = sum(self.cpu_history) / len(self.cpu_history) if self.cpu_history else 0
            memory_avg = sum(self.memory_history) / len(self.memory_history) if self.memory_history else 0

            if cpu_avg > 80 or memory_avg > 80:
                status_msg.data = f"WARNING: High resource usage - CPU: {cpu_avg:.1f}%, Memory: {memory_avg:.1f}%"
            else:
                status_msg.data = f"OK: CPU: {cpu_avg:.1f}%, Memory: {memory_avg:.1f}%"

            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

    def get_system_summary(self):
        """Get a summary of system performance"""
        summary = {
            'timestamp': time.time(),
            'cpu_avg': sum(self.cpu_history) / len(self.cpu_history) if self.cpu_history else 0,
            'memory_avg': sum(self.memory_history) / len(self.memory_history) if self.memory_history else 0,
            'gpu_avg': sum(self.gpu_history) / len(self.gpu_history) if self.gpu_history else 0,
            'cpu_peak': max(self.cpu_history) if self.cpu_history else 0,
            'memory_peak': max(self.memory_history) if self.memory_history else 0
        }
        return summary


def main(args=None):
    rclpy.init(args=args)
    monitor = SystemMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        summary = monitor.get_system_summary()
        print(f"System Summary: {summary}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### 2. Creating AI Model Performance Monitor

```bash
cat > ~/isaac_ros_ws/src/isaac_performance_monitor/performance_monitor/model_performance_monitor.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from vision_msgs.msg import Detection2DArray
import time
from collections import deque
import threading

class ModelPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('model_performance_monitor')

        # Subscribe to model input/output topics
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        # Publishers for performance metrics
        self.fps_publisher = self.create_publisher(Float32, '/model/fps', 10)
        self.latency_publisher = self.create_publisher(Float32, '/model/latency', 10)
        self.throughput_publisher = self.create_publisher(Float32, '/model/throughput', 10)
        self.accuracy_publisher = self.create_publisher(Float32, '/model/accuracy', 10)
        self.status_publisher = self.create_publisher(String, '/model/status', 10)

        # Performance tracking
        self.frame_times = deque(maxlen=30)  # Last 30 frames for FPS calculation
        self.processing_times = deque(maxlen=30)  # Last 30 processing times
        self.detection_counts = deque(maxlen=30)  # Detection counts per frame
        self.last_image_time = None
        self.image_count = 0
        self.detection_count = 0

        # Setup timer for periodic metrics publishing
        self.timer = self.create_timer(1.0, self.publish_metrics)

        self.get_logger().info('Model Performance Monitor initialized')

    def image_callback(self, msg):
        """Track image processing for performance measurement"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.image_count += 1

        if self.last_image_time is not None:
            frame_time = current_time - self.last_image_time
            self.frame_times.append(frame_time)

        self.last_image_time = current_time

    def detection_callback(self, msg):
        """Track detection processing"""
        self.detection_count += 1
        self.detection_counts.append(len(msg.detections))

        # Calculate processing time if we have corresponding image time
        if self.last_image_time is not None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            processing_time = current_time - self.last_image_time
            self.processing_times.append(processing_time)

    def publish_metrics(self):
        """Publish performance metrics"""
        # Calculate FPS
        fps = 0.0
        if len(self.frame_times) > 0:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

        fps_msg = Float32()
        fps_msg.data = fps
        self.fps_publisher.publish(fps_msg)

        # Calculate average processing latency
        avg_latency = 0.0
        if len(self.processing_times) > 0:
            avg_latency = sum(self.processing_times) / len(self.processing_times)

        latency_msg = Float32()
        latency_msg.data = avg_latency
        self.latency_publisher.publish(latency_msg)

        # Calculate throughput (detections per second)
        throughput = self.detection_count
        throughput_msg = Float32()
        throughput_msg.data = float(throughput)
        self.throughput_publisher.publish(throughput_msg)

        # Calculate average detections per frame
        avg_detections = 0.0
        if len(self.detection_counts) > 0:
            avg_detections = sum(self.detection_counts) / len(self.detection_counts)

        accuracy_msg = Float32()
        accuracy_msg.data = avg_detections  # Using avg detections as a proxy for activity
        self.accuracy_publisher.publish(accuracy_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f"FPS: {fps:.2f}, Latency: {avg_latency*1000:.2f}ms, Avg Detections: {avg_detections:.2f}"
        self.status_publisher.publish(status_msg)

        # Reset counters for next period
        self.detection_count = 0

    def get_performance_summary(self):
        """Get a summary of performance metrics"""
        summary = {
            'fps_avg': 0.0,
            'latency_avg_ms': 0.0,
            'throughput_avg': 0.0,
            'detections_per_frame_avg': 0.0,
            'total_samples': len(self.frame_times)
        }

        if len(self.frame_times) > 0:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            summary['fps_avg'] = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

        if len(self.processing_times) > 0:
            summary['latency_avg_ms'] = (sum(self.processing_times) / len(self.processing_times)) * 1000

        if len(self.detection_counts) > 0:
            summary['detections_per_frame_avg'] = sum(self.detection_counts) / len(self.detection_counts)

        return summary


def main(args=None):
    rclpy.init(args=args)
    monitor = ModelPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        summary = monitor.get_performance_summary()
        print(f"Model Performance Summary: {summary}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Navigation Performance Monitoring

### 1. Creating Navigation Performance Monitor

```bash
cat > ~/isaac_ros_ws/src/isaac_performance_monitor/performance_monitor/navigation_monitor.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import math
from collections import deque
import time

class NavigationPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('navigation_performance_monitor')

        # Subscribers for navigation-related topics
        self.path_subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for navigation metrics
        self.path_efficiency_publisher = self.create_publisher(Float32, '/nav/path_efficiency', 10)
        self.success_rate_publisher = self.create_publisher(Float32, '/nav/success_rate', 10)
        self.time_to_goal_publisher = self.create_publisher(Float32, '/nav/time_to_goal', 10)
        self.safety_score_publisher = self.create_publisher(Float32, '/nav/safety_score', 10)
        self.status_publisher = self.create_publisher(String, '/nav/status', 10)

        # Navigation tracking
        self.start_pose = None
        self.goal_pose = None
        self.current_pose = None
        self.path_length = 0.0
        self.actual_distance = 0.0
        self.start_time = None
        self.navigation_active = False
        self.success_count = 0
        self.attempt_count = 0
        self.collision_count = 0
        self.path_history = []
        self.velocity_history = deque(maxlen=100)

        # Setup timer for periodic metrics publishing
        self.timer = self.create_timer(1.0, self.publish_metrics)

        # Navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Navigation Performance Monitor initialized')

    def path_callback(self, msg):
        """Track planned path for efficiency calculation"""
        if len(msg.poses) > 1:
            # Calculate path length
            total_length = 0.0
            for i in range(len(msg.poses) - 1):
                p1 = msg.poses[i].pose.position
                p2 = msg.poses[i+1].pose.position
                segment_length = math.sqrt(
                    (p2.x - p1.x)**2 + (p2.y - p1.y)**2
                )
                total_length += segment_length

            self.path_length = total_length

    def odom_callback(self, msg):
        """Track robot position for navigation metrics"""
        self.current_pose = msg.pose.pose

        if self.start_pose and self.current_pose:
            # Calculate distance traveled
            dx = self.current_pose.position.x - self.start_pose.position.x
            dy = self.current_pose.position.y - self.start_pose.position.y
            self.actual_distance = math.sqrt(dx*dx + dy*dy)

    def cmd_vel_callback(self, msg):
        """Track velocity for safety metrics"""
        speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        self.velocity_history.append(speed)

    def start_navigation_tracking(self, start_pose, goal_pose):
        """Start tracking navigation performance"""
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.start_time = time.time()
        self.navigation_active = True
        self.path_length = 0.0
        self.actual_distance = 0.0

    def finish_navigation_tracking(self, success):
        """Finish navigation tracking and update statistics"""
        if self.start_time:
            elapsed_time = time.time() - self.start_time
            self.time_to_goal_publisher.publish(Float32(data=float(elapsed_time)))

        self.navigation_active = False
        self.attempt_count += 1

        if success:
            self.success_count += 1

    def publish_metrics(self):
        """Publish navigation performance metrics"""
        # Calculate path efficiency
        efficiency = 0.0
        if self.path_length > 0 and self.actual_distance > 0:
            efficiency = self.path_length / self.actual_distance

        efficiency_msg = Float32()
        efficiency_msg.data = efficiency
        self.path_efficiency_publisher.publish(efficiency_msg)

        # Calculate success rate
        success_rate = 0.0
        if self.attempt_count > 0:
            success_rate = (self.success_count / self.attempt_count) * 100

        success_msg = Float32()
        success_msg.data = success_rate
        self.success_rate_publisher.publish(success_msg)

        # Calculate safety score (simplified)
        avg_velocity = sum(self.velocity_history) / len(self.velocity_history) if self.velocity_history else 0.0
        # Safety score: lower velocities = higher safety
        safety_score = max(0, 100 - (avg_velocity * 10))  # Adjust scale as needed

        safety_msg = Float32()
        safety_msg.data = safety_score
        self.safety_score_publisher.publish(safety_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f"Efficiency: {efficiency:.2f}, Success Rate: {success_rate:.1f}%, Safety: {safety_score:.1f}"
        self.status_publisher.publish(status_msg)

    def get_navigation_summary(self):
        """Get a summary of navigation performance"""
        return {
            'success_rate_percent': (self.success_count / self.attempt_count * 100) if self.attempt_count > 0 else 0,
            'avg_path_efficiency': sum([p.length for p in self.path_history]) / len(self.path_history) if self.path_history else 0,
            'avg_time_to_goal': 0,  # Would need to track multiple navigation attempts
            'collision_count': self.collision_count,
            'total_attempts': self.attempt_count
        }


def main(args=None):
    rclpy.init(args=args)
    monitor = NavigationPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        summary = monitor.get_navigation_summary()
        print(f"Navigation Performance Summary: {summary}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Communication Performance Monitoring

### 1. Creating Communication Monitor

```bash
cat > ~/isaac_ros_ws/src/isaac_performance_monitor/performance_monitor/communication_monitor.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, LaserScan, Imu
import time
from collections import deque, defaultdict

class CommunicationMonitor(Node):
    def __init__(self):
        super().__init__('communication_monitor')

        # Topic-specific subscribers with timing
        self.topic_stats = defaultdict(lambda: {
            'count': 0,
            'latency_history': deque(maxlen=100),
            'frequency_history': deque(maxlen=100),
            'last_timestamp': None,
            'last_received': None
        })

        # Subscribe to common topics
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.create_message_callback('/camera/color/image_raw'),
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.create_message_callback('/scan'),
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.create_message_callback('/imu'),
            10
        )

        # Publishers for communication metrics
        self.latency_publisher = self.create_publisher(Float32, '/comm/avg_latency', 10)
        self.frequency_publisher = self.create_publisher(Float32, '/comm/frequency', 10)
        self.status_publisher = self.create_publisher(String, '/comm/status', 10)

        # Setup timer for periodic metrics publishing
        self.timer = self.create_timer(1.0, self.publish_metrics)

        self.get_logger().info('Communication Monitor initialized')

    def create_message_callback(self, topic_name):
        """Create a callback function for a specific topic"""
        def callback(msg):
            current_time = self.get_clock().now().nanoseconds / 1e9

            # Calculate latency (time from message creation to reception)
            if hasattr(msg.header, 'stamp'):
                msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                latency = current_time - msg_time
                if latency >= 0:  # Only consider positive latencies
                    self.topic_stats[topic_name]['latency_history'].append(latency)

            # Update statistics
            self.topic_stats[topic_name]['count'] += 1
            self.topic_stats[topic_name]['last_received'] = current_time

        return callback

    def publish_metrics(self):
        """Publish communication performance metrics"""
        # Calculate average latency across all topics
        total_latency = 0
        latency_count = 0

        for topic, stats in self.topic_stats.items():
            if stats['latency_history']:
                avg_topic_latency = sum(stats['latency_history']) / len(stats['latency_history'])
                total_latency += avg_topic_latency
                latency_count += 1

        avg_latency = total_latency / latency_count if latency_count > 0 else 0.0

        latency_msg = Float32()
        latency_msg.data = avg_latency
        self.latency_publisher.publish(latency_msg)

        # Calculate average frequency across all topics
        total_freq = 0
        freq_count = 0

        for topic, stats in self.topic_stats.items():
            if stats['last_received'] and stats['last_timestamp']:
                # Calculate frequency based on time intervals
                time_diff = stats['last_received'] - stats['last_timestamp']
                if time_diff > 0:
                    freq = 1.0 / time_diff
                    stats['frequency_history'].append(freq)
                    total_freq += freq
                    freq_count += 1

            # Update timestamp for next calculation
            stats['last_timestamp'] = stats['last_received']

        avg_freq = total_freq / freq_count if freq_count > 0 else 0.0

        freq_msg = Float32()
        freq_msg.data = avg_freq
        self.frequency_publisher.publish(freq_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f"Avg Latency: {avg_latency*1000:.2f}ms, Avg Freq: {avg_freq:.2f}Hz"
        self.status_publisher.publish(status_msg)

    def get_communication_summary(self):
        """Get a summary of communication performance"""
        summary = {
            'topics_monitored': list(self.topic_stats.keys()),
            'avg_latency_ms': 0.0,
            'avg_frequency': 0.0,
            'message_counts': {}
        }

        # Calculate overall statistics
        total_latency = 0
        latency_count = 0
        total_freq = 0
        freq_count = 0

        for topic, stats in self.topic_stats.items():
            summary['message_counts'][topic] = stats['count']

            if stats['latency_history']:
                avg_topic_latency = sum(stats['latency_history']) / len(stats['latency_history'])
                total_latency += avg_topic_latency
                latency_count += 1

            if stats['frequency_history']:
                avg_topic_freq = sum(stats['frequency_history']) / len(stats['frequency_history'])
                total_freq += avg_topic_freq
                freq_count += 1

        summary['avg_latency_ms'] = (total_latency / latency_count * 1000) if latency_count > 0 else 0.0
        summary['avg_frequency'] = total_freq / freq_count if freq_count > 0 else 0.0

        return summary


def main(args=None):
    rclpy.init(args=args)
    monitor = CommunicationMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        summary = monitor.get_communication_summary()
        print(f"Communication Performance Summary: {summary}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Performance Dashboard and Visualization

### 1. Creating Performance Dashboard

```bash
cat > ~/isaac_sim_shared/scripts/performance_dashboard.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import numpy as np
import threading
import time
from collections import deque
import tkinter as tk
from tkinter import ttk

class PerformanceDashboard(Node):
    def __init__(self):
        super().__init__('performance_dashboard')

        # Data storage for visualization
        self.cpu_data = deque(maxlen=100)
        self.memory_data = deque(maxlen=100)
        self.gpu_data = deque(maxlen=100)
        self.fps_data = deque(maxlen=100)
        self.latency_data = deque(maxlen=100)

        # Subscribe to performance metrics
        self.cpu_subscription = self.create_subscription(
            Float32,
            '/system/cpu_usage',
            self.cpu_callback,
            10
        )

        self.memory_subscription = self.create_subscription(
            Float32,
            '/system/memory_usage',
            self.memory_callback,
            10
        )

        self.gpu_subscription = self.create_subscription(
            Float32,
            '/system/gpu_usage',
            self.gpu_callback,
            10
        )

        self.fps_subscription = self.create_subscription(
            Float32,
            '/model/fps',
            self.fps_callback,
            10
        )

        self.latency_subscription = self.create_subscription(
            Float32,
            '/model/latency',
            self.latency_callback,
            10
        )

        # Setup visualization
        self.setup_visualization()

        # Start dashboard thread
        self.dashboard_thread = threading.Thread(target=self.run_dashboard)
        self.dashboard_thread.daemon = True
        self.dashboard_thread.start()

        self.get_logger().info('Performance Dashboard initialized')

    def cpu_callback(self, msg):
        self.cpu_data.append(msg.data)

    def memory_callback(self, msg):
        self.memory_data.append(msg.data)

    def gpu_callback(self, msg):
        self.gpu_data.append(msg.data)

    def fps_callback(self, msg):
        self.fps_data.append(msg.data)

    def latency_callback(self, msg):
        self.latency_data.append(msg.data * 1000)  # Convert to milliseconds

    def setup_visualization(self):
        """Setup the matplotlib visualization"""
        plt.ion()  # Interactive mode
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Isaac Sim AI-Robot Performance Dashboard', fontsize=14)

        # CPU usage plot
        self.cpu_line, = self.axs[0, 0].plot([], [], 'b-', label='CPU %')
        self.axs[0, 0].set_title('CPU Usage')
        self.axs[0, 0].set_ylim(0, 100)
        self.axs[0, 0].set_ylabel('Usage %')
        self.axs[0, 0].grid(True)
        self.axs[0, 0].legend()

        # Memory usage plot
        self.memory_line, = self.axs[0, 1].plot([], [], 'g-', label='Memory %')
        self.axs[0, 1].set_title('Memory Usage')
        self.axs[0, 1].set_ylim(0, 100)
        self.axs[0, 1].set_ylabel('Usage %')
        self.axs[0, 1].grid(True)
        self.axs[0, 1].legend()

        # FPS plot
        self.fps_line, = self.axs[1, 0].plot([], [], 'r-', label='FPS')
        self.axs[1, 0].set_title('Model FPS')
        self.axs[1, 0].set_ylim(0, 60)
        self.axs[1, 0].set_ylabel('Frames per Second')
        self.axs[1, 0].grid(True)
        self.axs[1, 0].legend()

        # Latency plot
        self.latency_line, = self.axs[1, 1].plot([], [], 'm-', label='Latency (ms)')
        self.axs[1, 1].set_title('Model Latency')
        self.axs[1, 1].set_ylim(0, 100)
        self.axs[1, 1].set_ylabel('Latency (ms)')
        self.axs[1, 1].grid(True)
        self.axs[1, 1].legend()

        # Set common x-axis
        for ax in self.axs.flat:
            ax.set_xlim(0, 100)

        self.fig.tight_layout()

    def update_plots(self, frame):
        """Update all plots with current data"""
        x = range(len(self.cpu_data))

        # Update CPU plot
        if self.cpu_data:
            self.cpu_line.set_data(x, list(self.cpu_data))
            self.axs[0, 0].set_xlim(max(0, len(self.cpu_data)-100), max(100, len(self.cpu_data)))

        # Update Memory plot
        if self.memory_data:
            self.memory_line.set_data(x, list(self.memory_data))
            self.axs[0, 1].set_xlim(max(0, len(self.memory_data)-100), max(100, len(self.memory_data)))

        # Update FPS plot
        if self.fps_data:
            self.fps_line.set_data(x, list(self.fps_data))
            self.axs[1, 0].set_xlim(max(0, len(self.fps_data)-100), max(100, len(self.fps_data)))

        # Update Latency plot
        if self.latency_data:
            self.latency_line.set_data(x, list(self.latency_data))
            self.axs[1, 1].set_xlim(max(0, len(self.latency_data)-100), max(100, len(self.latency_data)))

        return self.cpu_line, self.memory_line, self.fps_line, self.latency_line

    def run_dashboard(self):
        """Run the dashboard visualization"""
        try:
            ani = FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)
            plt.show()
        except Exception as e:
            self.get_logger().error(f'Error in dashboard: {e}')

    def get_system_health(self):
        """Calculate overall system health score"""
        if not self.cpu_data or not self.memory_data or not self.fps_data:
            return "Unknown"

        avg_cpu = sum(self.cpu_data) / len(self.cpu_data)
        avg_memory = sum(self.memory_data) / len(self.memory_data)
        avg_fps = sum(self.fps_data) / len(self.fps_data) if self.fps_data else 0

        # Health calculation (simplified)
        cpu_health = max(0, 100 - avg_cpu)  # Lower CPU = better
        memory_health = max(0, 100 - avg_memory)  # Lower memory = better
        fps_health = min(100, avg_fps)  # Higher FPS = better

        overall_health = (cpu_health + memory_health + fps_health) / 3

        if overall_health > 80:
            return f"Excellent ({overall_health:.1f})"
        elif overall_health > 60:
            return f"Good ({overall_health:.1f})"
        elif overall_health > 40:
            return f"Fair ({overall_health:.1f})"
        else:
            return f"Needs Attention ({overall_health:.1f})"


def main(args=None):
    rclpy.init(args=args)
    dashboard = PerformanceDashboard()

    try:
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        print(f"System Health: {dashboard.get_system_health()}")
        plt.close('all')
    finally:
        dashboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Performance Monitoring Launch Files

### 1. Creating Performance Monitoring Package

```bash
cd ~/isaac_ros_ws/src
source /opt/ros/humble/setup.bash

# Create performance monitoring package
ros2 pkg create --build-type ament_python isaac_performance_monitor --dependencies rclpy sensor_msgs geometry_msgs std_msgs message_filters tf2_ros tf2_geometry_msgs nav_msgs vision_msgs
```

### 2. Creating Setup Files for Performance Package

```bash
cat > ~/isaac_ros_ws/src/isaac_performance_monitor/setup.py << 'EOF
from setuptools import setup
from glob import glob
import os

package_name = 'isaac_performance_monitor'

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
    description='Performance monitoring tools for Isaac Sim AI-Robot systems',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor = isaac_performance_monitor.system_monitor:main',
            'model_performance_monitor = isaac_performance_monitor.model_performance_monitor:main',
            'navigation_monitor = isaac_performance_monitor.navigation_monitor:main',
            'communication_monitor = isaac_performance_monitor.communication_monitor:main',
        ],
    },
)
EOF
```

```bash
cat > ~/isaac_ros_ws/src/isaac_performance_monitor/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_performance_monitor</name>
  <version>0.0.1</version>
  <description>Performance monitoring tools for Isaac Sim AI-Robot systems</description>
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

### 3. Creating Performance Monitoring Launch File

```bash
mkdir -p ~/isaac_ros_ws/src/isaac_performance_monitor/launch
cat > ~/isaac_ros_ws/src/isaac_performance_monitor/launch/performance_monitoring.launch.py << 'EOF
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_visualization = LaunchConfiguration('enable_visualization', default='false')

    # System monitor node
    system_monitor = Node(
        package='isaac_performance_monitor',
        executable='system_monitor',
        name='system_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Model performance monitor node
    model_performance_monitor = Node(
        package='isaac_performance_monitor',
        executable='model_performance_monitor',
        name='model_performance_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation monitor node
    navigation_monitor = Node(
        package='isaac_performance_monitor',
        executable='navigation_monitor',
        name='navigation_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Communication monitor node
    communication_monitor = Node(
        package='isaac_performance_monitor',
        executable='communication_monitor',
        name='communication_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Performance dashboard (optional)
    performance_dashboard = Node(
        package='isaac_performance_monitor',
        executable='performance_dashboard',
        name='performance_dashboard',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=lambda context: LaunchConfiguration('enable_visualization').perform(context) == 'true'
    )

    # Return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'enable_visualization',
        default_value='false',
        description='Enable performance visualization dashboard'))

    # Add nodes
    ld.add_action(system_monitor)
    ld.add_action(TimerAction(
        period=1.0,
        actions=[model_performance_monitor]
    ))
    ld.add_action(TimerAction(
        period=2.0,
        actions=[navigation_monitor]
    ))
    ld.add_action(TimerAction(
        period=3.0,
        actions=[communication_monitor]
    ))
    ld.add_action(TimerAction(
        period=5.0,
        actions=[performance_dashboard]
    ))

    return ld
EOF
```

## Performance Logging and Analysis

### 1. Creating Performance Logger

```bash
cat > ~/isaac_sim_shared/scripts/performance_logger.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image, LaserScan, Imu
import json
import csv
import os
from datetime import datetime
from collections import defaultdict
import threading

class PerformanceLogger(Node):
    def __init__(self):
        super().__init__('performance_logger')

        # Data storage
        self.metrics = defaultdict(list)
        self.start_time = datetime.now()

        # Create logs directory
        self.logs_dir = "/workspace/shared_dir/logs"
        os.makedirs(self.logs_dir, exist_ok=True)

        # Subscribe to performance metrics
        self.cpu_subscription = self.create_subscription(
            Float32,
            '/system/cpu_usage',
            lambda msg: self.log_metric('cpu_usage', msg.data),
            10
        )

        self.memory_subscription = self.create_subscription(
            Float32,
            '/system/memory_usage',
            lambda msg: self.log_metric('memory_usage', msg.data),
            10
        )

        self.gpu_subscription = self.create_subscription(
            Float32,
            '/system/gpu_usage',
            lambda msg: self.log_metric('gpu_usage', msg.data),
            10
        )

        self.fps_subscription = self.create_subscription(
            Float32,
            '/model/fps',
            lambda msg: self.log_metric('fps', msg.data),
            10
        )

        self.latency_subscription = self.create_subscription(
            Float32,
            '/model/latency',
            lambda msg: self.log_metric('latency_ms', msg.data * 1000),
            10
        )

        # Setup timer for periodic logging
        self.log_timer = self.create_timer(10.0, self.flush_logs)

        self.get_logger().info('Performance Logger initialized')

    def log_metric(self, metric_name, value):
        """Log a performance metric"""
        timestamp = datetime.now().isoformat()
        self.metrics[metric_name].append({
            'timestamp': timestamp,
            'value': value
        })

    def flush_logs(self):
        """Flush logs to file periodically"""
        try:
            # Create filename with timestamp
            timestamp = self.start_time.strftime("%Y%m%d_%H%M%S")
            filename = f"performance_log_{timestamp}.json"
            filepath = os.path.join(self.logs_dir, filename)

            # Write metrics to JSON file
            with open(filepath, 'w') as f:
                json.dump(dict(self.metrics), f, indent=2)

            self.get_logger().info(f'Performance logs flushed to {filepath}')

            # Also create a summary CSV
            self.create_summary_csv(timestamp)

        except Exception as e:
            self.get_logger().error(f'Error flushing logs: {e}')

    def create_summary_csv(self, timestamp):
        """Create a summary CSV file"""
        try:
            filename = f"performance_summary_{timestamp}.csv"
            filepath = os.path.join(self.logs_dir, filename)

            with open(filepath, 'w', newline='') as csvfile:
                fieldnames = ['metric', 'avg_value', 'min_value', 'max_value', 'sample_count']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()

                for metric_name, values in self.metrics.items():
                    if values:
                        metric_values = [v['value'] for v in values]
                        avg_val = sum(metric_values) / len(metric_values)
                        min_val = min(metric_values)
                        max_val = max(metric_values)
                        count = len(metric_values)

                        writer.writerow({
                            'metric': metric_name,
                            'avg_value': avg_val,
                            'min_value': min_val,
                            'max_value': max_val,
                            'sample_count': count
                        })

            self.get_logger().info(f'Performance summary created: {filepath}')

        except Exception as e:
            self.get_logger().error(f'Error creating summary CSV: {e}')

    def get_performance_report(self):
        """Generate a performance report"""
        report = {
            'start_time': self.start_time.isoformat(),
            'end_time': datetime.now().isoformat(),
            'metrics_summary': {}
        }

        for metric_name, values in self.metrics.items():
            if values:
                metric_values = [v['value'] for v in values]
                report['metrics_summary'][metric_name] = {
                    'avg': sum(metric_values) / len(metric_values),
                    'min': min(metric_values),
                    'max': max(metric_values),
                    'count': len(metric_values),
                    'latest': metric_values[-1] if metric_values else None
                }

        return report


def main(args=None):
    rclpy.init(args=args)
    logger = PerformanceLogger()

    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        report = logger.get_performance_report()
        print(f"Performance Report: {report}")

        # Flush final logs
        logger.flush_logs()
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Performance Alerting System

### 1. Creating Performance Alert System

```bash
cat > ~/isaac_sim_shared/scripts/performance_alerts.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time
from collections import deque

class PerformanceAlerts(Node):
    def __init__(self):
        super().__init__('performance_alerts')

        # Alert thresholds
        self.thresholds = {
            'cpu_usage': {'warning': 80, 'critical': 90},
            'memory_usage': {'warning': 80, 'critical': 90},
            'gpu_usage': {'warning': 85, 'critical': 95},
            'fps': {'warning': 15, 'critical': 10},  # Low FPS is bad
            'latency_ms': {'warning': 50, 'critical': 100}
        }

        # Alert state tracking
        self.alert_states = {}
        self.alert_history = deque(maxlen=100)

        # Subscribe to metrics
        self.cpu_subscription = self.create_subscription(
            Float32,
            '/system/cpu_usage',
            lambda msg: self.check_threshold('cpu_usage', msg.data),
            10
        )

        self.memory_subscription = self.create_subscription(
            Float32,
            '/system/memory_usage',
            lambda msg: self.check_threshold('memory_usage', msg.data),
            10
        )

        self.gpu_subscription = self.create_subscription(
            Float32,
            '/system/gpu_usage',
            lambda msg: self.check_threshold('gpu_usage', msg.data),
            10
        )

        self.fps_subscription = self.create_subscription(
            Float32,
            '/model/fps',
            lambda msg: self.check_threshold('fps', msg.data),
            10
        )

        self.latency_subscription = self.create_subscription(
            Float32,
            '/model/latency',
            lambda msg: self.check_threshold('latency_ms', msg.data * 1000),
            10
        )

        # Publisher for alerts
        self.alert_publisher = self.create_publisher(String, '/system/alerts', 10)

        self.get_logger().info('Performance Alert System initialized')

    def check_threshold(self, metric_name, value):
        """Check if metric exceeds thresholds"""
        try:
            thresholds = self.thresholds[metric_name]

            # Determine alert level
            alert_level = None
            if metric_name in ['fps']:  # Lower is worse
                if value <= thresholds['critical']:
                    alert_level = 'CRITICAL'
                elif value <= thresholds['warning']:
                    alert_level = 'WARNING'
            else:  # Higher is worse
                if value >= thresholds['critical']:
                    alert_level = 'CRITICAL'
                elif value >= thresholds['warning']:
                    alert_level = 'WARNING'

            # Publish alert if threshold exceeded
            if alert_level:
                alert_msg = String()
                alert_msg.data = f"{alert_level}: {metric_name} = {value:.2f} (threshold: {thresholds.get('critical' if alert_level == 'CRITICAL' else 'warning')})"
                self.alert_publisher.publish(alert_msg)

                # Store in history
                alert_entry = {
                    'timestamp': time.time(),
                    'metric': metric_name,
                    'value': value,
                    'threshold': thresholds.get('critical' if alert_level == 'CRITICAL' else 'warning'),
                    'level': alert_level
                }
                self.alert_history.append(alert_entry)

                self.get_logger().warn(f"{alert_level} Alert: {alert_msg.data}")
            else:
                # Clear alert state if below warning
                if metric_name in self.alert_states:
                    del self.alert_states[metric_name]

        except KeyError:
            self.get_logger().error(f'Unknown metric: {metric_name}')

    def get_alert_summary(self):
        """Get summary of recent alerts"""
        if not self.alert_history:
            return "No alerts triggered"

        recent_alerts = list(self.alert_history)[-10:]  # Last 10 alerts
        summary = f"Recent alerts: {len(recent_alerts)} total, "
        summary += f"CRITICAL: {sum(1 for a in recent_alerts if a['level'] == 'CRITICAL')}, "
        summary += f"WARNING: {sum(1 for a in recent_alerts if a['level'] == 'WARNING')}"

        return summary


def main(args=None):
    rclpy.init(args=args)
    alerts = PerformanceAlerts()

    try:
        rclpy.spin(alerts)
    except KeyboardInterrupt:
        summary = alerts.get_alert_summary()
        print(f"Alert Summary: {summary}")
    finally:
        alerts.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Testing and Validation

### 1. Creating Performance Monitoring Test Script

```bash
cat > ~/test_performance_monitoring.sh << 'EOF'
#!/bin/bash

# Test script for performance monitoring tools

echo "Testing Performance Monitoring Tools..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if performance monitoring package is available
echo "Checking performance monitoring package..."
ros2 pkg list | grep performance_monitor

if [ $? -eq 0 ]; then
    echo "✓ Performance monitoring package found"
else
    echo "✗ Performance monitoring package not found"
    exit 1
fi

# Check for performance monitoring launch files
if [ -f "/workspace/shared_dir/src/isaac_performance_monitor/launch/performance_monitoring.launch.py" ]; then
    echo "✓ Performance monitoring launch file found"
else
    echo "✗ Performance monitoring launch file not found"
    exit 1
fi

# Build the performance monitoring package
echo "Building performance monitoring package..."
cd ~/isaac_ros_ws
colcon build --packages-select isaac_performance_monitor
source install/setup.bash

# Check if required Python packages are available
python3 -c "import psutil; import GPUtil; import matplotlib; print('✓ Required Python packages available')" || {
    echo "✗ Required Python packages not available"
    echo "Installing required packages..."
    pip3 install psutil GPUtil matplotlib numpy
}

# Test performance logging directory
if [ -d "/workspace/shared_dir/logs" ]; then
    echo "✓ Logs directory exists"
else
    mkdir -p /workspace/shared_dir/logs
    echo "✓ Created logs directory"
fi

echo "Performance Monitoring test completed."
echo "To run performance monitoring, use:"
echo "ros2 launch isaac_performance_monitor performance_monitoring.launch.py"
EOF

# Make executable
chmod +x ~/test_performance_monitoring.sh
```

### 2. Running Performance Monitoring Test

```bash
~/test_performance_monitoring.sh
```

## Troubleshooting Performance Monitoring

### Common Issues and Solutions

#### Issue: "Performance monitoring nodes consuming too many resources"
**Solution**: Adjust monitoring frequency
```bash
# Check current monitoring frequency
ros2 topic hz /system/cpu_usage
ros2 topic hz /model/fps

# Adjust timer intervals in the monitoring nodes
# Modify the timer periods in the source code
```

#### Issue: "No data being published to performance topics"
**Solution**: Verify that monitored systems are running
```bash
# Check if monitored topics exist
ros2 topic list | grep -E "(cpu_usage|memory_usage|fps|latency)"

# Verify Isaac Sim is publishing sensor data
ros2 topic echo /camera/color/image_raw --field height --field width
```

#### Issue: "Python packages not found for monitoring"
**Solution**: Install required packages
```bash
pip3 install psutil GPUtil matplotlib numpy
```

## Verification Checklist

- [ ] System resource monitoring node created
- [ ] AI model performance monitoring implemented
- [ ] Navigation performance monitoring created
- [ ] Communication performance monitoring implemented
- [ ] Performance dashboard visualization created
- [ ] Performance logging system implemented
- [ ] Performance alert system created
- [ ] Launch files created for monitoring system
- [ ] Test script created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After implementing performance monitoring:

1. **Test monitoring system** with Isaac Sim running
2. **Analyze performance data** and identify bottlenecks
3. **Set up alerts** for critical performance thresholds
4. **Create performance assessment tools** for students

The performance monitoring framework is now configured and ready for Module 3, providing students with tools to monitor and optimize AI-robot system performance.