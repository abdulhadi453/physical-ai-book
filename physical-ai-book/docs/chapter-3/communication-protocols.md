# AI-Robot Communication Protocols for Isaac Sim

## Overview

This document provides comprehensive guidelines for implementing AI-robot communication protocols in the NVIDIA Isaac Sim environment for Module 3. The focus is on establishing efficient, reliable, and secure communication between AI systems and robotic platforms using the Isaac ROS bridge.

## Introduction to AI-Robot Communication

### What is AI-Robot Communication?
AI-robot communication involves the exchange of data and commands between artificial intelligence systems and robotic platforms. In the Isaac Sim context, this includes:

- **Perception Data**: Camera images, LiDAR scans, sensor readings
- **Control Commands**: Movement commands, actuator controls
- **State Information**: Robot pose, sensor status, system health
- **AI Decisions**: Navigation goals, action plans, behavioral commands

### Key Communication Principles
1. **Reliability**: Ensuring data integrity and delivery
2. **Timeliness**: Meeting real-time performance requirements
3. **Efficiency**: Optimizing bandwidth and processing resources
4. **Security**: Protecting communication channels
5. **Scalability**: Supporting multiple robots and AI agents

## Isaac ROS Communication Architecture

### 1. Creating Communication Protocol Configuration

```bash
cat > ~/isaac_sim_shared/configs/communication_config.yaml << 'EOF
# AI-Robot Communication Protocol Configuration

communication:
  # General communication settings
  qos_profiles:
    sensor_data:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10
    control_commands:
      reliability: reliable
      durability: transient_local
      history: keep_last
      depth: 1
    status_updates:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 5

  # Topic configuration
  topics:
    perception:
      camera:
        topic: "/camera/color/image_raw"
        qos: "sensor_data"
        frequency: 30.0  # Hz
        compression: false
      depth:
        topic: "/camera/depth/image_rect_raw"
        qos: "sensor_data"
        frequency: 30.0
        compression: false
      lidar:
        topic: "/scan"
        qos: "sensor_data"
        frequency: 10.0
        compression: false
      imu:
        topic: "/imu"
        qos: "sensor_data"
        frequency: 100.0
        compression: false

    control:
      velocity:
        topic: "/cmd_vel"
        qos: "control_commands"
        frequency: 50.0
        priority: high
      joints:
        topic: "/joint_commands"
        qos: "control_commands"
        frequency: 100.0
        priority: high

    status:
      odometry:
        topic: "/odom"
        qos: "status_updates"
        frequency: 50.0
      battery:
        topic: "/battery_state"
        qos: "status_updates"
        frequency: 1.0
      tf:
        topic: "/tf"
        qos: "status_updates"
        frequency: 50.0

  # Communication optimization
  optimization:
    enable_compression: true
    compression_level: 1
    message_coalescing: true
    bandwidth_limit: 100.0  # Mbps
    message_buffer_size: 1024  # bytes

  # Security settings
  security:
    enable_encryption: false  # For simulation environment
    enable_authentication: false  # For simulation environment
    encryption_algorithm: "AES-256"
    authentication_method: "RSA-2048"

  # Performance monitoring
  monitoring:
    enable_profiling: true
    profile_output: "/workspace/shared_dir/logs/communication_profile.json"
    enable_bandwidth_monitoring: true
    enable_latency_monitoring: true
    enable_packet_loss_monitoring: true

  # Error handling and recovery
  error_handling:
    max_retries: 3
    retry_delay: 0.1  # seconds
    timeout: 1.0  # seconds
    heartbeat_interval: 5.0  # seconds
    failure_threshold: 5  # consecutive failures before recovery
EOF
```

### 2. Creating Communication Protocol Node

```bash
cat > ~/isaac_ros_ws/src/isaac_communication_protocol/communication_protocol/protocol_handler.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan, Imu, BatteryState
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float32
from builtin_interfaces.msg import Time
import time
import threading
from collections import deque, defaultdict
from typing import Dict, Any, Optional
import json

class CommunicationProtocolHandler(Node):
    def __init__(self):
        super().__init__('communication_protocol_handler')

        # Protocol configuration
        self.protocol_config = {
            'heartbeat_interval': 1.0,
            'message_timeout': 5.0,
            'retry_limit': 3,
            'priority_levels': {
                'high': 100,
                'normal': 50,
                'low': 10
            }
        }

        # Message queues for different priority levels
        self.message_queues = {
            'high': deque(maxlen=50),
            'normal': deque(maxlen=100),
            'low': deque(maxlen=200)
        }

        # Message statistics tracking
        self.message_stats = defaultdict(lambda: {
            'sent': 0,
            'received': 0,
            'errors': 0,
            'latency_history': deque(maxlen=100),
            'bandwidth_history': deque(maxlen=100)
        })

        # Communication state
        self.connection_status = True
        self.heartbeat_timestamp = None
        self.sequence_numbers = defaultdict(int)

        # QoS profiles for different message types
        self.qos_profiles = {
            'sensor_data': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
            'control_commands': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ),
            'status_updates': QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
        }

        # Subscribers for incoming messages
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.handle_camera_message,
            self.qos_profiles['sensor_data']
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.handle_lidar_message,
            self.qos_profiles['sensor_data']
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.handle_imu_message,
            self.qos_profiles['sensor_data']
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom_message,
            self.qos_profiles['status_updates']
        )

        # Publishers for outgoing messages
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            self.qos_profiles['control_commands']
        )

        self.status_publisher = self.create_publisher(
            String,
            '/communication/status',
            self.qos_profiles['status_updates']
        )

        self.heartbeat_publisher = self.create_publisher(
            Time,
            '/communication/heartbeat',
            self.qos_profiles['status_updates']
        )

        # Timers for protocol management
        self.heartbeat_timer = self.create_timer(
            self.protocol_config['heartbeat_interval'],
            self.send_heartbeat
        )

        self.stats_timer = self.create_timer(
            2.0,  # Update stats every 2 seconds
            self.publish_statistics
        )

        # Start protocol handler
        self.protocol_thread = threading.Thread(target=self.process_messages)
        self.protocol_thread.daemon = True
        self.protocol_thread.start()

        self.get_logger().info('Communication Protocol Handler initialized')

    def handle_camera_message(self, msg):
        """Handle incoming camera messages"""
        self.track_message('camera', 'received')

        # Process camera message with protocol handling
        processed_msg = self.apply_protocol_rules(msg, 'camera')

        # Add to appropriate queue based on priority
        self.enqueue_message(processed_msg, 'normal', 'camera')

    def handle_lidar_message(self, msg):
        """Handle incoming LiDAR messages"""
        self.track_message('lidar', 'received')

        # Process LiDAR message with protocol handling
        processed_msg = self.apply_protocol_rules(msg, 'lidar')

        # Add to appropriate queue based on priority
        self.enqueue_message(processed_msg, 'high', 'lidar')

    def handle_imu_message(self, msg):
        """Handle incoming IMU messages"""
        self.track_message('imu', 'received')

        # Process IMU message with protocol handling
        processed_msg = self.apply_protocol_rules(msg, 'imu')

        # Add to appropriate queue based on priority
        self.enqueue_message(processed_msg, 'high', 'imu')

    def handle_odom_message(self, msg):
        """Handle incoming odometry messages"""
        self.track_message('odometry', 'received')

        # Process odometry message with protocol handling
        processed_msg = self.apply_protocol_rules(msg, 'odometry')

        # Add to appropriate queue based on priority
        self.enqueue_message(processed_msg, 'normal', 'odometry')

    def apply_protocol_rules(self, msg, msg_type):
        """Apply communication protocol rules to messages"""
        # Add sequence number
        seq_num = self.sequence_numbers[msg_type]
        self.sequence_numbers[msg_type] += 1

        # Add timestamp if not present
        if not hasattr(msg.header, 'stamp') or msg.header.stamp.sec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        # Apply message validation rules
        validated_msg = self.validate_message(msg, msg_type)

        return validated_msg

    def validate_message(self, msg, msg_type):
        """Validate message according to protocol"""
        # Basic validation - in practice, this would be more extensive
        if msg_type == 'lidar':
            # Validate LiDAR ranges
            if hasattr(msg, 'ranges') and msg.ranges:
                valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
                if len(valid_ranges) != len(msg.ranges):
                    self.get_logger().warn(f'Invalid LiDAR ranges detected in message')

        return msg

    def enqueue_message(self, msg, priority, msg_type):
        """Add message to appropriate priority queue"""
        queue_item = {
            'message': msg,
            'priority': priority,
            'type': msg_type,
            'timestamp': time.time(),
            'sequence': self.sequence_numbers[msg_type]
        }

        self.message_queues[priority].append(queue_item)

    def send_heartbeat(self):
        """Send periodic heartbeat to maintain connection"""
        try:
            heartbeat_msg = Time()
            heartbeat_msg.sec = int(time.time())
            heartbeat_msg.nanosec = int((time.time() % 1) * 1e9)

            self.heartbeat_publisher.publish(heartbeat_msg)
            self.heartbeat_timestamp = time.time()

            self.get_logger().debug(f'Sent heartbeat at {heartbeat_msg.sec}.{heartbeat_msg.nanosec}')
        except Exception as e:
            self.get_logger().error(f'Error sending heartbeat: {e}')

    def process_messages(self):
        """Process messages in priority order"""
        while rclpy.ok():
            try:
                # Process high priority messages first
                self.process_queue('high')
                self.process_queue('normal')
                self.process_queue('low')

                # Small delay to prevent busy waiting
                time.sleep(0.001)

            except Exception as e:
                self.get_logger().error(f'Error processing messages: {e}')

    def process_queue(self, priority):
        """Process messages in a specific priority queue"""
        while len(self.message_queues[priority]) > 0:
            try:
                queue_item = self.message_queues[priority].popleft()

                # Process the message based on its type
                self.process_specific_message(queue_item)

            except IndexError:
                # Queue is empty
                break
            except Exception as e:
                self.get_logger().error(f'Error processing queue {priority}: {e}')

    def process_specific_message(self, queue_item):
        """Process a specific message based on its type"""
        msg = queue_item['message']
        msg_type = queue_item['type']

        # Handle different message types
        if msg_type.startswith('camera'):
            # Process camera data (e.g., send to perception system)
            self.process_camera_data(msg)
        elif msg_type.startswith('lidar'):
            # Process LiDAR data (e.g., send to navigation system)
            self.process_lidar_data(msg)
        elif msg_type.startswith('imu'):
            # Process IMU data (e.g., send to state estimator)
            self.process_imu_data(msg)
        elif msg_type.startswith('odometry'):
            # Process odometry data (e.g., send to localization system)
            self.process_odom_data(msg)

    def process_camera_data(self, msg):
        """Process camera data and send to perception system"""
        # In a real implementation, this would send to perception system
        # For simulation, just log the processing
        self.get_logger().debug(f'Processing camera data: {msg.width}x{msg.height}')

    def process_lidar_data(self, msg):
        """Process LiDAR data and send to navigation system"""
        # In a real implementation, this would send to navigation system
        self.get_logger().debug(f'Processing LiDAR data: {len(msg.ranges)} ranges')

    def process_imu_data(self, msg):
        """Process IMU data and send to state estimator"""
        # In a real implementation, this would send to state estimation
        self.get_logger().debug(f'Processing IMU data: {msg.linear_acceleration}')

    def process_odom_data(self, msg):
        """Process odometry data and send to localization system"""
        # In a real implementation, this would send to localization
        self.get_logger().debug(f'Processing odometry data: {msg.pose.pose.position}')

    def send_command(self, command_msg, priority='normal'):
        """Send a command with specified priority"""
        try:
            # Add protocol headers
            command_msg.header.stamp = self.get_clock().now().to_msg()

            # Track sent message
            self.track_message('command', 'sent')

            # Publish command
            self.cmd_vel_publisher.publish(command_msg)

            self.get_logger().debug(f'Sent command with priority: {priority}')

        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')
            self.track_message('command', 'error')

    def track_message(self, msg_type, event):
        """Track message statistics"""
        if event == 'sent':
            self.message_stats[msg_type]['sent'] += 1
        elif event == 'received':
            self.message_stats[msg_type]['received'] += 1
        elif event == 'error':
            self.message_stats[msg_type]['errors'] += 1

    def publish_statistics(self):
        """Publish communication statistics"""
        try:
            # Calculate statistics
            stats_msg = String()
            stats_data = {
                'connection_status': self.connection_status,
                'messages_sent': sum(stat['sent'] for stat in self.message_stats.values()),
                'messages_received': sum(stat['received'] for stat in self.message_stats.values()),
                'errors': sum(stat['errors'] for stat in self.message_stats.values()),
                'queues': {
                    'high': len(self.message_queues['high']),
                    'normal': len(self.message_queues['normal']),
                    'low': len(self.message_queues['low'])
                }
            }

            stats_msg.data = json.dumps(stats_data)
            self.status_publisher.publish(stats_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing statistics: {e}')

    def get_communication_summary(self):
        """Get communication performance summary"""
        return {
            'connection_status': self.connection_status,
            'message_stats': dict(self.message_stats),
            'queue_sizes': {
                'high': len(self.message_queues['high']),
                'normal': len(self.message_queues['normal']),
                'low': len(self.message_queues['low'])
            },
            'sequence_numbers': dict(self.sequence_numbers),
            'heartbeat_timestamp': self.heartbeat_timestamp
        }


def main(args=None):
    rclpy.init(args=args)
    protocol_handler = CommunicationProtocolHandler()

    try:
        rclpy.spin(protocol_handler)
    except KeyboardInterrupt:
        summary = protocol_handler.get_communication_summary()
        print(f"Communication Summary: {summary}")
    finally:
        protocol_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Message Serialization and Compression

### 1. Creating Message Serialization Utilities

```bash
cat > ~/isaac_ros_ws/src/isaac_communication_protocol/communication_protocol/message_serializer.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import cv2
import pickle
import gzip
import json
from typing import Any, Dict
import struct

class MessageSerializer:
    def __init__(self):
        self.compression_level = 6  # Default compression level

    def serialize_image(self, image_msg) -> bytes:
        """Serialize Image message with optional compression"""
        # Convert ROS Image to numpy array
        if image_msg.encoding == 'rgb8' or image_msg.encoding == 'bgr8':
            dtype = np.uint8
        elif image_msg.encoding == 'rgba8':
            dtype = np.uint8
        elif image_msg.encoding == 'mono8':
            dtype = np.uint8
        else:
            raise ValueError(f"Unsupported image encoding: {image_msg.encoding}")

        # Reshape image data
        height, width = image_msg.height, image_msg.width
        channels = 1 if 'mono' in image_msg.encoding else 3
        image_array = np.frombuffer(image_msg.data, dtype=dtype).reshape((height, width, channels))

        # Serialize image metadata
        metadata = {
            'width': width,
            'height': height,
            'encoding': image_msg.encoding,
            'step': image_msg.step,
            'header': {
                'stamp': {
                    'sec': image_msg.header.stamp.sec,
                    'nanosec': image_msg.header.stamp.nanosec
                },
                'frame_id': image_msg.header.frame_id
            }
        }

        # Pack image and metadata
        packed_data = {
            'metadata': metadata,
            'image_data': image_array
        }

        # Serialize to bytes
        serialized = pickle.dumps(packed_data)

        # Compress if requested
        compressed = gzip.compress(serialized, compresslevel=self.compression_level)

        return compressed

    def deserialize_image(self, serialized_data: bytes) -> object:
        """Deserialize Image message from compressed bytes"""
        # Decompress data
        decompressed = gzip.decompress(serialized_data)

        # Deserialize
        packed_data = pickle.loads(decompressed)

        # Extract metadata and image data
        metadata = packed_data['metadata']
        image_data = packed_data['image_data']

        # Convert back to ROS Image message
        from sensor_msgs.msg import Image
        image_msg = Image()
        image_msg.width = metadata['width']
        image_msg.height = metadata['height']
        image_msg.encoding = metadata['encoding']
        image_msg.step = metadata['step']
        image_msg.header.stamp.sec = metadata['header']['stamp']['sec']
        image_msg.header.stamp.nanosec = metadata['header']['stamp']['nanosec']
        image_msg.header.frame_id = metadata['header']['frame_id']

        # Convert image data back to bytes
        image_bytes = image_data.tobytes()
        image_msg.data = image_bytes

        return image_msg

    def serialize_laserscan(self, scan_msg) -> bytes:
        """Serialize LaserScan message with optional compression"""
        # Create dictionary with scan data
        scan_data = {
            'header': {
                'stamp': {
                    'sec': scan_msg.header.stamp.sec,
                    'nanosec': scan_msg.header.stamp.nanosec
                },
                'frame_id': scan_msg.header.frame_id
            },
            'angle_min': scan_msg.angle_min,
            'angle_max': scan_msg.angle_max,
            'angle_increment': scan_msg.angle_increment,
            'time_increment': scan_msg.time_increment,
            'scan_time': scan_msg.scan_time,
            'range_min': scan_msg.range_min,
            'range_max': scan_msg.range_max,
            'ranges': np.array(scan_msg.ranges, dtype=np.float32),
            'intensities': np.array(scan_msg.intensities, dtype=np.float32) if scan_msg.intensities else []
        }

        # Serialize to bytes
        serialized = pickle.dumps(scan_data)

        # Compress if requested
        compressed = gzip.compress(serialized, compresslevel=self.compression_level)

        return compressed

    def deserialize_laserscan(self, serialized_data: bytes) -> object:
        """Deserialize LaserScan message from compressed bytes"""
        # Decompress data
        decompressed = gzip.decompress(serialized_data)

        # Deserialize
        scan_data = pickle.loads(decompressed)

        # Convert back to ROS LaserScan message
        from sensor_msgs.msg import LaserScan
        scan_msg = LaserScan()
        scan_msg.header.stamp.sec = scan_data['header']['stamp']['sec']
        scan_msg.header.stamp.nanosec = scan_data['header']['stamp']['nanosec']
        scan_msg.header.frame_id = scan_data['header']['frame_id']
        scan_msg.angle_min = scan_data['angle_min']
        scan_msg.angle_max = scan_data['angle_max']
        scan_msg.angle_increment = scan_data['angle_increment']
        scan_msg.time_increment = scan_data['time_increment']
        scan_msg.scan_time = scan_data['scan_time']
        scan_msg.range_min = scan_data['range_min']
        scan_msg.range_max = scan_data['range_max']
        scan_msg.ranges = scan_data['ranges'].tolist()
        scan_msg.intensities = scan_data['intensities'].tolist() if scan_data['intensities'] else []

        return scan_msg

    def serialize_twist(self, twist_msg) -> bytes:
        """Serialize Twist message"""
        twist_data = {
            'linear': {
                'x': twist_msg.linear.x,
                'y': twist_msg.linear.y,
                'z': twist_msg.linear.z
            },
            'angular': {
                'x': twist_msg.angular.x,
                'y': twist_msg.angular.y,
                'z': twist_msg.angular.z
            }
        }

        return json.dumps(twist_data).encode('utf-8')

    def deserialize_twist(self, serialized_data: bytes) -> object:
        """Deserialize Twist message"""
        from geometry_msgs.msg import Twist
        twist_data = json.loads(serialized_data.decode('utf-8'))

        twist_msg = Twist()
        twist_msg.linear.x = twist_data['linear']['x']
        twist_msg.linear.y = twist_data['linear']['y']
        twist_msg.linear.z = twist_data['linear']['z']
        twist_msg.angular.x = twist_data['angular']['x']
        twist_msg.angular.y = twist_data['angular']['y']
        twist_msg.angular.z = twist_data['angular']['z']

        return twist_msg

    def calculate_compression_ratio(self, original_size: int, compressed_size: int) -> float:
        """Calculate compression ratio"""
        if original_size == 0:
            return 0.0
        return (original_size - compressed_size) / original_size * 100


class MessageCompressionNode(Node):
    def __init__(self):
        super().__init__('message_compression_node')

        # Initialize serializer
        self.serializer = MessageSerializer()

        # Publishers and subscribers for testing
        self.compressed_image_publisher = self.create_publisher(
            String,  # Using String to send compressed data
            '/compressed/image',
            10
        )

        self.compressed_scan_publisher = self.create_publisher(
            String,  # Using String to send compressed data
            '/compressed/scan',
            10
        )

        # Subscribers for original data
        self.original_image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_compress_callback,
            10
        )

        self.original_scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_compress_callback,
            10
        )

        self.get_logger().info('Message Compression Node initialized')

    def image_compress_callback(self, msg):
        """Compress and publish image message"""
        try:
            # Serialize and compress image
            compressed_data = self.serializer.serialize_image(msg)

            # Calculate compression statistics
            original_size = len(msg.data)
            compressed_size = len(compressed_data)
            ratio = self.serializer.calculate_compression_ratio(original_size, compressed_size)

            self.get_logger().debug(f'Image compression: {ratio:.2f}% reduction')

            # Publish compressed data as string
            compressed_msg = String()
            compressed_msg.data = compressed_data.hex()  # Convert to hex for transmission
            self.compressed_image_publisher.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error compressing image: {e}')

    def scan_compress_callback(self, msg):
        """Compress and publish scan message"""
        try:
            # Serialize and compress scan
            compressed_data = self.serializer.serialize_laserscan(msg)

            # Calculate compression statistics
            original_size = len(pickle.dumps(msg))
            compressed_size = len(compressed_data)
            ratio = self.serializer.calculate_compression_ratio(original_size, compressed_size)

            self.get_logger().debug(f'Scan compression: {ratio:.2f}% reduction')

            # Publish compressed data as string
            compressed_msg = String()
            compressed_msg.data = compressed_data.hex()  # Convert to hex for transmission
            self.compressed_scan_publisher.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error compressing scan: {e}')


def main(args=None):
    rclpy.init(args=args)
    compression_node = MessageCompressionNode()

    try:
        rclpy.spin(compression_node)
    except KeyboardInterrupt:
        pass
    finally:
        compression_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Communication Middleware Integration

### 1. Creating Communication Middleware

```bash
cat > ~/isaac_ros_ws/src/isaac_communication_protocol/communication_protocol/middleware.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import threading
import queue
import time
from typing import Dict, Any, Callable, Optional
from collections import defaultdict
import json

class CommunicationMiddleware(Node):
    def __init__(self):
        super().__init__('communication_middleware')

        # Middleware configuration
        self.middleware_config = {
            'max_queue_size': 1000,
            'processing_threads': 4,
            'message_timeout': 5.0,
            'retry_attempts': 3,
            'retry_delay': 0.1
        }

        # Message queues for different types
        self.input_queues = defaultdict(queue.Queue)
        self.output_queues = defaultdict(queue.Queue)

        # Message handlers registry
        self.message_handlers = {}
        self.topic_subscriptions = {}

        # Statistics tracking
        self.stats = {
            'messages_processed': 0,
            'messages_dropped': 0,
            'average_latency': 0.0,
            'throughput': 0.0
        }

        # Processing threads
        self.processing_threads = []
        self.running = True

        # Start processing threads
        for i in range(self.middleware_config['processing_threads']):
            thread = threading.Thread(target=self.process_messages, args=(i,))
            thread.daemon = True
            thread.start()
            self.processing_threads.append(thread)

        # Publishers for statistics
        self.stats_publisher = self.create_publisher(
            String,
            '/middleware/stats',
            10
        )

        # Timer for publishing statistics
        self.stats_timer = self.create_timer(1.0, self.publish_stats)

        self.get_logger().info('Communication Middleware initialized')

    def register_handler(self, topic_type: str, handler_func: Callable):
        """Register a message handler for a specific topic type"""
        self.message_handlers[topic_type] = handler_func
        self.get_logger().info(f'Registered handler for {topic_type}')

    def subscribe(self, topic: str, msg_type, callback: Callable, qos_profile=None):
        """Subscribe to a topic through middleware"""
        if qos_profile is None:
            from rclpy.qos import QoSProfile
            qos_profile = QoSProfile(depth=10)

        subscription = self.create_subscription(
            msg_type,
            topic,
            lambda msg: self.enqueue_message(topic, msg),
            qos_profile
        )

        self.topic_subscriptions[topic] = subscription
        return subscription

    def enqueue_message(self, topic: str, msg):
        """Enqueue a message for processing"""
        try:
            message_wrapper = {
                'topic': topic,
                'message': msg,
                'timestamp': time.time(),
                'sequence_id': self.generate_sequence_id()
            }

            # Add to input queue
            if self.input_queues[topic].qsize() < self.middleware_config['max_queue_size']:
                self.input_queues[topic].put(message_wrapper)
                self.stats['messages_processed'] += 1
            else:
                self.stats['messages_dropped'] += 1
                self.get_logger().warn(f'Message queue full for topic {topic}, dropping message')

        except Exception as e:
            self.get_logger().error(f'Error enqueuing message: {e}')

    def process_messages(self, thread_id: int):
        """Process messages in a dedicated thread"""
        while self.running:
            try:
                # Process messages from all queues
                for topic, q in self.input_queues.items():
                    try:
                        if not q.empty():
                            message_wrapper = q.get_nowait()

                            # Process the message
                            self.process_message(message_wrapper)

                    except queue.Empty:
                        # No messages in this queue, continue to next
                        continue
                    except Exception as e:
                        self.get_logger().error(f'Error processing message in thread {thread_id}: {e}')

                # Small delay to prevent busy waiting
                time.sleep(0.001)

            except Exception as e:
                self.get_logger().error(f'Error in processing thread {thread_id}: {e}')

    def process_message(self, message_wrapper: Dict):
        """Process a single message"""
        topic = message_wrapper['topic']
        msg = message_wrapper['message']
        timestamp = message_wrapper['timestamp']

        # Determine message type and call appropriate handler
        msg_type = type(msg).__name__.lower()

        if msg_type in self.message_handlers:
            try:
                # Call the registered handler
                result = self.message_handlers[msg_type](msg, topic)

                # Calculate latency
                latency = time.time() - timestamp
                self.update_latency_stats(latency)

                # Log processing result
                self.get_logger().debug(f'Processed {msg_type} message on {topic}, latency: {latency:.4f}s')

            except Exception as e:
                self.get_logger().error(f'Error in message handler for {msg_type}: {e}')
        else:
            self.get_logger().warn(f'No handler registered for message type: {msg_type}')

    def generate_sequence_id(self) -> int:
        """Generate a sequence ID for message ordering"""
        if not hasattr(self, '_sequence_counter'):
            self._sequence_counter = 0
        self._sequence_counter += 1
        return self._sequence_counter

    def update_latency_stats(self, latency: float):
        """Update latency statistics"""
        self.stats['average_latency'] = (
            self.stats['average_latency'] * 0.9 + latency * 0.1
        )  # Exponential moving average

    def publish_stats(self):
        """Publish middleware statistics"""
        try:
            stats_msg = String()
            stats_msg.data = json.dumps(self.stats)
            self.stats_publisher.publish(stats_msg)

            self.get_logger().debug(f'Middleware stats: {self.stats}')

        except Exception as e:
            self.get_logger().error(f'Error publishing stats: {e}')

    def add_topic_filter(self, topic: str, filter_func: Callable):
        """Add a filter function for a specific topic"""
        # In a real implementation, this would apply filters before processing
        pass

    def get_middleware_status(self) -> Dict:
        """Get middleware operational status"""
        queue_sizes = {topic: q.qsize() for topic, q in self.input_queues.items()}

        return {
            'stats': self.stats,
            'queue_sizes': queue_sizes,
            'handlers_registered': list(self.message_handlers.keys()),
            'subscriptions_active': len(self.topic_subscriptions),
            'threads_running': len(self.processing_threads)
        }

    def shutdown(self):
        """Gracefully shut down middleware"""
        self.running = False

        # Wait for threads to finish
        for thread in self.processing_threads:
            thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    middleware = CommunicationMiddleware()

    try:
        rclpy.spin(middleware)
    except KeyboardInterrupt:
        status = middleware.get_middleware_status()
        print(f"Middleware Status: {status}")
        middleware.shutdown()
    finally:
        middleware.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Communication Protocol Launch Files

### 1. Creating Communication Protocol Package

```bash
cd ~/isaac_ros_ws/src
source /opt/ros/humble/setup.bash

# Create communication protocol package
ros2 pkg create --build-type ament_python isaac_communication_protocol --dependencies rclpy sensor_msgs geometry_msgs std_msgs message_filters tf2_ros tf2_geometry_msgs nav_msgs vision_msgs builtin_interfaces
```

### 2. Creating Setup Files for Communication Package

```bash
cat > ~/isaac_ros_ws/src/isaac_communication_protocol/setup.py << 'EOF
from setuptools import setup
from glob import glob
import os

package_name = 'isaac_communication_protocol'

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
    description='Communication protocols for Isaac Sim AI-Robot systems',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'protocol_handler = isaac_communication_protocol.protocol_handler:main',
            'message_compression = isaac_communication_protocol.message_serializer:main',
            'communication_middleware = isaac_communication_protocol.middleware:main',
        ],
    },
)
EOF
```

```bash
cat > ~/isaac_ros_ws/src/isaac_communication_protocol/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_communication_protocol</name>
  <version>0.0.1</version>
  <description>Communication protocols for Isaac Sim AI-Robot systems</description>
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

### 3. Creating Communication Protocol Launch File

```bash
mkdir -p ~/isaac_ros_ws/src/isaac_communication_protocol/launch
cat > ~/isaac_ros_ws/src/isaac_communication_protocol/launch/communication_protocol.launch.py << 'EOF
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_compression = LaunchConfiguration('enable_compression', default='true')
    enable_middleware = LaunchConfiguration('enable_middleware', default='true')

    # Communication protocol handler node
    protocol_handler = Node(
        package='isaac_communication_protocol',
        executable='protocol_handler',
        name='communication_protocol_handler',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Message compression node (optional)
    message_compression = Node(
        package='isaac_communication_protocol',
        executable='message_compression',
        name='message_compression_node',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen',
        condition=lambda context: LaunchConfiguration('enable_compression').perform(context) == 'true'
    )

    # Communication middleware node (optional)
    communication_middleware = Node(
        package='isaac_communication_protocol',
        executable='communication_middleware',
        name='communication_middleware',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen',
        condition=lambda context: LaunchConfiguration('enable_middleware').perform(context) == 'true'
    )

    # Return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'enable_compression',
        default_value='true',
        description='Enable message compression'))

    ld.add_action(DeclareLaunchArgument(
        'enable_middleware',
        default_value='true',
        description='Enable communication middleware'))

    # Add nodes with timing delays for proper initialization
    ld.add_action(protocol_handler)
    ld.add_action(TimerAction(
        period=1.0,
        actions=[message_compression]
    ))
    ld.add_action(TimerAction(
        period=2.0,
        actions=[communication_middleware]
    ))

    return ld
EOF
```

## Communication Performance Optimization

### 1. Creating Performance Optimization Tools

```bash
cat > ~/isaac_sim_shared/scripts/communication_optimizer.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import time
import threading
from collections import deque, defaultdict
from typing import Dict, List
import json

class CommunicationOptimizer(Node):
    def __init__(self):
        super().__init__('communication_optimizer')

        # Performance tracking
        self.performance_metrics = defaultdict(lambda: {
            'bandwidth_usage': deque(maxlen=100),
            'latency_history': deque(maxlen=100),
            'message_frequency': deque(maxlen=100),
            'compression_ratio': deque(maxlen=100)
        })

        # Optimization parameters
        self.optimization_params = {
            'target_bandwidth': 100.0,  # Mbps
            'max_latency': 0.1,         # seconds
            'min_compression_ratio': 0.1,  # 10% reduction
            'adaptive_frequency': True,
            'adaptive_compression': True
        }

        # Subscribers for performance monitoring
        self.bandwidth_subscriber = self.create_subscription(
            Float32,
            '/communication/bandwidth_usage',
            self.bandwidth_callback,
            10
        )

        self.latency_subscriber = self.create_subscription(
            Float32,
            '/communication/latency',
            self.latency_callback,
            10
        )

        self.compression_subscriber = self.create_subscription(
            Float32,
            '/communication/compression_ratio',
            self.compression_callback,
            10
        )

        # Publishers for optimization commands
        self.frequency_publisher = self.create_publisher(
            String,
            '/communication/frequency_control',
            10
        )

        self.compression_publisher = self.create_publisher(
            String,
            '/communication/compression_control',
            10
        )

        self.optimization_status_publisher = self.create_publisher(
            String,
            '/communication/optimization_status',
            10
        )

        # Timer for periodic optimization
        self.optimization_timer = self.create_timer(2.0, self.optimize_communication)

        self.get_logger().info('Communication Optimizer initialized')

    def bandwidth_callback(self, msg):
        """Track bandwidth usage"""
        self.performance_metrics['bandwidth_usage'].append(msg.data)

    def latency_callback(self, msg):
        """Track latency"""
        self.performance_metrics['latency_history'].append(msg.data)

    def compression_callback(self, msg):
        """Track compression ratio"""
        self.performance_metrics['compression_ratio'].append(msg.data)

    def optimize_communication(self):
        """Perform communication optimization"""
        try:
            # Calculate current performance metrics
            current_metrics = self.calculate_current_metrics()

            # Determine optimization actions
            optimization_actions = self.determine_optimizations(current_metrics)

            # Apply optimizations
            self.apply_optimizations(optimization_actions)

            # Publish optimization status
            status_msg = String()
            status_msg.data = json.dumps({
                'timestamp': time.time(),
                'current_metrics': current_metrics,
                'optimization_actions': optimization_actions
            })
            self.optimization_status_publisher.publish(status_msg)

            self.get_logger().info(f'Applied optimizations: {optimization_actions}')

        except Exception as e:
            self.get_logger().error(f'Error in communication optimization: {e}')

    def calculate_current_metrics(self) -> Dict:
        """Calculate current communication metrics"""
        metrics = {}

        # Bandwidth usage
        if self.performance_metrics['bandwidth_usage']:
            metrics['avg_bandwidth'] = sum(self.performance_metrics['bandwidth_usage']) / len(self.performance_metrics['bandwidth_usage'])
        else:
            metrics['avg_bandwidth'] = 0.0

        # Latency
        if self.performance_metrics['latency_history']:
            metrics['avg_latency'] = sum(self.performance_metrics['latency_history']) / len(self.performance_metrics['latency_history'])
        else:
            metrics['avg_latency'] = 0.0

        # Compression ratio
        if self.performance_metrics['compression_ratio']:
            metrics['avg_compression'] = sum(self.performance_metrics['compression_ratio']) / len(self.performance_metrics['compression_ratio'])
        else:
            metrics['avg_compression'] = 0.0

        return metrics

    def determine_optimizations(self, current_metrics: Dict) -> Dict:
        """Determine optimization actions based on current metrics"""
        actions = {}

        # Bandwidth optimization
        if current_metrics['avg_bandwidth'] > self.optimization_params['target_bandwidth']:
            actions['reduce_bandwidth'] = True
            if self.optimization_params['adaptive_frequency']:
                actions['reduce_frequency'] = True
            if self.optimization_params['adaptive_compression']:
                actions['increase_compression'] = True

        # Latency optimization
        if current_metrics['avg_latency'] > self.optimization_params['max_latency']:
            actions['reduce_latency'] = True
            if self.optimization_params['adaptive_frequency']:
                actions['increase_frequency'] = False  # Reduce frequency to reduce latency

        # Compression optimization
        if current_metrics['avg_compression'] < self.optimization_params['min_compression_ratio']:
            actions['improve_compression'] = True

        return actions

    def apply_optimizations(self, actions: Dict):
        """Apply optimization actions"""
        # Apply frequency control
        if 'reduce_frequency' in actions:
            freq_msg = String()
            freq_msg.data = 'reduce_frequency'
            self.frequency_publisher.publish(freq_msg)

        # Apply compression control
        if 'increase_compression' in actions:
            comp_msg = String()
            comp_msg.data = 'increase_compression_level'
            self.compression_publisher.publish(comp_msg)

    def get_optimization_summary(self) -> Dict:
        """Get optimization summary"""
        current_metrics = self.calculate_current_metrics()
        return {
            'current_metrics': current_metrics,
            'optimization_enabled': True,
            'adaptive_frequency': self.optimization_params['adaptive_frequency'],
            'adaptive_compression': self.optimization_params['adaptive_compression'],
            'last_optimization_time': time.time()
        }


def main(args=None):
    rclpy.init(args=args)
    optimizer = CommunicationOptimizer()

    try:
        rclpy.spin(optimizer)
    except KeyboardInterrupt:
        summary = optimizer.get_optimization_summary()
        print(f"Optimization Summary: {summary}")
    finally:
        optimizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Communication Testing and Validation

### 1. Creating Communication Test Suite

```bash
cat > ~/isaac_sim_shared/scripts/communication_tests.py << 'EOF
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import time
import threading
from typing import Dict, List
import numpy as np

class CommunicationTestSuite(Node):
    def __init__(self):
        super().__init__('communication_test_suite')

        # Test results
        self.test_results = {}
        self.test_running = False

        # Communication statistics
        self.comm_stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'latency_measurements': [],
            'bandwidth_measurements': [],
            'packet_loss_rate': 0.0
        }

        # Publishers for test messages
        self.image_publisher = self.create_publisher(Image, '/test/image', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/test/scan', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/test/cmd_vel', 10)

        # Subscribers for response messages
        self.response_subscriber = self.create_subscription(
            String,
            '/test/response',
            self.response_callback,
            10
        )

        # Test control publishers
        self.test_status_publisher = self.create_publisher(
            String,
            '/test/status',
            10
        )

        self.get_logger().info('Communication Test Suite initialized')

    def response_callback(self, msg):
        """Handle test response messages"""
        self.comm_stats['messages_received'] += 1

    def run_all_tests(self) -> Dict:
        """Run all communication tests"""
        self.get_logger().info('Starting communication tests...')

        results = {}

        # Test 1: Bandwidth capacity
        results['bandwidth'] = self.test_bandwidth_capacity()
        time.sleep(1)

        # Test 2: Latency measurement
        results['latency'] = self.test_latency()
        time.sleep(1)

        # Test 3: Message throughput
        results['throughput'] = self.test_message_throughput()
        time.sleep(1)

        # Test 4: Packet loss rate
        results['packet_loss'] = self.test_packet_loss_rate()
        time.sleep(1)

        # Test 5: Connection stability
        results['stability'] = self.test_connection_stability()
        time.sleep(1)

        self.get_logger().info(f'Communication tests completed: {results}')
        return results

    def test_bandwidth_capacity(self) -> Dict:
        """Test maximum bandwidth capacity"""
        self.get_logger().info('Testing bandwidth capacity...')

        start_time = time.time()
        messages_sent = 0

        # Send large messages to test bandwidth
        for i in range(100):  # Send 100 test messages
            # Create a large image message
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'test'
            img_msg.width = 640
            img_msg.height = 480
            img_msg.encoding = 'rgb8'
            img_msg.step = 640 * 3

            # Create large image data
            large_data = [i % 256 for _ in range(640 * 480 * 3)]  # RGB data
            img_msg.data = bytes(large_data)

            self.image_publisher.publish(img_msg)
            messages_sent += 1
            time.sleep(0.01)  # Small delay between messages

        end_time = time.time()
        duration = end_time - start_time
        total_bytes = messages_sent * len(img_msg.data)
        bandwidth_mbps = (total_bytes * 8) / (duration * 1e6)

        result = {
            'bandwidth_mbps': bandwidth_mbps,
            'messages_sent': messages_sent,
            'duration': duration,
            'passed': bandwidth_mbps > 1.0  # Require at least 1 Mbps
        }

        self.get_logger().info(f'BW test: {bandwidth_mbps:.2f} Mbps, Passed: {result["passed"]}')
        return result

    def test_latency(self) -> Dict:
        """Test message latency"""
        self.get_logger().info('Testing message latency...')

        latency_measurements = []

        # Send messages and measure round-trip time
        for i in range(10):
            send_time = time.time()

            # Send test message
            test_msg = String()
            test_msg.data = f'test_latency_{i}_{send_time}'
            self.test_status_publisher.publish(test_msg)

            # Wait for response (simplified - in reality would use timestamps)
            time.sleep(0.1)

            # Record latency
            recv_time = time.time()
            latency = recv_time - send_time
            latency_measurements.append(latency)

        avg_latency = sum(latency_measurements) / len(latency_measurements)
        max_latency = max(latency_measurements)

        result = {
            'avg_latency_ms': avg_latency * 1000,
            'max_latency_ms': max_latency * 1000,
            'measurements': len(latency_measurements),
            'passed': avg_latency < 0.1  # Require < 100ms average
        }

        self.get_logger().info(f'Latency test: avg={avg_latency*1000:.2f}ms, Passed: {result["passed"]}')
        return result

    def test_message_throughput(self) -> Dict:
        """Test message throughput"""
        self.get_logger().info('Testing message throughput...')

        start_time = time.time()
        messages_sent = 0

        # Send messages as fast as possible
        for i in range(1000):
            cmd_msg = Twist()
            cmd_msg.linear.x = float(i % 100) / 100.0
            cmd_msg.angular.z = float(i % 100) / 100.0

            self.cmd_publisher.publish(cmd_msg)
            messages_sent += 1

        end_time = time.time()
        duration = end_time - start_time
        throughput = messages_sent / duration

        result = {
            'throughput_hz': throughput,
            'messages_sent': messages_sent,
            'duration': duration,
            'passed': throughput > 50.0  # Require at least 50 Hz throughput
        }

        self.get_logger().info(f'Throughput test: {throughput:.2f} Hz, Passed: {result["passed"]}')
        return result

    def test_packet_loss_rate(self) -> Dict:
        """Test packet loss rate"""
        self.get_logger().info('Testing packet loss rate...')

        # Send numbered messages and check which ones arrive
        total_sent = 100
        expected_responses = set(range(total_sent))
        received_responses = set()

        for i in range(total_sent):
            test_msg = String()
            test_msg.data = f'ping_{i}'
            self.test_status_publisher.publish(test_msg)

            # In a real test, we'd wait for acknowledgment
            # For simulation, we'll just record what we expect
            if i % 2 == 0:  # Simulate 50% success rate for testing
                received_responses.add(i)

            time.sleep(0.01)

        lost_packets = total_sent - len(received_responses)
        loss_rate = lost_packets / total_sent

        result = {
            'total_sent': total_sent,
            'received': len(received_responses),
            'lost': lost_packets,
            'loss_rate': loss_rate,
            'passed': loss_rate < 0.1  # Require < 10% loss rate
        }

        self.get_logger().info(f'Loss test: {loss_rate*100:.2f}% loss, Passed: {result["passed"]}')
        return result

    def test_connection_stability(self) -> Dict:
        """Test connection stability over time"""
        self.get_logger().info('Testing connection stability...')

        # Simulate connection stability test
        # In reality, this would involve monitoring connection status over time
        stability_score = 0.95  # Simulated high stability

        result = {
            'stability_score': stability_score,
            'test_duration': 10.0,  # seconds
            'passed': stability_score > 0.9  # Require > 90% stability
        }

        self.get_logger().info(f'Stability test: {stability_score*100:.2f}%, Passed: {result["passed"]}')
        return result

    def get_test_summary(self) -> Dict:
        """Get summary of all communication tests"""
        return {
            'timestamp': time.time(),
            'total_tests': len(self.test_results),
            'passed_tests': sum(1 for result in self.test_results.values() if result.get('passed', False)),
            'test_results': self.test_results,
            'communication_stats': self.comm_stats
        }


def main(args=None):
    rclpy.init(args=args)
    test_suite = CommunicationTestSuite()

    try:
        results = test_suite.run_all_tests()
        summary = test_suite.get_test_summary()
        print(f"Communication Test Summary: {summary}")
    except Exception as e:
        print(f"Error running communication tests: {e}")
    finally:
        test_suite.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

## Testing and Validation

### 1. Creating Communication Protocol Test Script

```bash
cat > ~/test_communication_protocols.sh << 'EOF'
#!/bin/bash

# Test script for AI-robot communication protocols

echo "Testing AI-Robot Communication Protocols..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if communication protocol package is available
echo "Checking communication protocol package..."
ros2 pkg list | grep communication_protocol

if [ $? -eq 0 ]; then
    echo "✓ Communication protocol package found"
else
    echo "✗ Communication protocol package not found"
    exit 1
fi

# Check for communication configuration files
if [ -f "/workspace/shared_dir/configs/communication_config.yaml" ]; then
    echo "✓ Communication configuration file found"
else
    echo "✗ Communication configuration file not found"
    exit 1
fi

# Build the communication protocol package
echo "Building communication protocol package..."
cd ~/isaac_ros_ws
colcon build --packages-select isaac_communication_protocol
source install/setup.bash

# Check if launch files exist
if [ -f "/workspace/shared_dir/src/isaac_communication_protocol/launch/communication_protocol.launch.py" ]; then
    echo "✓ Communication protocol launch file found"
else
    echo "✗ Communication protocol launch file not found"
    exit 1
fi

# Test Python dependencies
python3 -c "import numpy; import cv2; import pickle; import gzip; print('✓ Required Python packages available')" || {
    echo "✗ Required Python packages not available"
    echo "Installing required packages..."
    pip3 install numpy opencv-python
}

echo "AI-Robot Communication Protocol test completed."
echo "To run the communication system, use:"
echo "ros2 launch isaac_communication_protocol communication_protocol.launch.py"
EOF

# Make executable
chmod +x ~/test_communication_protocols.sh
```

### 2. Running Communication Protocol Test

```bash
~/test_communication_protocols.sh
```

## Troubleshooting Communication Protocols

### Common Issues and Solutions

#### Issue: "High latency in communication between AI and robot"
**Solution**: Check QoS profiles and message frequency
```bash
# Check current QoS settings
ros2 topic info /camera/color/image_raw
ros2 topic info /cmd_vel

# Verify message frequency
ros2 topic hz /camera/color/image_raw
ros2 topic hz /cmd_vel
```

#### Issue: "Bandwidth limitations affecting performance"
**Solution**: Enable message compression and adjust frequency
```bash
# Check bandwidth usage
ros2 run isaac_communication_protocol communication_optimizer
```

#### Issue: "Messages not being delivered consistently"
**Solution**: Check connection stability and retry mechanisms
```bash
# Monitor communication statistics
ros2 topic echo /communication/stats
ros2 topic echo /communication/optimization_status
```

## Verification Checklist

- [ ] Communication protocol handler created and functional
- [ ] Message serialization and compression implemented
- [ ] Communication middleware created
- [ ] QoS profiles configured appropriately
- [ ] Performance optimization tools implemented
- [ ] Communication test suite created
- [ ] Launch files created for communication system
- [ ] Configuration files created and validated
- [ ] Test script created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After implementing the AI-robot communication protocols:

1. **Test communication performance** with Isaac Sim running
2. **Validate message throughput and latency** under various conditions
3. **Optimize communication parameters** based on performance metrics
4. **Create communication exercises** for students

The AI-robot communication protocol framework is now configured and ready for Module 3, providing students with tools to establish efficient and reliable communication between AI systems and robotic platforms.