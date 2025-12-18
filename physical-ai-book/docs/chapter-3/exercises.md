# Module 3 Exercise Framework: The AI-Robot Brain (NVIDIA Isaac™)

## Overview

This document provides the comprehensive exercise framework for Module 3, which covers the NVIDIA Isaac platform for creating AI-robot brains. The exercises are designed to reinforce learning objectives through hands-on practice with simulation, perception, navigation, and AI decision-making systems.

## Exercise Design Philosophy

### Learning Through Practice
The exercises in Module 3 follow a progressive difficulty model:
- **Foundation Exercises**: Basic setup and configuration tasks
- **Integration Exercises**: Combining multiple systems
- **Application Exercises**: Real-world scenario implementations
- **Challenge Exercises**: Advanced problem-solving tasks

### Assessment Integration
Each exercise includes:
- Clear objectives and success criteria
- Step-by-step instructions
- Self-assessment checkpoints
- Performance metrics
- Troubleshooting guidance

## Exercise Categories

### 1. Simulation Setup Exercises
Focus on Isaac Sim environment configuration and basic robot simulation

### 2. Perception System Exercises
Implement AI-powered perception using Isaac ROS packages

### 3. Navigation System Exercises
Configure and test Nav2 integration with Isaac Sim

### 4. AI Decision Making Exercises
Create intelligent behavior using perception and navigation

### 5. Integration Challenge Exercises
Combine all systems for complete AI-robot applications

## Exercise Framework Structure

### Exercise Template
Each exercise follows this structure:

```
## Exercise [Number]: [Title]
**Duration**: [Time estimate]
**Difficulty**: [Beginner/Intermediate/Advanced]
**Learning Objectives**:
- [Specific objective 1]
- [Specific objective 2]

**Prerequisites**:
- [Required knowledge/skills]

**Setup Requirements**:
- [Hardware/software needed]

**Steps**:
1. [Detailed step 1]
2. [Detailed step 2]

**Success Criteria**:
- [Measurable outcome 1]
- [Measurable outcome 2]

**Assessment Questions**:
- [Knowledge check question 1]
- [Knowledge check question 2]

**Troubleshooting Tips**:
- [Common issue 1 and solution]
- [Common issue 2 and solution]

**Extension Activities**:
- [Optional advanced challenges]
```

## Foundation Exercises (Lessons 1-3)

### Exercise 1: Basic Isaac Sim Setup
**Duration**: 45 minutes
**Difficulty**: Beginner
**Learning Objectives**:
- Launch Isaac Sim with Docker
- Navigate the Isaac Sim interface
- Load a basic scene with a robot
- Verify ROS communication

**Prerequisites**:
- Completed Module 3 setup guide
- Docker with NVIDIA Container Toolkit
- ROS 2 Humble environment

**Setup Requirements**:
- Computer with NVIDIA GPU
- Isaac Sim Docker image
- Module 3 configuration files

**Steps**:
1. Launch Isaac Sim using the configured script:
   ```bash
   ~/launch_isaac_sim_configured.sh
   ```
2. Open Isaac Sim in your browser at http://localhost:5000
3. Load the basic navigation scene from `/workspace/shared_dir/scenes/basic_navigation.usd`
4. Verify the Carter robot appears in the scene
5. Check ROS communication by running:
   ```bash
   # In a new terminal
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 topic list | grep -E "(scan|image|imu)"
   ```

**Success Criteria**:
- Isaac Sim launches successfully
- Robot model appears in the scene
- ROS topics for sensors are available
- Isaac Sim interface is navigable

**Assessment Questions**:
1. What is the default port for Isaac Sim web interface?
2. How can you verify that sensor data is being published by the simulated robot?
3. What does the `xhost +local:docker` command do?

**Troubleshooting Tips**:
- If Isaac Sim fails to launch: Check GPU access with `nvidia-smi`
- If ROS topics are not available: Verify ROS_DOMAIN_ID is set correctly
- If Docker container exits immediately: Check available disk space

**Extension Activities**:
- Try loading different scenes from the Isaac Sim assets library
- Experiment with different robot models

### Exercise 2: ROS Bridge Communication
**Duration**: 60 minutes
**Difficulty**: Beginner
**Learning Objectives**:
- Configure Isaac ROS bridge
- Subscribe to sensor topics
- Publish commands to the robot
- Verify bidirectional communication

**Prerequisites**:
- Exercise 1 completed
- Isaac ROS workspace built
- Basic ROS 2 knowledge

**Setup Requirements**:
- Running Isaac Sim with robot
- Isaac ROS bridge extensions enabled
- ROS 2 Humble environment sourced

**Steps**:
1. In Isaac Sim, enable the ROS bridge extension:
   - Go to Window > Extensions
   - Search for "ROS" and enable "Isaac ROS Bridge"
2. Create a simple ROS node to subscribe to camera images:
   ```python
   # Create file ~/isaac_ros_ws/src/simple_subscriber.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge

   class SimpleSubscriber(Node):
       def __init__(self):
           super().__init__('simple_subscriber')
           self.subscription = self.create_subscription(
               Image,
               '/camera/color/image_raw',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning
           self.bridge = CvBridge()

       def listener_callback(self, msg):
           self.get_logger().info(f'Received image: {msg.width}x{msg.height}')

   def main(args=None):
       rclpy.init(args=args)
       simple_subscriber = SimpleSubscriber()
       rclpy.spin(simple_subscriber)
       simple_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
3. Run the subscriber:
   ```bash
   cd ~/isaac_ros_ws
   source install/setup.bash
   python3 src/simple_subscriber.py
   ```
4. Create a simple publisher for robot commands:
   ```python
   # Create file ~/isaac_ros_ws/src/simple_publisher.py
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist

   class SimplePublisher(Node):
       def __init__(self):
           super().__init__('simple_publisher')
           self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)

       def timer_callback(self):
           msg = Twist()
           msg.linear.x = 0.1  # Move forward slowly
           msg.angular.z = 0.0  # No rotation
           self.publisher.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg)

   def main(args=None):
       rclpy.init(args=args)
       simple_publisher = SimplePublisher()
       rclpy.spin(simple_publisher)
       simple_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
5. Run the publisher:
   ```bash
   cd ~/isaac_ros_ws
   source install/setup.bash
   python3 src/simple_publisher.py
   ```

**Success Criteria**:
- Camera image subscriber receives data
- Robot responds to velocity commands
- Bidirectional communication verified
- No errors in ROS communication

**Assessment Questions**:
1. What is the purpose of the CV Bridge in ROS?
2. How do you create a ROS publisher for velocity commands?
3. What is the typical message type for robot velocity commands?

**Troubleshooting Tips**:
- If no images received: Check Isaac Sim camera configuration
- If robot doesn't move: Verify topic remapping in Isaac Sim
- If high latency: Check network configuration and bandwidth

**Extension Activities**:
- Create a node that processes camera images and publishes commands based on image content
- Implement a simple obstacle avoidance behavior

### Exercise 3: Basic Perception Pipeline
**Duration**: 90 minutes
**Difficulty**: Intermediate
**Learning Objectives**:
- Implement object detection using Isaac ROS
- Configure perception pipeline parameters
- Visualize detection results
- Integrate perception with basic control

**Prerequisites**:
- Exercises 1 and 2 completed
- Isaac ROS perception packages installed
- Basic Python programming skills

**Setup Requirements**:
- Isaac Sim with robot and objects in scene
- Isaac ROS perception pipeline built
- Perception test scene loaded

**Steps**:
1. Launch Isaac Sim with perception test scene:
   ```bash
   # Use the perception test scene
   # Load /workspace/shared_dir/scenes/perception_test.usd in Isaac Sim
   ```
2. Build the perception pipeline package:
   ```bash
   cd ~/isaac_ros_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select isaac_ros_perception_pipeline
   source install/setup.bash
   ```
3. Launch the basic perception pipeline:
   ```bash
   ros2 launch isaac_ros_perception_pipeline basic_perception_pipeline.launch.py
   ```
4. Create a perception processing node:
   ```python
   # Create file ~/isaac_ros_ws/src/perception_processor.py
   import rclpy
   from rclpy.node import Node
   from vision_msgs.msg import Detection2DArray
   from geometry_msgs.msg import Twist
   import math

   class PerceptionProcessor(Node):
       def __init__(self):
           super().__init__('perception_processor')

           # Subscribe to object detections
           self.detection_subscription = self.create_subscription(
               Detection2DArray,
               '/object_detections',
               self.detection_callback,
               10
           )

           # Publisher for robot commands
           self.cmd_publisher = self.create_publisher(
               Twist,
               '/cmd_vel',
               10
           )

           self.last_detections = []
           self.get_logger().info('Perception Processor initialized')

       def detection_callback(self, msg):
           self.last_detections = msg.detections
           self.get_logger().info(f'Detected {len(self.last_detections)} objects')

           # Simple behavior: move toward the first detected object
           if len(self.last_detections) > 0:
               detection = self.last_detections[0]
               center_x = detection.bbox.center.x

               cmd_msg = Twist()
               cmd_msg.linear.x = 0.1  # Move forward

               # Turn toward object if not centered
               image_center = 320  # Assuming 640x480 image
               if center_x < image_center - 50:
                   cmd_msg.angular.z = 0.2  # Turn left
               elif center_x > image_center + 50:
                   cmd_msg.angular.z = -0.2  # Turn right
               else:
                   cmd_msg.angular.z = 0.0  # Go straight

               self.cmd_publisher.publish(cmd_msg)

   def main(args=None):
       rclpy.init(args=args)
       processor = PerceptionProcessor()

       try:
           rclpy.spin(processor)
       except KeyboardInterrupt:
           pass
       finally:
           processor.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
5. Run the perception processor:
   ```bash
   cd ~/isaac_ros_ws
   source install/setup.bash
   python3 src/perception_processor.py
   ```

**Success Criteria**:
- Object detection system runs without errors
- Robot moves toward detected objects
- Detection results are visualized
- Perception system integrates with control

**Assessment Questions**:
1. What is the difference between object detection and semantic segmentation?
2. How do you access bounding box information from Detection2DArray?
3. What is the purpose of the vision_msgs package in ROS?

**Troubleshooting Tips**:
- If no detections: Check camera configuration and scene objects
- If poor detection quality: Verify model parameters and confidence thresholds
- If robot behavior is erratic: Check coordinate frame transformations

**Extension Activities**:
- Implement different behaviors based on object class
- Add confidence threshold filtering for detections

### Exercise 4: Nav2 Basic Navigation
**Duration**: 120 minutes
**Difficulty**: Intermediate
**Learning Objectives**:
- Launch Nav2 with Isaac Sim
- Send navigation goals to the robot
- Monitor navigation performance
- Configure basic navigation parameters

**Prerequisites**:
- Exercise 1 completed
- Nav2 packages installed
- Isaac Sim navigation scene loaded

**Setup Requirements**:
- Isaac Sim with navigation scene
- Nav2 configuration files
- RViz2 for visualization

**Steps**:
1. Launch Isaac Sim with navigation scene
2. Build and source Isaac Nav2 package:
   ```bash
   cd ~/isaac_ros_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select isaac_nav2
   source install/setup.bash
   ```
3. Launch Nav2 with Isaac Sim:
   ```bash
   ros2 launch isaac_nav2 nav2_isaac_sim_bringup.launch.py
   ```
4. In another terminal, launch RViz:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   rviz2 -d ~/isaac_ros_ws/install/isaac_nav2/share/isaac_nav2/rviz/nav2_isaac_sim_view.rviz
   ```
5. Send a navigation goal using the RViz interface:
   - Click "2D Goal Pose" button
   - Click on the map where you want the robot to go
   - Set the orientation by dragging the arrow
6. Monitor navigation performance using command line tools:
   ```bash
   # Check navigation status
   ros2 action list | grep navigate
   ros2 action info /navigate_to_pose

   # Monitor costmaps
   ros2 topic echo /global_costmap/costmap
   ```

**Success Criteria**:
- Nav2 system launches without errors
- Robot successfully navigates to goals
- Costmaps update correctly
- Navigation performance is monitored

**Assessment Questions**:
1. What are the main components of the Nav2 architecture?
2. How do global and local planners differ in their function?
3. What is the purpose of costmaps in navigation?

**Troubleshooting Tips**:
- If navigation fails: Check map, localization, and sensor data
- If robot gets stuck: Verify obstacle detection and recovery behaviors
- If path is not optimal: Adjust planner parameters

**Extension Activities**:
- Implement a patrol pattern with multiple waypoints
- Add dynamic obstacle avoidance to the navigation system

### Exercise 5: Perception-Action Integration
**Duration**: 150 minutes
**Difficulty**: Advanced
**Learning Objectives**:
- Integrate perception and navigation systems
- Create AI decision-making based on perception
- Implement complex robot behaviors
- Evaluate integrated system performance

**Prerequisites**:
- Exercises 1-4 completed
- Understanding of both perception and navigation systems
- Advanced Python programming skills

**Setup Requirements**:
- Complete Isaac Sim setup with perception and navigation
- All previous exercises working
- Complex scene with multiple objects

**Steps**:
1. Create an integrated perception-navigation node:
   ```python
   # Create file ~/isaac_ros_ws/src/integrated_system.py
   import rclpy
   from rclpy.node import Node
   from vision_msgs.msg import Detection2DArray
   from geometry_msgs.msg import Twist, PoseStamped
   from nav_msgs.msg import Odometry
   from std_msgs.msg import String
   import math
   from rclpy.action import ActionClient
   from nav2_msgs.action import NavigateToPose

   class IntegratedSystem(Node):
       def __init__(self):
           super().__init__('integrated_system')

           # Perception components
           self.detection_subscription = self.create_subscription(
               Detection2DArray,
               '/object_detections',
               self.detection_callback,
               10
           )

           # Navigation components
           self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

           # State management
           self.robot_pose = None
           self.detected_objects = []
           self.current_task = "exploring"  # exploring, navigating_to_object, etc.

           # Task timer
           self.task_timer = self.create_timer(2.0, self.task_management)

           self.get_logger().info('Integrated Perception-Navigation System initialized')

       def detection_callback(self, msg):
           self.detected_objects = msg.detections
           self.get_logger().info(f'Detected {len(self.detected_objects)} objects')

       def task_management(self):
           """Manage robot tasks based on perception"""
           if self.current_task == "exploring":
               if len(self.detected_objects) > 0:
                   # Found an object, navigate to it
                   self.navigate_to_object(self.detected_objects[0])
                   self.current_task = "navigating_to_object"
               else:
                   # No objects found, continue exploring
                   self.explore_area()

       def navigate_to_object(self, detection):
           """Navigate to a detected object"""
           # This is a simplified version - in practice, you'd convert
           # image coordinates to world coordinates using camera calibration
           goal_pose = PoseStamped()
           goal_pose.header.frame_id = 'map'
           goal_pose.header.stamp = self.get_clock().now().to_msg()

           # Set a goal near where the object was detected
           goal_pose.pose.position.x = 1.0  # Example coordinates
           goal_pose.pose.position.y = 1.0
           goal_pose.pose.orientation.w = 1.0

           # Send navigation goal
           goal_msg = NavigateToPose.Goal()
           goal_msg.pose = goal_pose

           self.nav_to_pose_client.wait_for_server()
           self.nav_to_pose_client.send_goal_async(goal_msg)

       def explore_area(self):
           """Simple exploration behavior"""
           cmd_msg = Twist()
           cmd_msg.linear.x = 0.2
           cmd_msg.angular.z = 0.1  # Gentle turning for exploration

           # Publish to cmd_vel for basic exploration if navigation not active
           cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
           cmd_publisher.publish(cmd_msg)

   def main(args=None):
       rclpy.init(args=args)
       system = IntegratedSystem()

       try:
           rclpy.spin(system)
       except KeyboardInterrupt:
           pass
       finally:
           system.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
2. Launch the complete integrated system:
   ```bash
   # Terminal 1: Launch Isaac Sim
   ~/launch_isaac_sim_configured.sh

   # Terminal 2: Launch Nav2
   cd ~/isaac_ros_ws
   source install/setup.bash
   ros2 launch isaac_nav2 nav2_isaac_sim_bringup.launch.py

   # Terminal 3: Launch perception pipeline
   ros2 launch isaac_ros_perception_pipeline basic_perception_pipeline.launch.py

   # Terminal 4: Run the integrated system
   ros2 run isaac_ros_perception_pipeline integrated_system
   ```

**Success Criteria**:
- Perception and navigation systems work together
- Robot demonstrates intelligent behavior
- System handles transitions between tasks
- Performance metrics are tracked

**Assessment Questions**:
1. How do you integrate perception and navigation systems?
2. What are the challenges in combining multiple AI systems?
3. How do you handle state transitions in an integrated system?

**Troubleshooting Tips**:
- If systems don't communicate: Check topic names and frame IDs
- If behavior is inconsistent: Verify timing and state management
- If performance is poor: Check system resource usage

**Extension Activities**:
- Implement more complex decision-making logic
- Add multiple robot coordination
- Create a learning system that improves over time

## Assessment Exercises

### Exercise 6: Module Integration Challenge
**Duration**: 180 minutes
**Difficulty**: Advanced
**Learning Objectives**:
- Integrate all Module 3 components
- Demonstrate complete AI-robot system
- Evaluate system performance
- Document system behavior

**Prerequisites**:
- All previous exercises completed
- Complete understanding of Isaac Sim, perception, and navigation
- Ability to troubleshoot complex systems

**Setup Requirements**:
- Full Module 3 environment
- Complex test scenario
- Performance monitoring tools

**Steps**:
1. Set up a complex scenario with multiple objects and navigation challenges
2. Implement a complete AI-robot system that:
   - Perceives the environment using Isaac ROS
   - Makes decisions based on perception
   - Navigates to relevant locations
   - Demonstrates intelligent behavior
3. Document the system architecture and implementation
4. Evaluate performance using established metrics

**Success Criteria**:
- Complete system integration demonstrated
- All learning objectives achieved
- Performance meets requirements
- Documentation is comprehensive

**Assessment Questions**:
1. How does your system handle uncertainty in perception?
2. What decision-making framework did you implement?
3. How do you ensure system safety during autonomous operation?

**Troubleshooting Tips**:
- Break down the system into components for debugging
- Use logging and visualization tools extensively
- Test components individually before integration

**Extension Activities**:
- Optimize system performance
- Add additional capabilities
- Prepare for Module 4 integration

## Performance Evaluation Framework

### Quantitative Metrics
- **Perception Accuracy**: Percentage of correctly detected objects
- **Navigation Success Rate**: Percentage of successful goal reaches
- **Path Efficiency**: Ratio of optimal path to actual path length
- **System Response Time**: Time from perception to action
- **Resource Utilization**: CPU, GPU, and memory usage

### Qualitative Assessment
- **Behavior Appropriateness**: How well the robot's actions match the situation
- **Robustness**: How well the system handles unexpected situations
- **Efficiency**: How effectively the system uses available resources
- **Safety**: How safely the system operates

## Exercise Submission Guidelines

### Required Components
1. **Implementation Code**: All custom nodes and scripts
2. **Configuration Files**: Parameter files and launch configurations
3. **Documentation**: Explanation of approach and results
4. **Performance Data**: Metrics and evaluation results
5. **Video Evidence**: Screencast of system in operation (optional but recommended)

### Evaluation Rubric
- **Functionality (40%)**: Does the system work as intended?
- **Code Quality (20%)**: Is the code well-structured and documented?
- **Performance (20%)**: Does the system meet performance requirements?
- **Innovation (10%)**: Does the solution show creative thinking?
- **Documentation (10%)**: Is the work well-documented and explained?

## Troubleshooting and Support

### Common Exercise Issues
- **Environment Setup**: Ensure all prerequisites are met
- **ROS Communication**: Check topic names and frame IDs
- **Performance**: Monitor system resources and optimize as needed
- **Integration**: Test components individually before integration

### Support Resources
- Module 3 documentation
- Isaac Sim user guide
- ROS 2 tutorials
- Peer collaboration opportunities

## Next Steps

After completing the exercises:
1. **Review and Refine**: Analyze system performance and identify improvements
2. **Prepare for Assessment**: Complete Module 3 assessment
3. **Plan Advanced Projects**: Consider capstone project ideas
4. **Explore Module 4**: Begin preparation for humanoid robotics content

This exercise framework provides a comprehensive learning path through Module 3, from basic setup to advanced integration, ensuring students gain practical experience with NVIDIA Isaac platform for AI-robot brain development.