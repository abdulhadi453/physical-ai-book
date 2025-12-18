# Lesson 2: Navigation and Path Planning with Nav2

## Overview

This lesson focuses on implementing navigation systems using Nav2 integrated with the NVIDIA Isaac platform. You'll learn how to create path planning algorithms that allow robots to navigate complex environments autonomously.

## Learning Objectives

After completing this lesson, you will be able to:

- Integrate Nav2 with NVIDIA Isaac simulation
- Configure path planning algorithms
- Implement navigation behaviors
- Create obstacle avoidance systems
- Evaluate navigation performance

## Introduction to Nav2

Navigation2 (Nav2) is the navigation stack for ROS 2, designed to provide path planning, navigation, and obstacle avoidance capabilities for mobile robots. When integrated with NVIDIA Isaac, it enables realistic simulation of navigation tasks.

### Key Components of Nav2

1. **Navigation Server**: Centralized navigation system
2. **Planners**: Global and local path planning algorithms
3. **Controllers**: Trajectory controllers for robot motion
4. **Recovery Behaviors**: Actions when navigation fails
5. **Sensors**: Integration with perception systems

### Nav2 Architecture

```
[Goals] -> [Navigation Server] -> [Global Planner] -> [Path]
                                   |                    |
                                   v                    v
                            [Local Planner] -> [Controller] -> [Robot]
                                   |                    |
                                   v                    v
                            [Recovery Behaviors] <- [Sensors]
```

## Agent Interaction Points

### AI Assistant Request: Nav2 Architecture Explanation
**Context**: Understanding Nav2 architecture
**Request**: "Explain the Nav2 architecture and how each component interacts with the others"
**Expected Output**: Detailed explanation of Nav2 components and their interactions

### AI Assistant Request: Troubleshoot Nav2 Integration
**Context**: Nav2 integration with Isaac Sim
**Request**: "I'm getting errors when launching Nav2 with Isaac Sim. What should I check?"
**Expected Output**: Troubleshooting guide for Nav2-Isaac integration issues

### AI Assistant Request: Path Planning Algorithm Selection
**Context**: Choosing path planning algorithms
**Request**: "Which path planning algorithm should I use for my specific scenario?"
**Expected Output**: Decision guide for selecting appropriate planners based on scenario requirements

## Nav2 Integration with Isaac

### Prerequisites

- Completed Lesson 1 (Isaac Sim + ROS integration)
- Nav2 packages installed in ROS 2 environment
- Robot model with appropriate sensors (LiDAR, camera, IMU)

### Installation and Setup

```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Launching Nav2 with Isaac

```bash
# Launch Isaac Sim with Nav2 configuration
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/nav2_params.yaml
```

## Path Planning Algorithms

### Global Planners

Global planners compute a path from the start to goal positions based on a map of the environment.

#### A* Planner
- Optimal path finding algorithm
- Guarantees shortest path
- Computationally intensive for large maps

#### Dijkstra Planner
- Similar to A* but without heuristic
- Guarantees optimal solution
- Slower than A* in practice

#### NavFn Planner
- Fast navigation function
- Uses wavefront propagation
- Good for real-time applications

### Local Planners

Local planners generate velocity commands to follow the global path while avoiding obstacles.

#### DWA Local Planner
- Dynamic Window Approach
- Considers robot dynamics
- Good obstacle avoidance

#### Trajectory Rollout
- Evaluates multiple trajectory options
- Balances speed and safety
- Computationally efficient

### Configuration Example

```yaml
# nav2_params.yaml
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
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Note: the 'default_nav_through_poses_bt_xml' and 'default_nav_to_pose_bt_xml'
    # will be used as the default behavior tree for the navigator.
    # You can specify a different behavior tree file at runtime with the
    # 'behavior_tree' parameter in the action goal.
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
```

## Navigation Behaviors

### Pure Navigation
- Simple path following
- Minimal obstacle avoidance
- Good for known environments

### Navigation with Recovery
- Includes recovery behaviors when stuck
- Handles navigation failures
- More robust than pure navigation

### Navigation with Smoothing
- Applies path smoothing algorithms
- Creates more natural robot motion
- Better for dynamic environments

## Obstacle Avoidance Systems

### Costmap Configuration

Costmaps represent the environment with obstacle information:

```yaml
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
```

### Dynamic Obstacle Avoidance

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Create subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create publisher
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.safe_distance = 0.5  # meters
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s

    def scan_callback(self, msg):
        # Find minimum distance in front of robot
        min_distance = min(msg.ranges)

        twist = Twist()

        if min_distance > self.safe_distance:
            # Move forward safely
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        else:
            # Rotate to avoid obstacle
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed

        self.cmd_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    avoidance = ObstacleAvoidance()
    rclpy.spin(avoidance)
    avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Agent Interaction Points for Exercises

### AI Assistant Request: Exercise 3 Guidance
**Context**: Exercise 3 - Basic Navigation Setup
**Request**: "I'm having trouble configuring Nav2 parameters for my Isaac Sim scenario. Can you help?"
**Expected Output**: Step-by-step guide for Nav2 parameter configuration

### AI Assistant Request: Exercise 4 Guidance
**Context**: Exercise 4 - Advanced Navigation with Recovery
**Request**: "How do I implement custom recovery behaviors for my navigation system?"
**Expected Output**: Guide for implementing custom recovery behaviors

## Exercise 3: Basic Navigation Setup

### Objective
Configure Nav2 with Isaac Sim and execute basic navigation commands.

### Steps
1. Launch Isaac Sim with a navigation scenario
2. Configure Nav2 parameters for the simulation
3. Send navigation goals to the robot
4. Monitor navigation performance

### Expected Outcome
Robot successfully navigates to specified goals while avoiding obstacles.

## Exercise 4: Advanced Navigation with Recovery

### Objective
Implement navigation with recovery behaviors for robust operation.

### Steps
1. Configure recovery behaviors in Nav2
2. Create scenarios that trigger recovery behaviors
3. Test navigation in challenging environments
4. Evaluate recovery effectiveness

### Expected Outcome
Robot handles navigation failures and recovers successfully.

## Performance Evaluation

### Metrics for Navigation Performance

1. **Success Rate**: Percentage of successful navigation attempts
2. **Time to Goal**: Time taken to reach the goal
3. **Path Efficiency**: Actual path length vs optimal path
4. **Safety**: Number of collisions or near-misses
5. **Smoothness**: Robot motion quality

### Evaluation Tools

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class NavigationEvaluator(Node):
    def __init__(self):
        super().__init__('navigation_evaluator')

        self.start_position = None
        self.goal_position = None
        self.path_length = 0.0
        self.has_started = False

    def calculate_path_efficiency(self, optimal_distance):
        if self.path_length > 0 and optimal_distance > 0:
            efficiency = optimal_distance / self.path_length
            return efficiency
        return 0.0

    def calculate_path_length(self, path):
        length = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i+1].pose.position
            segment_length = math.sqrt(
                (p2.x - p1.x)**2 + (p2.y - p1.y)**2
            )
            length += segment_length
        return length
```

## Assessment Questions

1. What are the main components of the Nav2 navigation stack?
2. How does the global planner differ from the local planner?
3. What are recovery behaviors and why are they important?
4. How do costmaps help with obstacle avoidance in navigation?
5. What metrics would you use to evaluate navigation performance?

## Summary

This lesson covered the integration of Nav2 with NVIDIA Isaac for robot navigation. You learned about the Nav2 architecture, path planning algorithms, navigation behaviors, and obstacle avoidance systems. The exercises provided hands-on experience with configuring and evaluating navigation systems in simulation.

## Agent Interaction Points for Review

### AI Assistant Request: Lesson Review
**Context**: Review of Lesson 2 content
**Request**: "Summarize the key navigation concepts from Lesson 2 and suggest best practices"
**Expected Output**: Concise summary of navigation concepts and recommended best practices

### AI Assistant Request: Troubleshooting Summary
**Context**: Common issues in Lesson 2
**Request**: "What are the most common navigation issues students face in Lesson 2 and how to solve them?"
**Expected Output**: Compilation of common navigation issues and their solutions