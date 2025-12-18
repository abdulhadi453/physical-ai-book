---
sidebar_position: 18
---

# Exercise 3: Advanced Navigation

## Overview
This exercise integrates all previously learned concepts to implement advanced navigation capabilities using a multi-sensor approach. You will create a complete navigation system that uses LiDAR, depth camera, and IMU data to navigate complex environments with dynamic obstacles and challenging terrain.

## Learning Objectives
By the end of this exercise, you will be able to:
- Implement a complete navigation stack with perception, planning, and control
- Integrate multiple sensors for robust environment understanding
- Handle dynamic obstacles and changing environments
- Implement path planning and trajectory execution
- Validate navigation performance in complex scenarios

## Prerequisites
- Completion of Exercises 1 and 2
- Understanding of sensor fusion concepts
- Basic knowledge of path planning algorithms
- Familiarity with control systems for robot navigation

## Exercise Setup

### Environment Configuration
Create a complex navigation environment with:
- Multiple rooms connected by corridors
- Various obstacle types (static and dynamic)
- Different floor surfaces and elevations
- Doorways and narrow passages
- Moving obstacles to test dynamic navigation

### Robot Configuration
Configure your robot with:
- Complete sensor suite (LiDAR, depth camera, IMU)
- Differential drive or similar mobility platform
- Appropriate computational resources for real-time processing
- Communication interface for navigation commands

## Exercise Steps

### Step 1: Environment Mapping and Localization
1. **SLAM Implementation**: Implement Simultaneous Localization and Mapping using sensor data
2. **Map Building**: Create a comprehensive map of the environment
3. **Localization**: Maintain accurate robot pose estimation within the map
4. **Map Update**: Handle dynamic changes in the environment

### Step 2: Path Planning
1. **Global Planner**: Implement a global path planner (A*, Dijkstra, or similar)
2. **Local Planner**: Implement a local planner for obstacle avoidance
3. **Path Optimization**: Optimize paths for safety, efficiency, and smoothness
4. **Dynamic Replanning**: Handle path updates when obstacles are detected

### Step 3: Motion Control
1. **Trajectory Following**: Implement controllers to follow planned trajectories
2. **Obstacle Avoidance**: Implement reactive avoidance for unexpected obstacles
3. **Motion Smoothing**: Ensure smooth transitions between waypoints
4. **Safety Systems**: Implement emergency stopping and safe recovery

### Step 4: System Integration and Testing
1. **Complete System**: Integrate all components into a working navigation system
2. **Performance Testing**: Test navigation performance in various scenarios
3. **Robustness Testing**: Test system behavior under challenging conditions
4. **Validation**: Validate navigation accuracy and safety

## Implementation Tasks

### Task 1: SLAM System Implementation
Create a basic SLAM system that integrates sensor data:

```python
import numpy as np
from scipy.spatial import KDTree
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class Pose2D:
    x: float
    y: float
    theta: float

class SimpleSLAM:
    def __init__(self, map_resolution: float = 0.1):
        self.map_resolution = map_resolution
        self.occupancy_grid = {}  # Dictionary-based occupancy grid
        self.robot_path = []      # Path history for loop closure
        self.current_pose = Pose2D(0, 0, 0)

    def update_map(self, lidar_points: np.ndarray, robot_pose: Pose2D):
        """Update occupancy grid with LiDAR data"""
        # Convert LiDAR points to global coordinates
        cos_theta = np.cos(robot_pose.theta)
        sin_theta = np.sin(robot_pose.theta)

        # Transform points from robot frame to global frame
        global_points = []
        for point in lidar_points:
            # Rotate and translate point
            x_rot = point[0] * cos_theta - point[1] * sin_theta
            y_rot = point[0] * sin_theta + point[1] * cos_theta
            x_global = x_rot + robot_pose.x
            y_global = y_rot + robot_pose.y
            global_points.append((x_global, y_global))

        # Update occupancy grid
        for point in global_points:
            grid_x = int(point[0] / self.map_resolution)
            grid_y = int(point[1] / self.map_resolution)
            grid_key = (grid_x, grid_y)

            # Simple occupancy update (occupied = 1, free = 0)
            if grid_key not in self.occupancy_grid:
                self.occupancy_grid[grid_key] = 0
            self.occupancy_grid[grid_key] = 1  # Mark as occupied

        # Update robot pose
        self.current_pose = robot_pose
        self.robot_path.append(robot_pose)

    def get_map_bounds(self):
        """Get the bounds of the current map"""
        if not self.occupancy_grid:
            return (0, 0, 0, 0)

        x_coords = [key[0] for key in self.occupancy_grid.keys()]
        y_coords = [key[1] for key in self.occupancy_grid.keys()]

        return (min(x_coords), max(x_coords), min(y_coords), max(y_coords))

    def is_occupied(self, x: float, y: float) -> bool:
        """Check if a point is occupied in the map"""
        grid_x = int(x / self.map_resolution)
        grid_y = int(y / self.map_resolution)
        grid_key = (grid_x, grid_y)

        return self.occupancy_grid.get(grid_key, 0) > 0.5

class NavigationSystem:
    def __init__(self):
        self.slam = SimpleSLAM()
        self.path = []
        self.current_goal = None
        self.path_index = 0

    def set_goal(self, goal_x: float, goal_y: float):
        """Set navigation goal"""
        self.current_goal = (goal_x, goal_y)

    def plan_path(self, start_pose: Pose2D, goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Plan path using A* algorithm"""
        # Simplified A* implementation
        start_grid = (int(start_pose.x / 0.5), int(start_pose.y / 0.5))
        goal_grid = (int(goal[0] / 0.5), int(goal[1] / 0.5))

        # Use occupancy grid for path planning
        # In practice, you'd implement a proper A* algorithm
        # This is a simplified version for demonstration

        # Generate a simple path (in reality, implement A* with obstacle checking)
        path = []
        current = start_grid
        target = goal_grid

        # Simple grid-based path to target
        while current != target:
            dx = target[0] - current[0]
            dy = target[1] - current[1]

            if abs(dx) > abs(dy):
                next_x = current[0] + (1 if dx > 0 else -1)
                next_y = current[1]
            else:
                next_x = current[0]
                next_y = current[1] + (1 if dy > 0 else -1)

            current = (next_x, next_y)
            path.append((current[0] * 0.5, current[1] * 0.5))  # Convert back to meters

            if len(path) > 1000:  # Prevent infinite loops
                break

        return path
```

### Task 2: Local Planner and Obstacle Avoidance
Implement local planning and reactive obstacle avoidance:

```python
class LocalPlanner:
    def __init__(self, robot_radius: float = 0.3, safety_distance: float = 0.5):
        self.robot_radius = robot_radius
        self.safety_distance = safety_distance
        self.obstacle_threshold = robot_radius + safety_distance

    def check_collision_free_path(self, start: Tuple[float, float],
                                end: Tuple[float, float],
                                slam_system: SimpleSLAM) -> bool:
        """Check if path is collision-free"""
        # Simple line-of-sight collision checking
        steps = 10  # Number of steps to check along the path
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])

            # Check if this point is occupied
            if slam_system.is_occupied(x, y):
                return False
        return True

    def reactive_avoidance(self, lidar_points: np.ndarray,
                          current_velocity: Tuple[float, float],
                          target_direction: Tuple[float, float]) -> Tuple[float, float]:
        """Implement reactive obstacle avoidance"""
        # Find closest obstacles in front of robot
        front_threshold = 0.5  # Only consider obstacles within 0.5m
        min_distance = float('inf')
        closest_obstacle = None

        for point in lidar_points:
            distance = np.sqrt(point[0]**2 + point[1]**2)
            if distance < min_distance and distance < front_threshold:
                min_distance = distance
                closest_obstacle = point

        if closest_obstacle is not None and min_distance < self.obstacle_threshold:
            # Obstacle detected, modify velocity
            # Simple reactive approach: slow down and turn away
            avoidance_factor = 1.0 - (min_distance / self.obstacle_threshold)
            turn_direction = 1.0 if closest_obstacle[1] > 0 else -1.0  # Turn away from obstacle

            # Reduce forward velocity and add lateral component
            new_vx = current_velocity[0] * (1 - avoidance_factor * 0.5)
            new_vy = current_velocity[1] + turn_direction * avoidance_factor * 0.3

            return (new_vx, new_vy)

        # No obstacle, return original velocity
        return current_velocity
```

### Task 3: Motion Controller
Implement a motion controller for trajectory following:

```python
class MotionController:
    def __init__(self, linear_kp: float = 1.0, angular_kp: float = 2.0):
        self.linear_kp = linear_kp
        self.angular_kp = angular_kp
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s

    def compute_velocity(self, current_pose: Pose2D,
                        target_pose: Pose2D) -> Tuple[float, float]:
        """Compute linear and angular velocities to reach target pose"""
        # Calculate error
        dx = target_pose.x - current_pose.x
        dy = target_pose.y - current_pose.y
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate target angle
        target_angle = np.arctan2(dy, dx)
        angle_error = target_angle - current_pose.theta

        # Normalize angle error to [-π, π]
        while angle_error > np.pi:
            angle_error -= 2 * np.pi
        while angle_error < -np.pi:
            angle_error += 2 * np.pi

        # Compute velocities
        linear_vel = min(self.linear_kp * distance, self.max_linear_vel)
        angular_vel = self.angular_kp * angle_error

        # Limit angular velocity
        angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)

        return (linear_vel, angular_vel)

    def follow_path(self, current_pose: Pose2D, path: List[Tuple[float, float]],
                   current_index: int) -> Tuple[float, float, int]:
        """Follow a path and return velocity commands"""
        if current_index >= len(path):
            return (0.0, 0.0)  # Stop when path is complete

        target_x, target_y = path[current_index]
        target_pose = Pose2D(target_x, target_y, 0)  # Heading not used here

        # Check if we've reached current waypoint
        dx = target_x - current_pose.x
        dy = target_y - current_pose.y
        distance_to_waypoint = np.sqrt(dx**2 + dy**2)

        # Move to next waypoint if close enough
        if distance_to_waypoint < 0.2:  # 20cm threshold
            current_index += 1
            if current_index < len(path):
                target_x, target_y = path[current_index]
                target_pose = Pose2D(target_x, target_y, 0)

        # Compute velocity to follow path
        linear_vel, angular_vel = self.compute_velocity(current_pose, target_pose)

        return (linear_vel, angular_vel, current_index)
```

### Task 4: Complete Navigation System
Integrate all components into a complete system:

```python
class CompleteNavigationSystem:
    def __init__(self):
        self.slam = SimpleSLAM()
        self.local_planner = LocalPlanner()
        self.motion_controller = MotionController()
        self.global_path = []
        self.current_waypoint = 0
        self.navigation_active = False
        self.emergency_stop = False

    def update_sensors(self, lidar_points: np.ndarray,
                      depth_image: Optional[np.ndarray],
                      imu_data: dict,
                      robot_pose: Pose2D):
        """Process sensor data and update navigation system"""
        if self.emergency_stop:
            return (0.0, 0.0)  # Emergency stop

        # Update SLAM with sensor data
        self.slam.update_map(lidar_points, robot_pose)

        # If navigation is active, compute navigation commands
        if self.navigation_active and self.current_waypoint < len(self.global_path):
            # Get current waypoint
            target_x, target_y = self.global_path[self.current_waypoint]
            target_pose = Pose2D(target_x, target_y, 0)

            # Follow the path
            linear_vel, angular_vel, self.current_waypoint = self.motion_controller.follow_path(
                robot_pose, self.global_path, self.current_waypoint
            )

            # Apply local obstacle avoidance
            current_vel = (linear_vel, 0)  # Simplified for differential drive
            safe_vel = self.local_planner.reactive_avoidance(
                lidar_points, current_vel, (target_x - robot_pose.x, target_y - robot_pose.y)
            )

            # Convert back to linear/angular velocities for differential drive
            linear_cmd = safe_vel[0]
            angular_cmd = angular_vel  # Keep original angular control

            return (linear_cmd, angular_cmd)

        # If not navigating, return zero velocity
        return (0.0, 0.0)

    def set_navigation_goal(self, goal_x: float, goal_y: float, start_pose: Pose2D):
        """Set navigation goal and plan path"""
        # Plan global path
        self.global_path = self.slam.plan_path(start_pose, (goal_x, goal_y))
        self.current_waypoint = 0
        self.navigation_active = True

    def stop_navigation(self):
        """Stop current navigation"""
        self.navigation_active = False
        self.emergency_stop = False

    def emergency_stop(self):
        """Emergency stop the robot"""
        self.emergency_stop = True
```

### Task 5: Performance Evaluation
Implement functions to evaluate navigation performance:

```python
class NavigationEvaluator:
    def __init__(self):
        self.metrics = {
            'path_length': 0.0,
            'time_taken': 0.0,
            'collisions': 0,
            'goal_reached': False,
            'path_efficiency': 0.0,
            'navigation_success': False
        }

    def evaluate_navigation(self, path_taken: List[Tuple[float, float]],
                           goal_reached: bool,
                           execution_time: float,
                           collisions: int,
                           start_pos: Tuple[float, float],
                           goal_pos: Tuple[float, float]) -> dict:
        """Evaluate navigation performance"""
        # Calculate path length
        path_length = 0.0
        for i in range(1, len(path_taken)):
            dx = path_taken[i][0] - path_taken[i-1][0]
            dy = path_taken[i][1] - path_taken[i-1][1]
            path_length += np.sqrt(dx**2 + dy**2)

        # Calculate optimal path length (straight line)
        optimal_length = np.sqrt((goal_pos[0] - start_pos[0])**2 +
                                (goal_pos[1] - start_pos[1])**2)

        # Calculate metrics
        path_efficiency = optimal_length / path_length if path_length > 0 else 0

        self.metrics.update({
            'path_length': path_length,
            'time_taken': execution_time,
            'collisions': collisions,
            'goal_reached': goal_reached,
            'path_efficiency': path_efficiency,
            'navigation_success': goal_reached and collisions == 0
        })

        return self.metrics

    def get_navigation_score(self) -> float:
        """Calculate overall navigation score"""
        if not self.metrics['goal_reached']:
            return 0.0

        # Calculate weighted score
        efficiency_score = min(self.metrics['path_efficiency'], 1.0)
        safety_score = 1.0 if self.metrics['collisions'] == 0 else 0.0
        time_score = 1.0 / (1.0 + self.metrics['time_taken'] / 100)  # Normalize time

        # Weighted average (adjust weights as needed)
        score = 0.5 * efficiency_score + 0.3 * safety_score + 0.2 * time_score
        return score
```

## Expected Outcomes

### Technical Outcomes
- Working navigation system that can reach specified goals
- Robust obstacle avoidance in dynamic environments
- Accurate localization and mapping capabilities
- Smooth trajectory following with safety considerations
- Quantitative performance metrics for navigation quality

### Learning Outcomes
- Understanding of complete navigation system architecture
- Experience with multi-sensor integration for navigation
- Knowledge of path planning and motion control
- Skills in system validation and performance evaluation
- Appreciation for real-world navigation challenges

## Assessment Questions

1. **Analysis**: How does your navigation system handle situations where the planned path becomes blocked by unexpected obstacles?

2. **Problem-Solving**: What strategies did you implement to ensure the robot doesn't get stuck in local minima during navigation?

3. **Application**: How would you modify your navigation system to handle environments with poor lighting where the depth camera might be less effective?

4. **Evaluation**: What metrics did you use to evaluate navigation performance, and why are these important for real-world applications?

## Advanced Challenges

1. **Dynamic Obstacle Prediction**: Extend your system to predict the future positions of moving obstacles and plan accordingly

2. **Multi-Goal Navigation**: Implement navigation to multiple goals in an efficient sequence

3. **Exploration**: Add exploration capabilities to discover unknown areas of the environment

4. **Human-Robot Interaction**: Implement navigation that considers human presence and comfort in shared spaces

## Validation and Testing

### Performance Metrics
- **Success Rate**: Percentage of goals reached successfully
- **Path Efficiency**: Ratio of actual path length to optimal path length
- **Safety**: Number of collisions or near-misses
- **Time Efficiency**: Time taken to reach goals
- **Robustness**: Performance under various environmental conditions

### Test Scenarios
1. **Simple Navigation**: Move between two points with no obstacles
2. **Obstacle Avoidance**: Navigate around static obstacles
3. **Dynamic Obstacles**: Handle moving obstacles in the environment
4. **Complex Environments**: Navigate through cluttered or narrow spaces
5. **Long-term Operation**: Extended navigation sessions to test system stability

## Resources
- [ROS Navigation Stack Documentation](http://wiki.ros.org/navigation)
- [Path Planning Algorithms Comparison](https://ieeexplore.ieee.org/document/8202188)
- [SLAM Tutorial](https://www.cs.cmu.edu/~16831-f14/notes/LEC/16831_lecture29_trotz_2014.pdf)
- [Motion Control in Robotics](https://www.springer.com/gp/book/9783319515941)

## Next Steps
After completing this exercise, you will have implemented a complete navigation system. Consider exploring advanced topics such as:
- Learning-based navigation approaches
- Multi-robot coordination
- Navigation in 3D environments
- Integration with higher-level task planning systems