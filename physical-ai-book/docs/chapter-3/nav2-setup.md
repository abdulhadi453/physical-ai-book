# Nav2 Navigation Stack Integration with Isaac Sim

## Overview

This document provides comprehensive instructions for setting up and integrating the Navigation2 (Nav2) stack with NVIDIA Isaac Sim. Nav2 is the navigation stack for ROS 2, providing path planning, navigation, and obstacle avoidance capabilities for mobile robots when integrated with Isaac Sim.

## Introduction to Nav2

### What is Nav2?
Navigation2 (Nav2) is the navigation stack for ROS 2, designed to provide path planning, navigation, and obstacle avoidance capabilities for mobile robots. When integrated with NVIDIA Isaac Sim, it enables realistic simulation of navigation tasks with high-fidelity physics and sensor simulation.

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

## Prerequisites

Before setting up Nav2 integration, ensure you have:
- Completed Isaac Sim and Isaac ROS bridge setup
- Nav2 packages installed in ROS 2 environment
- Robot model with appropriate sensors (LiDAR, camera, IMU)
- Basic understanding of ROS 2 concepts

## Nav2 Installation and Setup

### 1. Verify Nav2 Installation
```bash
# Check if Nav2 packages are installed
dpkg -l | grep nav2

# If not installed, install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox ros-humble-robot-localization
```

### 2. Source ROS Environment
```bash
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash
```

### 3. Test Nav2 Installation
```bash
# Check available Nav2 packages
ros2 pkg list | grep nav2

# Verify Nav2 launch files
ls /opt/ros/humble/share/nav2_bringup/launch/
```

## Nav2 Configuration for Isaac Sim

### 1. Create Nav2 Configuration Directory
```bash
mkdir -p ~/isaac_ros_ws/src/isaac_nav2/config
```

### 2. Create Complete Nav2 Parameters File
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/config/nav2_params.yaml << 'EOF'
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
    enable_groot_monitoring: False
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

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      global_plan_overwrite_orientation: True
      max_global_plan_lookahead_dist: 3.0
      feasibility_check_expected_pose_update_rate: 1.0
      global_plan_prune_distance: 1.0
      use_dwb: True
      max_vel_obstacle: 0.4
      preserve_goal_on_costmap_resize: True
      prune_plan: True
      controller_frequency: 20.0

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
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        enabled: True
        map_subscribe_transient_local: True
      inflation_layer:
        enabled: True
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: True
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_queue_size: 10
    do_multithreading: true
    start_waypoint_at_tolerance: 0.1
EOF
```

### 3. Create Nav2 Map Configuration
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/config/map_params.yaml << 'EOF'
map_server:
  ros__parameters:
    yaml_filename: "turtlebot3_world.yaml"
    save_map_timeout: 5.0
    use_sim_time: True

map_saver:
  ros__parameters:
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True
    use_sim_time: True
EOF
```

## Isaac Sim Robot Configuration for Navigation

### 1. Create Robot Description for Navigation
```bash
mkdir -p ~/isaac_ros_ws/src/isaac_nav2/urdf
cat > ~/isaac_ros_ws/src/isaac_nav2/urdf/carter_nav2.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="carter">
  <xacro:arg name="namespace" default="carter"/>
  <xacro:arg name="use_sim_time" default="true"/>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Base Footprint -->
  <link name="base_footprint"/>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- IMU -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Laser -->
  <link name="laser_frame">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.18 0.0 0.19" rpy="0 0 0"/>
  </joint>

  <!-- Camera -->
  <link name="camera_frame">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_frame"/>
    <origin xyz="0.2 0.0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Wheels -->
  <xacro:macro name="wheel" params="prefix reflect joint_pos_x joint_pos_y">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.04"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <material name="black">
          <color rgba="0.2 0.2 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.04"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${joint_pos_x} ${joint_pos_y} 0.0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <gazebo reference="${prefix}_wheel">
      <mu1 value="100.0"/>
      <mu2 value="100.0"/>
      <kp value="10000000.0"/>
      <kd value="1.0"/>
      <fdir1 value="1 0 0"/>
      <material>Gazebo/Black</material>
    </gazebo>
  </xacro:macro>

  <xacro:wheel prefix="front_left" reflect="1" joint_pos_x="0.15" joint_pos_y="0.15"/>
  <xacro:wheel prefix="front_right" reflect="-1" joint_pos_x="0.15" joint_pos_y="-0.15"/>
  <xacro:wheel prefix="back_left" reflect="1" joint_pos_x="-0.15" joint_pos_y="0.15"/>
  <xacro:wheel prefix="back_right" reflect="-1" joint_pos_x="-0.15" joint_pos_y="-0.15"/>

  <!-- Differential Drive Controller -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>$(arg namespace)</namespace>
        <remapping>cmd_vel:=cmd_vel_nav</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>30</update_rate>
      <left_joint>front_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- Laser Plugin -->
  <gazebo reference="laser_frame">
    <sensor name="laser" type="ray">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>$(arg namespace)</namespace>
          <remapping>out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Plugin -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>$(arg namespace)</namespace>
          <remapping>imu:=imu</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Camera Plugin -->
  <gazebo reference="camera_frame">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>$(arg namespace)</namespace>
          <remapping>image_raw:=camera/color/image_raw</remapping>
          <remapping>camera_info:=camera/color/camera_info</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>
</robot>
EOF
```

## Nav2 Launch Files for Isaac Sim

### 1. Create Launch Directory
```bash
mkdir -p ~/isaac_ros_ws/src/isaac_nav2/launch
```

### 2. Create Nav2 Bringup Launch File for Isaac Sim
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/launch/nav2_isaac_sim_bringup.launch.py << 'EOF'
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    params_file = LaunchConfiguration('params_file', default='nav2_params.yaml')
    namespace = LaunchConfiguration('namespace', default='')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(
        FindPackageShare('isaac_nav2').find('isaac_nav2'),
        'maps', 'turtlebot3_world.yaml'))

    # Package names
    nav2_bringup_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    isaac_nav2_pkg = FindPackageShare('isaac_nav2').find('isaac_nav2')

    # Launch files
    localization_launch_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'localization_launch.py'
    ])

    navigation_launch_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    ])

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='nav2_map_server',
        name='map_server',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}],
        output='screen'
    )

    # Lifecycle manager for map server
    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server']}],
        output='screen'
    )

    # Localization launch (AMCL)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': PathJoinSubstitution([
                FindPackageShare('isaac_nav2'),
                'config',
                params_file
            ])
        }.items()
    )

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': PathJoinSubstitution([
                FindPackageShare('isaac_nav2'),
                'config',
                params_file
            ])
        }.items()
    )

    # Velocity smoother for navigation commands
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[{
            'use_sim_time': use_sim_time,
            'smoothed_vel_topic': '/cmd_vel_smoothed',
            'velocity_scaling_topic': '/navigation_velocity_smoother/raw_cmd_vel',
            'feedback_topic': '/odom',
            'robot_base_frame': 'base_link',
            'publish_odom': True,
            'publish_tf': False,
            'forward_sampling_horizon': 0.1,
            'angular_dist_threshold': 0.01,
            'velocity_threshold': 0.01,
            'max_velocity': 1.0,
            'min_velocity': -1.0,
            'max_accel': 1.0,
            'max_decel': -1.0
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/smoothed_cmd_vel', '/cmd_vel')
        ]
    )

    # Transform publisher for navigation frames
    nav_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='nav_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value='nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=os.path.join(isaac_nav2_pkg, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load'))

    ld.add_action(DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'))

    # Add nodes and launch descriptions
    ld.add_action(lifecycle_manager_map_server)
    ld.add_action(map_server_node)
    ld.add_action(TimerAction(
        period=3.0,
        actions=[localization_launch]
    ))
    ld.add_action(TimerAction(
        period=6.0,
        actions=[navigation_launch]
    ))
    ld.add_action(velocity_smoother_node)
    ld.add_action(nav_transform_publisher)

    return ld
EOF
```

### 3. Create Isaac Sim Navigation Demo Launch File
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/launch/isaac_nav2_demo.launch.py << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    params_file = LaunchConfiguration('params_file', default='nav2_params.yaml')

    # Package names
    isaac_nav2_pkg = FindPackageShare('isaac_nav2').find('isaac_nav2')

    # Launch Isaac Sim navigation bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('isaac_nav2'),
                'launch',
                'nav2_isaac_sim_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # RViz2 for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('isaac_nav2'),
        'rviz',
        'nav2_isaac_sim_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation 2D pose estimate node (for initial pose setting)
    initial_pose_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_initial_pose',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['initial_pose_publisher']}],
        output='screen'
    )

    # Return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value='nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    # Add nodes and launch descriptions
    ld.add_action(nav2_bringup_launch)
    ld.add_action(TimerAction(
        period=10.0,
        actions=[rviz_node]
    ))
    ld.add_action(initial_pose_node)

    return ld
EOF
```

## RViz Configuration for Navigation

### 1. Create RViz Directory
```bash
mkdir -p ~/isaac_ros_ws/src/isaac_nav2/rviz
```

### 2. Create RViz Configuration for Isaac Sim Navigation
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/rviz/nav2_isaac_sim_view.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /TF1/Frames1
        - /LaserScan1
        - /Map1
        - /Local Costmap1
        - /Global Costmap1
        - /Global Planner1
        - /Local Planner1
        - /RobotModel1
      Splitter Ratio: 0.5833333134651184
    Tree Height: 857
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      Robot Description: robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
        base_footprint:
          Value: true
        base_link:
          Value: true
        camera_frame:
          Value: true
        imu_link:
          Value: true
        laser_frame:
          Value: true
        map:
          Value: true
        odom:
          Value: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        map:
          odom:
            base_footprint:
              base_link:
                camera_frame:
                  {}
                imu_link:
                  {}
                laser_frame:
                  {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Local Costmap
      Topic:
        Depth: 1
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Global Costmap
      Topic:
        Depth: 1
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Global Planner
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_plan
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 255; 0; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Local Planner
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_plan
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: MarkerArray
      Namespaces:
        {}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /waypoints
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.7853981852531433
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.7853981852531433
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1043
  Hide Left Dock: false
  Hide Right Dock: true
  Navigation 2:
    collapsed: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001560000039ffc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000039f000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f0000039ffc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d0000039f000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000003910000039f00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: true
  Width: 1853
  X: 67
  Y: 27
EOF
```

## Isaac Sim Navigation Integration Package

### 1. Create Isaac Sim Navigation Package
```bash
cd ~/isaac_ros_ws/src
source /opt/ros/humble/setup.bash

# Create package for Isaac Sim navigation integration
ros2 pkg create --build-type ament_cmake isaac_nav2 --dependencies rclcpp rclpy nav2_common nav2_msgs nav2_bringup
```

### 2. Create CMakeLists.txt
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(isaac_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps if any
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and cpplint is enabled
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF
```

### 3. Create Package XML
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_nav2</name>
  <version>0.0.1</version>
  <description>Isaac Sim integration with Navigation2 stack</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>nav2_common</depend>
  <depend>nav2_msgs</depend>
  <depend>nav2_bringup</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

## Navigation Performance and Testing

### 1. Create Navigation Performance Test Script
```bash
cat > ~/test_nav2_integration.sh << 'EOF'
#!/bin/bash

# Test script for Isaac Sim Nav2 integration

echo "Testing Isaac Sim Nav2 integration..."

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check if Nav2 packages are available
echo "Checking Nav2 packages..."
ros2 pkg list | grep nav2

if [ $? -eq 0 ]; then
    echo "✓ Nav2 packages found"
else
    echo "✗ Nav2 packages not found"
    exit 1
fi

# Check if Isaac Nav2 package is available
echo "Checking Isaac Nav2 package..."
ros2 pkg list | grep isaac_nav2

if [ $? -eq 0 ]; then
    echo "✓ Isaac Nav2 package found"
else
    echo "✗ Isaac Nav2 package not found"
    exit 1
fi

# Test Nav2 launch files
echo "Checking Nav2 launch files..."
ls ~/isaac_ros_ws/install/isaac_nav2/share/isaac_nav2/launch/ | grep -i nav

if [ $? -eq 0 ]; then
    echo "✓ Nav2 launch files found"
else
    echo "✗ Nav2 launch files not found"
    exit 1
fi

# Build the Isaac Nav2 package
echo "Building Isaac Nav2 package..."
cd ~/isaac_ros_ws
colcon build --packages-select isaac_nav2
source install/setup.bash

echo "Isaac Sim Nav2 integration test completed."
EOF

# Make executable
chmod +x ~/test_nav2_integration.sh
```

### 2. Run Navigation Integration Test
```bash
~/test_nav2_integration.sh
```

## Navigation Behavior Trees Configuration

### 1. Create Custom Behavior Trees
```bash
mkdir -p ~/isaac_ros_ws/src/isaac_nav2/behavior_trees
```

### 2. Create Navigation Behavior Tree with Isaac-specific Recovery
```bash
cat > ~/isaac_ros_ws/src/isaac_nav2/behavior_trees/navigate_to_pose_w_isaac_recovery.xml << 'EOF'
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <ReactiveSequence name="task_sequence">
        <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="{goal_radius}">
          <ComputePathToPose goal="{current_goal}" path="{path}" planner_id="GridBased">
            <PipelineSequence name="global_pipeline">
              <RateController hz="1.0">
                <IsPathValid path="{path}"/>
              </RateController>
            </PipelineSequence>
          </ComputePathToPose>
          <FollowPath path="{path}" controller_id="FollowPath">
            <PipelineSequence name="local_pipeline">
              <RateController hz="20.0">
                <IsPathValid path="{path}"/>
              </RateController>
            </PipelineSequence>
          </FollowPath>
        </RemovePassedGoals>
      </ReactiveSequence>
      <ReactiveFallback name="global_recover">
        <GoalUpdated/>
        <Sequence name="global_recovery">
          <ClearEntireCostmap name="global_clear" service_name="global_costmap/clear_entirely_global_costmap"/>
          <Wait wait_duration="5"/>
        </Sequence>
      </ReactiveFallback>
    </Sequence>
  </BehaviorTree>
</root>
EOF
```

## Troubleshooting Nav2 Integration

### Common Issues and Solutions

#### Issue: "Nav2 nodes not starting properly"
**Solution**: Check parameters and timing
```bash
# Verify parameters file
ros2 param list | grep nav2

# Check timing issues
echo "use_sim_time should be set to true in all nodes"
```

#### Issue: "Robot not moving in Isaac Sim"
**Solution**: Check TF tree and command remapping
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Check command topic remapping
ros2 topic echo /cmd_vel
```

#### Issue: "Costmaps not updating"
**Solution**: Verify sensor data and frame IDs
```bash
# Check if sensor data is available
ros2 topic echo /scan

# Verify frame IDs in costmap configuration
grep -r "frame" ~/isaac_ros_ws/src/isaac_nav2/config/
```

## Verification Checklist

- [ ] Nav2 packages installed and configured
- [ ] Isaac Sim robot description created
- [ ] Nav2 parameters file created and configured
- [ ] Launch files created for Isaac Sim integration
- [ ] RViz configuration created for navigation visualization
- [ ] Isaac Nav2 package created and built
- [ ] Custom behavior trees configured
- [ ] Test script created and functional
- [ ] Troubleshooting guide reviewed

## Next Steps

After implementing Nav2 integration:

1. **Test navigation** with Isaac Sim
2. **Run navigation exercises** from Lesson 2
3. **Integrate with perception system** for obstacle avoidance
4. **Create navigation assessment tasks**

The Nav2 navigation stack is now integrated with Isaac Sim and ready for Module 3, providing students with a complete navigation system for AI-robot applications.