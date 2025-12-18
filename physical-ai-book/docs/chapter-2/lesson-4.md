---
sidebar_position: 7
---

# Gazebo Physics Environment Setup Guide

## Overview
This lesson provides a comprehensive guide to setting up physics simulation environments using Gazebo, a powerful open-source robotics simulator. Gazebo provides accurate and efficient simulation of robots in complex indoor and outdoor environments.

## Learning Objectives
By the end of this lesson, you will be able to:
- Install and configure Gazebo for physics simulation
- Create basic simulation environments with custom physics parameters
- Configure gravity, friction, and collision properties
- Validate physics behavior in the simulation environment

## Prerequisites
- Basic understanding of physics simulation concepts
- Linux or Windows development environment
- Familiarity with ROS/ROS2 (optional but recommended)

## Gazebo Installation

### Ubuntu Installation
```bash
sudo apt update
sudo apt install gazebo libgazebo-dev
```

### Windows Installation
1. Download the latest Gazebo release from the official website
2. Follow the installation wizard
3. Verify installation by running `gz sim --version`

### macOS Installation
```bash
brew install gazebo
```

## Basic Environment Creation

### Creating a New World
1. Launch Gazebo:
   ```bash
   gz sim
   ```

2. Create a new world file (`.sdf` format):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="my_world">
       <physics type="ode">
         <gravity>0 0 -9.8</gravity>
       </physics>
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>
     </world>
   </sdf>
   ```

### Physics Engine Configuration
Gazebo supports multiple physics engines:
- **ODE** (Open Dynamics Engine): Default, good for most applications
- **Bullet**: Good for complex collision detection
- **Simbody**: Advanced multibody dynamics

## Physics Parameters Configuration

### Gravity Settings
The gravity vector defines the gravitational force in the simulation:
```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
</physics>
```
- `0 0 -9.8`: Earth's gravity (default)
- `0 0 0`: Zero gravity environment
- `0 0 -1.6`: Moon's gravity (~1/6 of Earth's)

### Friction Properties
Configure surface friction for realistic object interactions:
```xml
<material>
  <pbr>
    <metal>
      <albedo_map>file://media/materials/textures/pnc_road.png</albedo_map>
    </metal>
  </pbr>
  <script>
    <uri>file://media/materials/scripts/gazebo.material</uri>
    <name>Gazebo/White</name>
  </script>
</material>
```

### Collision Detection
Fine-tune collision detection parameters:
```xml
<collision>
  <max_contacts>10</max_contacts>
  <surface>
    <contact>
      <ode>
        <max_vel>100.0</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

## Practical Example: Simple Physics Environment

Let's create a simple environment with a ball and a plane:

1. Create a world file `simple_physics.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_physics">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ball model -->
    <model name="ball">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.004</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.004</iyy>
            <iyz>0</iyz>
            <izz>0.004</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

2. Launch the world:
```bash
gz sim simple_physics.sdf
```

## Physics Validation

### Checking Physics Behavior
1. Verify that objects fall with the expected acceleration
2. Test collision responses between different objects
3. Validate friction coefficients by testing object movement
4. Check for stability in complex multi-object scenarios

### Performance Optimization
- Adjust `max_step_size` for accuracy vs performance trade-off
- Use appropriate `real_time_factor` for real-time or faster simulation
- Optimize collision meshes for complex objects
- Limit the number of simultaneous physics interactions

## Troubleshooting Common Issues

### Physics Instability
- **Symptom**: Objects vibrating or flying apart
- **Solution**: Reduce `max_step_size` or adjust solver parameters

### Slow Performance
- **Symptom**: Low simulation speed
- **Solution**: Simplify collision meshes or reduce physics complexity

### Unexpected Behavior
- **Symptom**: Objects not responding as expected
- **Solution**: Check mass properties and collision geometries

## Advanced Physics Features

### Custom Physics Plugins
Gazebo supports custom physics plugins for specialized behaviors:
```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class CustomPhysicsPlugin : public gazebo::WorldPlugin
{
  public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Custom physics implementation
  }
};
```

### Real-time Physics Adjustment
Use Gazebo's service interface to adjust physics parameters during simulation:
```bash
# Change gravity during simulation
rosservice call /gazebo/set_physics_properties "time_step: 0.001
max_update_rate: 1000.0
gravity: {x: 0.0, y: 0.0, z: -9.8}
ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, ode_m_erp: 0.2, ode_cfm: 0.0, ode_prop_erp: 0.2, ode_prop_cfm: 0.0}"
```

## Best Practices

### Physics Parameter Selection
- Start with default parameters and adjust as needed
- Test with simple scenarios before complex ones
- Document parameter changes for reproducibility
- Consider the target hardware performance

### Environment Design
- Use realistic physical properties for accurate simulation
- Validate simulation results against theoretical models
- Include environmental factors (wind, friction variations)
- Plan for scalability to multiple objects

## Summary
Gazebo provides a robust platform for physics simulation with extensive configuration options. Understanding and properly configuring physics parameters is crucial for creating realistic and stable simulation environments.

## Next Steps
After mastering Gazebo physics setup, proceed to the Unity physics environment setup to understand cross-platform simulation approaches.