---
sidebar_position: 9
---

# Physics Parameters Configuration Guide

## Overview
This lesson provides a comprehensive guide to configuring physics parameters in simulation environments. Proper configuration of physics parameters is crucial for achieving realistic and stable simulations that accurately represent real-world physics behavior.

## Learning Objectives
By the end of this lesson, you will be able to:
- Understand the fundamental physics parameters that affect simulation behavior
- Configure gravity, friction, and collision properties appropriately
- Optimize physics parameters for different simulation scenarios
- Validate and tune physics parameters for accuracy and performance

## Key Physics Parameters

### Gravity
Gravity is the fundamental force that affects all objects with mass in the simulation:

**Gazebo Configuration:**
```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>  <!-- Earth's gravity -->
</physics>
```

**Unity Configuration:**
```
Edit > Project Settings > Physics > Gravity
Default: (0, -9.81, 0)
```

**Common Gravity Values:**
- Earth: (0, -9.81, 0) m/s²
- Moon: (0, -1.62, 0) m/s²
- Mars: (0, -3.71, 0) m/s²
- Zero Gravity: (0, 0, 0) m/s²

### Mass and Inertia
Mass determines how objects respond to forces, while inertia affects rotational behavior:

**Gazebo Configuration:**
```xml
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
```

**Unity Configuration:**
- Set via Rigidbody component's Mass property
- Inertia is automatically calculated based on shape and mass

### Friction Properties
Friction affects how objects interact when sliding against each other:

**Gazebo Configuration:**
```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>      <!-- Static friction coefficient -->
      <mu2>1.0</mu2>    <!-- Dynamic friction coefficient -->
    </ode>
  </friction>
</surface>
```

**Unity Configuration:**
- Use Physic Materials with Static/Dynamic Friction values
- Range: 0 (no friction) to 1+ (high friction)

### Bounce and Restitution
Bounce properties determine how objects behave when colliding:

**Gazebo Configuration:**
```xml
<surface>
  <bounce>
    <restitution_coefficient>0.5</restitution_coefficient>
  </bounce>
</surface>
```

**Unity Configuration:**
- Set Bounciness property in Physic Materials
- Range: 0 (no bounce) to 1+ (perfectly elastic)

## Parameter Optimization Strategies

### Accuracy vs Performance Trade-offs

**Time Step Configuration:**
- **Smaller time steps**: More accurate but slower simulation
- **Larger time steps**: Faster but potentially unstable

**Gazebo:**
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller = more accurate -->
  <real_time_factor>1</real_time_factor>
</physics>
```

**Unity:**
- Edit > Project Settings > Time > Fixed Timestep
- Default: 0.02 seconds (50 FPS physics)

### Solver Configuration
The physics solver determines how accurately forces and collisions are calculated:

**Gazebo (ODE Solver):**
- **solver_iterations**: Higher values = more stable but slower
- **sor_pgs_w**: Successive Over-Relaxation parameter (typically 1.3)

**Unity:**
- **Solver Iteration Count**: Higher values = more stable but slower
- **Default**: 6 for velocity, 1 for position

## Environment-Specific Parameter Tuning

### Indoor Environments
- **Gravity**: Standard Earth gravity (0, -9.81, 0)
- **Friction**: Higher values (0.7-1.0) for surfaces like carpet, wood
- **Damping**: Moderate linear and angular damping for stability

### Outdoor Environments
- **Gravity**: Standard Earth gravity, but consider terrain variations
- **Friction**: Variable based on surface type (grass, dirt, concrete)
- **Environmental forces**: Consider wind effects

### Microgravity Environments
- **Gravity**: Very low or zero (0, 0, 0) for space simulations
- **Friction**: Less critical but still important for contact interactions
- **Damping**: Often increased to prevent perpetual motion

## Practical Configuration Examples

### Example 1: Warehouse Robot Simulation
```xml
<!-- Gazebo World Configuration -->
<world name="warehouse">
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Concrete floor -->
  <model name="floor">
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>
          <mu2>0.6</mu2>
        </ode>
      </friction>
    </surface>
  </model>

  <!-- Robot with rubber wheels -->
  <model name="robot">
    <surface>
      <friction>
        <ode>
          <mu>0.9</mu>
          <mu2>0.8</mu2>
        </ode>
      </friction>
    </surface>
  </model>
</world>
```

### Example 2: Mobile Robot with Sensors
```csharp
// Unity Robot Configuration
public class RobotPhysicsConfig : MonoBehaviour
{
    public float robotMass = 10f;
    public float wheelFriction = 0.8f;
    public float linearDamping = 0.1f;
    public float angularDamping = 0.1f;

    void Start()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.mass = robotMass;
        rb.drag = linearDamping;
        rb.angularDrag = angularDamping;

        // Configure wheel colliders with appropriate friction
        ConfigureWheels();
    }

    void ConfigureWheels()
    {
        // Set up wheel-specific physics properties
        // for realistic robot movement
    }
}
```

## Validation and Testing

### Theoretical Validation
Compare simulation results with theoretical physics equations:
- **Free fall**: Distance = 0.5 × gravity × time²
- **Projectile motion**: Trajectory calculations
- **Collision response**: Conservation of momentum

### Experimental Validation
- Compare with real-world measurements when possible
- Use standardized test scenarios
- Validate across different parameter ranges

### Performance Validation
- Monitor simulation stability over time
- Check for energy conservation in closed systems
- Verify parameter changes produce expected behavior changes

## Common Parameter Tuning Challenges

### Instability Issues
**Symptoms:**
- Objects vibrating or shaking
- Objects flying apart unexpectedly
- Simulation becoming chaotic

**Solutions:**
- Reduce time step size
- Increase solver iterations
- Lower mass ratios between objects
- Add appropriate damping

### Performance Issues
**Symptoms:**
- Slow simulation speed
- High CPU usage
- Frame rate drops

**Solutions:**
- Increase time step size (within stability limits)
- Reduce solver iterations
- Simplify collision meshes
- Limit the number of simultaneous interactions

### Accuracy Issues
**Symptoms:**
- Objects not behaving as expected
- Energy not conserved
- Inconsistent results

**Solutions:**
- Decrease time step size
- Increase solver iterations
- Verify mass and inertia properties
- Check for configuration errors

## Advanced Parameter Configuration

### Dynamic Parameter Adjustment
Adjust physics parameters during simulation:

**Gazebo Service Call:**
```bash
ros2 service call /world/set_physics_properties gazebo_msgs/srv/SetPhysicsProperties "{gravity: {x: 0.0, y: 0.0, z: -4.9}}"
```

**Unity Runtime Adjustment:**
```csharp
Physics.gravity = new Vector3(0, -4.9f, 0);  // Moon gravity
```

### Parameter Sweep Studies
Systematically vary parameters to understand their effects:
- Gravity from 0 to 19.6 m/s²
- Friction coefficients from 0.1 to 1.5
- Time step sizes from 0.001 to 0.01

## Quality Assurance for Physics Parameters

### Documentation Standards
- Record all parameter values used
- Document the rationale for chosen values
- Note any deviations from standard values
- Include validation results

### Reproducibility
- Use version control for configuration files
- Document software versions used
- Include environmental conditions
- Provide clear parameter initialization procedures

## Best Practices Summary

1. **Start with defaults**: Begin with standard physics parameters
2. **Validate incrementally**: Test simple scenarios before complex ones
3. **Document changes**: Keep track of parameter modifications
4. **Balance accuracy and performance**: Find optimal trade-offs
5. **Test edge cases**: Verify behavior under extreme conditions
6. **Compare with theory**: Validate against known physics equations

## Summary
Proper configuration of physics parameters is essential for creating realistic and stable simulation environments. Understanding the relationships between different parameters and their effects on simulation behavior enables the creation of accurate digital twins for robotics applications.

## Next Steps
After mastering physics parameter configuration, proceed to learn about sensor simulation to complete the digital twin environment setup.