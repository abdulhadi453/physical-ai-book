---
sidebar_position: 4
---

# Exercise 1: Basic Environment Creation

## Overview
In this exercise, you will create a basic simulation environment with physics properties and test simple robot movements. This exercise will help you understand the fundamentals of physics simulation in both Gazebo and Unity environments.

## Learning Objectives
By the end of this exercise, you will be able to:
- Set up a basic physics simulation environment
- Configure fundamental physics parameters (gravity, friction, collision properties)
- Create simple geometric objects in the simulation
- Test basic robot movement in the environment

## Prerequisites
- Basic understanding of physics simulation concepts
- Familiarity with either Gazebo or Unity (or both)
- Access to the simulation environment of your choice

## Exercise Steps

### Step 1: Environment Setup
1. Launch your chosen simulation environment (Gazebo or Unity)
2. Create a new empty scene/world
3. Configure basic physics parameters:
   - Set gravity to Earth's standard (9.81 m/s²)
   - Set default friction coefficients
   - Configure collision detection parameters

### Step 2: Basic Object Creation
1. Create a ground plane with appropriate physical properties
2. Add a simple geometric object (cube, sphere, or cylinder)
3. Configure the object's physical properties:
   - Mass
   - Material properties
   - Collision shapes

### Step 3: Robot Integration
1. Add a basic robot model to the environment
2. Ensure the robot responds to gravity
3. Test basic movement commands

### Step 4: Physics Validation
1. Run the simulation
2. Observe how objects interact with each other and the environment
3. Verify that physics parameters are correctly applied

## Expected Outcomes
- The environment should render with realistic physics behavior
- Objects should respond appropriately to simulated forces
- The robot should move as expected based on commands
- Physics interactions should be stable and predictable

## Assessment Questions
1. What happens when you modify the gravity parameter in your simulation?
2. How do different friction coefficients affect object movement?
3. What are the key factors that determine collision behavior?

## Advanced Challenges
- Try creating multiple objects with different physical properties
- Experiment with different environmental conditions (e.g., zero gravity)
- Test how objects of different masses interact

## Resources
- [Gazebo Physics Documentation](http://gazebosim.org/tutorials?tut=physics)
- [Unity Physics Documentation](https://docs.unity3d.com/Manual/PhysicsSection.html)
- Physics simulation best practices

## Next Steps
After completing this exercise, proceed to Exercise 2: Sensor Integration to learn about sensor simulation in digital twin environments.