# Quickstart Guide: Module 1 - Foundations of Physical AI

## Overview

This quickstart guide provides the essential steps to set up and begin working with Module 1: Foundations of Physical AI. This module focuses on introducing students to fundamental concepts of Physical AI and embodied intelligence with a simulation-first approach using ROS 2, Gazebo, and NVIDIA Isaac platforms.

## Prerequisites

### System Requirements
- Operating System: Ubuntu 22.04 LTS (recommended) or equivalent Linux distribution
- RAM: 8GB minimum (16GB recommended)
- CPU: 4 cores minimum (8 cores recommended)
- Storage: 20GB free space
- Internet connection for initial setup

### Software Dependencies
- Node.js 20+ and npm
- Python 3.8-3.11
- ROS 2 Humble Hawksbill
- Gazebo Garden
- Git

## Setup Steps

### 1. Clone the Repository
```bash
git clone https://github.com/physical-ai-book/hackathon-book.git
cd hackathon-book/physical-ai-book
```

### 2. Install Node.js Dependencies
```bash
npm install
```

### 3. Set up ROS 2 Environment
```bash
# Install ROS 2 Humble Hawksbill (follow official installation guide)
# Source the ROS 2 setup script
source /opt/ros/humble/setup.bash
```

### 4. Install Additional Python Dependencies
```bash
pip3 install -r requirements.txt  # if available in the project
```

### 5. Start the Docusaurus Development Server
```bash
npm start
```

The development server will start and open your browser to http://localhost:3000 where you can view the Module 1 content.

## Module 1 Structure

Module 1: Foundations of Physical AI is organized into the following sections:

### Section 1.1: Introduction to Physical AI and Embodied Intelligence
- **Learning Objectives**: Define Physical AI, distinguish from traditional AI
- **Content**: Theoretical concepts, real-world context, and foundational exercises
- **Simulation Exercise**: Basic perception-action loop demonstration

### Section 1.2: Core Principles of Perception-Action Loops
- **Learning Objectives**: Explain perception-action loop, identify components
- **Content**: Theoretical concepts with robotics applications
- **Simulation Exercise**: Simple sensor-actuator interaction

### Section 1.3: Mathematical Foundations for Robotics and AI Integration
- **Learning Objectives**: Apply basic math to robotics problems
- **Content**: Mathematical concepts applied to robotics
- **Simulation Exercise**: Mathematical modeling of simple robot movement

### Section 1.4: Introduction to Robot Embodiment and Its Role in Intelligence
- **Learning Objectives**: Explain embodiment effects on intelligence
- **Content**: Theoretical concepts about physical form and intelligence
- **Simulation Exercise**: Embodiment impact demonstration

## Simulation Environment Setup

### ROS 2 Workspace Setup
```bash
# Create a workspace for Module 1 exercises
mkdir -p ~/module1_ws/src
cd ~/module1_ws
colcon build
source install/setup.bash
```

### Running Simulation Exercises
Each section includes simulation exercises that can be run in the appropriate environment:

```bash
# Example for running a basic simulation
cd ~/module1_ws
source install/setup.bash
ros2 launch <package_name> <launch_file>.py
```

## Assessment and Progress Tracking

### Formative Assessments
- Each section includes 3-5 questions to check understanding
- Immediate feedback is provided for each answer
- Track your progress through the built-in progress indicators

### Summative Assessment
- Complete Module 1 with a comprehensive assessment
- Achieve 85% or higher to demonstrate mastery
- Prepare for Module 2 readiness with the final assessment

## Key Components

### Custom Docusaurus Components
- `ExerciseBox`: Interactive exercise containers with instructions and validation
- `ConceptCard`: Theoretical concept explanations with examples
- `ResourceLink`: Curated resources with difficulty ratings
- `PrerequisiteIndicator`: Track required knowledge before proceeding
- `SummarySection`: Key takeaways and next steps

### AI-Enhanced Features
- RAG-based chatbot for personalized learning support
- Intelligent tutoring system for concept clarification
- Adaptive content based on learning progress

## Getting Help

### Documentation
- Check the main textbook introduction for general setup
- Review the contributing guide for content structure details
- Refer to the component documentation for interactive features

### Community Support
- Use GitHub Discussions for questions about Physical AI concepts
- Report issues with exercises or content through GitHub Issues
- Contribute improvements through pull requests

## Next Steps

After completing Module 1: Foundations of Physical AI, you will be prepared to:

1. Progress to Module 2: Simulation and Control Systems (ROS 2, Gazebo, NVIDIA Isaac)
2. Apply Physical AI concepts to more complex robotic systems
3. Use your simulation-first foundation for advanced robotics learning

## Troubleshooting

### Common Issues

**Problem**: Docusaurus server fails to start
**Solution**: Ensure all Node.js dependencies are installed with `npm install`

**Problem**: Simulation exercises don't run
**Solution**: Verify ROS 2 Humble Hawksbill is properly installed and sourced

**Problem**: Assessment scores not saving
**Solution**: Check that your browser allows local storage for the application

### Performance Tips

- Close unnecessary applications to free up system resources for simulations
- Use a wired internet connection for downloading large simulation assets
- Monitor system resources during complex simulation exercises