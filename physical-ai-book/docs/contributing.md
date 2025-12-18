---
sidebar_position: 100
title: 'Contributing'
---

# Contributing to AI-Native Textbook for Physical AI & Humanoid Robotics

We welcome contributions from the community to help improve this AI-Native Textbook for Physical AI & Humanoid Robotics! This guide explains how you can contribute to the project, which is designed for the competitive hackathon focused on creating AI-enhanced educational materials.

## Ways to Contribute

### Content Contributions
- Adding new lessons on ROS 2, Gazebo, and NVIDIA Isaac platforms
- Improving existing content on Physical AI and humanoid robotics concepts
- Creating hands-on exercises for Vision-Language-Action integration
- Developing examples for humanoid locomotion and control systems
- Contributing AI-tutoring content and interactive elements

### Technical Contributions
- Enhancing AI integration features (RAG-based chatbots, personalization)
- Improving simulation integration with ROS 2 and Gazebo
- Adding NVIDIA Isaac platform components and examples
- Creating multilingual support features
- Building AI-agent components for personalized learning

### Community Contributions
- Answering questions in discussions about Physical AI concepts
- Reviewing pull requests for technical accuracy
- Providing feedback on new AI-enhanced learning features
- Translating content to other languages for global accessibility
- Sharing best practices for humanoid robotics education

## Getting Started

1. Fork the repository
2. Create a new branch for your contribution (e.g., `feature/chapter-3-lesson-2`)
3. Make your changes following the content guidelines below
4. Test your changes locally to ensure proper rendering
5. Submit a pull request with a clear description of your contribution
6. Link any relevant issues or user stories

## Content Guidelines

### Lesson Structure
All lessons should follow the 8-component structure designed for Physical AI education:
1. Learning Objectives (specific and measurable)
2. Prerequisites (clearly defined requirements)
3. Theoretical Concepts (with Physical AI focus)
4. Real World Context (practical applications)
5. Hands-On Exercise (simulation-based when possible)
6. Exercise Solutions (with explanations)
7. Summary (key takeaways and next steps)
8. Further Reading (with difficulty ratings)

### Physical AI & Robotics Focus
- Emphasize the connection between AI and physical systems
- Include examples using ROS 2, Gazebo, and NVIDIA Isaac
- Address Vision-Language-Action integration where relevant
- Consider safety and ethics in physical AI systems
- Focus on simulation-first approaches with optional real hardware

### Writing Style
- Use clear, accessible language for CS/AI background learners
- Explain robotics concepts with practical examples
- Include code snippets and configuration examples
- Provide multiple modalities for concept understanding
- Connect theory to real-world Physical AI applications

## Technical Requirements

### Development Setup
```bash
# Clone your fork
git clone https://github.com/YOUR-USERNAME/hackathon-book.git

# Navigate to the project directory
cd hackathon-book/physical-ai-book

# Install dependencies
npm install

# Start the development server
npm start
```

### Component Usage
When creating lessons, use the custom components:
- `ExerciseBox` for hands-on exercises with clear instructions
- `ConceptCard` for important Physical AI concepts
- `ResourceLink` for further reading with difficulty ratings
- `PrerequisiteIndicator` for tracking learning requirements
- `SummarySection` for comprehensive lesson summaries

### AI Integration
- Consider how to integrate AI tutoring capabilities
- Include examples that work with RAG-based systems
- Structure content for personalization and adaptive learning
- Format content for multilingual support

## Chapter-Specific Guidelines

### Chapter 1: Foundations of Physical AI (Module 1)
- Focus on foundational concepts of Physical AI and embodied intelligence
- Cover perception-action loops, mathematical foundations, and embodiment principles
- Emphasize the difference from traditional AI and the importance of physical interaction
- Include simulation exercises that demonstrate core Physical AI concepts
- Ensure content aligns with the 8-component lesson structure (Learning Objectives, Prerequisites, Theoretical Concepts, Real World Context, Hands-On Exercise, Exercise Solutions, Summary, Further Reading)
- Include assessment questions that validate understanding per functional requirement FR8

### Chapter 2: Simulation and Control Systems
- Include ROS 2 architecture and communication patterns
- Provide Gazebo simulation examples
- Cover NVIDIA Isaac integration basics

### Chapter 3: Vision-Language-Action Integration
- Emphasize multimodal perception systems
- Include language-guided control examples
- Cover action planning and execution

### Chapter 4: Humanoid Systems and Locomotion
- Address humanoid kinematics and dynamics
- Include walking and balance control
- Cover human-robot interaction principles

### Chapter 5: Advanced Applications and Deployment
- Focus on real-world deployment considerations
- Address safety and ethics in physical AI
- Cover simulation-to-reality transfer

## Pull Request Process

1. Describe your changes with specific impact on Physical AI learning
2. Link any relevant hackathon requirements or objectives
3. Ensure all changes follow the content guidelines above
4. Test that all code examples and exercises work as expected
5. Wait for review and address feedback from maintainers
6. Your PR will be merged once approved and tested

## Questions?

If you have questions about contributing to this Physical AI & Humanoid Robotics textbook, feel free to open a discussion in the GitHub repository or contact the maintainers. We're excited to work together to create an exceptional AI-native learning resource!