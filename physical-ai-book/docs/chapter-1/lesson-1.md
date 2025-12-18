---
sidebar_position: 2
title: 'Module 1: Communication Primitives (Topics, Services, Actions)'
description: Understanding ROS 2 communication patterns and their applications
tags: [ros2, communication, topics, services, actions, ai-integration]
---

# Communication Primitives (Topics, Services, Actions)

## Learning Objectives
- Students will be able to differentiate between ROS 2 communication patterns (nodes, topics, services, actions)
- Students will understand when to use each communication primitive for specific use cases
- Students will demonstrate the ability to select appropriate patterns for given scenarios

## Prerequisites
- Understanding of ROS 2 architecture from Module 1 intro
- Basic Python knowledge

## Content (500 words max)

ROS 2 provides three primary communication patterns that enable different types of interactions between nodes: topics, services, and actions. Each pattern serves a specific purpose and is suited for different types of communication needs.

**Topics** implement a publish-subscribe pattern where data flows continuously from publishers to subscribers. This is ideal for sensor data streams, state updates, or any information that needs to be broadcast continuously. For example, a camera node publishes image data to a topic, and multiple nodes (object detection, visualization, storage) can subscribe to receive the same stream. Topics are asynchronous and unidirectional, making them perfect for real-time data distribution.

**Services** implement a request-response pattern where a client sends a request and waits for a response from a server. This is ideal for operations that have a clear start and end, like saving a map, transforming coordinates, or performing a computation. The communication is synchronous, meaning the client waits for the server to complete the operation before continuing.

**Actions** are designed for long-running tasks that require feedback and the ability to cancel. They combine aspects of both topics and services, providing goal specification, continuous feedback, and result reporting. Actions are perfect for navigation tasks, where you want to send a goal (navigate to point X), receive continuous feedback (progress updates), and potentially cancel the operation if needed.

Understanding when to use each pattern is crucial for effective robotic system design. Use topics for continuous data streams, services for discrete operations with clear input/output, and actions for long-running tasks that need monitoring and control.

## AI Agent Interaction Points
- **Explanation**: AI agents can provide additional examples of when to use each communication pattern
- **Debugging**: AI agents can help troubleshoot communication pattern selection issues
- **Examples**: AI agents can provide real-world examples from humanoid robotics

## Urdu Translation Markers
- **Technical Term**: Topics - ٹاپکس
- **Technical Term**: Services - سروسز
- **Technical Term**: Actions - ایکشنز
- **Key Concept**: Publish-Subscribe - پبلش-سبسکرائب

## Exercise
Create a scenario where a humanoid robot needs to navigate to a location, avoid obstacles, and report its status. Identify which communication patterns (topics, services, actions) would be appropriate for each part of the interaction and justify your choices.

## Exercise Solution and Discussion
The navigation task would use actions (for long-running navigation with feedback), sensor data would use topics (for continuous obstacle detection), and map queries might use services (for discrete location lookups). Common mistakes include using services for continuous data or topics for goal-oriented tasks.

## Summary and Key Takeaways
- Topics: Continuous data streams, publish-subscribe pattern
- Services: Request-response for discrete operations
- Actions: Long-running tasks with feedback and cancellation
- Pattern selection is crucial for effective system design

## Further Reading
- [ROS 2 Topics Documentation](https://docs.ros.org/en/humble/Concepts/About-Topics.html)
- [ROS 2 Services Documentation](https://docs.ros.org/en/humble/Concepts/About-Services.html)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Concepts/About-Actions.html)

## Assessment
1. Differentiate between ROS 2 topics, services, and actions.
2. Identify appropriate use cases for each communication pattern.
3. Explain when to use each pattern in a humanoid robot system.