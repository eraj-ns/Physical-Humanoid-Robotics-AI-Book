# Chapter 6 Outline: The AI to ROS Bridge

## Learning Objectives
- Understand the importance of decoupling the AI from the ROS 2 system.
- Design a ROS 2 node that acts as a bridge to an AI agent.
- Use a simple queue or other communication mechanism between the AI and the ROS 2 bridge.
- Implement safety features like command filtering and rate limiting.

## Sections
1.  **Why Decouple AI?**: Maintainability, Testability, and Safety
2.  **The Bridge Pattern**: A Dedicated Node for AI Communication
3.  **Communication Mechanisms**: Queues, Sockets, or Files
4.  **Implementing the Bridge**: Translating AI output to ROS 2 messages
5.  **Safety First**: Command Filtering and Rate Limiting

## Labs
- **Lab 6.1**: Creating a bridge node that takes simple text commands and publishes them as `geometry_msgs/Twist`.
