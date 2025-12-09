---
id: specs-001-ros2-nervous-system-research
---
# Research for Module 1: The Robotic Nervous System (ROS 2)

This document outlines the research performed to ensure the quality and accuracy of the textbook module.

## Research Task 1: Best practices for ROS 2 educational content

**Decision**: The content will be structured with a "theory, then practice" approach in each chapter. Core concepts will be explained first, followed by hands-on labs. This aligns with standard engineering pedagogy.

**Rationale**: Students learn robotics best by doing. A strong theoretical foundation prevents the labs from being just "copy-paste" exercises.

**Alternatives considered**: A "practice-first" approach was considered but rejected, as it can lead to a shallow understanding of the underlying principles.

## Research Task 2: Best practices for `rclpy`

**Decision**: `rclpy` examples will emphasize the use of timers and callbacks for asynchronous operations, which is the standard ROS 2 way. The use of classes for organizing nodes will be demonstrated for all but the simplest examples.

**Rationale**: This structure promotes readable, maintainable, and scalable ROS 2 code. It mirrors the structure of most professional ROS 2 projects.

**Alternatives considered**: Simple, procedural scripts were considered for the examples but rejected as they don't teach the students the proper way to structure ROS 2 nodes.

## Research Task 3: Best practices for Docusaurus project structure

**Decision**: The project will follow the standard Docusaurus v3 structure. Markdown files for each chapter will be placed in a `module1` directory. Code examples will be in a separate `src` directory, organized by chapter, and linked from the Markdown files.

**Rationale**: This separation of content and code keeps the project organized and makes it easy to maintain both the text and the examples. It also allows for the code to be easily tested independently.

**Alternatives considered**: Embedding code directly in the Markdown files was considered but rejected because it makes the code harder to test and maintain.

## Research Task 4: Patterns for integrating ROS 2 with Python AI agents

**Decision**: The integration will be done via a dedicated ROS 2 node that acts as a bridge. The AI agent will run in its own process and communicate with the bridge node, which will then translate the agent's output into ROS 2 messages.

**Rationale**: This decouples the AI agent from the ROS 2 system, allowing the agent to be developed and tested independently. It also provides a clear interface and a place to implement safety features like command filtering and rate limiting.

**Alternatives considered**: Running the AI agent directly within a ROS 2 node was considered but rejected because it would tightly couple the agent to the ROS 2 system and make it harder to manage its lifecycle.
