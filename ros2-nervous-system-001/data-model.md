---
id: specs-001-ros2-nervous-system-data-model
---
# Data Model for Module 1

This document describes the key conceptual entities in the Robotic Nervous System module. As this is an educational module, these are not database models but rather concepts that students will learn about.

## Entity: ROS 2 Node

A node is the fundamental building block of a ROS 2 system. Each node should be responsible for a single, module purpose (e.g., one node for controlling a camera, one node for controlling the wheels, etc.).

**Attributes**:
- **Name**: A unique name within the ROS 2 graph.
- **Namespace**: A grouping mechanism to avoid name collisions.
- **Publishers**: A list of topics the node publishes messages to.
- **Subscribers**: A list of topics the node subscribes to.
- **Services**: A list of services the node provides.
- **Clients**: A list of services the node uses.
- **Actions**: A list of actions the node provides or uses.
- **Parameters**: A set of configurable parameters.

## Entity: Topic

Topics are named buses for nodes to exchange messages. They are the primary way of moving data in a ROS 2 system.

**Attributes**:
- **Name**: A unique name.
- **Message Type**: The type of message that can be sent over the topic (e.g., `std_msgs/String`, `sensor_msgs/Image`).
- **QoS Profile**: Quality of Service settings that control the reliability, durability, and other aspects of the communication.

## Entity: URDF Model

A Unified Robot Description Format (URDF) model is an XML file that represents a robot model.

**Attributes**:
- **Links**: The rigid parts of the robot. Each link has inertial, visual, and collision properties.
- **Joints**: The connections between links. Each joint has a type (e.g., revolute, prismatic, fixed), an axis of rotation, and limits.
- **Transmissions**: Used to model the relationship between an actuator and a joint.

## Entity: AI Agent

An AI Agent is a standalone program that encapsulates some form of intelligence.

**Attributes**:
- **Logic**: The core decision-making part of the agent (e.g., a neural network, a state machine, a set of rules).
- **Inputs**: The data the agent uses to make decisions (e.g., sensor data, user commands).
- **Outputs**: The commands the agent generates (e.g., motor commands, speech).

**Relationships**:
- The AI Agent **communicates with** a ROS 2 Node (the "bridge") to interact with the robot. It does not directly interact with the ROS 2 graph.
