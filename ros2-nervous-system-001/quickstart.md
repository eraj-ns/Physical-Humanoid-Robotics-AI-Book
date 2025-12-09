---
id: specs-001-ros2-nervous-system-quickstart
---
# Quickstart Guide for Module 1

This guide provides the essential steps to get started with the "Robotic Nervous System" module.

## 1. System Requirements

- A computer running Ubuntu 22.04 LTS.
- An NVIDIA RTX-enabled GPU is recommended for later modules.
- A Jetson Orin Nano/NX is required for the edge deployment chapter.

## 2. Install ROS 2

Follow the official ROS 2 documentation to install ROS 2 Humble or Iron. The textbook will provide detailed, step-by-step instructions.

```bash
# Example for ROS 2 Humble
sudo apt update && sudo apt install ros-humble-desktop
```

## 3. Create a Colcon Workspace

A "colcon workspace" is a directory where you will store your ROS 2 packages.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

## 4. Source the Environment

Before using any ROS 2 command, you need to source the setup script.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 5. Run a "Hello World" Example

Run a simple publisher/subscriber example to verify your installation.

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_py listener
```

After these steps, you will be ready to start the hands-on labs in the textbook.
