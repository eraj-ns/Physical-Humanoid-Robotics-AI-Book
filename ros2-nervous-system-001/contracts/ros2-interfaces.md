---
id: specs-001-ros2-nervous-system-contracts-ros2-interfaces
---
# ROS 2 Contracts for Module 1

This document outlines the key ROS 2 message and service types that will be used in this module. These are the "contracts" that define how data is exchanged between nodes.

## Message Types

### `std_msgs/String`
- **Usage**: For simple text-based communication. Will be used in the introductory "hello world" examples.
- **Fields**:
  - `string data`

### `sensor_msgs/Image`
- **Usage**: For transmitting images from a camera sensor.
- **Fields**:
  - `Header header`
  - `uint32 height`
  - `uint32 width`
  - `string encoding`
  - `uint8 is_bigendian`
  - `uint32 step`
  - `uint8[] data`

### `sensor_msgs/Imu`
- **Usage**: For transmitting inertial measurement unit data.
- **Fields**:
  - `Header header`
  - `Quaternion orientation`
  - `float64[9] orientation_covariance`
  - `Vector3 angular_velocity`
  - `float64[9] angular_velocity_covariance`
  - `Vector3 linear_acceleration`
  - `float64[9] linear_acceleration_covariance`

### `geometry_msgs/Twist`
- **Usage**: For sending velocity commands to a robot's base.
- **Fields**:
  - `Vector3 linear`
  - `Vector3 angular`

## Service Types

### `std_srvs/SetBool`
- **Usage**: For simple services that enable or disable a feature.
- **Request**:
  - `bool data`
- **Response**:
  - `bool success`
  - `string message`

## Action Types

### `control_msgs/action/FollowJointTrajectory`
- **Usage**: For sending a sequence of joint positions to a robot controller.
- **Goal**:
  - `trajectory_msgs/JointTrajectory trajectory`
- **Result**:
  - `int32 error_code`
- **Feedback**:
  - `trajectory_msgs/JointTrajectoryPoint desired`
  - `trajectory_msgs/JointTrajectoryPoint actual`
  - `trajectory_msgs/JointTrajectoryPoint error`
