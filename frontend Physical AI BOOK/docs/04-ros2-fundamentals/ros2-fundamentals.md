---
sidebar_position: 4
title: "Chapter 4: ROS 2 Fundamentals"
---

# ROS 2 Fundamentals

## Overview

Robot Operating System 2 (ROS 2) provides the foundational middleware for developing Physical AI systems. This chapter covers the core concepts, architecture, and practical implementation of ROS 2 for creating distributed robotic applications that connect sensors, AI processing, and actuators in real-time systems.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the architecture and design principles of ROS 2
- Create and manage ROS 2 packages, nodes, and communication interfaces
- Implement message passing and service architectures
- Configure Quality of Service (QoS) settings for real-time requirements
- Design distributed systems using ROS 2 middleware
- Integrate sensors and actuators using ROS 2 interfaces

## Introduction to ROS 2

ROS 2 is the next-generation Robot Operating System designed for production robotics applications. Unlike its predecessor, ROS 2 addresses critical requirements for Physical AI systems:

- **Real-time performance**: Deterministic message delivery for safety-critical applications
- **Security**: Built-in authentication, encryption, and access control
- **Scalability**: Support for multi-robot systems and cloud integration
- **Reliability**: Robust communication with Quality of Service controls

### ROS 1 vs. ROS 2 Key Differences

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom TCP/UDP | DDS-based middleware |
| Real-time | Limited support | First-class real-time support |
| Security | No built-in security | Built-in security features |
| Multi-robot | Complex setup | Native multi-robot support |
| Deployment | Desktop-focused | Production-ready |

## ROS 2 Architecture

### DDS (Data Distribution Service)
ROS 2 uses DDS as its underlying communication middleware, providing:

- **Publisher/Subscriber Model**: Asynchronous message passing
- **Client/Service Model**: Synchronous request/response communication
- **Quality of Service (QoS)**: Configurable reliability and performance settings
- **Discovery**: Automatic node discovery and connection

### Core Components

#### Nodes
- **Definition**: Independent processes that perform computation
- **Implementation**: Can be written in C++, Python, or other supported languages
- **Communication**: Exchange messages through topics, services, and actions
- **Lifecycle**: Can be configured with lifecycle states (unconfigured, inactive, active)

#### Topics
- **Publish/Subscribe**: Asynchronous, one-to-many communication
- **Message Types**: Strongly typed messages defined in .msg files
- **Serialization**: Efficient binary serialization for performance
- **Transport**: Can use shared memory, TCP, UDP, or other transports

#### Services
- **Request/Response**: Synchronous, one-to-one communication
- **Message Types**: Defined in .srv files with request and response structures
- **Blocking**: Client waits for service response
- **Use Cases**: Configuration, calibration, one-time requests

#### Actions
- **Goal/Feedback/Result**: Asynchronous long-running tasks with feedback
- **Message Types**: Defined in .action files with goal, feedback, and result structures
- **Cancellation**: Support for canceling long-running tasks
- **Use Cases**: Navigation, manipulation, calibration sequences

## Quality of Service (QoS) in ROS 2

QoS profiles control communication behavior for different requirements:

### Reliability Policy
- **RELIABLE**: All messages delivered (TCP-like)
- **BEST_EFFORT**: Messages may be lost (UDP-like)

### Durability Policy
- **TRANSIENT_LOCAL**: Late-joining subscribers receive last message
- **VOLATILE**: Only new messages delivered to subscribers

### History Policy
- **KEEP_LAST**: Store N most recent messages
- **KEEP_ALL**: Store all messages (requires reliable transport)

### Deadline and Lifespan
- **Deadline**: Maximum time between consecutive messages
- **Lifespan**: Maximum time before message expiration

## Setting Up a ROS 2 Environment

### Installation Requirements
- **OS Support**: Ubuntu 20.04/22.04, Windows 10/11, macOS (limited)
- **Python**: Python 3.8 or higher
- **C++**: C++17 compiler (GCC 7.4+, Clang 8+, MSVC 2019+)
- **DDS Implementation**: Fast DDS, Cyclone DDS, RTI Connext DDS

### Workspace Structure
```
ros2_workspace/
├── src/
│   ├── robot_control/
│   ├── perception_stack/
│   ├── navigation/
│   └── custom_msgs/
├── build/
├── install/
└── log/
```

## Creating ROS 2 Packages

### Package Structure
```
my_robot_package/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── my_node.cpp
├── include/my_robot_package/
│   └── my_header.hpp
├── launch/
│   └── my_launch_file.launch.py
├── config/
│   └── my_config.yaml
├── test/
│   └── test_my_node.cpp
└── msg/
    └── MyCustomMessage.msg
```

### Package.xml Example
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>My robot package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Practical Implementation: Sensor Integration

### Creating a Sensor Node
```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SensorNode : public rclcpp::Node
{
public:
    SensorNode() : Node("sensor_node")
    {
        // Create publisher for laser scan data
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "laser_scan", 10);

        // Create timer for sensor data acquisition
        timer_ = this->create_wall_timer(
            50ms, std::bind(&SensorNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = sensor_msgs::msg::LaserScan();
        // Fill message with sensor data
        message.header.stamp = this->now();
        message.header.frame_id = "laser_frame";

        // Publish sensor data
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};
```

### Creating a Control Node
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1  # Move forward at 0.1 m/s
        msg.angular.z = 0.0  # No rotation
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')
```

## Launch Files and System Configuration

### Python Launch Files
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='sensor_node',
            name='sensor_node',
            parameters=[
                {'param1': 'value1'},
                {'param2': 10}
            ]
        ),
        Node(
            package='my_robot_package',
            executable='control_node',
            name='control_node'
        )
    ])
```

### YAML Configuration
```yaml
/**:
  ros__parameters:
    use_sim_time: false

sensor_node:
  ros__parameters:
    update_rate: 10.0
    frame_id: "laser_frame"

control_node:
  ros__parameters:
    max_velocity: 1.0
    acceleration_limit: 0.5
```

## MCP-Grounded Technical Content

All technical specifications, API calls, parameters, and capabilities mentioned in this chapter are verified through Context7 MCP Server documentation. For any technical content that cannot be verified through MCP documentation, we explicitly state "Not covered in this book."

## Chapter Exercises

1. **Package Creation**: Create a ROS 2 package for a simple robot that publishes sensor data (e.g., simulated IMU data) and subscribes to velocity commands. Implement proper QoS settings for real-time performance.

2. **Node Communication**: Design a distributed system with three nodes: sensor node, processing node, and actuator node. Implement the communication pattern using topics, services, and actions as appropriate. Justify your choices.

3. **Launch Configuration**: Create a launch file that starts multiple robot nodes in a simulated environment. Include parameter configuration and proper namespace management for multi-robot scenarios.

## Summary

ROS 2 provides the essential middleware for developing Physical AI systems, enabling distributed communication between sensors, AI processing, and actuators. Understanding the architecture, communication patterns, and QoS settings is crucial for building robust and performant robotic applications. The publisher/subscriber, client/service, and action patterns provide the foundation for creating complex Physical AI systems that operate in real-time environments.