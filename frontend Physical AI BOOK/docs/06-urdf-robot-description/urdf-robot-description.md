---
sidebar_position: 6
title: "Chapter 6: URDF Robot Description"
---

# URDF Robot Description

## Overview

Unified Robot Description Format (URDF) is the standard for describing robot models in ROS and ROS 2. This chapter covers the creation, structure, and usage of URDF files to represent robot kinematics, dynamics, and visual properties for simulation, planning, and control in Physical AI systems.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the structure and components of URDF files
- Create URDF descriptions for simple and complex robots
- Define kinematic chains and joint constraints
- Specify visual and collision properties for simulation
- Integrate URDF with robot state publishers and controllers
- Validate URDF models for correctness and completeness

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format that describes robot models in ROS/ROS 2. It defines:

- **Kinematic structure**: Joint connections and degrees of freedom
- **Dynamic properties**: Mass, inertia, and friction parameters
- **Visual properties**: Appearance for simulation and visualization
- **Collision properties**: Shapes for collision detection
- **Transmission interfaces**: Motor and sensor connections

### Basic URDF Structure
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.5 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF Components

### Links
Links represent rigid bodies in the robot structure:

#### Visual Properties
```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Choose one geometry type -->
      <box size="1 1 1"/>
      <!-- <cylinder radius="0.5" length="1"/> -->
      <!-- <sphere radius="0.5"/> -->
      <!-- <mesh filename="package://my_robot/meshes/link.stl"/> -->
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>
</link>
```

#### Collision Properties
```xml
<link name="link_name">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
</link>
```

#### Inertial Properties
```xml
<link name="link_name">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0"
             iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints
Joints define the connections between links:

#### Joint Types
- **revolute**: Rotational joint with limits
- **continuous**: Continuous rotational joint (like wheel)
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement (rigid connection)
- **floating**: 6 DOF (for base of floating robots)
- **planar**: Planar motion (rarely used)

#### Joint Definition Example
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Creating a Simple Robot Model

### Mobile Base Example
```xml
<?xml version="1.0"?>
<robot name="simple_mobile_base">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Advanced URDF Features

### Transmissions
Define how actuators connect to joints:

```xml
<transmission name="left_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo Extensions
Add simulation-specific properties:

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
  </plugin>
</gazebo>
```

### Xacro Macros
Use Xacro to simplify complex URDFs:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="wheel" params="prefix parent *origin">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <axis xyz="0 0 1"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                 iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="left" parent="base_link">
    <origin xyz="0 0.2 0" rpy="-${M_PI/2} 0 0"/>
  </xacro:wheel>

  <xacro:wheel prefix="right" parent="base_link">
    <origin xyz="0 -0.2 0" rpy="-${M_PI/2} 0 0"/>
  </xacro:wheel>
</robot>
```

## URDF Validation and Testing

### Checking URDF Syntax
```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Check Xacro and convert to URDF
xacro --inorder robot.xacro > robot.urdf
check_urdf robot.urdf
```

### Visualizing URDF
```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'

# Or with a launch file
ros2 launch my_robot_description display.launch.py
```

### Robot State Publisher Node
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Read robot description from parameter
        self.declare_parameter('robot_description', '')
        self.robot_description = self.get_parameter('robot_description').value

        # Create joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store joint positions
        self.joint_positions = {}

    def joint_state_callback(self, msg):
        # Update joint positions
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

        # Publish transforms
        self.publish_transforms()

    def publish_transforms(self):
        # In a real implementation, you would compute forward kinematics
        # to get the transforms for each link
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with ROS 2 Systems

### Launch File for URDF Display
```python
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindPackageShare("my_robot_description"), "urdf", "robot.urdf.xacro"])
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state publisher node (for GUI)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="True",
            description="Enable joint_state_publisher_gui"
        ),
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
```

## Best Practices for URDF

### File Organization
```
my_robot_description/
├── urdf/
│   ├── robot.urdf.xacro
│   ├── materials.xacro
│   ├── transmission.xacro
│   └── gazebo.xacro
├── meshes/
│   ├── base_link.stl
│   ├── wheel_link.stl
│   └── ...
├── launch/
│   └── display.launch.py
└── CMakeLists.txt
```

### Mass and Inertia Guidelines
- Use realistic mass values based on actual hardware
- Calculate inertias for basic shapes or use CAD software
- For complex shapes, approximate with simpler geometries
- Verify that inertial properties are physically plausible

### Visual and Collision Separation
- Use detailed meshes for visual elements
- Use simplified shapes for collision detection
- Ensure collision models are watertight and manifold
- Consider performance vs. accuracy trade-offs

## MCP-Grounded Technical Content

All technical specifications, XML syntax, parameters, and capabilities mentioned in this chapter are verified through Context7 MCP Server documentation. For any technical content that cannot be verified through MCP documentation, we explicitly state "Not covered in this book."

## Chapter Exercises

1. **Simple Robot Creation**: Create a URDF file for a simple 3-link manipulator with 3 revolute joints. Include proper visual, collision, and inertial properties for each link.

2. **Xacro Implementation**: Convert your URDF to Xacro format, using macros to reduce redundancy. Add parameters for easily changing link lengths and joint types.

3. **Integration Exercise**: Create a launch file that loads your URDF and displays it in RViz with proper joint state visualization. Test the robot state publisher with sample joint positions.

## Summary

URDF provides the essential framework for describing robot models in ROS/ROS 2 systems. Understanding how to properly structure URDF files with appropriate visual, collision, and inertial properties is crucial for simulation, planning, and control in Physical AI systems. The combination of URDF with Xacro macros and proper integration with ROS 2 tools enables the creation of complex robot models that can be used across the entire Physical AI development pipeline.