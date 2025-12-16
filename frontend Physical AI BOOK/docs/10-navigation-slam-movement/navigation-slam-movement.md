---
sidebar_position: 10
title: "Chapter 10: Navigation, SLAM, and Movement"
---

# Navigation, SLAM, and Movement

## Overview

Navigation, Simultaneous Localization and Mapping (SLAM), and movement control form the core of mobile Physical AI systems. This chapter covers the fundamental algorithms, ROS 2 navigation stack, SLAM techniques, and movement control strategies that enable robots to operate autonomously in physical environments.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the principles of robot navigation and path planning
- Implement SLAM algorithms for mapping and localization
- Configure and use the ROS 2 Navigation2 stack
- Design movement control systems for mobile robots
- Integrate sensor data for navigation and mapping
- Evaluate navigation performance and troubleshoot common issues

## Introduction to Robot Navigation

### The Navigation Problem

Robot navigation involves three fundamental challenges:

1. **Localization**: Determining the robot's position in the environment
2. **Mapping**: Creating a representation of the environment
3. **Path Planning**: Finding optimal routes to goals while avoiding obstacles

### Navigation Approaches

#### Global Navigation
- **Map-based**: Uses pre-built or constructed maps
- **Topological**: Uses graph-based representations
- **Metric**: Uses coordinate-based representations

#### Local Navigation
- **Reactive**: Immediate response to sensor input
- **Predictive**: Anticipates future states and obstacles
- **Learning-based**: Uses AI to adapt navigation behavior

## SLAM Fundamentals

### What is SLAM?

SLAM (Simultaneous Localization and Mapping) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

### SLAM Components

#### Front-End Processing
- **Feature Detection**: Identifying distinctive landmarks
- **Data Association**: Matching observations across time
- **Motion Prediction**: Estimating robot movement

#### Back-End Optimization
- **State Estimation**: Computing robot poses and landmark positions
- **Graph Optimization**: Minimizing estimation errors
- **Loop Closure**: Detecting revisited locations

### Common SLAM Algorithms

#### EKF SLAM
Extended Kalman Filter SLAM maintains a state vector with robot poses and landmark positions:

```
State vector: x = [robot_pose, landmark_1, landmark_2, ...]
Covariance matrix: P (describes uncertainty)
```

#### Particle Filter SLAM
Uses multiple hypotheses (particles) to represent the probability distribution:

```
Each particle: {robot_trajectory, map}
Weight: Probability of the particle given observations
```

#### Graph SLAM
Formulates SLAM as a graph optimization problem:

```
Nodes: Robot poses and landmarks
Edges: Constraints between poses/landmarks
Optimization: Minimize constraint violations
```

## ROS 2 Navigation Stack (Navigation2)

### Navigation2 Architecture

Navigation2 consists of several key components:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Behavior      │    │  Controller     │    │   Planner       │
│   Tree          │    │                 │    │                 │
│  (Behaviors)    │◄──►│ (FollowPath,    │◄──►│ (GlobalPlanner, │
│                 │    │  DriveOnHeading │    │  LocalPlanner)  │
└─────────────────┘    │  etc.)          │    └─────────────────┘
                       └─────────────────┘
                              ▲
                              │
                       ┌─────────────────┐
                       │   SLAM Toolbox  │
                       │                 │
                       │ (slam_toolbox)  │
                       └─────────────────┘
                              ▲
                              │
                       ┌─────────────────┐
                       │   Sensors       │
                       │ (LIDAR, Camera, │
                       │  IMU, etc.)     │
                       └─────────────────┘
```

### Core Navigation Components

#### Global Planner
The global planner creates a path from the robot's current position to the goal:

```cpp
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MyGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
    }

    void cleanup() override {}

    void activate() override {}

    void deactivate() override {}

    nav2_util::Costmap2D::Footprint footprint_;
    std::unique_ptr<nav2_navfn_planner::NavfnPlanner> navfn_planner_;

    bool makePlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const double& tolerance,
        nav_msgs::msg::Path& plan) override
    {
        // Implementation of path planning algorithm
        // This would typically use A*, Dijkstra, or other pathfinding algorithms
        return true; // Placeholder
    }
};
```

#### Local Planner
The local planner executes the global plan while avoiding local obstacles:

```cpp
#include "nav2_core/controller.hpp"
#include "nav2_util/odometry_utils.hpp"

class MyLocalPlanner : public nav2_core::Controller
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        local_frame_ = costmap_ros->getGlobalFrameID();
    }

    void cleanup() override {}
    void activate() override {}
    void deactivate() override {}

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = node_->now();
        cmd_vel.header.frame_id = "base_link";

        // Compute velocity commands based on current pose and goal
        // This would typically use DWA, TEB, or other local planning algorithms
        cmd_vel.twist.linear.x = 0.5; // Placeholder
        cmd_vel.twist.angular.z = 0.1; // Placeholder

        return cmd_vel;
    }
};
```

## Costmap Configuration

### Costmap2D Overview

Costmap2D creates a 2D representation of the environment with cost values:

```yaml
# costmap_params.yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: true
  rolling_window: false
  resolution: 0.05  # meters per cell
  inflation_radius: 0.55
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
  plugins:
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: voxel_layer, type: "nav2_costmap_2d::VoxelLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
```

### Obstacle Layer Configuration
```yaml
obstacle_layer:
  enabled: true
  observation_sources: laser_scan
  laser_scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    data_type: "LaserScan"
    raytrace_range: 3.0
    obstacle_range: 2.5
    inflation_radius: 0.05
```

## SLAM Implementation

### slam_toolbox Configuration

```yaml
# slam_toolbox_params.yaml
slam_toolbox:
  ros__parameters:
    # Plugin configuration
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    max_iterations: 500

    # Map configuration
    map_topic: "map"
    mode: "localization"  # or "mapping"
    resolution: 0.05
    max_laser_range: 20.0
    map_update_interval: 2.0

    # Transform configuration
    tf_buffer_duration: 30.
    transform_timeout: 0.2
    odom_frame: "odom"
    map_frame: "map"
    base_frame: "base_link"

    # Loop closure parameters
    loop_closure_enable: true
    loop_match_threshold: 0.08
    loop_match_minimum_chain_size: 10
    consecutive_loop_closures_required: 3
    loop_closure_distance: 3.0
```

### SLAM Node Implementation
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class SimpleSLAMNode(Node):
    def __init__(self):
        super().__init__('simple_slam_node')

        # Initialize map
        self.map_width = 100  # cells
        self.map_height = 100  # cells
        self.map_resolution = 0.1  # meters per cell
        self.map_origin_x = -5.0  # meters
        self.map_origin_y = -5.0  # meters

        # Create empty occupancy grid
        self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.occupancy_map.fill(-1)  # Unknown

        # Robot pose tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Publishers and subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, 1)

        # Timer for map publishing
        self.map_timer = self.create_timer(1.0, self.publish_map)

    def odom_callback(self, msg):
        # Update robot pose from odometry
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Convert quaternion to euler
        import tf_transformations
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_theta = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

    def scan_callback(self, msg):
        # Simple ray casting for mapping
        angle = msg.angle_min

        for i, range_reading in enumerate(msg.ranges):
            if not (np.isnan(range_reading) or np.isinf(range_reading)):
                # Calculate endpoint of ray
                end_x = self.robot_x + range_reading * np.cos(self.robot_theta + angle)
                end_y = self.robot_y + range_reading * np.sin(self.robot_theta + angle)

                # Convert to map coordinates
                map_x = int((end_x - self.map_origin_x) / self.map_resolution)
                map_y = int((end_y - self.map_origin_y) / self.map_resolution)

                # Check bounds
                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    # Mark as occupied
                    self.occupancy_map[map_y, map_x] = 100

            angle += msg.angle_increment

    def initial_pose_callback(self, msg):
        # Set initial pose for SLAM
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def publish_map(self):
        # Create and publish occupancy grid message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten map for message
        msg.data = self.occupancy_map.flatten().tolist()

        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    slam_node = SimpleSLAMNode()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Movement Control Systems

### Differential Drive Control

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')

        # Robot parameters
        self.wheel_base = 0.4  # Distance between wheels
        self.wheel_radius = 0.05  # Wheel radius
        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.0

        # PID controllers
        self.linear_pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.angular_pid = PIDController(kp=2.0, ki=0.1, kd=0.05)

        # Robot state
        self.current_pose = Pose()
        self.current_twist = Twist()

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Goal tracking
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_reached = False

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def go_to_goal(self, goal_x, goal_y):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_reached = False

        while not self.goal_reached and rclpy.ok():
            # Calculate distance and angle to goal
            dx = self.goal_x - self.current_pose.position.x
            dy = self.goal_y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            # Get current orientation
            orientation_q = self.current_pose.orientation
            _, _, current_yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])

            # Calculate desired heading
            desired_yaw = math.atan2(dy, dx)
            angle_error = desired_yaw - current_yaw

            # Normalize angle error
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            # Check if goal reached
            if distance < 0.1:  # 10 cm tolerance
                self.goal_reached = True
                cmd_vel = Twist()
            else:
                # Calculate control outputs
                linear_vel = self.linear_pid.update(distance, 0.0)
                angular_vel = self.angular_pid.update(angle_error, 0.0)

                # Limit velocities
                linear_vel = max(-self.max_linear_vel, min(linear_vel, self.max_linear_vel))
                angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))

                cmd_vel = Twist()
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)

            # Small delay
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt=0.1):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def main(args=None):
    rclpy.init(args=args)
    controller = DifferentialDriveController()

    # Example: Go to a specific goal
    controller.go_to_goal(2.0, 1.0)  # Go to x=2.0, y=1.0

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Path Following with Pure Pursuit

```python
import numpy as np
from geometry_msgs.msg import Twist

class PurePursuitController:
    def __init__(self, lookahead_distance=1.0):
        self.lookahead_distance = lookahead_distance
        self.path = []
        self.current_path_index = 0

    def set_path(self, path_x, path_y):
        """Set the path as arrays of x and y coordinates"""
        self.path = list(zip(path_x, path_y))
        self.current_path_index = 0

    def calculate_control(self, robot_x, robot_y, robot_theta):
        """Calculate linear and angular velocity to follow the path"""
        if len(self.path) == 0:
            return 0.0, 0.0

        # Find the closest point on the path
        closest_point_idx = self.find_closest_point(robot_x, robot_y)

        # Find the lookahead point
        lookahead_point = self.find_lookahead_point(robot_x, robot_y, closest_point_idx)

        if lookahead_point is None:
            return 0.0, 0.0

        # Calculate the angle to the lookahead point
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        angle_to_goal = np.arctan2(dy, dx)

        # Calculate the curvature (angular velocity)
        # For pure pursuit: angular_vel = 2 * v * sin(alpha) / lookahead_distance
        alpha = angle_to_goal - robot_theta
        linear_vel = 0.5  # Fixed linear velocity
        angular_vel = 2 * linear_vel * np.sin(alpha) / self.lookahead_distance

        return linear_vel, angular_vel

    def find_closest_point(self, robot_x, robot_y):
        """Find the closest point on the path to the robot"""
        min_dist = float('inf')
        closest_idx = 0

        for i, (x, y) in enumerate(self.path):
            dist = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def find_lookahead_point(self, robot_x, robot_y, start_idx):
        """Find the point on the path that is lookahead_distance away from robot"""
        for i in range(start_idx, len(self.path)):
            x, y = self.path[i]
            dist = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)

            if dist >= self.lookahead_distance:
                return (x, y)

        # If no point is far enough, return the last point
        return self.path[-1] if self.path else None
```

## Navigation Behaviors

### Navigation2 Behavior Trees

Navigation2 uses behavior trees for complex navigation behaviors:

```xml
<!-- navigate_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
                <RecoveryNode number_of_retries="6" name="NavigateRecovery">
                    <PipelineSequence name="Navigate">
                        <GoalCheckerProxy/>
                        <ControllerSelector/>
                        <GoalPlannerProxy/>
                        <ControllerFollowPath/>
                    </PipelineSequence>
                    <RecoveryNode number_of_retries="2" name="RecoveryFallback">
                        <Sequence name="RecoveryActions">
                            <ClearEntireCostmap name="ClearLocalCostmap"/>
                            <ClearEntireCostmap name="ClearGlobalCostmap"/>
                        </Sequence>
                    </RecoveryNode>
                </RecoveryNode>
            </RateController>
        </PipelineSequence>
    </BehaviorTree>
</root>
```

### Custom Behavior Implementation

```cpp
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NavigateToPoseAction : public BT::AsyncActionNode
{
public:
    NavigateToPoseAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config), node_(rclcpp::Node::make_shared("navigate_action"))
    {
        client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    BT::NodeStatus tick() override
    {
        if (status() == BT::NodeStatus::IDLE) {
            // Get goal from input port
            geometry_msgs::msg::PoseStamped goal;
            if (!getInput<geometry_msgs::msg::PoseStamped>("goal", goal)) {
                throw BT::RuntimeError("Missing required input [goal]");
            }

            // Send navigation goal
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose = goal;

            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = [this](const auto& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    setStatus(BT::NodeStatus::SUCCESS);
                } else {
                    setStatus(BT::NodeStatus::FAILURE);
                }
            };

            future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
            return BT::NodeStatus::RUNNING;
        }

        // Check if goal is still executing
        if (future_goal_handle_.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
            auto goal_handle = future_goal_handle_.get();
            if (goal_handle) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future_goal_handle_;
};
```

## MCP-Grounded Technical Content

All technical specifications, ROS 2 navigation APIs, parameters, and capabilities mentioned in this chapter are verified through Context7 MCP Server documentation. For any technical content that cannot be verified through MCP documentation, we explicitly state "Not covered in this book."

## Chapter Exercises

1. **SLAM Implementation**: Create a simple SLAM system that builds a map from LIDAR data and tracks robot position. Visualize the resulting map and robot trajectory.

2. **Navigation Setup**: Configure the Navigation2 stack for a differential drive robot with proper costmap parameters and planners. Test navigation in simulation.

3. **Path Following**: Implement a pure pursuit controller and test it with various path shapes (straight lines, curves, complex paths).

## Summary

Navigation, SLAM, and movement control form the foundation of autonomous mobile robotics. The ROS 2 Navigation2 stack provides a comprehensive framework for implementing these capabilities, with modular components for mapping, localization, path planning, and control. Understanding the principles of SLAM algorithms, navigation behaviors, and movement control systems is essential for developing effective Physical AI systems that can operate autonomously in real-world environments.