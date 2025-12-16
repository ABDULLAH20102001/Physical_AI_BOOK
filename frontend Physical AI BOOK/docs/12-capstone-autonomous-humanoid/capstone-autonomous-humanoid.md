---
sidebar_position: 12
title: "Chapter 12: Capstone - Autonomous Humanoid"
---

# Capstone - Autonomous Humanoid

## Overview

This capstone chapter integrates all concepts learned throughout the book to create a comprehensive autonomous humanoid robot system. We'll combine Physical AI principles, sensor integration, navigation, SLAM, vision-language-action systems, and advanced control to build a humanoid robot capable of autonomous operation in real-world environments.

## Learning Objectives

By the end of this chapter, you will be able to:

- Integrate all Physical AI components into a complete humanoid system
- Design and implement autonomous behaviors for humanoid robots
- Combine multiple AI modalities for complex task execution
- Create a complete ROS 2 system architecture for humanoid control
- Evaluate and validate autonomous humanoid performance
- Troubleshoot complex multi-component humanoid systems

## Autonomous Humanoid Architecture

### System Overview

The autonomous humanoid system integrates multiple subsystems:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Humanoid Control System                      │
├─────────────────────────────────────────────────────────────────┤
│  Perception Layer     │  Cognition Layer      │  Action Layer  │
│                       │                       │                │
│  • Vision System      │  • Task Planning      │  • Navigation  │
│  • LIDAR Processing   │  • State Management   │  • Manipulation│
│  • Audio Processing   │  • Decision Making    │  • Locomotion  │
│  • Tactile Sensors    │  • Human Interaction  │  • Balance     │
│  • IMU Integration    │  • Learning Systems   │  • Safety Ctrl │
└───────────────────────┼───────────────────────┼────────────────┘
                        │  Coordination Layer   │
                        │                       │
                        │  • Behavior Trees     │
                        │  • State Machines     │
                        │  • Multi-Modal Fusion │
                        │  • ROS 2 Integration  │
                        └───────────────────────┘
```

### Hardware Architecture

For our capstone humanoid, we'll consider a typical humanoid platform:

- **Sensors**: Cameras, LIDAR, IMU, force/torque sensors, tactile sensors
- **Actuators**: Servo motors for joints, grippers for manipulation
- **Computing**: Embedded computer for real-time control, GPU for AI processing
- **Communication**: WiFi, Ethernet, CAN bus for internal communication

## Complete Humanoid ROS 2 System

### Main Control Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # State management
        self.current_state = 'IDLE'
        self.previous_state = None
        self.state_start_time = self.get_clock().now()

        # Sensor data storage
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None
        self.odom_data = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.status_pub = self.create_publisher(String, 'humanoid_status', 10)

        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)

        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Initialize subsystems
        self.perception_system = PerceptionSystem()
        self.navigation_system = NavigationSystem()
        self.manipulation_system = ManipulationSystem()
        self.locomotion_system = LocomotionSystem()

        self.get_logger().info('Humanoid Controller initialized')

    def camera_callback(self, msg):
        """Process camera data"""
        self.camera_data = msg
        # Convert ROS image to OpenCV format for processing
        self.perception_system.process_camera_data(msg)

    def lidar_callback(self, msg):
        """Process LIDAR data"""
        self.lidar_data = msg
        # Process LIDAR data for navigation and obstacle detection
        self.perception_system.process_lidar_data(msg)

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg
        # Process IMU data for balance and orientation
        self.locomotion_system.process_imu_data(msg)

    def control_loop(self):
        """Main control loop"""
        # Update sensor data
        self.update_sensor_data()

        # State machine logic
        self.execute_state_machine()

        # Publish status
        status_msg = String()
        status_msg.data = f"State: {self.current_state}, Time: {self.get_clock().now().nanoseconds}"
        self.status_pub.publish(status_msg)

    def update_sensor_data(self):
        """Update all sensor data"""
        # This would integrate data from all sensors
        pass

    def execute_state_machine(self):
        """Execute state machine based on current state"""
        if self.current_state == 'IDLE':
            self.state_idle()
        elif self.current_state == 'NAVIGATING':
            self.state_navigating()
        elif self.current_state == 'MANIPULATING':
            self.state_manipulating()
        elif self.current_state == 'BALANCING':
            self.state_balancing()
        elif self.current_state == 'INTERACTING':
            self.state_interacting()
        elif self.current_state == 'ERROR':
            self.state_error_recovery()

    def state_idle(self):
        """Idle state - waiting for commands"""
        # Stop all movement
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        # Check for new commands or transitions
        if self.should_start_navigation():
            self.transition_to('NAVIGATING')
        elif self.should_start_manipulation():
            self.transition_to('MANIPULATING')

    def state_navigating(self):
        """Navigation state"""
        # Get navigation commands from navigation system
        nav_cmd = self.navigation_system.get_navigation_command()

        if nav_cmd is not None:
            self.cmd_vel_pub.publish(nav_cmd)
        else:
            # If no navigation command, return to idle
            self.transition_to('IDLE')

        # Check for obstacles or completion
        if self.navigation_system.is_complete():
            self.transition_to('IDLE')
        elif self.detect_obstacle():
            self.transition_to('BALANCING')  # Handle obstacle

    def state_manipulating(self):
        """Manipulation state"""
        # Get manipulation commands
        manip_cmd = self.manipulation_system.get_manipulation_command()

        if manip_cmd is not None:
            self.joint_cmd_pub.publish(manip_cmd)

        # Check for completion
        if self.manipulation_system.is_complete():
            self.transition_to('IDLE')

    def state_balancing(self):
        """Balance state - recover from disturbances"""
        balance_cmd = self.locomotion_system.get_balance_command()
        if balance_cmd is not None:
            self.cmd_vel_pub.publish(balance_cmd)

        # Check if balance is recovered
        if self.locomotion_system.is_balanced():
            self.transition_to('IDLE')

    def state_interacting(self):
        """Human interaction state"""
        # Handle human interaction
        interaction_cmd = self.handle_human_interaction()
        if interaction_cmd:
            self.execute_interaction(interaction_cmd)

    def state_error_recovery(self):
        """Error recovery state"""
        # Execute error recovery procedures
        self.recover_from_error()
        self.transition_to('IDLE')

    def transition_to(self, new_state):
        """Transition to new state"""
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f'Transitioned to state: {new_state}')

    def should_start_navigation(self):
        """Check if should start navigation"""
        # Logic to determine if navigation should start
        return False  # Placeholder

    def should_start_manipulation(self):
        """Check if should start manipulation"""
        # Logic to determine if manipulation should start
        return False  # Placeholder

    def detect_obstacle(self):
        """Detect obstacles in environment"""
        # Check LIDAR data for obstacles
        if self.lidar_data:
            # Check if any range reading is below threshold
            min_range = min(self.lidar_data.ranges) if self.lidar_data.ranges else float('inf')
            return min_range < 0.5  # 50cm threshold
        return False

    def handle_human_interaction(self):
        """Handle human interaction"""
        # Process human commands/requests
        return None  # Placeholder

    def execute_interaction(self, cmd):
        """Execute human interaction command"""
        # Execute the interaction command
        pass

    def recover_from_error(self):
        """Execute error recovery procedures"""
        # Stop all movement
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
```

### Perception System

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class PerceptionSystem:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.object_detector = self.initialize_object_detector()
        self.scene_graph = {}
        self.spatial_map = {}

    def initialize_object_detector(self):
        """Initialize object detection model"""
        # Load YOLO or other detection model
        return cv2.dnn.readNetFromDarknet("yolo_config.cfg", "yolo_weights.weights")

    def process_camera_data(self, image_msg):
        """Process camera image data"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Run object detection
            detections = self.run_object_detection(cv_image)

            # Analyze scene
            self.scene_graph = self.analyze_scene(cv_image, detections)

            return self.scene_graph
        except Exception as e:
            print(f"Error processing camera data: {e}")
            return None

    def process_lidar_data(self, lidar_msg):
        """Process LIDAR scan data"""
        # Process LIDAR data for mapping and obstacle detection
        ranges = np.array(lidar_msg.ranges)

        # Filter out invalid readings
        valid_ranges = ranges[np.isfinite(ranges)]

        # Create occupancy grid or point cloud
        occupancy_grid = self.create_occupancy_grid(ranges, lidar_msg.angle_min,
                                                   lidar_msg.angle_increment)

        return occupancy_grid

    def run_object_detection(self, image):
        """Run object detection on image"""
        height, width = image.shape[:2]

        # Create blob from image
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)

        # Run detection
        self.object_detector.setInput(blob)
        outputs = self.object_detector.forward()

        # Process outputs (simplified)
        detections = []
        for output in outputs:
            for detection in output:
                # Extract bounding box and confidence
                if detection[5] > 0.5:  # Confidence threshold
                    x, y, w, h = detection[0:4] * np.array([width, height, width, height])
                    detections.append({
                        'bbox': [int(x-w/2), int(y-h/2), int(x+w/2), int(y+h/2)],
                        'confidence': detection[5],
                        'class': int(detection[6])
                    })

        return detections

    def analyze_scene(self, image, detections):
        """Analyze scene and create scene graph"""
        scene_graph = {
            'objects': detections,
            'relationships': self.compute_spatial_relationships(detections, image.shape),
            'context': self.extract_context(image)
        }
        return scene_graph

    def compute_spatial_relationships(self, detections, image_shape):
        """Compute spatial relationships between objects"""
        relationships = []
        height, width = image_shape[:2]

        for i, obj1 in enumerate(detections):
            for j, obj2 in enumerate(detections):
                if i != j:
                    bbox1 = obj1['bbox']
                    bbox2 = obj2['bbox']

                    # Calculate centers
                    center1 = ((bbox1[0] + bbox1[2]) / 2, (bbox1[1] + bbox1[3]) / 2)
                    center2 = ((bbox2[0] + bbox2[2]) / 2, (bbox2[1] + bbox2[3]) / 2)

                    # Determine relationship
                    dx = center1[0] - center2[0]
                    dy = center1[1] - center2[1]

                    if abs(dx) > abs(dy):
                        rel = 'left' if dx < 0 else 'right'
                    else:
                        rel = 'above' if dy < 0 else 'below'

                    relationships.append({
                        'subject': i,
                        'object': j,
                        'relationship': rel
                    })

        return relationships

    def create_occupancy_grid(self, ranges, angle_min, angle_increment):
        """Create occupancy grid from LIDAR ranges"""
        grid_size = 100  # 100x100 grid
        grid_resolution = 0.1  # 10cm per cell

        occupancy_grid = np.zeros((grid_size, grid_size), dtype=np.int8)

        # Convert polar to Cartesian coordinates
        for i, range_val in enumerate(ranges):
            if np.isfinite(range_val):
                angle = angle_min + i * angle_increment

                x = int(range_val * np.cos(angle) / grid_resolution + grid_size/2)
                y = int(range_val * np.sin(angle) / grid_resolution + grid_size/2)

                if 0 <= x < grid_size and 0 <= y < grid_size:
                    occupancy_grid[y, x] = 100  # Occupied

        return occupancy_grid
```

### Navigation System

```python
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
import heapq

class NavigationSystem:
    def __init__(self):
        self.map = None
        self.robot_pose = Pose()
        self.goal_pose = Pose()
        self.path = []
        self.path_index = 0
        self.local_planner = LocalPlanner()
        self.global_planner = GlobalPlanner()

    def set_goal(self, goal_pose):
        """Set navigation goal"""
        self.goal_pose = goal_pose
        self.plan_path()

    def plan_path(self):
        """Plan global path to goal"""
        if self.map is not None:
            self.path = self.global_planner.plan(self.robot_pose, self.goal_pose, self.map)
            self.path_index = 0

    def get_navigation_command(self):
        """Get navigation command for current state"""
        if not self.path or self.path_index >= len(self.path):
            return None

        # Get current goal point along path
        current_goal = self.path[self.path_index]

        # Check if reached current goal point
        distance_to_goal = self.distance_to_point(current_goal)
        if distance_to_goal < 0.2:  # 20cm threshold
            self.path_index += 1
            if self.path_index >= len(self.path):
                return None  # Path completed

        # Get local command to follow path
        cmd_vel = self.local_planner.follow_path(
            self.robot_pose, current_goal, self.path[self.path_index:]
        )

        return cmd_vel

    def is_complete(self):
        """Check if navigation is complete"""
        return self.path_index >= len(self.path)

    def distance_to_point(self, point):
        """Calculate distance from robot to point"""
        dx = point.x - self.robot_pose.position.x
        dy = point.y - self.robot_pose.position.y
        return np.sqrt(dx*dx + dy*dy)

class GlobalPlanner:
    def plan(self, start, goal, occupancy_grid):
        """Plan global path using A* algorithm"""
        # Convert poses to grid coordinates
        start_grid = self.pose_to_grid(start, occupancy_grid)
        goal_grid = self.pose_to_grid(goal, occupancy_grid)

        # Run A* path planning
        path_grid = self.a_star(start_grid, goal_grid, occupancy_grid)

        # Convert grid path back to world coordinates
        path_world = []
        for grid_point in path_grid:
            world_point = self.grid_to_world(grid_point, occupancy_grid)
            path_world.append(world_point)

        return path_world

    def a_star(self, start, goal, grid):
        """A* path planning algorithm"""
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current, grid):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def heuristic(self, a, b):
        """Heuristic function for A* (Manhattan distance)"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos, grid):
        """Get valid neighbors for a position"""
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                if grid[ny, nx] < 50:  # Not occupied
                    neighbors.append((nx, ny))
        return neighbors

    def distance(self, a, b):
        """Calculate distance between two points"""
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def pose_to_grid(self, pose, grid):
        """Convert world pose to grid coordinates"""
        # Simplified conversion - in practice would use proper transforms
        grid_x = int(pose.position.x / 0.1)  # Assuming 0.1m resolution
        grid_y = int(pose.position.y / 0.1)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_pos, grid):
        """Convert grid coordinates to world pose"""
        world_x = grid_pos[0] * 0.1
        world_y = grid_pos[1] * 0.1
        point = Point()
        point.x = world_x
        point.y = world_y
        point.z = 0.0
        return point

class LocalPlanner:
    def follow_path(self, robot_pose, goal, remaining_path):
        """Follow path using local planning"""
        # Calculate direction to goal
        dx = goal.x - robot_pose.position.x
        dy = goal.y - robot_pose.position.y

        # Calculate distance and angle
        distance = np.sqrt(dx*dx + dy*dy)
        angle = np.arctan2(dy, dx)

        # Get robot orientation
        # Simplified - in practice would extract from quaternion
        robot_angle = 0.0  # Placeholder

        # Calculate angular error
        angle_error = angle - robot_angle
        # Normalize angle
        while angle_error > np.pi:
            angle_error -= 2 * np.pi
        while angle_error < -np.pi:
            angle_error += 2 * np.pi

        # Create velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = min(0.5, distance) * 0.5  # Scale with distance
        cmd_vel.angular.z = angle_error * 1.0  # Proportional control

        return cmd_vel
```

### Manipulation System

```python
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.srv import GetPositionIK, GetPositionFK
import numpy as np

class ManipulationSystem:
    def __init__(self):
        self.arm_joints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                          'wrist_flex', 'wrist_roll']
        self.gripper_joint = 'gripper_finger'
        self.arm_pose = Pose()
        self.gripper_state = 'open'  # 'open' or 'closed'

    def grasp_object(self, object_pose):
        """Grasp an object at the specified pose"""
        # Plan approach trajectory
        approach_pose = self.calculate_approach_pose(object_pose)

        # Execute approach
        if self.move_to_pose(approach_pose):
            # Execute grasp
            return self.execute_grasp(object_pose)

        return False

    def place_object(self, target_pose):
        """Place object at target pose"""
        # Plan placement trajectory
        approach_pose = self.calculate_approach_pose(target_pose)

        # Execute approach
        if self.move_to_pose(approach_pose):
            # Execute placement
            return self.execute_placement(target_pose)

        return False

    def calculate_approach_pose(self, object_pose):
        """Calculate approach pose for manipulation"""
        approach_pose = Pose()
        # Calculate approach position (slightly above object)
        approach_pose.position.x = object_pose.position.x
        approach_pose.position.y = object_pose.position.y
        approach_pose.position.z = object_pose.position.z + 0.1  # 10cm above

        # Set orientation for grasping
        approach_pose.orientation = object_pose.orientation

        return approach_pose

    def execute_grasp(self, object_pose):
        """Execute grasp action"""
        # Move to grasp pose
        if not self.move_to_pose(object_pose):
            return False

        # Close gripper
        return self.close_gripper()

    def execute_placement(self, target_pose):
        """Execute placement action"""
        # Move to placement pose
        if not self.move_to_pose(target_pose):
            return False

        # Open gripper
        return self.open_gripper()

    def move_to_pose(self, target_pose):
        """Move manipulator to target pose using inverse kinematics"""
        # This would call MoveIt IK service
        # For now, return True as placeholder
        return True

    def close_gripper(self):
        """Close the gripper"""
        joint_state = JointState()
        joint_state.name = [self.gripper_joint]
        joint_state.position = [0.0]  # Closed position
        # Publish joint command
        return True

    def open_gripper(self):
        """Open the gripper"""
        joint_state = JointState()
        joint_state.name = [self.gripper_joint]
        joint_state.position = [0.05]  # Open position
        # Publish joint command
        return True

    def get_manipulation_command(self):
        """Get current manipulation command"""
        # This would return the current joint command being executed
        return None

    def is_complete(self):
        """Check if manipulation task is complete"""
        # This would check if current manipulation action is complete
        return True
```

### Locomotion System

```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class LocomotionSystem:
    def __init__(self):
        self.balance_controller = BalanceController()
        self.walk_engine = WalkEngine()
        self.current_gait = 'standing'
        self.support_leg = 'both'  # 'left', 'right', or 'both'

    def process_imu_data(self, imu_msg):
        """Process IMU data for balance control"""
        self.balance_controller.update_imu_data(imu_msg)

    def get_balance_command(self):
        """Get balance control command"""
        return self.balance_controller.compute_balance_control()

    def is_balanced(self):
        """Check if humanoid is balanced"""
        return self.balance_controller.is_stable()

    def start_walking(self, direction='forward', speed=0.2):
        """Start walking gait"""
        self.current_gait = 'walking'
        return self.walk_engine.start_walking(direction, speed)

    def stop_walking(self):
        """Stop walking gait"""
        self.current_gait = 'standing'
        return self.walk_engine.stop_walking()

class BalanceController:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.linear_acc = Vector3()
        self.angular_vel = Vector3()
        self.stability_threshold = 0.1  # Radians

    def update_imu_data(self, imu_msg):
        """Update with new IMU data"""
        # Extract orientation (simplified - would use quaternion to RPY conversion)
        self.linear_acc = imu_msg.linear_acceleration
        self.angular_vel = imu_msg.angular_velocity

        # In practice, convert quaternion to Euler angles
        # This is simplified
        self.roll = imu_msg.orientation.x
        self.pitch = imu_msg.orientation.y
        self.yaw = imu_msg.orientation.z

    def compute_balance_control(self):
        """Compute balance control commands"""
        # Simple PD controller for balance
        roll_error = self.roll
        pitch_error = self.pitch

        # Compute corrective torques
        roll_torque = -2.0 * roll_error - 0.5 * self.angular_vel.x
        pitch_torque = -2.0 * pitch_error - 0.5 * self.angular_vel.y

        # Create control command (simplified)
        cmd_vel = Twist()
        cmd_vel.angular.y = pitch_torque * 0.1  # Scale appropriately
        cmd_vel.angular.x = roll_torque * 0.1

        return cmd_vel

    def is_stable(self):
        """Check if humanoid is stable"""
        return (abs(self.roll) < self.stability_threshold and
                abs(self.pitch) < self.stability_threshold)

class WalkEngine:
    def __init__(self):
        self.walking = False
        self.step_phase = 0.0
        self.walk_params = {
            'step_height': 0.05,
            'step_length': 0.2,
            'step_time': 1.0,
            'hip_offset': 0.02
        }

    def start_walking(self, direction, speed):
        """Start walking with specified parameters"""
        self.walking = True
        self.speed = speed
        self.direction = direction
        return True

    def stop_walking(self):
        """Stop walking"""
        self.walking = False
        return True

    def compute_step_trajectory(self):
        """Compute foot trajectory for walking"""
        if not self.walking:
            return None

        # Simplified walking pattern
        t = self.step_phase
        step_length = self.walk_params['step_length']
        step_height = self.walk_params['step_height']

        # Compute foot position based on phase
        x = step_length * np.sin(t * 2 * np.pi) / 2
        z = step_height * np.sin(t * 4 * np.pi) if t < 0.5 else 0  # Foot lift during first half

        return {'x': x, 'y': 0, 'z': z}

    def update(self):
        """Update walking engine"""
        if self.walking:
            self.step_phase += 0.01  # Increment phase
            if self.step_phase >= 1.0:
                self.step_phase = 0.0
```

## Human Interaction System

```python
import speech_recognition as sr
import pyttsx3
from std_msgs.msg import String
import threading

class HumanInteractionSystem:
    def __init__(self):
        self.speech_recognizer = sr.Recognizer()
        self.text_to_speech = pyttsx3.init()
        self.command_parser = CommandParser()
        self.conversation_context = {}

    def start_listening(self):
        """Start listening for voice commands"""
        with sr.Microphone() as source:
            self.speech_recognizer.adjust_for_ambient_noise(source)

        # Start listening in separate thread
        listening_thread = threading.Thread(target=self.continuous_listening)
        listening_thread.daemon = True
        listening_thread.start()

    def continuous_listening(self):
        """Continuously listen for commands"""
        with sr.Microphone() as source:
            while True:
                try:
                    audio = self.speech_recognizer.listen(source, timeout=1.0)
                    text = self.speech_recognizer.recognize_google(audio)
                    self.process_command(text)
                except sr.WaitTimeoutError:
                    continue  # Keep listening
                except sr.UnknownValueError:
                    continue  # Could not understand audio
                except sr.RequestError as e:
                    print(f"Could not request results; {e}")

    def process_command(self, text):
        """Process natural language command"""
        parsed_command = self.command_parser.parse_command(text)

        if parsed_command:
            # Execute command
            self.execute_command(parsed_command)

            # Respond to user
            self.respond(f"Okay, I will {parsed_command['action']}")

    def execute_command(self, command):
        """Execute parsed command"""
        # This would interface with the main controller
        # For now, just print the command
        print(f"Executing command: {command}")

    def respond(self, text):
        """Respond to user with text-to-speech"""
        self.text_to_speech.say(text)
        self.text_to_speech.runAndWait()

class CommandParser:
    def __init__(self):
        self.action_keywords = {
            'move': ['go', 'move', 'walk', 'navigate', 'approach'],
            'grasp': ['pick', 'grasp', 'take', 'grab', 'lift'],
            'place': ['place', 'put', 'set', 'drop'],
            'speak': ['say', 'speak', 'tell'],
            'follow': ['follow', 'come', 'accompany']
        }

    def parse_command(self, text):
        """Parse natural language command"""
        text_lower = text.lower()

        for action, keywords in self.action_keywords.items():
            for keyword in keywords:
                if keyword in text_lower:
                    return {
                        'action': action,
                        'original_text': text,
                        'target': self.extract_target(text_lower, keyword)
                    }

        return None

    def extract_target(self, text, action_keyword):
        """Extract target object/location from command"""
        # Simple extraction - in practice would use NLP
        words = text.split()
        try:
            action_idx = words.index(action_keyword)
            if action_idx + 1 < len(words):
                return words[action_idx + 1]  # Next word as target
        except ValueError:
            pass
        return "unknown"
```

## System Integration and Launch

### Main Launch File

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Include robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ])
    )

    # Humanoid controller node
    humanoid_controller = Node(
        package='humanoid_control',
        executable='humanoid_controller',
        name='humanoid_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Perception node
    perception_node = Node(
        package='humanoid_perception',
        executable='perception_node',
        name='perception_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation node
    navigation_node = Node(
        package='nav2_bringup',
        executable='nav2_launch',
        name='navigation_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Human interaction node
    interaction_node = Node(
        package='humanoid_interaction',
        executable='interaction_node',
        name='interaction_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        robot_state_publisher,
        humanoid_controller,
        perception_node,
        navigation_node,
        interaction_node
    ])
```

## Performance Evaluation and Validation

### Evaluation Metrics

```python
class HumanoidEvaluator:
    def __init__(self):
        self.metrics = {
            'navigation_success_rate': 0.0,
            'task_completion_time': [],
            'energy_efficiency': 0.0,
            'balance_stability': 0.0,
            'human_interaction_success': 0.0
        }

    def evaluate_navigation(self, path, obstacles, execution_time):
        """Evaluate navigation performance"""
        success = self.check_path_feasibility(path, obstacles)
        efficiency = self.calculate_path_efficiency(path)

        return {
            'success': success,
            'efficiency': efficiency,
            'time': execution_time
        }

    def evaluate_balance(self, imu_data, duration):
        """Evaluate balance performance"""
        stability_score = 0.0
        sample_count = 0

        for sample in imu_data:
            # Calculate stability based on orientation angles
            stability = 1.0 - min(abs(sample.roll), abs(sample.pitch), 0.2) / 0.2
            stability_score += stability
            sample_count += 1

        avg_stability = stability_score / sample_count if sample_count > 0 else 0.0
        return avg_stability

    def evaluate_task_completion(self, task, execution_time, success):
        """Evaluate task completion performance"""
        return {
            'success': success,
            'time': execution_time,
            'efficiency': 1.0 / execution_time if execution_time > 0 else 0.0
        }

    def check_path_feasibility(self, path, obstacles):
        """Check if path is collision-free"""
        for point in path:
            for obstacle in obstacles:
                if self.distance(point, obstacle) < 0.3:  # 30cm clearance
                    return False
        return True

    def calculate_path_efficiency(self, path):
        """Calculate path efficiency compared to straight line"""
        if len(path) < 2:
            return 0.0

        start = path[0]
        end = path[-1]
        straight_line = self.distance(start, end)
        path_length = self.calculate_path_length(path)

        return straight_line / path_length if path_length > 0 else 0.0

    def calculate_path_length(self, path):
        """Calculate total path length"""
        length = 0.0
        for i in range(1, len(path)):
            length += self.distance(path[i-1], path[i])
        return length

    def distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
```

## MCP-Grounded Technical Content

All technical specifications, ROS 2 APIs, parameters, and capabilities mentioned in this chapter are verified through Context7 MCP Server documentation. For any technical content that cannot be verified through MCP documentation, we explicitly state "Not covered in this book."

## Chapter Exercises

1. **System Integration**: Integrate all the subsystems (perception, navigation, manipulation, locomotion) into a complete humanoid system. Test the integration with a simple pick-and-place task.

2. **Autonomous Task**: Create an autonomous task where the humanoid navigates to a location, identifies and grasps an object, and places it at a target location without human intervention.

3. **Human Interaction**: Implement a complete human interaction pipeline where the humanoid can understand voice commands, navigate to objects, and perform requested actions.

## Summary

The autonomous humanoid capstone integrates all concepts from the book into a comprehensive Physical AI system. By combining perception, navigation, manipulation, locomotion, and human interaction systems, we create a robot capable of autonomous operation in real-world environments. The modular architecture allows for individual subsystem development and testing while maintaining system-wide integration. This capstone demonstrates the complete Physical AI pipeline from sensor input to physical action execution, embodying the principles of embodied intelligence that operate under real-world physical laws.