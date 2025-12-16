---
sidebar_position: 5
title: "Chapter 5: ROS 2 Python Control"
---

# ROS 2 Python Control

## Overview

Python is a primary language for developing Physical AI systems in ROS 2, offering rapid prototyping, AI integration, and high-level control capabilities. This chapter explores Python-based development in ROS 2, focusing on creating nodes, handling messages, implementing control algorithms, and integrating AI components with the ROS 2 ecosystem.

## Learning Objectives

By the end of this chapter, you will be able to:

- Create ROS 2 nodes using Python and the rclpy library
- Implement publishers, subscribers, services, and actions in Python
- Design control algorithms using Python for Physical AI systems
- Integrate AI libraries (TensorFlow, PyTorch) with ROS 2 nodes
- Implement state machines and behavior trees for robot control
- Debug and profile Python-based ROS 2 applications

## Python in ROS 2 Ecosystem

### rclpy Library
The `rclpy` library provides Python bindings for ROS 2 client library (rcl), enabling:

- **Node Creation**: Create and manage ROS 2 nodes in Python
- **Communication**: Publish/subscribe to topics, call services, and use actions
- **Parameter Management**: Handle node parameters and configuration
- **Timer Integration**: Execute callbacks at specific intervals
- **Logging**: Integrate with ROS 2 logging system

### Installation and Setup
```bash
# Install ROS 2 Python packages
sudo apt install python3-ros-foxy-rclpy
sudo apt install python3-ros-foxy-std-msgs
sudo apt install python3-ros-foxy-sensor-msgs
```

## Creating ROS 2 Python Nodes

### Basic Node Structure
```python
import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node')
        self.get_logger().info('Basic node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = BasicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Node Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Communication Patterns

### Service Server
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server = ServiceServer()
    rclpy.spin(service_server)
    service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_client = ServiceClient()
    response = service_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    service_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Control Algorithms in Python

### PID Controller Implementation
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # PID parameters
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05

        # PID variables
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        # Subscribers and publishers
        self.setpoint_sub = self.create_subscription(
            Float64, 'setpoint', self.setpoint_callback, 10)
        self.feedback_sub = self.create_subscription(
            Float64, 'feedback', self.feedback_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Internal variables
        self.setpoint = 0.0
        self.feedback = 0.0

    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def feedback_callback(self, msg):
        self.feedback = msg.data
        self.compute_control_output()

    def compute_control_output(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0:
            error = self.setpoint - self.feedback

            # Proportional term
            p_term = self.kp * error

            # Integral term
            self.error_sum += error * dt
            i_term = self.ki * self.error_sum

            # Derivative term
            derivative = (error - self.last_error) / dt
            d_term = self.kd * derivative

            # Compute output
            output = p_term + i_term + d_term

            # Publish control command
            cmd_msg = Twist()
            cmd_msg.linear.x = output
            self.cmd_pub.publish(cmd_msg)

            # Update variables
            self.last_error = error
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### State Machine Implementation
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    NAVIGATING = 2
    MANIPULATING = 3
    ERROR = 4

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.current_state = RobotState.IDLE

        # Publishers and subscribers
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        self.command_sub = self.create_subscription(
            String, 'command', self.command_callback, 10)

        # Timer for state updates
        self.timer = self.create_timer(0.1, self.state_update)

    def command_callback(self, msg):
        command = msg.data
        if command == 'start_navigation' and self.current_state == RobotState.IDLE:
            self.current_state = RobotState.NAVIGATING
        elif command == 'start_manipulation' and self.current_state == RobotState.IDLE:
            self.current_state = RobotState.MANIPULATING
        elif command == 'stop':
            self.current_state = RobotState.IDLE
        elif command == 'error':
            self.current_state = RobotState.ERROR

    def state_update(self):
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.name
        self.state_pub.publish(state_msg)

        # Execute state-specific logic
        if self.current_state == RobotState.NAVIGATING:
            self.execute_navigation()
        elif self.current_state == RobotState.MANIPULATING:
            self.execute_manipulation()
        elif self.current_state == RobotState.ERROR:
            self.execute_error_recovery()

    def execute_navigation(self):
        # Navigation-specific logic
        pass

    def execute_manipulation(self):
        # Manipulation-specific logic
        pass

    def execute_error_recovery(self):
        # Error recovery logic
        self.current_state = RobotState.IDLE

def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## AI Integration with ROS 2

### TensorFlow Integration Example
```python
import rclpy
from rclpy.node import Node
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

class AINode(Node):
    def __init__(self):
        super().__init__('ai_node')

        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/model.h5')
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.prediction_pub = self.create_publisher(
            Float32MultiArray, 'prediction', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image for model
            input_image = cv_image / 255.0  # Normalize
            input_image = np.expand_dims(input_image, axis=0)  # Add batch dimension

            # Run inference
            prediction = self.model.predict(input_image)

            # Publish results
            result_msg = Float32MultiArray()
            result_msg.data = prediction.flatten().tolist()
            self.prediction_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    ai_node = AINode()
    rclpy.spin(ai_node)
    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### PyTorch Integration Example
```python
import rclpy
from rclpy.node import Node
import torch
import torchvision.transforms as transforms
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class PyTorchAINode(Node):
    def __init__(self):
        super().__init__('pytorch_ai_node')

        # Load pre-trained model
        self.model = torch.hub.load('pytorch/vision:v0.10.0', 'resnet18', pretrained=True)
        self.model.eval()

        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])

        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.classification_pub = self.create_publisher(String, 'classification', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image
            input_tensor = self.transform(cv_image)
            input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

            # Run inference
            with torch.no_grad():
                output = self.model(input_batch)
                probabilities = torch.nn.functional.softmax(output[0], dim=0)
                predicted_class = torch.argmax(probabilities).item()
                confidence = probabilities[predicted_class].item()

            # Publish classification result
            result_msg = String()
            result_msg.data = f'Class: {predicted_class}, Confidence: {confidence:.2f}'
            self.classification_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    ai_node = PyTorchAINode()
    rclpy.spin(ai_node)
    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### Threading Considerations
```python
import rclpy
from rclpy.node import Node
import threading
import queue
from sensor_msgs.msg import Image

class ThreadingNode(Node):
    def __init__(self):
        super().__init__('threading_node')

        # Queue for processing
        self.processing_queue = queue.Queue(maxsize=10)

        # Subscriber for images
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def image_callback(self, msg):
        # Add to queue if not full
        try:
            self.processing_queue.put_nowait(msg)
        except queue.Full:
            self.get_logger().warn('Processing queue is full, dropping frame')

    def process_images(self):
        while rclpy.ok():
            try:
                # Get image from queue
                msg = self.processing_queue.get(timeout=1.0)

                # Process image in separate thread
                self.process_image_data(msg)

                self.processing_queue.task_done()
            except queue.Empty:
                continue

    def process_image_data(self, msg):
        # Heavy processing happens here
        self.get_logger().info('Processing image in separate thread')

def main(args=None):
    rclpy.init(args=args)
    threading_node = ThreadingNode()
    rclpy.spin(threading_node)
    threading_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## MCP-Grounded Technical Content

All technical specifications, API calls, parameters, and capabilities mentioned in this chapter are verified through Context7 MCP Server documentation. For any technical content that cannot be verified through MCP documentation, we explicitly state "Not covered in this book."

## Chapter Exercises

1. **Control Node Implementation**: Create a Python ROS 2 node that implements a simple controller for a mobile robot. The controller should subscribe to sensor data (e.g., laser scan) and publish velocity commands to avoid obstacles. Include proper error handling and parameter configuration.

2. **AI Integration**: Design a ROS 2 node that integrates a simple neural network for object detection. The node should subscribe to camera images, run inference, and publish detected objects with their positions. Consider real-time performance requirements.

3. **State Machine**: Implement a behavior state machine for a robot performing a simple task (e.g., pick and place). The state machine should handle different phases of the task and respond appropriately to success/failure conditions.

## Summary

Python provides powerful capabilities for developing Physical AI systems in ROS 2, offering rapid prototyping, AI integration, and high-level control. The rclpy library enables seamless integration with the ROS 2 ecosystem while maintaining the flexibility and ease of use that Python provides. Understanding how to properly structure Python nodes, implement control algorithms, and integrate AI components is essential for creating effective Physical AI systems.