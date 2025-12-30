---
sidebar_position: 3
---

# Chapter 2: Connecting Python AI Agents to ROS Controllers with rclpy

## Introduction to rclpy

The **rclpy** package is the Python client library for ROS 2. It provides the Python API for developing ROS 2 applications, allowing Python programs to interact with ROS 2 systems. This is particularly important for connecting AI agents written in Python to ROS-based robot control systems.

Python is widely used in AI and machine learning, making rclpy the perfect bridge between sophisticated AI algorithms and robotic systems. Whether you're working with neural networks, planning algorithms, or any other Python-based AI solution, rclpy enables seamless integration with ROS 2.

## Setting up rclpy

To use rclpy, you first need to initialize the ROS 2 client library:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 client library
    # Create and run your nodes here
    rclpy.shutdown()  # Clean shutdown
```

## Creating ROS 2 Nodes in Python

### Publisher Nodes

Publisher nodes send messages to topics. This is useful when your AI agent needs to send commands or data to other parts of the robot system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIPublisher(Node):
    def __init__(self):
        super().__init__('ai_publisher')
        self.publisher = self.create_publisher(String, 'ai_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # This could be where your AI logic runs
        ai_response = "Command from AI agent"
        msg = String()
        msg.data = ai_response
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    ai_publisher = AIPublisher()
    rclpy.spin(ai_publisher)
    ai_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Nodes

Subscriber nodes receive messages from topics. This is useful when your AI agent needs to process sensor data or other information from the robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AISubscriber(Node):
    def __init__(self):
        super().__init__('ai_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_sensors',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Process the received data - this could feed into your AI algorithm
        self.get_logger().info(f'Received: "{msg.data}"')
        # Here you could process the data with your AI agent
        self.process_with_ai(msg.data)

    def process_with_ai(self, sensor_data):
        # Your AI processing logic would go here
        print(f"AI processing: {sensor_data}")

def main(args=None):
    rclpy.init(args=args)
    ai_subscriber = AISubscriber()
    rclpy.spin(ai_subscriber)
    ai_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting AI Agents to Robot Control

### Example: AI-Based Navigation

Here's a more complete example showing how to connect an AI agent to robot navigation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class NavigationAI(Node):
    def __init__(self):
        super().__init__('navigation_ai')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_navigation_callback)

        # Robot state
        self.laser_data = None

    def scan_callback(self, msg):
        self.laser_data = msg.ranges

    def ai_navigation_callback(self):
        if self.laser_data is None:
            return

        # Simple AI algorithm: avoid obstacles
        cmd_vel = self.simple_navigation_ai(self.laser_data)
        self.cmd_vel_publisher.publish(cmd_vel)

    def simple_navigation_ai(self, laser_ranges):
        # Convert to numpy array for easier processing
        ranges = np.array(laser_ranges)
        ranges = ranges[~np.isnan(ranges)]  # Remove NaN values

        if len(ranges) == 0:
            # If no valid readings, stop
            cmd = Twist()
            return cmd

        # Find the closest obstacle
        min_distance = np.min(ranges)

        cmd = Twist()
        if min_distance < 1.0:  # If obstacle is closer than 1m
            # Turn to avoid obstacle
            cmd.angular.z = 0.5
        else:
            # Move forward
            cmd.linear.x = 0.5

        return cmd

def main(args=None):
    rclpy.init(args=args)
    nav_ai = NavigationAI()
    rclpy.spin(nav_ai)
    nav_ai.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Clients and Servers

### Service Client (AI requesting robot services)

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.srv import SetBool

class AIServiceClient(Node):
    def __init__(self):
        super().__init__('ai_service_client')
        self.cli = self.create_client(SetBool, 'robot_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def call_service(self, request_data):
        request = SetBool.Request()
        request.data = request_data
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    ai_client = AIServiceClient()

    # AI logic could call robot services
    response = ai_client.call_service(True)
    print(f'Service response: {response.success}')

    ai_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration Patterns

### Pattern 1: AI Agent as Controller
The AI agent makes decisions and sends commands to the robot. Sensor data flows to the AI, which then publishes control commands.

### Pattern 2: AI Agent as Supervisor
The AI agent supervises lower-level controllers, intervening when necessary based on high-level goals or safety considerations.

### Pattern 3: AI Agent as Planner
The AI agent creates plans or trajectories that are executed by lower-level controllers.

## Best Practices

1. **Error Handling**: Always implement proper error handling for ROS communication
2. **Resource Management**: Clean up nodes and resources properly
3. **Threading**: Be aware of threading implications when combining AI processing with ROS callbacks
4. **Rate Limiting**: Don't overwhelm the ROS system with too many messages
5. **Data Validation**: Validate sensor data before feeding it to AI algorithms

## Practical Exercise

Create a simple AI agent that:
1. Subscribes to a sensor topic (like laser scan data)
2. Applies a simple decision-making algorithm
3. Publishes velocity commands to control a robot
4. Uses a service to request specific robot capabilities when needed

## Summary

The rclpy library provides the essential bridge between Python-based AI agents and ROS 2 robot control systems. By understanding how to create publishers, subscribers, and service clients in Python, you can integrate sophisticated AI algorithms with robotic systems. The key is to structure your AI agent as a ROS node that can communicate with other parts of the robot system through topics and services.