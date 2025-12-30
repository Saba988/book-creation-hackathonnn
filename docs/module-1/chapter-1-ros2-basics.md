---
sidebar_position: 2
---

# Chapter 1: ROS 2 Basics

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 serves as the "nervous system" of a robot, providing the infrastructure for different software components to communicate with each other. It enables distributed computing where different parts of a robot's software can run on different computers and still communicate seamlessly.

## Key Concepts

### Nodes

A **Node** is a process that performs computation. In ROS 2, nodes are the fundamental building blocks of a robot application. Each node typically handles a specific task, such as sensor processing, motion planning, or control.

Nodes are lightweight and designed to be modular. A complete robot system usually consists of many nodes working together. For example, a mobile robot might have nodes for:
- Sensor data acquisition
- Localization
- Path planning
- Motor control
- User interface

In Python, a basic ROS 2 node looks like this:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Messages

**Topics** are named buses over which nodes exchange messages. They form the backbone of ROS 2's communication system. Topics allow for asynchronous communication between nodes using a publish/subscribe model.

A node can publish messages to a topic, and other nodes can subscribe to that topic to receive the messages. This creates a decoupled system where publishers don't need to know about subscribers, and vice versa.

Messages are the data structures that travel between nodes. They have a specific type and structure. For example, sensor data might be published as a `sensor_msgs/LaserScan` message, while velocity commands might be sent as `geometry_msgs/Twist` messages.

### Services

**Services** provide a request/response communication pattern between nodes. Unlike topics which enable asynchronous, one-way communication, services allow for synchronous, two-way communication.

A service has a client and a server. The client sends a request to the server, which processes the request and sends back a response. Services are useful for operations that need confirmation or return specific results, such as:
- Saving a map
- Setting parameters
- Triggering specific actions

## Communication Architecture

### DDS - Data Distribution Service

ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware. DDS provides:
- Real-time communication capabilities
- Quality of Service (QoS) settings
- Discovery mechanisms
- Data persistence options

DDS enables ROS 2 to work in complex, distributed systems where reliability and performance are critical.

### Quality of Service (QoS)

QoS settings allow you to specify how messages should be delivered based on your application's requirements. You can configure:
- Reliability (best effort vs. reliable)
- Durability (volatile vs. transient)
- History (keep last N messages vs. keep all)
- Deadline and lifespan constraints

## Practical Exercise

To solidify your understanding, try the following:

1. Run a simple ROS 2 publisher and subscriber
2. Observe how they communicate through topics
3. Notice the decoupled nature of the communication

You can use the built-in demo examples:
```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_py listener
```

## Summary

ROS 2 provides a robust framework for robot software development through its node-based architecture. The publish/subscribe model using topics enables flexible communication between different parts of a robot system, while services provide synchronous request/response interactions when needed.

Understanding these fundamental concepts is crucial for working with ROS 2 systems, as they form the basis for all robot communication and coordination.