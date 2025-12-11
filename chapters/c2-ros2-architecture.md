---
id: c2-ros2-architecture
title: "Chapter 2: ROS 2 Humble Architecture"
sidebar_label: "C2: ROS 2 Architecture"
sidebar_position: 2
---

# Chapter 2: ROS 2 Humble Architecture

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Install** ROS 2 Humble on Ubuntu 22.04 and create a functional colcon workspace
2. **Describe** the ROS 2 computational graph (Nodes, Topics, Messages) and explain how it enables distributed robot control
3. **Implement** a basic rclpy Publisher node demonstrating node structure and topic publishing

## Overview

In Chapter 1, we established that Physical AI systems require real-time sensor integration and actuator control. **ROS 2 (Robot Operating System 2)** serves as the "nervous system" of humanoid robots, providing the middleware infrastructure that bridges high-level AI decision layers to low-level physical actuator commands.

Unlike traditional monolithic software architectures, humanoid robots demand **distributed computing**: sensor drivers run on dedicated processors, AI models execute on GPUs, and motor controllers operate on real-time hardware. ROS 2's computational graph architecture enables this distribution while maintaining coordinated behavior.

This chapter introduces ROS 2 Humble Hawksbill (the Long-Term Support release) as the foundation for humanoid robotics development.

## Key Concepts

- **ROS 2**: Open-source middleware framework providing communication infrastructure and hardware abstraction for robot software
- **Node**: Independent computational unit encapsulating a specific function (sensor driver, controller, planner)
- **Topic**: Named communication channel for asynchronous message passing between nodes
- **Message**: Structured data packet transmitted over Topics (e.g., `std_msgs/String`, `sensor_msgs/Imu`)
- **Publisher**: Node component that sends messages to a Topic
- **Subscriber**: Node component that receives messages from a Topic
- **rclpy**: Python client library for ROS 2, providing API for creating nodes and communication
- **QoS (Quality of Service)**: Policy controlling message delivery behavior (reliability, durability, history)

## ROS 2 Installation (Ubuntu 22.04)

### Setup Commands

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Creating Your First ROS 2 Publisher Node

### Minimal Publisher Example

This example demonstrates the core pattern of ROS 2 node creation and topic publishing.

```python
# minimal_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        # Initialize node with name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create publisher: topic name 'topic', message type String, queue size 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create timer: calls timer_callback every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0  # Message counter

    def timer_callback(self):
        # Create and populate message
        msg = String()
        msg.data = 'Hello World: %d' % self.i

        # Publish message to topic
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create and spin the node (keeps it running)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Cleanup on shutdown
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Publisher

```bash
# Run the publisher node
python3 minimal_publisher.py

# In another terminal, echo messages to verify
ros2 topic echo /topic
```

The publisher creates a **Node** named `minimal_publisher`, establishes a **Publisher** on the **Topic** `/topic`, and uses a timer **callback** to send **Messages** every 0.5 seconds. This pattern forms the foundation of ROS 2 communication.

## Summary

ROS 2 Humble provides the distributed middleware infrastructure essential for humanoid robotics. The computational graph architecture—built on **Nodes**, **Topics**, and **Messages**—enables modular, scalable robot control systems. The **rclpy** library allows Python developers to create nodes that integrate with the broader ROS 2 ecosystem.

In Chapter 3, we will extend this foundation by implementing Subscriber nodes and exploring Service-based communication for request-response patterns.

## Review Questions

1. **Conceptual**: Why does ROS 2 use a distributed computational graph instead of a monolithic architecture for humanoid robots?

2. **Applied**: Modify the `MinimalPublisher` to publish at 10 Hz instead of 2 Hz. What parameter must change?

3. **Structural**: What are the three required components for a ROS 2 Publisher node as demonstrated in `minimal_publisher.py`?
