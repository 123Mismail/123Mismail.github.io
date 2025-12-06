---
id: c3-ros2-actions
title: "Chapter 3: ROS 2 Actions and Services"
sidebar_label: "C3: Actions & Services"
sidebar_position: 3
---

# Chapter 3: ROS 2 Actions and Services

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Distinguish** between ROS 2 Actions and Services and explain when to use each communication pattern
2. **Implement** an Action Server and Client with goal handling, feedback, and result processing
3. **Create** a Service Server and Client for synchronous request-response operations

## Overview

In Chapter 2, we explored ROS 2's publish/subscribe pattern using Topics for continuous data streams. However, humanoid robots require additional communication paradigms for goal-oriented tasks. When a humanoid must navigate to a target location, execute a multi-step manipulation sequence, or wait for sensor calibration, simple message passing is insufficient.

**Actions** provide goal-oriented, long-running operations with feedback and cancellation support. **Services** enable synchronous request-response interactions for short-duration queries. Together, these patterns complement Topics to create the complete communication infrastructure for complex robotic systems.

This chapter demonstrates how Actions coordinate multi-step behaviors (like walking gaits or manipulation sequences) while Services handle queries (parameter requests, state checks). Understanding these patterns is essential for implementing the control architectures in subsequent chapters.

## Key Concepts

- **Action**: Asynchronous goal-oriented communication with three components: Goal (request), Feedback (progress updates), and Result (final outcome)
- **Service**: Synchronous request-response communication pattern where a client waits for a server's reply
- **Goal**: Target specification sent by an Action Client to an Action Server (e.g., "navigate to position X,Y")
- **Feedback**: Incremental progress updates sent from Action Server to Client during goal execution
- **Result**: Final outcome returned by Action Server when goal completes (success or failure)
- **Action Server**: Node component that accepts goals, executes operations, publishes feedback, and returns results
- **Action Client**: Node component that sends goals, receives feedback, and processes results
- **Service Server**: Node component that listens for requests and returns responses
- **Service Client**: Node component that sends requests and waits for responses

## Actions vs Services: When to Use Each

| Pattern | Duration | Feedback | Cancellable | Use Case |
|---------|----------|----------|-------------|----------|
| **Action** | Long (seconds to minutes) | Yes (continuous updates) | Yes | Navigation, gait execution, manipulation sequences |
| **Service** | Short (milliseconds) | No | No | Parameter queries, sensor readings, state checks |
| **Topic** | Continuous | N/A | N/A | Sensor streams, command velocities, state updates |

**Rule of Thumb**: Use Actions for any operation that takes >1 second or requires progress monitoring. Use Services for quick queries that expect immediate responses.

## ROS 2 Actions: Goal-Oriented Communication

### Action Definition File Format

Actions are defined in `.action` files with three sections separated by `---`:

```action
# Goal (request from client to server)
int32 order
---
# Result (final response from server to client)
int32[] sequence
---
# Feedback (incremental updates during execution)
int32[] partial_sequence
```

### Action Server Example

This example implements a Fibonacci sequence generator as an Action Server, demonstrating goal handling, feedback publishing, and result return.

```python
# fibonacci_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server: action type, node, action name, execute callback
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Initialize feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Generate Fibonacci sequence up to goal.order
        for i in range(1, goal_handle.request.order):
            # Publish feedback with current partial sequence
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            )
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)

        # Mark goal as succeeded and return result
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)

if __name__ == '__main__':
    main()
```

### Action Client Example with Feedback

This client sends a Fibonacci goal, monitors feedback during execution, and retrieves the final result.

```python
# fibonacci_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Wait for action server to be available
        self._action_client.wait_for_server()

        # Send goal with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## ROS 2 Services: Request-Response Communication

### Service Definition File Format

Services are defined in `.srv` files with two sections separated by `---`:

```srv
# Request (from client to server)
int64 a
int64 b
---
# Response (from server to client)
int64 sum
```

### Service Server Example

```python
# add_two_ints_server.py
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service: service type, service name, callback function
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # Process request and populate response
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: a=%d b=%d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    rclpy.spin(add_two_ints_server)

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
# add_two_ints_client.py
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client: service type, service name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Call service asynchronously
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    future = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    # Spin until response received
    rclpy.spin_until_future_complete(client, future)
    response = future.result()

    client.get_logger().info('Result: %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 provides three complementary communication patterns: **Topics** for continuous data streams, **Actions** for goal-oriented operations with feedback and cancellation, and **Services** for synchronous request-response queries. Actions enable humanoid robots to execute long-running behaviors (navigation, manipulation) while reporting progress and allowing cancellation. Services provide quick parameter queries and state checks.

In Chapter 4, we will apply these communication patterns to robot description formats (URDF), enabling us to model humanoid robot kinematics and visualize robot configurations.

## Review Questions

1. **Conceptual**: Why are Actions preferable to Services for implementing a humanoid walking gait controller?

2. **Applied**: Modify the Fibonacci Action Server to limit execution to 30 seconds and return a partial result if the goal cannot complete in time.

3. **Structural**: What are the three callback functions in the `FibonacciActionClient`, and what role does each play in the action lifecycle?
