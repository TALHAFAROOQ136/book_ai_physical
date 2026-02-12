---
sidebar_position: 2
---

# ROS 2 Nodes, Topics, and Services

**The Three Communication Patterns That Power Every Robot**

---

## Overview

Every ROS 2 system is built from three fundamental communication patterns: **Topics** for streaming data, **Services** for request-response queries, and **Actions** for long-running tasks with feedback. In this chapter, you will implement all three from scratch, understand when to use each, and build a multi-node system that coordinates a humanoid robot's perception and movement.

## Nodes: The Building Blocks

A **node** is a single-purpose process in ROS 2. Good robotics design follows the Unix philosophy: each node does one thing well. A typical humanoid robot system might have dozens of nodes:

```
Camera Driver Node ──► Image Processing Node ──► Object Detection Node
                                                        │
IMU Sensor Node ──► Pose Estimation Node               │
                          │                             │
                          ▼                             ▼
                    Balance Controller ◄── Motion Planner
                          │
                          ▼
                    Joint Command Node ──► Hardware Interface
```

### Creating a ROS 2 Package

Before writing nodes, you need a package — a container for related nodes, message definitions, and configuration files.

```bash
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python humanoid_control \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs

cd humanoid_control
```

This creates:

```
humanoid_control/
├── humanoid_control/        # Python module
│   └── __init__.py
├── resource/
│   └── humanoid_control
├── test/
├── package.xml              # Package metadata and dependencies
├── setup.py                 # Python package configuration
└── setup.cfg
```

### Anatomy of a Node

Every ROS 2 Python node follows the same structure:

```python
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')  # Unique name in the ROS graph
        self.get_logger().info('Node has started!')

    def destroy_node(self):
        self.get_logger().info('Node is shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)           # Initialize ROS 2
    node = MyNode()
    rclpy.spin(node)                # Keep the node alive and processing
    node.destroy_node()
    rclpy.shutdown()                # Clean up


if __name__ == '__main__':
    main()
```

Key elements:
- **`rclpy.init()`** — Initializes the ROS 2 communication layer.
- **`Node` class** — Base class providing access to topics, services, timers, and logging.
- **`rclpy.spin()`** — Event loop that processes incoming messages and timer callbacks.
- **`rclpy.shutdown()`** — Cleans up resources when the node exits.

---

## Topics: Publish-Subscribe Communication

Topics are the workhorse of ROS 2 communication. They implement the **publish-subscribe** pattern: publishers send messages to a named channel, and any number of subscribers receive those messages. There is no direct connection between publisher and subscriber — they are fully decoupled.

### When to Use Topics

- Continuous sensor data (camera images, IMU readings, joint states)
- One-to-many or many-to-many communication
- Data that is produced regardless of whether anyone is listening
- High-frequency data streams

### Message Types

ROS 2 messages are strongly typed. Common message types include:

| Package | Message | Use Case |
|---------|---------|----------|
| `std_msgs` | `String`, `Int32`, `Float64`, `Bool` | Simple data |
| `sensor_msgs` | `Image`, `Imu`, `LaserScan`, `JointState` | Sensor data |
| `geometry_msgs` | `Twist`, `Pose`, `Point`, `Quaternion` | Positions and velocities |
| `nav_msgs` | `Odometry`, `Path`, `OccupancyGrid` | Navigation data |

### Building a Publisher

Let us create a node that simulates a humanoid robot's IMU sensor, publishing orientation and acceleration data:

```python
# humanoid_control/imu_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
import math
import random


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Create a publisher on the /humanoid/imu topic
        self.publisher_ = self.create_publisher(
            Imu,                    # Message type
            '/humanoid/imu',        # Topic name
            10                      # Queue size (QoS depth)
        )

        # Publish at 100 Hz (every 10ms)
        self.timer = self.create_timer(0.01, self.publish_imu)
        self.start_time = self.get_clock().now()
        self.get_logger().info('IMU publisher started at 100 Hz')

    def publish_imu(self):
        msg = Imu()

        # Set the timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate orientation (quaternion) — robot swaying slightly
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        sway = 0.02 * math.sin(elapsed * 2.0)  # Gentle side-to-side sway

        msg.orientation.x = 0.0
        msg.orientation.y = sway
        msg.orientation.z = 0.0
        msg.orientation.w = math.sqrt(1.0 - sway * sway)

        # Simulate linear acceleration (gravity + noise)
        msg.linear_acceleration.x = random.gauss(0.0, 0.1)
        msg.linear_acceleration.y = random.gauss(0.0, 0.1)
        msg.linear_acceleration.z = 9.81 + random.gauss(0.0, 0.05)

        # Simulate angular velocity
        msg.angular_velocity.x = random.gauss(0.0, 0.01)
        msg.angular_velocity.y = 2.0 * 0.02 * math.cos(elapsed * 2.0)
        msg.angular_velocity.z = random.gauss(0.0, 0.01)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Building a Subscriber

Now let us create a **balance monitor** node that subscribes to the IMU data and detects when the robot is tilting dangerously:

```python
# humanoid_control/balance_monitor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math


class BalanceMonitor(Node):
    def __init__(self):
        super().__init__('balance_monitor')

        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/humanoid/imu',
            self.imu_callback,
            10
        )

        # Publish balance status
        self.status_pub = self.create_publisher(String, '/humanoid/balance_status', 10)

        # Tilt threshold in radians (~5.7 degrees)
        self.tilt_threshold = 0.1
        self.get_logger().info('Balance monitor started')

    def imu_callback(self, msg: Imu):
        # Extract pitch and roll from quaternion
        q = msg.orientation
        # Simplified pitch calculation from quaternion
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )

        total_tilt = math.sqrt(pitch ** 2 + roll ** 2)

        status = String()
        if total_tilt > self.tilt_threshold:
            status.data = f'WARNING: Tilt={math.degrees(total_tilt):.1f}deg - UNSTABLE'
            self.get_logger().warn(status.data)
        else:
            status.data = f'OK: Tilt={math.degrees(total_tilt):.1f}deg - STABLE'

        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Running the Publisher-Subscriber Pair

Register both nodes in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'imu_publisher = humanoid_control.imu_publisher:main',
        'balance_monitor = humanoid_control.balance_monitor:main',
    ],
},
```

Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_control
source install/setup.bash

# Terminal 1
ros2 run humanoid_control imu_publisher

# Terminal 2
ros2 run humanoid_control balance_monitor

# Terminal 3 — inspect the data
ros2 topic echo /humanoid/balance_status
```

### Custom Message Types

Standard messages do not always fit your needs. You can define custom messages for your humanoid robot:

```
# Create a message package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake humanoid_msgs
mkdir humanoid_msgs/msg
```

Define a custom message for humanoid joint commands:

```
# humanoid_msgs/msg/JointCommand.msg
string joint_name          # e.g., "left_knee", "right_shoulder_pitch"
float64 target_position    # Target angle in radians
float64 max_velocity       # Maximum angular velocity (rad/s)
float64 max_effort         # Maximum torque (Nm)
```

After building, use it like any other message:

```python
from humanoid_msgs.msg import JointCommand

cmd = JointCommand()
cmd.joint_name = 'left_knee'
cmd.target_position = 1.57  # 90 degrees
cmd.max_velocity = 2.0
cmd.max_effort = 50.0
```

---

## Services: Request-Response Communication

Services implement a **synchronous request-response** pattern. A client sends a request and blocks until the server responds. This is fundamentally different from topics — services are for one-time computations or queries, not continuous data streams.

### When to Use Services

- One-time queries ("What is the robot's current pose?")
- Computations that return a result ("Plan a path from A to B")
- Configuration changes ("Set the camera resolution to 1080p")
- State transitions ("Enable/disable the balance controller")

### Service Definition

A service has two parts: a **request** and a **response**. Here is a custom service for commanding the humanoid to move to a specific pose:

```
# humanoid_msgs/srv/MoveToPose.srv
# Request
geometry_msgs/Pose target_pose    # Where to move
float64 duration                  # How long the motion should take (seconds)
---
# Response
bool success                      # Did the motion complete?
string message                    # Description of result or error
float64 actual_duration           # How long it actually took
```

### Building a Service Server

```python
# humanoid_control/pose_service.py
import rclpy
from rclpy.node import Node
from humanoid_msgs.srv import MoveToPose
import time


class PoseService(Node):
    def __init__(self):
        super().__init__('pose_service')

        self.srv = self.create_service(
            MoveToPose,
            '/humanoid/move_to_pose',
            self.handle_move_request
        )

        self.get_logger().info('Pose service ready — waiting for requests...')

    def handle_move_request(self, request, response):
        target = request.target_pose
        self.get_logger().info(
            f'Received move request: position=({target.position.x:.2f}, '
            f'{target.position.y:.2f}, {target.position.z:.2f}), '
            f'duration={request.duration:.1f}s'
        )

        # Simulate motion execution
        start_time = time.time()

        # In a real system, this would send joint commands
        # and wait for the motion to complete
        self.get_logger().info('Executing motion...')

        # Simulate the motion taking some time
        # (In production, this would be non-blocking with callbacks)

        elapsed = time.time() - start_time
        response.success = True
        response.message = f'Motion completed to ({target.position.x:.2f}, {target.position.y:.2f}, {target.position.z:.2f})'
        response.actual_duration = elapsed

        self.get_logger().info(f'Motion complete: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Building a Service Client

```python
# humanoid_control/pose_client.py
import rclpy
from rclpy.node import Node
from humanoid_msgs.srv import MoveToPose
from geometry_msgs.msg import Pose, Point, Quaternion


class PoseClient(Node):
    def __init__(self):
        super().__init__('pose_client')
        self.client = self.create_client(MoveToPose, '/humanoid/move_to_pose')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pose service...')

        self.get_logger().info('Connected to pose service!')

    def send_move_request(self, x, y, z, duration=2.0):
        request = MoveToPose.Request()
        request.target_pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        request.duration = duration

        self.get_logger().info(f'Sending move request to ({x}, {y}, {z})...')
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    client = PoseClient()

    # Send a move request
    future = client.send_move_request(1.0, 0.5, 0.0, duration=3.0)
    rclpy.spin_until_future_complete(client, future)

    result = future.result()
    if result.success:
        client.get_logger().info(f'Success: {result.message} in {result.actual_duration:.2f}s')
    else:
        client.get_logger().error(f'Failed: {result.message}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Actions: Long-Running Tasks with Feedback

Actions combine the best of topics and services. They are for **long-running tasks** where you need to send a goal, receive periodic progress feedback, and get a final result. Actions can also be **cancelled** mid-execution.

### When to Use Actions

- Walking to a destination (takes seconds to minutes)
- Picking up an object (multi-step process)
- Performing a full-body gesture (complex motion sequence)
- Any task where the caller needs progress updates

### Action Definition

```
# humanoid_msgs/action/Walk.action
# Goal — what we want the robot to do
geometry_msgs/Pose target_pose
float64 speed               # meters per second
---
# Result — final outcome
bool success
float64 distance_traveled
float64 time_taken
---
# Feedback — periodic updates during execution
float64 distance_remaining
float64 estimated_time_remaining
string current_state         # e.g., "walking", "turning", "avoiding_obstacle"
```

### Building an Action Server

```python
# humanoid_control/walk_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from humanoid_msgs.action import Walk
import math
import time


class WalkServer(Node):
    def __init__(self):
        super().__init__('walk_server')

        self._action_server = ActionServer(
            self,
            Walk,
            '/humanoid/walk',
            self.execute_callback
        )

        self.get_logger().info('Walk action server ready')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(
            f'Walking to ({goal_handle.request.target_pose.position.x:.1f}, '
            f'{goal_handle.request.target_pose.position.y:.1f}) '
            f'at {goal_handle.request.speed:.1f} m/s'
        )

        feedback_msg = Walk.Feedback()
        target = goal_handle.request.target_pose.position
        total_distance = math.sqrt(target.x ** 2 + target.y ** 2)
        speed = goal_handle.request.speed
        start_time = time.time()

        distance_covered = 0.0
        step_size = speed * 0.1  # Update every 100ms

        while distance_covered < total_distance:
            # Check if the action was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Walk cancelled!')
                result = Walk.Result()
                result.success = False
                result.distance_traveled = distance_covered
                result.time_taken = time.time() - start_time
                return result

            # Simulate walking progress
            distance_covered = min(distance_covered + step_size, total_distance)
            remaining = total_distance - distance_covered

            # Send feedback
            feedback_msg.distance_remaining = remaining
            feedback_msg.estimated_time_remaining = remaining / speed if speed > 0 else 0.0
            feedback_msg.current_state = 'walking' if remaining > 0.5 else 'approaching'
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)  # 10 Hz feedback rate

        # Goal completed
        goal_handle.succeed()
        result = Walk.Result()
        result.success = True
        result.distance_traveled = total_distance
        result.time_taken = time.time() - start_time

        self.get_logger().info(
            f'Walk complete: {result.distance_traveled:.1f}m in {result.time_taken:.1f}s'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WalkServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Putting It All Together: A Multi-Node System

Here is how these patterns combine in a humanoid robot:

```
┌──────────────┐  Topic:/imu    ┌──────────────┐
│  IMU Sensor  │───────────────►│   Balance     │
│  (Publisher)  │               │   Monitor     │
└──────────────┘               │  (Subscriber)  │
                                └───────┬───────┘
                                        │ Topic:/balance_status
                                        ▼
┌──────────────┐  Service      ┌──────────────┐
│  Path Planner │◄────────────►│  Walk         │
│  (Server)     │  /plan_path  │  Coordinator  │
└──────────────┘               └───────┬───────┘
                                        │ Action:/walk
                                        ▼
                                ┌──────────────┐
                                │  Walk Server  │
                                │  (Action)     │
                                └───────┬───────┘
                                        │ Topic:/joint_commands
                                        ▼
                                ┌──────────────┐
                                │  Motor Driver │
                                └──────────────┘
```

- **Topics** carry the continuous sensor streams (IMU at 100Hz, camera at 30fps).
- **Services** handle one-time queries (path planning, configuration).
- **Actions** manage long-running motions (walking, gestures) with cancellation support.

---

## Exercises

### Exercise 1: Joint State Publisher
Create a node that publishes `sensor_msgs/JointState` messages for a 6-joint humanoid leg (hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll). Simulate a walking cycle where joints oscillate sinusoidally.

### Exercise 2: Emergency Stop Service
Create a service that immediately halts all robot motion. The service server should publish a zero-velocity command to `/humanoid/cmd_vel` when called.

### Exercise 3: Multi-Node Launch
Write a ROS 2 launch file (`launch/humanoid_system.launch.py`) that starts all nodes simultaneously: IMU publisher, balance monitor, pose service, and walk server.

---

:::tip Key Takeaway
Choose the right communication pattern for the job: **Topics** for continuous data streams, **Services** for synchronous request-response, and **Actions** for long-running tasks with feedback. A well-designed ROS 2 system uses all three patterns together, each for what it does best.
:::
