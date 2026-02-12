---
sidebar_position: 3
---

# Bridging Python Agents to ROS 2 with rclpy

**Connecting AI Intelligence to Robot Control**

---

## Overview

You already know how to build AI systems in Python — neural networks, language models, planning algorithms. Now it is time to connect that intelligence to a physical robot. The **rclpy** library is the bridge: it lets you write ROS 2 nodes entirely in Python, accessing the full power of the ROS 2 ecosystem while leveraging Python's vast AI/ML libraries.

In this chapter, you will build increasingly sophisticated Python agents that perceive the world through sensors, make intelligent decisions, and command a humanoid robot's actuators — all through rclpy.

## Why Python for Robotics?

Robotics traditionally favored C++ for its raw performance. But modern robotics has shifted:

| Concern | C++ (rclcpp) | Python (rclpy) |
|---------|-------------|----------------|
| **Performance** | Fastest execution | Sufficient for most AI/control loops |
| **AI/ML ecosystem** | Limited | PyTorch, TensorFlow, OpenAI, NumPy, SciPy |
| **Development speed** | Slow iteration | Rapid prototyping |
| **Debugging** | Complex toolchain | Simple print/pdb debugging |
| **Community** | Robotics-specific | Massive general-purpose + robotics |

The modern approach is to use **Python for high-level AI and decision-making** and **C++ only for performance-critical drivers** (e.g., real-time motor control at 1kHz+). For this textbook, Python is our primary language.

## rclpy Fundamentals

### The Node Lifecycle

Every rclpy node goes through a lifecycle:

```
Create → Initialize → Active (spinning) → Shutdown → Destroy
```

The most important concept is **spinning**. When you call `rclpy.spin(node)`, ROS 2 enters an event loop that:
1. Checks for incoming messages on subscribed topics
2. Fires timer callbacks at their scheduled intervals
3. Processes incoming service requests
4. Handles action goals and feedback
5. Repeats until the node is shut down

### Timer-Based Control Loops

Most robot control uses **timer callbacks** — functions that execute at a fixed rate:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Publish velocity commands at 50 Hz
        self.cmd_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.step_phase = 0.0
        self.get_logger().info('Walking controller started at 50 Hz')

    def control_loop(self):
        """Called every 20ms — the main control loop."""
        cmd = Twist()

        # Simple forward walking command
        cmd.linear.x = 0.3   # 0.3 m/s forward
        cmd.angular.z = 0.0  # No turning

        self.cmd_pub.publish(cmd)
        self.step_phase += 0.02


def main(args=None):
    rclpy.init(args=args)
    node = WalkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Multi-Subscription Nodes

Real robots need to process multiple data streams simultaneously. rclpy handles this naturally — each subscription has its own callback:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, JointState
from std_msgs.msg import String


class SensorFusion(Node):
    """Fuses data from multiple sensors into a unified state estimate."""

    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to multiple sensor streams
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid/imu', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/humanoid/joint_states', self.joint_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/humanoid/camera/rgb', self.camera_callback, 10)

        # Publish fused state
        self.state_pub = self.create_publisher(String, '/humanoid/state', 10)

        # Internal state storage
        self.latest_imu = None
        self.latest_joints = None
        self.latest_image = None

        # Fusion runs at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.fuse_state)
        self.get_logger().info('Sensor fusion node started')

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def joint_callback(self, msg: JointState):
        self.latest_joints = msg

    def camera_callback(self, msg: Image):
        self.latest_image = msg

    def fuse_state(self):
        """Combine all sensor data into a unified state estimate."""
        if self.latest_imu is None or self.latest_joints is None:
            return

        # Extract key information
        pitch = self.latest_imu.orientation.y
        num_joints = len(self.latest_joints.position) if self.latest_joints.position else 0
        has_image = self.latest_image is not None

        state = String()
        state.data = (
            f'pitch={pitch:.3f}, '
            f'joints={num_joints}, '
            f'camera={"active" if has_image else "no data"}'
        )
        self.state_pub.publish(state)
```

## Building Intelligent Agents

Now let us move beyond simple publishers and subscribers to build agents that **perceive, decide, and act**.

### Pattern: The Reactive Agent

A reactive agent maps sensor inputs directly to motor outputs — no memory, no planning, just stimulus-response. This is the simplest agent architecture but surprisingly effective for balance control and obstacle avoidance.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
import math


class ReactiveWalker(Node):
    """
    A reactive agent that walks forward while:
    - Maintaining balance (using IMU)
    - Avoiding obstacles (using laser scan)
    """

    def __init__(self):
        super().__init__('reactive_walker')

        # Sensors (input)
        self.create_subscription(Imu, '/humanoid/imu', self.on_imu, 10)
        self.create_subscription(LaserScan, '/humanoid/scan', self.on_scan, 10)

        # Actuators (output)
        self.cmd_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self.act)

        # Perception state
        self.tilt = 0.0
        self.min_obstacle_distance = float('inf')
        self.obstacle_direction = 0.0  # -1 left, 0 center, 1 right

        self.get_logger().info('Reactive walker agent started')

    def on_imu(self, msg: Imu):
        """Perceive: Extract tilt from IMU orientation."""
        q = msg.orientation
        self.tilt = math.asin(2.0 * (q.w * q.y - q.z * q.x))

    def on_scan(self, msg: LaserScan):
        """Perceive: Find closest obstacle and its direction."""
        if not msg.ranges:
            return

        min_dist = float('inf')
        min_idx = 0

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and r < min_dist:
                min_dist = r
                min_idx = i

        self.min_obstacle_distance = min_dist

        # Determine if obstacle is left, center, or right
        angle = msg.angle_min + min_idx * msg.angle_increment
        self.obstacle_direction = -1.0 if angle > 0.1 else (1.0 if angle < -0.1 else 0.0)

    def act(self):
        """Decide and Act: Generate velocity commands based on perception."""
        cmd = Twist()

        # Rule 1: If tilting too much, stop and stabilize
        if abs(self.tilt) > 0.15:  # ~8.6 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(f'Stabilizing! Tilt: {math.degrees(self.tilt):.1f}deg')
            self.cmd_pub.publish(cmd)
            return

        # Rule 2: If obstacle is close, turn away
        if self.min_obstacle_distance < 1.0:
            cmd.linear.x = 0.1  # Slow down
            # Turn away from obstacle
            cmd.angular.z = 0.5 * self.obstacle_direction
            self.get_logger().info(
                f'Avoiding obstacle at {self.min_obstacle_distance:.1f}m'
            )
            self.cmd_pub.publish(cmd)
            return

        # Rule 3: Default — walk forward
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ReactiveWalker())
    rclpy.shutdown()
```

### Pattern: The Stateful Agent

A stateful agent maintains internal memory and uses it to make decisions. This enables more complex behaviors like sequential task execution:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
from enum import Enum


class RobotState(Enum):
    IDLE = 'idle'
    WALKING = 'walking'
    REACHING = 'reaching'
    GRASPING = 'grasping'
    RETURNING = 'returning'


class StatefulAgent(Node):
    """
    A stateful agent that executes a pick-and-place task:
    1. Walk to the object
    2. Reach for it
    3. Grasp it
    4. Return to starting position
    """

    def __init__(self):
        super().__init__('stateful_agent')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointState, '/humanoid/arm_command', 10)
        self.gripper_pub = self.create_publisher(String, '/humanoid/gripper', 10)

        # Subscribers
        self.create_subscription(
            PoseStamped, '/humanoid/pose', self.on_pose, 10)
        self.create_subscription(
            PoseStamped, '/object/pose', self.on_object_pose, 10)

        # State machine
        self.state = RobotState.IDLE
        self.robot_pose = None
        self.object_pose = None
        self.home_pose = None

        # Control loop
        self.create_timer(0.1, self.update)  # 10 Hz
        self.get_logger().info('Stateful pick-and-place agent ready')

    def on_pose(self, msg: PoseStamped):
        self.robot_pose = msg.pose
        if self.home_pose is None:
            self.home_pose = msg.pose  # Remember starting position

    def on_object_pose(self, msg: PoseStamped):
        self.object_pose = msg.pose

    def distance_to(self, target_pose) -> float:
        if self.robot_pose is None or target_pose is None:
            return float('inf')
        dx = target_pose.position.x - self.robot_pose.position.x
        dy = target_pose.position.y - self.robot_pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def update(self):
        """State machine update — runs at 10 Hz."""

        if self.state == RobotState.IDLE:
            if self.object_pose is not None:
                self.state = RobotState.WALKING
                self.get_logger().info('Target acquired. Walking to object...')

        elif self.state == RobotState.WALKING:
            dist = self.distance_to(self.object_pose)
            if dist < 0.5:  # Close enough to reach
                self.stop_walking()
                self.state = RobotState.REACHING
                self.get_logger().info('In range. Reaching for object...')
            else:
                self.walk_toward(self.object_pose)

        elif self.state == RobotState.REACHING:
            # Send arm to reaching pose
            self.extend_arm()
            self.state = RobotState.GRASPING
            self.get_logger().info('Arm extended. Grasping...')

        elif self.state == RobotState.GRASPING:
            gripper_cmd = String()
            gripper_cmd.data = 'close'
            self.gripper_pub.publish(gripper_cmd)
            self.state = RobotState.RETURNING
            self.get_logger().info('Object grasped. Returning home...')

        elif self.state == RobotState.RETURNING:
            dist = self.distance_to(self.home_pose)
            if dist < 0.3:
                self.stop_walking()
                self.state = RobotState.IDLE
                self.get_logger().info('Task complete! Back at home position.')
            else:
                self.walk_toward(self.home_pose)

    def walk_toward(self, target_pose):
        """Generate velocity commands to walk toward a target."""
        if self.robot_pose is None or target_pose is None:
            return

        dx = target_pose.position.x - self.robot_pose.position.x
        dy = target_pose.position.y - self.robot_pose.position.y
        angle_to_target = math.atan2(dy, dx)

        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.5 * angle_to_target  # Proportional turning
        self.cmd_pub.publish(cmd)

    def stop_walking(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def extend_arm(self):
        js = JointState()
        js.name = ['shoulder_pitch', 'shoulder_roll', 'elbow', 'wrist']
        js.position = [0.5, 0.0, -1.0, 0.0]  # Reaching pose
        self.arm_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StatefulAgent())
    rclpy.shutdown()
```

## Quality of Service (QoS) in Practice

QoS profiles determine how messages are delivered. Choosing the right QoS is critical for robot reliability:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# For critical joint commands — never miss a message
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# For high-frequency sensor data — latest value matters most
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only keep the latest reading
)

# For configuration that late-joiners should receive
config_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


# Usage
self.create_publisher(JointState, '/joint_commands', reliable_qos)
self.create_subscription(Image, '/camera/rgb', self.on_image, sensor_qos)
```

## Launch Files: Starting Multi-Node Systems

Real systems have many nodes. Launch files start them all at once:

```python
# launch/humanoid_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
        ),
        Node(
            package='humanoid_control',
            executable='balance_monitor',
            name='balance_monitor',
            output='screen',
        ),
        Node(
            package='humanoid_control',
            executable='reactive_walker',
            name='reactive_walker',
            output='screen',
            parameters=[{
                'max_speed': 0.5,
                'obstacle_threshold': 1.0,
            }],
        ),
    ])
```

Run with:

```bash
ros2 launch humanoid_control humanoid_system.launch.py
```

## Parameters: Runtime Configuration

Parameters let you configure nodes without recompiling:

```python
class ConfigurableWalker(Node):
    def __init__(self):
        super().__init__('configurable_walker')

        # Declare parameters with defaults
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('obstacle_threshold', 1.0)
        self.declare_parameter('balance_enabled', True)

    def control_loop(self):
        # Read parameters (can be changed at runtime)
        max_speed = self.get_parameter('max_speed').value
        threshold = self.get_parameter('obstacle_threshold').value
        balance_on = self.get_parameter('balance_enabled').value

        # Use parameters in control logic
        if balance_on and abs(self.tilt) > 0.15:
            speed = 0.0
        else:
            speed = min(max_speed, self.desired_speed)
```

Change parameters at runtime from the command line:

```bash
ros2 param set /configurable_walker max_speed 0.5
ros2 param set /configurable_walker balance_enabled false
ros2 param get /configurable_walker max_speed
```

---

## Exercises

### Exercise 1: Perception Pipeline
Create a Python node that subscribes to `/humanoid/camera/depth` (`sensor_msgs/Image`), converts the depth image to a NumPy array, finds the closest object in the image, and publishes its estimated distance and bearing on a custom topic.

### Exercise 2: Behavior Tree Agent
Extend the StatefulAgent to handle failure cases: if the robot drops the object during RETURNING, transition back to WALKING. If the object disappears from detection during WALKING, transition to a SEARCHING state that rotates in place.

### Exercise 3: Multi-Robot Coordinator
Create two walker agents with different namespaces (`/robot1/` and `/robot2/`). Write a coordinator node that assigns them different target positions and monitors their progress via action feedback.

---

:::tip Key Takeaway
**rclpy** is the bridge between Python's AI ecosystem and ROS 2's robotics infrastructure. By mastering timer-based control loops, multi-subscription data fusion, and stateful agent patterns, you can build intelligent robot behaviors entirely in Python — leveraging the same language you use for machine learning and AI development.
:::
