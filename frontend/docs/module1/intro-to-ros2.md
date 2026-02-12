---
sidebar_position: 1
---

# Introduction to ROS 2

**The Robot Operating System 2 — The Nervous System of Modern Robots**

---

## What is ROS 2?

The Robot Operating System 2 (ROS 2) is not actually an operating system in the traditional sense. It is a **middleware framework** — a set of software libraries, tools, and conventions that provide the communication infrastructure every robot needs. Think of ROS 2 as the nervous system of a robot: it carries signals between sensors, processors, and actuators, enabling all the parts to work together as a coordinated whole.

Just as your nervous system allows your eyes to send visual information to your brain, which then sends movement commands to your muscles, ROS 2 allows a robot's cameras to send image data to AI models, which then send control commands to motors. Without this communication backbone, even the most sophisticated AI would be trapped in isolation — able to think but unable to act.

## Why ROS 2? (And Not ROS 1?)

The original ROS (now called ROS 1) was created at Stanford and Willow Garage in 2007. It revolutionized robotics research by providing a standard way for robot components to communicate. However, ROS 1 was designed for single-robot research in controlled lab environments. It had critical limitations:

| Limitation | ROS 1 | ROS 2 |
|-----------|-------|-------|
| **Real-time support** | No guaranteed timing | Built on DDS with real-time capabilities |
| **Multi-robot** | Single roscore bottleneck | Decentralized discovery, no single point of failure |
| **Security** | No authentication or encryption | Built-in DDS security (authentication, encryption, access control) |
| **Platform** | Linux only | Linux, Windows, macOS, RTOS |
| **Reliability** | Best-effort only | Configurable QoS (reliable, best-effort, transient local) |
| **Lifecycle** | No standard node lifecycle | Managed nodes with startup/shutdown states |

ROS 2 was built from the ground up to address these gaps. It uses the **Data Distribution Service (DDS)** standard as its communication layer — the same protocol used in military systems, air traffic control, and financial trading platforms where reliability is non-negotiable.

For humanoid robotics specifically, ROS 2's advantages are critical:
- **Real-time control**: A humanoid robot walking requires millisecond-precision joint commands. Missed deadlines cause falls.
- **Multi-sensor fusion**: Humanoids have cameras, IMUs, force/torque sensors, and joint encoders — all streaming data simultaneously.
- **Safety**: A robot operating near humans must have secure, reliable communication between its perception and control systems.

## ROS 2 Architecture Overview

ROS 2 follows a **distributed computing** model. There is no central server. Instead, independent programs called **nodes** discover each other automatically and exchange data through well-defined communication patterns.

### The Core Concepts

```
┌─────────────────────────────────────────────────┐
│                  ROS 2 Graph                     │
│                                                  │
│  ┌──────────┐    Topic     ┌──────────────┐      │
│  │  Camera   │──────────►  │  Perception  │      │
│  │  Node     │  /image     │  Node        │      │
│  └──────────┘              └──────┬───────┘      │
│                                   │              │
│                              Topic│/objects      │
│                                   │              │
│  ┌──────────┐    Service   ┌──────▼───────┐      │
│  │  Planner  │◄──────────► │  Controller  │      │
│  │  Node     │  /plan_path │  Node        │      │
│  └──────────┘              └──────┬───────┘      │
│                                   │              │
│                             Action│/walk         │
│                                   │              │
│                            ┌──────▼───────┐      │
│                            │  Motor Driver │      │
│                            │  Node         │      │
│                            └──────────────┘      │
└─────────────────────────────────────────────────┘
```

**1. Nodes** — Individual processes that perform specific tasks (camera driver, object detector, path planner, motor controller). Each node is a self-contained unit that can be developed, tested, and deployed independently.

**2. Topics** — Named channels for streaming data. A node **publishes** messages to a topic, and any number of nodes can **subscribe** to receive those messages. This is the **publish-subscribe** pattern — ideal for continuous data like sensor readings.

**3. Services** — Request-response communication. A node sends a request and waits for a response. This is like a function call across the network — used for one-time queries like "calculate a path from A to B."

**4. Actions** — Long-running tasks with feedback. A node sends a goal ("walk to the kitchen"), receives periodic feedback ("50% complete, currently in hallway"), and eventually gets a result ("arrived at kitchen"). Actions can be cancelled mid-execution.

**5. Parameters** — Configuration values that nodes can read and modify at runtime. For example, a camera node might have parameters for resolution, frame rate, and exposure.

### Quality of Service (QoS)

One of ROS 2's most powerful features is **Quality of Service** profiles. You can configure each topic's communication reliability:

- **Reliable**: Every message is guaranteed to arrive (like TCP). Used for critical commands.
- **Best Effort**: Messages may be dropped if the network is congested (like UDP). Used for high-frequency sensor data where the latest reading matters more than receiving every single one.
- **Transient Local**: Late-joining subscribers receive the last published message. Used for configuration or state information.

This is crucial for humanoid robots where some data (joint commands) must never be lost, while other data (camera frames at 30fps) can tolerate occasional drops.

## The ROS 2 Ecosystem

ROS 2 is not just middleware — it comes with a rich ecosystem of tools and packages:

### Development Tools

| Tool | Purpose |
|------|---------|
| **colcon** | Build system for compiling ROS 2 packages |
| **ros2 cli** | Command-line tools for inspecting topics, services, nodes |
| **RViz2** | 3D visualization of robot models, sensor data, and paths |
| **rqt** | GUI-based debugging and introspection tools |
| **rosbag2** | Record and replay message streams for testing |

### Key Packages for Humanoid Robotics

| Package | Purpose |
|---------|---------|
| **rclpy** | Python client library for writing ROS 2 nodes |
| **rclcpp** | C++ client library for performance-critical nodes |
| **tf2** | Transform library for tracking coordinate frames (where is the left hand relative to the torso?) |
| **Nav2** | Navigation stack for autonomous movement and path planning |
| **MoveIt2** | Motion planning for robotic arms and manipulators |
| **ros2_control** | Hardware abstraction layer for joint controllers |
| **robot_state_publisher** | Broadcasts the robot's joint state as transform frames |

## Setting Up Your ROS 2 Environment

ROS 2 distributions follow a naming convention using alphabetical turtle names. For this textbook, we use **ROS 2 Humble Hawksbill** (LTS) or **ROS 2 Iron Irwini**.

### Prerequisites

- **Ubuntu 22.04 LTS** (recommended) or Windows 10/11 with WSL2
- **Python 3.10+**
- At least **8 GB RAM** and **20 GB disk space**

### Installation (Ubuntu 22.04)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2 in every terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

```bash
# Terminal 1: Run the demo talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run the demo listener
ros2 run demo_nodes_cpp listener
```

If you see the listener receiving messages from the talker, ROS 2 is working correctly. You have just witnessed your first publish-subscribe communication.

### Create Your First Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the empty workspace
colcon build

# Source the workspace
source install/setup.bash
```

This workspace is where all your robot packages will live. The `src/` directory holds your source code, and `colcon build` compiles everything into the `install/` directory.

## Your First ROS 2 Commands

The `ros2` CLI is your primary debugging tool. Here are the essential commands:

```bash
# List all running nodes
ros2 node list

# List all active topics
ros2 topic list

# See messages flowing on a topic
ros2 topic echo /topic_name

# Check the message type of a topic
ros2 topic info /topic_name

# Publish a test message to a topic
ros2 topic pub /test std_msgs/msg/String "data: 'Hello ROS 2'"

# List all available services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Empty

# View the node graph visually
rqt_graph
```

These commands will be your constant companions as you develop and debug robot systems throughout this course.

## What is Coming Next

In this module, you will progressively build your ROS 2 skills:

1. **Nodes, Topics, and Services** — Deep dive into each communication pattern with hands-on code examples. You will build a publisher-subscriber pair, create a custom service, and implement an action server.

2. **Python Agents with rclpy** — Bridge your Python AI knowledge to robot control. You will write intelligent nodes that process sensor data, make decisions, and send commands to actuators.

3. **URDF for Humanoids** — Describe a humanoid robot's physical structure in code. You will create a URDF model with joints, links, and sensors that simulators and controllers can understand.

By the end of this module, you will have a complete ROS 2 application that controls a humanoid robot's movements, processes sensor inputs, and coordinates multi-node systems — the foundation for everything that follows.

---

:::tip Key Takeaway
ROS 2 is the **communication backbone** of modern robotics. It is not an operating system but a middleware framework that lets all parts of a robot — sensors, AI models, controllers, and actuators — talk to each other reliably and efficiently. Master ROS 2, and you master the language that robots speak.
:::
