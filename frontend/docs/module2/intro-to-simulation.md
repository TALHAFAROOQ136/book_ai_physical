---
sidebar_position: 1
---

# Introduction to Simulation

**The Digital Twin — Why Simulation-First is the Future of Robotics**

---

## Overview

Training a humanoid robot in the real world is expensive, slow, and dangerous. A single fall can destroy thousands of dollars of hardware in seconds. A miscalculated joint command can strip gears, crack actuators, or send the robot crashing into equipment — or people. Even when things go well, real-world training is limited by wall-clock time: you cannot make gravity faster.

This is why modern robotics teams spend **90% of their development time in simulation** and only 10% on real hardware. Simulation is not a shortcut — it is the most effective development methodology available.

In this module, you will learn to build **digital twins** — virtual replicas of your robots and their environments where you can develop, test, and refine behaviors without any risk. You will work with two industry-standard simulators: **Gazebo** for physics-accurate robotics simulation and **Unity** for high-fidelity rendering and complex human-robot interaction scenarios.

## What is a Digital Twin?

A digital twin is a virtual model that accurately mirrors a physical system. For robotics, this means:

- The **robot model** (from your URDF) is loaded with accurate mass, inertia, joint limits, and sensor specifications
- The **environment** includes realistic physics — gravity, friction, collisions, contact forces
- The **sensors** produce synthetic data that closely matches real hardware — camera images, LiDAR point clouds, IMU readings
- The **actuators** respond to commands with realistic dynamics — motor delays, torque limits, backlash

When the digital twin is accurate enough, code developed in simulation transfers directly to the real robot with minimal adjustment. This concept — **sim-to-real transfer** — is the foundation of modern robotics development.

```
┌──────────────────────────────────────────────────┐
│              THE DIGITAL TWIN                     │
│                                                    │
│   Real World              Simulation               │
│   ┌──────────┐           ┌──────────┐             │
│   │  Physical │  mirror   │  Virtual │             │
│   │  Robot    │◄────────►│  Robot   │             │
│   └──────────┘           └──────────┘             │
│                                                    │
│   ┌──────────┐           ┌──────────┐             │
│   │  Real     │  mirror   │  Simulated│            │
│   │  Sensors  │◄────────►│  Sensors │             │
│   └──────────┘           └──────────┘             │
│                                                    │
│   ┌──────────┐           ┌──────────┐             │
│   │  Physical │  mirror   │  Virtual │             │
│   │  World    │◄────────►│  World   │             │
│   └──────────┘           └──────────┘             │
│                                                    │
│        SAME CODE runs on both sides               │
└──────────────────────────────────────────────────┘
```

## Why Simulate?

### 1. Safety

A humanoid robot weighs 20-80 kg and has joints that can exert hundreds of Newton-meters of torque. Untested code can cause:
- The robot to fall and damage itself or surroundings
- Joints to exceed safe limits, destroying actuators
- Unexpected interactions with humans in the workspace

In simulation, your robot can fall a thousand times with zero cost.

### 2. Speed

Real-world training is limited by physics — you cannot speed up time. In simulation:
- **Gazebo** can run at 2-10x real time for physics simulation
- **NVIDIA Isaac Sim** can run thousands of parallel environments on GPUs
- A training run that would take months in the real world can complete in hours

### 3. Scale

You need diverse training data. In the real world, rearranging furniture, changing lighting, and varying object textures takes hours of manual labor. In simulation, you can procedurally generate thousands of unique environments automatically — a technique called **domain randomization**.

### 4. Repeatability

Debugging a real-world failure is difficult because you cannot rewind time. In simulation, you can:
- Record and replay exact scenarios
- Add breakpoints and step through physics frame by frame
- Inject specific faults to test recovery behaviors
- Run regression tests on every code change

### 5. Impossible Scenarios

Some scenarios are too dangerous or impractical to test in reality:
- What happens when the robot's knee actuator fails mid-stride?
- How does the system respond to an earthquake?
- Can the robot navigate in complete darkness using only LiDAR?

Simulation lets you test edge cases that matter but cannot be safely reproduced.

## Gazebo vs Unity: When to Use Each

This module covers two simulators, each with different strengths:

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Primary Strength** | Physics accuracy | Visual fidelity |
| **ROS 2 Integration** | Native, first-class | Via ROS-TCP-Connector plugin |
| **Physics Engine** | ODE, Bullet, DART, Simbody | PhysX (NVIDIA) |
| **Rendering** | Basic OpenGL | Photorealistic (HDRP, ray tracing) |
| **Sensor Simulation** | Excellent (LiDAR, camera, IMU, F/T) | Good (requires custom scripts) |
| **Performance** | CPU-based, single environment | GPU-accelerated, parallel |
| **Cost** | Free, open-source | Free personal license |
| **Best For** | Control development, sensor testing | Visual AI training, HRI research |
| **Learning Curve** | Moderate (SDF/URDF + plugins) | Steep (C# scripting + Unity editor) |

### The Practical Approach

Most robotics teams use **both**:

1. **Gazebo** for daily development — testing control algorithms, validating sensor pipelines, debugging ROS 2 node interactions. Its tight ROS 2 integration means your nodes cannot tell the difference between simulated and real sensors.

2. **Unity** for perception training — generating photorealistic synthetic datasets for computer vision, testing human-robot interaction in visually realistic environments, and creating compelling demos.

## The Sim-to-Real Gap

The difference between simulated and real-world performance is called the **sim-to-real gap**. Key sources of this gap include:

| Source | Problem | Mitigation |
|--------|---------|------------|
| **Physics accuracy** | Simulated contacts and friction differ from reality | Tune physics parameters, use system identification |
| **Sensor noise** | Real sensors have noise, latency, and artifacts | Add realistic noise models in simulation |
| **Visual differences** | Simulated images look different from real cameras | Domain randomization, photorealistic rendering |
| **Actuator dynamics** | Real motors have delays, backlash, and nonlinearities | Model motor dynamics, add response delays |
| **Environmental variety** | Training environments lack real-world diversity | Domain randomization (textures, lighting, objects) |

Throughout this module, you will learn techniques to minimize the sim-to-real gap, making your simulated results transfer reliably to physical hardware.

## The Simulation-First Workflow

Here is the workflow you will follow throughout this module and the rest of the course:

```
1. DESIGN        →  Create URDF model (Module 1) ✓
2. SIMULATE      →  Load into Gazebo/Unity (This Module)
3. DEVELOP       →  Write control/perception code
4. TEST          →  Run scenarios, collect metrics
5. ITERATE       →  Fix bugs, tune parameters
6. VALIDATE      →  Run comprehensive test suite
7. TRANSFER      →  Deploy to real hardware
8. FINE-TUNE     →  Adjust for real-world differences
```

Steps 2-6 happen entirely in simulation and can iterate in minutes. Step 7 happens only when you have high confidence from simulation testing.

## Setting Up Your Simulation Environment

### Gazebo Installation

Gazebo (formerly Ignition Gazebo) is installed alongside ROS 2:

```bash
# If not already installed with ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
gz sim --version
```

### Unity Installation (for later chapters)

1. Download **Unity Hub** from [unity.com](https://unity.com)
2. Install **Unity 2022.3 LTS** (or newer)
3. Install the **ROS-TCP-Connector** package via Unity Package Manager
4. Install the **URDF Importer** package for loading robot models

### Workspace Setup

```bash
# Create a simulation workspace
cd ~/ros2_ws/src

# Create a package for simulation launch files and worlds
ros2 pkg create --build-type ament_python humanoid_simulation \
  --dependencies rclpy gazebo_ros launch launch_ros

# Create directories for world files and models
mkdir -p humanoid_simulation/worlds
mkdir -p humanoid_simulation/models
mkdir -p humanoid_simulation/launch
```

## What is Coming Next

In this module, you will progressively build your simulation skills:

1. **Gazebo Physics** — Load your humanoid URDF into Gazebo, simulate gravity and collisions, build custom worlds with obstacles, and connect ROS 2 controllers to simulated joints.

2. **Unity for Human-Robot Interaction** — Import your robot into Unity's photorealistic environment, simulate human-robot scenarios, and generate synthetic training data for perception models.

3. **Simulating Sensors** — Add and configure LiDAR, depth cameras, and IMUs in simulation. Produce synthetic sensor data that matches real hardware, enabling you to develop perception pipelines entirely in simulation.

By the end of this module, you will have a fully functional digital twin of your humanoid robot — a virtual testing ground where you can develop, test, and validate every algorithm before it ever touches real hardware.

---

:::tip Key Takeaway
Simulation is not a substitute for the real world — it is a **force multiplier**. By developing in simulation first, you arrive at real hardware testing with code that already works, behaviors that are already tuned, and edge cases that are already handled. The best robotics teams simulate first, simulate often, and simulate everything.
:::
