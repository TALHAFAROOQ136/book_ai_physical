---
sidebar_position: 4
---

# Understanding URDF for Humanoid Robots

**Describing a Robot's Body in Code**

---

## Overview

Before a simulator can render your robot, before a controller can move its joints, and before a planner can calculate its motions — the system needs a precise mathematical description of the robot's physical structure. This is what **URDF (Unified Robot Description Format)** provides.

URDF is an XML-based file format that describes a robot's **links** (rigid body parts), **joints** (connections between links), **sensors**, **visual appearance**, and **collision geometry**. It is the standard robot description format in the ROS ecosystem, and every tool — from Gazebo simulation to RViz visualization to MoveIt motion planning — reads URDF.

In this chapter, you will learn to describe a humanoid robot from the ground up: torso, limbs, joints, sensors, and all.

## The Building Blocks: Links and Joints

A URDF model is a **tree structure** of links connected by joints. Think of it like a skeleton:

```
                    ┌──────────┐
                    │   HEAD   │
                    └────┬─────┘
                         │ neck_joint
                    ┌────┴─────┐
          ┌─────────┤  TORSO   ├──────────┐
          │         └────┬─────┘          │
  left_shoulder    hip_joint      right_shoulder
          │              │                │
    ┌─────┴─────┐  ┌────┴─────┐   ┌─────┴─────┐
    │ LEFT ARM  │  │  PELVIS  │   │ RIGHT ARM │
    └─────┬─────┘  ├──────────┤   └─────┬─────┘
     left_elbow    │          │    right_elbow
    ┌─────┴─────┐  │          │   ┌─────┴─────┐
    │ LEFT FORE │  │          │   │RIGHT FORE │
    └─────┬─────┘  │          │   └─────┬─────┘
     left_wrist  left_hip  right_hip  right_wrist
    ┌─────┴─────┐  │          │   ┌─────┴─────┐
    │ LEFT HAND │  │          │   │RIGHT HAND │
    └───────────┘  │          │   └───────────┘
             ┌─────┴──┐  ┌───┴──────┐
             │LEFT LEG│  │RIGHT LEG │
             └────┬───┘  └───┬──────┘
           left_knee    right_knee
             ┌────┴───┐  ┌───┴──────┐
             │L LOWER │  │R LOWER   │
             └────┬───┘  └───┬──────┘
          left_ankle    right_ankle
             ┌────┴───┐  ┌───┴──────┐
             │L FOOT  │  │R FOOT    │
             └────────┘  └──────────┘
```

### Links

A **link** is a rigid body part. Each link has three properties:

1. **Visual** — How the link looks (mesh or geometric shape, color, material)
2. **Collision** — Simplified geometry used for physics collision detection
3. **Inertial** — Mass, center of mass, and inertia tensor for physics simulation

```xml
<link name="torso">
  <!-- What it looks like -->
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
    <material name="blue">
      <color rgba="0.2 0.4 0.8 1.0"/>
    </material>
  </visual>

  <!-- Simplified shape for collision detection -->
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
  </collision>

  <!-- Physics properties -->
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="10.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0"
             iyy="0.08" iyz="0.0"
             izz="0.05"/>
  </inertial>
</link>
```

#### Geometry Types

URDF supports several primitive shapes and custom meshes:

| Type | XML | Use Case |
|------|-----|----------|
| Box | `<box size="x y z"/>` | Torso, feet, simple links |
| Cylinder | `<cylinder radius="r" length="l"/>` | Arms, legs, joints |
| Sphere | `<sphere radius="r"/>` | Head, joint covers |
| Mesh | `<mesh filename="package://..."/>` | Detailed visual models (STL/DAE) |

### Joints

A **joint** connects two links — a parent and a child — and defines how they can move relative to each other.

```xml
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.12 0.35" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation axis: pitch (around Y) -->
  <limit lower="-2.0" upper="2.0"
         effort="50.0" velocity="3.14"/>
</joint>
```

#### Joint Types

| Type | Motion | Use Case |
|------|--------|----------|
| **revolute** | Rotation with limits | Elbows, knees, shoulder pitch/roll |
| **continuous** | Unlimited rotation | Wheels, wrists |
| **prismatic** | Linear sliding | Telescoping limbs, grippers |
| **fixed** | No motion | Sensor mounts, decorative parts |
| **floating** | 6-DOF free motion | Base link of a free-floating robot |
| **planar** | 2D translation + rotation | Mobile bases on a flat surface |

#### Joint Properties

- **`<origin>`** — Position and orientation of the joint relative to the parent link
- **`<axis>`** — The axis of rotation (for revolute) or translation (for prismatic)
- **`<limit>`** — Physical constraints:
  - `lower` / `upper` — Angular limits in radians
  - `effort` — Maximum force/torque in Nm
  - `velocity` — Maximum angular velocity in rad/s
- **`<dynamics>`** — Friction and damping coefficients

---

## Building a Humanoid URDF: Step by Step

Let us build a simplified humanoid robot URDF. We will start with the torso and progressively add limbs.

### Step 1: The Base and Torso

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link (root of the kinematic tree) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="body_color">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.2"/>
      <mass value="12.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.15" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.8" rpy="0 0 0"/>
  </joint>
```

### Step 2: The Head

```xml
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="head_color">
        <color rgba="0.9 0.8 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Neck joint: allows head to pan and tilt -->
  <joint name="neck_yaw" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.45" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>
```

### Step 3: Arms (Left Arm Example)

A humanoid arm typically has 6-7 degrees of freedom. Here is a simplified 4-DOF arm:

```xml
  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="body_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15"/>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.17 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="1.57" effort="50" velocity="3.14"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.13" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.26"/>
      </geometry>
      <material name="body_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.13" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.26"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.13"/>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="30" velocity="3.14"/>
    <dynamics damping="0.3" friction="0.1"/>
  </joint>
```

### Step 4: Legs (Left Leg Example)

Humanoid legs need at least 6 DOF each for stable walking: hip (3 DOF), knee (1 DOF), ankle (2 DOF).

```xml
  <!-- Left Upper Leg (Thigh) -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="body_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2"/>
      <mass value="4.0"/>
      <inertia ixx="0.06" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.0 0.1 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3.14"/>
    <dynamics damping="1.0" friction="0.2"/>
  </joint>

  <!-- Left Lower Leg (Shin) -->
  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="body_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2"/>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.006"/>
    </inertial>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="80" velocity="3.14"/>
    <dynamics damping="0.8" friction="0.15"/>
  </joint>

  <!-- Left Foot -->
  <link name="left_foot">
    <visual>
      <origin xyz="0.04 0 -0.015" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.03"/>
      </geometry>
      <material name="shoe_color">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.04 0 -0.015" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.04 0 -0.015"/>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_ankle_pitch" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.8" upper="0.8" effort="60" velocity="3.14"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

</robot>
```

---

## Adding Sensors to URDF

Humanoid robots need sensors. In URDF, sensors are added as links with fixed joints attached to the appropriate body parts, plus **Gazebo plugins** that simulate sensor behavior:

### Camera (Head-Mounted)

```xml
<!-- Camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.03 0.08 0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1.0"/>
    </material>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo camera plugin -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>image_raw:=/humanoid/camera/rgb</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU (Torso-Mounted)

```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </visual>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor type="imu" name="torso_imu">
    <update_rate>100.0</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=/humanoid/imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

---

## Xacro: URDF with Superpowers

Writing URDF by hand is verbose — especially for a humanoid where left and right limbs are mirror images. **Xacro** (XML Macro) extends URDF with variables, math, and macros:

### Variables and Math

```xml
<xacro:property name="upper_arm_length" value="0.3"/>
<xacro:property name="upper_arm_radius" value="0.04"/>
<xacro:property name="upper_arm_mass" value="2.0"/>

<!-- Use in geometry -->
<cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>

<!-- Math expressions -->
<origin xyz="0 0 ${-upper_arm_length / 2}"/>
```

### Macros for Repeated Structures

Instead of writing the arm twice, define it once and instantiate it for both sides:

```xml
<xacro:macro name="arm" params="side reflect">
  <link name="${side}_upper_arm">
    <visual>
      <origin xyz="0 0 ${-upper_arm_length/2}"/>
      <geometry>
        <cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="${upper_arm_mass}"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="${side}_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="${side}_upper_arm"/>
    <origin xyz="0.17 ${reflect * 0.12} 0.35"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="1.57" effort="50" velocity="3.14"/>
  </joint>

  <!-- Forearm, wrist, hand follow the same pattern... -->
</xacro:macro>

<!-- Instantiate for both sides -->
<xacro:arm side="left" reflect="1"/>
<xacro:arm side="right" reflect="-1"/>
```

### Processing Xacro Files

```bash
# Convert xacro to plain URDF
xacro humanoid.urdf.xacro > humanoid.urdf

# Or use it directly in launch files (preferred)
robot_description = Command(['xacro ', 'path/to/humanoid.urdf.xacro'])
```

---

## Visualizing Your URDF

### RViz2

The quickest way to see your robot model:

```bash
# Launch the robot state publisher
ros2 launch humanoid_description display.launch.py

# This typically includes:
# 1. robot_state_publisher — broadcasts transforms from URDF
# 2. joint_state_publisher_gui — sliders to move each joint
# 3. rviz2 — 3D visualization with the robot model loaded
```

With the joint state publisher GUI, you can drag sliders to move each joint and see the robot model update in real time. This is invaluable for verifying that joints rotate in the correct direction and that link dimensions are correct.

### Checking Your URDF

```bash
# Validate URDF syntax
check_urdf humanoid.urdf

# Visualize the link-joint tree
urdf_to_graphviz humanoid.urdf
# Creates a PDF showing the kinematic tree structure
```

---

## From URDF to Simulation

Your URDF is the single source of truth that feeds into multiple tools:

```
                    URDF / Xacro
                         │
         ┌───────────────┼───────────────┐
         ▼               ▼               ▼
     Gazebo          RViz2          MoveIt2
   (Physics Sim)  (Visualization)  (Motion Planning)
         │               │               │
         ▼               ▼               ▼
   Sensor Data     Debug Views     Trajectories
   Collision       TF Frames       Joint Paths
   Dynamics        Joint States    Collision Check
```

In Module 2, you will take this URDF and load it into Gazebo to simulate your humanoid walking, falling, and interacting with objects in a physics-enabled world. The accuracy of your URDF directly determines how realistic the simulation will be.

---

## Exercises

### Exercise 1: Complete the Humanoid
Using the arm and leg examples above, create a complete humanoid URDF with both arms (4 DOF each) and both legs (3 DOF each) using Xacro macros. Verify it in RViz2 using the joint state publisher GUI.

### Exercise 2: Add Sensors
Add a depth camera to the head, an IMU to the torso, and force/torque sensors to both feet. Define the appropriate Gazebo plugins for each sensor.

### Exercise 3: Inertia Calculator
The inertia tensor values are critical for accurate simulation. Write a Python script that takes link geometry (box, cylinder, or sphere), dimensions, and mass, then calculates the correct inertia tensor values using standard formulas. Use it to verify the values in your URDF.

---

:::tip Key Takeaway
URDF is the **blueprint** of your robot — every simulator, controller, and planner reads it to understand the robot's physical structure. Invest time in getting your URDF right: accurate dimensions, realistic masses, correct joint limits, and proper sensor placements. A good URDF is the foundation that makes everything else — simulation, control, and AI — work correctly.
:::
