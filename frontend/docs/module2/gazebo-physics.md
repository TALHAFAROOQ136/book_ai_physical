---
sidebar_position: 2
---

# Simulating Physics in Gazebo

**Gravity, Collisions, and the Laws That Govern Your Robot**

---

## Overview

Gazebo is the most widely used open-source robotics simulator in the ROS ecosystem. It provides a physics-enabled 3D environment where your robot model experiences realistic gravity, friction, collisions, and contact forces. In this chapter, you will load your humanoid URDF into Gazebo, build custom worlds, and connect ROS 2 controllers to simulated joints — turning your static robot description into a moving, interactive digital twin.

## Loading Your Humanoid into Gazebo

### The Launch File

The standard way to spawn a robot in Gazebo is through a ROS 2 launch file that starts Gazebo, loads the URDF, and spawns the robot model:

```python
# launch/humanoid_gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('humanoid_simulation')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Robot description from URDF/Xacro
    robot_description = Command([
        'xacro ',
        os.path.join(pkg_dir, 'urdf', 'humanoid.urdf.xacro')
    ])

    return LaunchDescription([
        # Launch Gazebo with an empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(pkg_dir, 'worlds', 'humanoid_lab.world'),
                'verbose': 'true',
            }.items(),
        ),

        # Publish robot description to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'humanoid',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '1.0',  # Spawn 1m above ground (robot will fall to standing)
            ],
            output='screen',
        ),
    ])
```

### Running the Simulation

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_simulation
source install/setup.bash

# Launch Gazebo with the humanoid
ros2 launch humanoid_simulation humanoid_gazebo.launch.py
```

You should see Gazebo open with your humanoid robot model standing (or falling, if the physics are not yet tuned) in the simulated world.

## Understanding Gazebo Physics

### The Physics Engine

Gazebo supports multiple physics engines. Each has different strengths:

| Engine | Strengths | Best For |
|--------|-----------|----------|
| **ODE** (default) | Stable, well-tested | General-purpose robotics |
| **Bullet** | Good soft-body and collision | Object manipulation |
| **DART** | Accurate rigid body dynamics | Humanoid locomotion |
| **Simbody** | Biomechanics modeling | Human-robot interaction |

For humanoid robots, **DART** (Dynamic Animation and Robotics Toolkit) is often preferred because it handles articulated rigid body dynamics — exactly what humanoid walking requires.

### Configuring Physics in SDF World Files

```xml
<!-- worlds/humanoid_lab.world -->
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="humanoid_lab">

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>          <!-- More iterations = more accuracy -->
          <sor>1.3</sor>              <!-- Successive over-relaxation -->
        </solver>
        <constraints>
          <cfm>0.0</cfm>             <!-- Constraint force mixing -->
          <erp>0.2</erp>             <!-- Error reduction parameter -->
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

### Key Physics Parameters

| Parameter | What It Controls | Humanoid Impact |
|-----------|-----------------|-----------------|
| `max_step_size` | Physics time step | Smaller = more accurate but slower. 0.001s is standard for humanoids |
| `real_time_factor` | Speed vs accuracy | 1.0 = real time. Set > 1.0 for faster training |
| `iters` | Solver iterations per step | More iterations = more stable contacts. 50+ for humanoid feet |
| `cfm` | Constraint softness | 0 = rigid constraints. Small values prevent jitter |
| `erp` | Error correction speed | 0.2 is standard. Higher = faster correction but possible oscillation |
| `friction` | Surface friction | Critical for foot-ground contact during walking |

## Gravity and Falling

The first test of any humanoid simulation: does the robot stand or fall?

### Why Robots Fall in Simulation

When you first spawn a humanoid in Gazebo, it will likely collapse. This happens because:

1. **No joint controllers**: Without active control, joints are free to move under gravity
2. **Poor inertia values**: Incorrect mass/inertia in the URDF causes unrealistic dynamics
3. **Missing friction**: Without proper foot-ground friction, the robot slides

### Adding Joint Controllers

To keep the robot standing, you need **joint position controllers** that hold joints at their desired positions:

```yaml
# config/humanoid_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # 1kHz control loop

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: joint_trajectory_controller/JointTrajectoryController

position_controller:
  ros__parameters:
    joints:
      - left_hip_pitch
      - left_knee
      - left_ankle_pitch
      - right_hip_pitch
      - right_knee
      - right_ankle_pitch
      - left_shoulder_pitch
      - left_elbow
      - right_shoulder_pitch
      - right_elbow
      - neck_yaw

    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### The ros2_control Integration

Add the `ros2_control` hardware interface to your URDF for Gazebo:

```xml
<!-- Add inside your URDF <robot> tag -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <joint name="left_hip_pitch">
    <command_interface name="position">
      <param name="min">-1.57</param>
      <param name="max">1.57</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <!-- Repeat for all joints... -->
</ros2_control>

<!-- Gazebo ros2_control plugin -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find humanoid_simulation)/config/humanoid_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

## Collisions and Contact Physics

### Understanding Collision Geometry

Every link in your URDF has two geometries:
- **Visual**: What you see (can be complex meshes)
- **Collision**: What the physics engine uses (should be simplified)

```xml
<!-- Good: Simplified collision geometry -->
<collision>
  <geometry>
    <box size="0.2 0.1 0.03"/>  <!-- Simple box for foot -->
  </geometry>
</collision>

<!-- Visual can be detailed -->
<visual>
  <geometry>
    <mesh filename="package://humanoid_description/meshes/foot.stl"/>
  </geometry>
</visual>
```

Using simplified collision geometry improves simulation performance without affecting visual quality.

### Surface Properties

Foot-ground contact is critical for humanoid walking. Configure surface properties in Gazebo:

```xml
<!-- In your URDF, add Gazebo-specific properties to the foot link -->
<gazebo reference="left_foot">
  <mu1>1.0</mu1>           <!-- Friction coefficient (direction 1) -->
  <mu2>1.0</mu2>           <!-- Friction coefficient (direction 2) -->
  <kp>1000000.0</kp>       <!-- Contact stiffness -->
  <kd>100.0</kd>           <!-- Contact damping -->
  <minDepth>0.001</minDepth>
  <maxVel>0.1</maxVel>
  <material>Gazebo/DarkGrey</material>
</gazebo>
```

| Parameter | Effect | Typical Value |
|-----------|--------|---------------|
| `mu1`, `mu2` | Friction coefficients | 0.8-1.2 for rubber on floor |
| `kp` | Contact stiffness | 10^5 to 10^7 |
| `kd` | Contact damping | 10 to 1000 |
| `minDepth` | Minimum penetration before contact | 0.001 m |
| `maxVel` | Maximum contact correction velocity | 0.1 m/s |

### Collision Detection and Response

When two objects collide in Gazebo, the physics engine:

1. **Detects overlap** between collision geometries
2. **Calculates contact points** and normal forces
3. **Applies friction** based on surface properties
4. **Resolves penetration** by pushing objects apart
5. **Publishes contact data** on ROS 2 topics (if configured)

You can subscribe to contact data in your ROS 2 nodes:

```python
from gazebo_msgs.msg import ContactsState

class FootContactMonitor(Node):
    def __init__(self):
        super().__init__('foot_contact_monitor')
        self.create_subscription(
            ContactsState,
            '/left_foot/contact',
            self.on_contact,
            10
        )

    def on_contact(self, msg: ContactsState):
        for state in msg.states:
            total_force = state.total_wrench.force
            self.get_logger().info(
                f'Contact force: ({total_force.x:.1f}, '
                f'{total_force.y:.1f}, {total_force.z:.1f}) N'
            )
```

## Building Custom Worlds

### A Robotics Lab Environment

```xml
<!-- worlds/humanoid_lab.world -->
<sdf version="1.7">
  <world name="humanoid_lab">
    <!-- Physics and lighting (from above) -->

    <!-- Lab floor (textured) -->
    <model name="lab_floor">
      <static>true</static>
      <link name="floor">
        <collision name="floor_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Table with objects -->
    <model name="table">
      <static>true</static>
      <pose>2.0 0 0 0 0 0</pose>
      <link name="table_link">
        <collision name="table_top">
          <pose>0 0 0.4 0 0 0</pose>
          <geometry>
            <box><size>1.2 0.6 0.04</size></box>
          </geometry>
        </collision>
        <visual name="table_visual">
          <pose>0 0 0.4 0 0 0</pose>
          <geometry>
            <box><size>1.2 0.6 0.04</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
          </material>
        </visual>
        <!-- Table legs -->
        <collision name="leg1">
          <pose>0.55 0.25 0.2 0 0 0</pose>
          <geometry><cylinder><radius>0.03</radius><length>0.4</length></cylinder></geometry>
        </collision>
        <visual name="leg1_v">
          <pose>0.55 0.25 0.2 0 0 0</pose>
          <geometry><cylinder><radius>0.03</radius><length>0.4</length></cylinder></geometry>
        </visual>
        <!-- Repeat for other 3 legs... -->
      </link>
    </model>

    <!-- Graspable object on table -->
    <model name="red_cup">
      <pose>2.0 0 0.45 0 0 0</pose>
      <link name="cup_link">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.00005</izz>
          </inertia>
        </inertial>
        <collision name="cup_collision">
          <geometry>
            <cylinder><radius>0.04</radius><length>0.12</length></cylinder>
          </geometry>
        </collision>
        <visual name="cup_visual">
          <geometry>
            <cylinder><radius>0.04</radius><length>0.12</length></cylinder>
          </geometry>
          <material>
            <ambient>0.9 0.1 0.1 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle walls for navigation -->
    <model name="wall1">
      <static>true</static>
      <pose>4.0 2.0 0.5 0 0 0</pose>
      <link name="wall_link">
        <collision name="wall_col">
          <geometry><box><size>0.2 3.0 1.0</size></box></geometry>
        </collision>
        <visual name="wall_vis">
          <geometry><box><size>0.2 3.0 1.0</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Dynamic Objects

Unlike static models, dynamic objects respond to physics — they can be pushed, knocked over, and grasped:

```xml
<!-- A ball that the robot can kick -->
<model name="soccer_ball">
  <pose>1.5 0.5 0.11 0 0 0</pose>
  <link name="ball_link">
    <inertial>
      <mass>0.45</mass>  <!-- Standard soccer ball mass -->
      <inertia>
        <ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>
      </inertia>
    </inertial>
    <collision name="ball_col">
      <geometry><sphere><radius>0.11</radius></sphere></geometry>
      <surface>
        <bounce>
          <restitution_coefficient>0.6</restitution_coefficient>
        </bounce>
        <friction>
          <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
        </friction>
      </surface>
    </collision>
    <visual name="ball_vis">
      <geometry><sphere><radius>0.11</radius></sphere></geometry>
      <material><ambient>1 1 1 1</ambient></material>
    </visual>
  </link>
</model>
```

The `restitution_coefficient` controls bounciness (0 = no bounce, 1 = perfectly elastic).

## Controlling Your Robot in Gazebo

### Sending Joint Commands via ROS 2

With `ros2_control` configured, you can command joints through the standard ROS 2 interface:

```python
# humanoid_simulation/stand_controller.py
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class StandController(Node):
    """Commands the humanoid to a standing pose."""

    def __init__(self):
        super().__init__('stand_controller')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/position_controller/joint_trajectory',
            10
        )

        # Send standing pose after 2 seconds (wait for controllers to load)
        self.create_timer(2.0, self.send_standing_pose)
        self.pose_sent = False

    def send_standing_pose(self):
        if self.pose_sent:
            return

        msg = JointTrajectory()
        msg.joint_names = [
            'left_hip_pitch', 'left_knee', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee', 'right_ankle_pitch',
            'left_shoulder_pitch', 'left_elbow',
            'right_shoulder_pitch', 'right_elbow',
            'neck_yaw',
        ]

        point = JointTrajectoryPoint()
        # Standing pose: all joints at 0 (straight)
        point.positions = [0.0] * 11
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points = [point]
        self.pub.publish(msg)
        self.pose_sent = True
        self.get_logger().info('Standing pose command sent!')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StandController())
    rclpy.shutdown()
```

### Reading Joint States

Monitor the actual joint positions and velocities during simulation:

```python
from sensor_msgs.msg import JointState

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.create_subscription(
            JointState,
            '/joint_states',
            self.on_joint_state,
            10
        )

    def on_joint_state(self, msg: JointState):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if abs(vel) > 0.1:  # Only log moving joints
                self.get_logger().info(
                    f'{name}: pos={pos:.3f} rad, vel={vel:.3f} rad/s'
                )
```

## Simulation Performance Tips

| Tip | Impact |
|-----|--------|
| Use simplified collision geometry | 5-10x faster collision detection |
| Reduce `real_time_update_rate` for training | Faster-than-real-time simulation |
| Disable shadows and complex lighting | 2-3x faster rendering |
| Use headless mode (`--headless`) | No rendering overhead for batch testing |
| Reduce sensor update rates during training | Less computation per time step |
| Limit solver iterations for prototyping | Faster but less accurate physics |

```bash
# Run Gazebo in headless mode (no GUI) for automated testing
gz sim --headless -r humanoid_lab.world
```

---

## Exercises

### Exercise 1: Humanoid Standing Test
Load your humanoid URDF into Gazebo with joint controllers. Tune the standing pose so the robot remains upright for at least 30 seconds. Monitor the balance using the IMU topic.

### Exercise 2: Obstacle Course World
Create a Gazebo world file with a flat floor, three walls forming a corridor, a ramp with a 10-degree incline, and a doorway the robot must fit through. Verify collision detection works by manually teleporting the robot into walls.

### Exercise 3: Object Interaction
Add 5 dynamic objects to your world (different shapes and masses). Write a ROS 2 node that commands the robot's arm to push an object off a table and verify the physics simulation (object should fall and bounce realistically).

---

:::tip Key Takeaway
Gazebo transforms your static URDF into a living, physics-enabled digital twin. The key to realistic simulation is **tuning**: accurate mass/inertia values, appropriate friction coefficients, and correctly configured joint controllers. A well-tuned Gazebo simulation produces results that transfer directly to real hardware — making it the most valuable tool in your robotics development workflow.
:::
