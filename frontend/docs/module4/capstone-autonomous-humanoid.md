---
sidebar_position: 4
---

# Capstone Project: The Autonomous Humanoid

**Integrating Voice, Vision, Planning, and Navigation into One System**

---

## Overview

This is the final project of the course. You will integrate every concept from all four modules into a single, end-to-end autonomous humanoid robot system. The robot will receive a voice command from a human, use an LLM to plan the task, localize itself with Visual SLAM, navigate through an environment with obstacles using Nav2, detect a target object with computer vision, and manipulate it — all in simulation.

This capstone is not about building new components. It is about **integration** — making all the pieces you have built work together as a cohesive system.

## The Mission

```
┌─────────────────────────────────────────────────────────────┐
│                    CAPSTONE MISSION                          │
│                                                               │
│  "Hey robot, go to the kitchen and bring me the red cup"     │
│                                                               │
│  The robot must:                                              │
│  1. Hear and understand the voice command     (Module 4)     │
│  2. Plan the task using an LLM                (Module 4)     │
│  3. Localize itself in the environment        (Module 3)     │
│  4. Navigate to the kitchen, avoiding         (Module 3)     │
│     obstacles along the way                                   │
│  5. Detect the red cup using computer vision  (Module 3)     │
│  6. Grasp the cup                             (Module 2+3)   │
│  7. Navigate back to the user                 (Module 3)     │
│  8. Hand over the cup                         (Module 2)     │
│  9. Confirm task completion verbally          (Module 4)     │
│                                                               │
│  All running on the ROS 2 framework           (Module 1)     │
└─────────────────────────────────────────────────────────────┘
```

## System Architecture

### The Complete Node Graph

```
                        ┌──────────────┐
                        │  Microphone  │
                        └──────┬───────┘
                               │ audio
                               ▼
                        ┌──────────────┐
                        │  Wake Word   │
                        │  Detector    │
                        └──────┬───────┘
                               │ trigger
                               ▼
                        ┌──────────────┐
                        │  Whisper     │──────────────────────┐
                        │  (STT)       │                      │
                        └──────┬───────┘                      │
                               │ /voice/transcript            │
                               ▼                              │
                        ┌──────────────┐                      │
                        │  Intent      │                      │
                        │  Parser      │                      │
                        └──────┬───────┘                      │
                               │ /voice/parsed_intent         │
                               ▼                              │
  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐ │
  │  Scene        │────►│  LLM Task    │     │  TTS Node    │◄┘
  │  Builder      │     │  Planner     │     │  (Speech)    │
  └──────┬────────┘     └──────┬───────┘     └──────────────┘
         │                     │ /action_plan
         │                     ▼
         │              ┌──────────────┐
         │              │  Plan        │
         │              │  Executor    │
         │              └──────┬───────┘
         │                     │
         │          ┌──────────┼──────────┐
         │          │          │          │
         │          ▼          ▼          ▼
  ┌──────────┐ ┌────────┐ ┌────────┐ ┌────────┐
  │  Object  │ │ Nav2   │ │ Grasp  │ │ Gait   │
  │Detection │ │ Stack  │ │Pipeline│ │ Bridge │
  └──────────┘ └────────┘ └────────┘ └────────┘
         │          │          │          │
         └──────────┴──────────┴──────────┘
                        │
                 ┌──────┴───────┐
                 │  Gazebo /    │
                 │  Isaac Sim   │
                 └──────────────┘
```

### ROS 2 Topic Map

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/humanoid/voice/transcript` | `String` | Whisper Node | Intent Parser |
| `/humanoid/voice/parsed_intent` | `String` | Intent Parser | Task Planner |
| `/humanoid/voice/say` | `String` | Plan Executor | TTS Node |
| `/humanoid/scene_description` | `String` | Scene Builder | Task Planner |
| `/humanoid/action_plan` | `String` | Task Planner | Plan Executor |
| `/humanoid/execution/status` | `String` | Plan Executor | Scene Builder |
| `/humanoid/camera/image_raw` | `Image` | Gazebo/Isaac | Image Pipeline |
| `/humanoid/detectnet/detections` | `Detection2DArray` | Isaac ROS | Scene Builder, Grounding |
| `/visual_slam/tracking/odometry` | `Odometry` | cuVSLAM | Nav2, Scene Builder |
| `/humanoid/scan` | `LaserScan` | Gazebo/Isaac | Nav2 Costmap |
| `/joint_states` | `JointState` | Gazebo/Isaac | Robot State Publisher |
| `/cmd_vel` | `Twist` | Nav2 Controller | Gait Bridge |
| `/humanoid/gait_command` | `String` | Gait Bridge | Gait Controller |

## Building the Simulation World

### The Capstone Environment

Create an apartment-like environment with distinct rooms and obstacles:

```python
# capstone/launch/capstone_world.launch.py
"""Launch the capstone simulation world with the humanoid robot."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('humanoid_simulation')

    use_isaac = LaunchConfiguration('use_isaac', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_isaac', default_value='false',
                             description='Use Isaac Sim instead of Gazebo'),

        # Launch Gazebo with capstone world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'humanoid_gazebo.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(sim_pkg, 'worlds', 'capstone_apartment.world'),
            }.items(),
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
        ),
    ])
```

### The Apartment World

```xml
<!-- worlds/capstone_apartment.world -->
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="capstone_apartment">

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver><type>quick</type><iters>50</iters></solver>
      </ode>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <include><uri>model://sun</uri></include>

    <!-- Living Room (start position: 0,0) -->
    <model name="living_room_floor">
      <static>true</static>
      <link name="floor">
        <collision name="col"><geometry>
          <box><size>6 5 0.01</size></box>
        </geometry></collision>
        <visual name="vis"><geometry>
          <box><size>6 5 0.01</size></box>
        </geometry>
          <material><ambient>0.7 0.65 0.55 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Kitchen (at position 5, 2) -->
    <model name="kitchen_floor">
      <static>true</static>
      <pose>5 2 0 0 0 0</pose>
      <link name="floor">
        <collision name="col"><geometry>
          <box><size>4 3 0.01</size></box>
        </geometry></collision>
        <visual name="vis"><geometry>
          <box><size>4 3 0.01</size></box>
        </geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Kitchen counter with red cup -->
    <model name="kitchen_counter">
      <static>true</static>
      <pose>5.5 2.5 0.45 0 0 0</pose>
      <link name="counter">
        <collision name="col"><geometry>
          <box><size>1.5 0.6 0.9</size></box>
        </geometry></collision>
        <visual name="vis"><geometry>
          <box><size>1.5 0.6 0.9</size></box>
        </geometry>
          <material><ambient>0.4 0.3 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- THE RED CUP — the target object -->
    <model name="red_cup">
      <pose>5.5 2.5 0.95 0 0 0</pose>
      <link name="cup">
        <inertial>
          <mass>0.15</mass>
          <inertia>
            <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.00005</izz>
          </inertia>
        </inertial>
        <collision name="col"><geometry>
          <cylinder><radius>0.035</radius><length>0.10</length></cylinder>
        </geometry></collision>
        <visual name="vis"><geometry>
          <cylinder><radius>0.035</radius><length>0.10</length></cylinder>
        </geometry>
          <material><ambient>0.9 0.1 0.1 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Wall between living room and kitchen (with doorway) -->
    <model name="dividing_wall_left">
      <static>true</static>
      <pose>3 1.5 0.5 0 0 0</pose>
      <link name="wall">
        <collision name="col"><geometry>
          <box><size>0.15 2.0 1.0</size></box>
        </geometry></collision>
        <visual name="vis"><geometry>
          <box><size>0.15 2.0 1.0</size></box>
        </geometry>
          <material><ambient>0.9 0.9 0.85 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="dividing_wall_right">
      <static>true</static>
      <pose>3 3.5 0.5 0 0 0</pose>
      <link name="wall">
        <collision name="col"><geometry>
          <box><size>0.15 1.5 1.0</size></box>
        </geometry></collision>
        <visual name="vis"><geometry>
          <box><size>0.15 1.5 1.0</size></box>
        </geometry>
          <material><ambient>0.9 0.9 0.85 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Obstacle: couch in living room -->
    <model name="couch">
      <static>true</static>
      <pose>-1 1 0.3 0 0 0</pose>
      <link name="couch">
        <collision name="col"><geometry>
          <box><size>1.8 0.8 0.6</size></box>
        </geometry></collision>
        <visual name="vis"><geometry>
          <box><size>1.8 0.8 0.6</size></box>
        </geometry>
          <material><ambient>0.3 0.3 0.6 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Obstacle: coffee table -->
    <model name="coffee_table">
      <static>true</static>
      <pose>0.5 1 0.2 0 0 0</pose>
      <link name="table">
        <collision name="col"><geometry>
          <box><size>0.8 0.5 0.4</size></box>
        </geometry></collision>
        <visual name="vis"><geometry>
          <box><size>0.8 0.5 0.4</size></box>
        </geometry>
          <material><ambient>0.5 0.35 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## The Master Launch File

### Bringing Everything Together

```python
# capstone/launch/capstone_full.launch.py
"""Master launch file for the capstone autonomous humanoid project."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('humanoid_simulation')
    perception_pkg = get_package_share_directory('humanoid_perception')
    nav_pkg = get_package_share_directory('humanoid_navigation')
    voice_pkg = get_package_share_directory('humanoid_voice')
    planning_pkg = get_package_share_directory('humanoid_planning')

    return LaunchDescription([

        # ============================================================
        # LAYER 0: SIMULATION (Module 2)
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'humanoid_gazebo.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(sim_pkg, 'worlds', 'capstone_apartment.world'),
            }.items(),
        ),

        # ============================================================
        # LAYER 1: PERCEPTION (Module 3) — delayed to let sim start
        # ============================================================
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(perception_pkg, 'launch', 'humanoid_perception.launch.py')
                    ),
                ),
            ],
        ),

        # ============================================================
        # LAYER 2: NAVIGATION (Module 3)
        # ============================================================
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav_pkg, 'launch', 'humanoid_navigation.launch.py')
                    ),
                ),
            ],
        ),

        # ============================================================
        # LAYER 3: VOICE PIPELINE (Module 4)
        # ============================================================
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(voice_pkg, 'launch', 'voice_pipeline.launch.py')
                    ),
                ),
            ],
        ),

        # ============================================================
        # LAYER 4: COGNITIVE PLANNING (Module 4)
        # ============================================================
        TimerAction(
            period=10.0,
            actions=[
                # Scene builder
                Node(
                    package='humanoid_planning',
                    executable='scene_builder',
                    name='scene_builder',
                    output='screen',
                ),
                # LLM task planner
                Node(
                    package='humanoid_planning',
                    executable='task_planner',
                    name='task_planner',
                    parameters=[{
                        'model': 'gpt-4o',
                        'temperature': 0.1,
                        'max_plan_steps': 15,
                    }],
                    output='screen',
                ),
                # Grounding node
                Node(
                    package='humanoid_planning',
                    executable='grounding_node',
                    name='grounding_node',
                    output='screen',
                ),
                # Plan executor
                Node(
                    package='humanoid_planning',
                    executable='plan_executor',
                    name='plan_executor',
                    output='screen',
                ),
            ],
        ),

        # ============================================================
        # VISUALIZATION
        # ============================================================
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(sim_pkg, 'config', 'capstone.rviz')],
            output='screen',
        ),
    ])
```

## Running the Capstone

### Step-by-Step Execution

```bash
# Terminal 1: Launch the full system
export OPENAI_API_KEY="your-api-key-here"
ros2 launch capstone capstone_full.launch.py

# Wait for all nodes to start (~15 seconds)
# Gazebo window should show the apartment world with the humanoid standing
# RViz should show the costmap and robot model

# Terminal 2: Monitor the system
ros2 topic echo /humanoid/execution/status

# Terminal 3: Issue a voice command (or simulate one)
ros2 topic pub --once /humanoid/voice/transcript std_msgs/String \
  '{data: "go to the kitchen and bring me the red cup"}'
```

### Expected Execution Flow

```
[whisper_node]      Transcript: "go to the kitchen and bring me the red cup"
[intent_parser]     Intent: {type: "complex", raw_text: "go to the kitchen..."}
[llm_intent_parser] LLM parsed: {type: "complex", steps: [...]}
[scene_builder]     Scene: Robot at (0.0, 0.0), no objects detected
[task_planner]      Planning: "go to the kitchen and bring me the red cup"
[task_planner]      Plan generated: 9 steps
[task_planner]        Step 1: say({"text": "I'll get the red cup from the kitchen"})
[task_planner]        Step 2: navigate_to({"location": "kitchen"})
[task_planner]        Step 3: scan_area({})
[task_planner]        Step 4: detect_object({"object_name": "red cup"})
[task_planner]        Step 5: grasp({"object_name": "red cup"})
[task_planner]        Step 6: verify_grasp({})
[task_planner]        Step 7: navigate_to({"location": "user"})
[task_planner]        Step 8: handover({"object_name": "red cup"})
[task_planner]        Step 9: say({"text": "Here's your red cup!"})
[plan_executor]     Executing step 1/9: say
[tts_node]          Speaking: "I'll get the red cup from the kitchen"
[plan_executor]     Step 1 succeeded
[plan_executor]     Executing step 2/9: navigate_to(kitchen)
[cmd_vel_to_gait]   WALK_FAST: forward=0.25m, lateral=0.00m, turn=12.3deg
[nav2]              Navigation goal accepted
...
[nav2]              Navigation succeeded
[plan_executor]     Step 2 succeeded
[plan_executor]     Executing step 3/9: scan_area
...
[plan_executor]     Step 9 succeeded
[plan_executor]     Plan execution complete!
[tts_node]          Speaking: "Here's your red cup!"
```

## Evaluation Criteria

### Grading Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| **Voice Recognition** | 15 | Whisper correctly transcribes 3/3 test commands |
| **Intent Parsing** | 10 | Intents are correctly structured with resolved locations/objects |
| **LLM Planning** | 20 | Plans are logical, include verification steps, and handle edge cases |
| **Navigation** | 20 | Robot navigates from living room to kitchen through doorway without collision |
| **Object Detection** | 15 | Red cup is detected and localized when the robot reaches the kitchen |
| **Grasping** | 10 | Robot successfully grasps the cup (or demonstrates the action pipeline) |
| **Integration** | 10 | All components communicate correctly via ROS 2 topics |
| **Total** | **100** | |

### Test Scenarios

Run these three scenarios to validate your system:

**Scenario 1: Simple Navigation**
```
Command: "Go to the kitchen"
Expected: Robot navigates to kitchen, avoids obstacles, confirms arrival.
Pass criteria: Robot reaches kitchen within 2m tolerance in under 60 seconds.
```

**Scenario 2: Fetch Task (The Primary Mission)**
```
Command: "Bring me the red cup from the kitchen"
Expected: Full pipeline — plan, navigate, detect, grasp, return, handover.
Pass criteria: All plan steps execute. Navigation succeeds. Cup is detected.
```

**Scenario 3: Ambiguous Command**
```
Command: "Clean up"
Expected: LLM asks for clarification or generates a reasonable cleaning plan.
Pass criteria: Robot does not crash. Plan is reasonable. Robot communicates.
```

## Troubleshooting Guide

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Robot does not move | Nav2 not connected to `/cmd_vel` | Check Nav2 params and gait bridge |
| SLAM drift | Camera topic not publishing | Verify Gazebo camera plugin |
| LLM returns empty plan | API key not set or model error | Check `OPENAI_API_KEY` env var |
| Navigation fails at doorway | Costmap inflation too large | Reduce `inflation_radius` |
| Object not detected | Camera not pointing at kitchen counter | Add `look_at` or `scan_area` step |
| TTS not speaking | Piper/espeak not installed | Install TTS engine or use `espeak` fallback |
| Nodes not connecting | Topic name mismatch | Use `ros2 topic list` and `ros2 node info` |

## Extensions and Challenges

After completing the base capstone, try these extensions:

### Extension 1: Multi-Object Fetch
Modify the command to "bring me the red cup and the book." The planner should generate a multi-trip plan or use both hands.

### Extension 2: Dynamic Obstacles
Add a simulated human walking through the apartment. The robot must replan when its path is blocked.

### Extension 3: Isaac Sim Upgrade
Replace Gazebo with Isaac Sim for photorealistic rendering. Use Replicator to generate training data for the object detector, then deploy the trained model.

### Extension 4: Multi-Turn Dialogue
Implement a conversation where the robot asks clarifying questions: "I see a red cup and a blue cup. Which one would you like?"

---

## Exercises

### Exercise 1: System Integration Test
Launch the full capstone system. Verify that all ROS 2 topics listed in the topic map are publishing at their expected rates. Use `ros2 topic hz` for each topic and document the results.

### Exercise 2: The Primary Mission
Execute Scenario 2 (fetch the red cup). Record the full execution log showing each plan step, navigation progress, and detection results. Identify any failure points and document how you resolved them.

### Exercise 3: Robustness Testing
Run Scenario 2 five times. Document the success rate, average completion time, and any failures. For each failure, identify the root cause and propose a fix. Aim for at least 3/5 successful completions.

---

:::tip Key Takeaway
This capstone demonstrates that building an autonomous humanoid robot is not about any single breakthrough — it is about **integration**. Voice recognition, language planning, visual perception, navigation, and manipulation are each well-understood components. The engineering challenge is making them work together reliably, handling failures gracefully, and creating a system that is greater than the sum of its parts. You have now built that system — from URDF to voice command.
:::
