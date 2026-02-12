---
sidebar_position: 4
---

# Nav2 for Bipedal Navigation

**Path Planning, Costmaps, and Footstep-Aware Navigation for Humanoid Robots**

---

## Overview

Navigation for wheeled robots is a solved problem — Nav2 handles it elegantly. But humanoid robots are not wheeled. They step over obstacles, climb stairs, balance on uneven terrain, and must plan paths in terms of **discrete footstep placements** rather than continuous velocity commands. In this chapter, you will configure Nav2 for bipedal navigation, build costmaps that encode humanoid-specific constraints, implement footstep-aware planning, and handle the dynamic obstacles that humanoid robots encounter in real-world environments.

## Nav2 Architecture

Nav2 is a modular navigation framework built on ROS 2. Understanding its architecture is essential before adapting it for bipedal movement.

```
┌────────────────────────────────────────────────────────────┐
│                      NAV2 ARCHITECTURE                      │
│                                                              │
│  ┌────────────┐    ┌──────────────┐    ┌────────────────┐  │
│  │  BT         │    │  Planner     │    │  Controller    │  │
│  │  Navigator   │──►│  Server      │──►│  Server        │  │
│  │             │    │  (Global)    │    │  (Local)       │  │
│  └────────────┘    └──────────────┘    └────────────────┘  │
│        │                  │                    │             │
│        │                  ▼                    ▼             │
│        │           ┌──────────┐         ┌──────────┐       │
│        │           │ Costmap  │         │ Costmap  │       │
│        │           │ 2D       │         │ 2D       │       │
│        │           │ (Global) │         │ (Local)  │       │
│        │           └──────────┘         └──────────┘       │
│        │                                                    │
│        ▼                                                    │
│  ┌────────────────────────────────────────────────┐        │
│  │  Recovery Server (spin, backup, wait, clear)    │        │
│  └────────────────────────────────────────────────┘        │
└────────────────────────────────────────────────────────────┘
```

### Core Components

| Component | Role | Bipedal Adaptation |
|-----------|------|-------------------|
| **BT Navigator** | Behavior tree controlling navigation flow | Custom BT nodes for gait transitions |
| **Planner Server** | Computes global paths | Footstep-aware path planning |
| **Controller Server** | Follows paths locally | Gait pattern generator instead of `cmd_vel` |
| **Costmap 2D** | Represents navigable space | Step height, slope, and gap constraints |
| **Recovery Server** | Handles failures | Balance recovery, stance widening |

## Costmaps for Bipedal Robots

### The Standard Costmap Problem

Standard Nav2 costmaps assume a circular or rectangular robot footprint and classify space as free or occupied. For a humanoid, the navigability of terrain depends on factors that the standard costmap ignores:

| Factor | Wheeled Robot | Humanoid Robot |
|--------|---------------|----------------|
| **Step height** | Any curb is impassable | Can step over obstacles < 15 cm |
| **Slope** | Limited to ~10° | Can handle 20-30° with balance control |
| **Gaps** | Must bridge completely | Can step across gaps < 40 cm |
| **Stairs** | Impassable | Navigable with stair-climbing gait |
| **Narrow passages** | Footprint clearance only | Can turn sideways, squeeze through |

### Custom Costmap Layers

Add humanoid-specific layers to the Nav2 costmap:

```python
# humanoid_navigation/terrain_costmap_layer.py
"""Custom costmap layer that encodes terrain traversability for bipedal robots."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import numpy as np


class TerrainCostmapLayer(Node):
    """Analyzes 3D terrain and produces a costmap encoding bipedal constraints."""

    # Humanoid-specific parameters
    MAX_STEP_HEIGHT = 0.15       # meters — maximum single step height
    MAX_SLOPE_ANGLE = 25.0       # degrees — maximum traversable slope
    MAX_GAP_WIDTH = 0.35         # meters — maximum gap the robot can step over
    FOOT_LENGTH = 0.25           # meters — minimum flat surface for foot placement
    ROBOT_WIDTH = 0.45           # meters — hip width for narrow passage check

    def __init__(self):
        super().__init__('terrain_costmap_layer')

        self.create_subscription(
            PointCloud2,
            '/humanoid/points',
            self.on_pointcloud,
            10,
        )

        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/humanoid/terrain_costmap',
            10,
        )

        # Costmap parameters
        self.resolution = 0.05  # 5 cm per cell
        self.width = 200        # 10 m
        self.height = 200       # 10 m
        self.origin_x = -5.0
        self.origin_y = -5.0

        self.get_logger().info('Terrain costmap layer active')

    def on_pointcloud(self, msg: PointCloud2):
        """Process 3D point cloud to generate terrain costmap."""
        # Parse point cloud into numpy array
        points = self.parse_pointcloud(msg)
        if points is None or len(points) == 0:
            return

        # Initialize costmap
        costmap = np.zeros((self.height, self.width), dtype=np.int8)

        # Analyze each cell
        for cy in range(self.height):
            for cx in range(self.width):
                world_x = self.origin_x + cx * self.resolution
                world_y = self.origin_y + cy * self.resolution

                # Get points in this cell
                cell_mask = (
                    (points[:, 0] >= world_x) &
                    (points[:, 0] < world_x + self.resolution) &
                    (points[:, 1] >= world_y) &
                    (points[:, 1] < world_y + self.resolution)
                )
                cell_points = points[cell_mask]

                if len(cell_points) == 0:
                    costmap[cy, cx] = -1  # Unknown
                    continue

                cost = self.compute_bipedal_cost(cell_points)
                costmap[cy, cx] = cost

        self.publish_costmap(costmap)

    def compute_bipedal_cost(self, cell_points: np.ndarray) -> int:
        """Compute traversability cost for a single cell."""
        heights = cell_points[:, 2]
        height_range = np.max(heights) - np.min(heights)

        # Check step height constraint
        if height_range > self.MAX_STEP_HEIGHT:
            return 100  # Lethal — too tall to step over

        # Check slope (approximate from height variance)
        if len(cell_points) >= 3:
            # Fit a plane to estimate slope
            slope_deg = self.estimate_slope(cell_points)
            if slope_deg > self.MAX_SLOPE_ANGLE:
                return 100  # Lethal — too steep

            # Moderate slopes get increasing cost
            if slope_deg > 10.0:
                return int(50 + (slope_deg - 10) * 3.3)  # 50-100 range

        # Flat, traversable terrain
        return 0  # Free

    def estimate_slope(self, points: np.ndarray) -> float:
        """Estimate terrain slope in degrees from a set of 3D points."""
        if len(points) < 3:
            return 0.0

        # Simple slope estimate from PCA of height variations
        centered = points - np.mean(points, axis=0)
        cov = np.cov(centered.T)
        eigenvalues = np.linalg.eigvalsh(cov)

        # Ratio of smallest to largest eigenvalue indicates planarity
        if eigenvalues[-1] > 0:
            slope_ratio = np.sqrt(eigenvalues[0] / eigenvalues[-1])
            return np.degrees(np.arctan(slope_ratio))
        return 0.0

    def parse_pointcloud(self, msg):
        """Parse PointCloud2 message to numpy array. Simplified."""
        # In production, use sensor_msgs_py or ros2_numpy
        return None  # Placeholder — implement with your point cloud library

    def publish_costmap(self, costmap: np.ndarray):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.data = costmap.flatten().tolist()
        self.costmap_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TerrainCostmapLayer())
    rclpy.shutdown()
```

## Nav2 Configuration for Humanoids

### Parameter File

```yaml
# config/nav2_humanoid_params.yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry
    bt_loop_duration: 10          # ms — behavior tree tick rate
    default_server_timeout: 20
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_bt_node

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.25           # Humanoid approximate radius
      resolution: 0.05             # 5 cm resolution
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /humanoid/scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.1
          obstacle_max_range: 8.0
          obstacle_min_range: 0.1

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55     # Slightly larger than robot radius

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5                     # 5m x 5m local window
      height: 5
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan pointcloud
        scan:
          topic: /humanoid/scan
          data_type: "LaserScan"
          max_obstacle_height: 2.0
          clearing: true
          marking: true
        pointcloud:
          topic: /humanoid/depth/points
          data_type: "PointCloud2"
          max_obstacle_height: 1.5
          min_obstacle_height: 0.05  # Ignore ground plane
          clearing: true
          marking: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 2.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.25              # Goal tolerance (meters)
      use_astar: true              # A* instead of Dijkstra
      allow_unknown: false

controller_server:
  ros__parameters:
    controller_frequency: 20.0     # 20 Hz control loop
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.1  # Must move 10cm in check period
      movement_time_allowance: 15.0  # 15 seconds to make progress

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15       # 15 cm position tolerance
      yaw_goal_tolerance: 0.1       # ~5.7 degrees heading tolerance
      stateful: true

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5               # Humanoid max forward speed (m/s)
      min_vel_y: -0.1              # Small lateral velocity for side-stepping
      max_vel_y: 0.1
      max_vel_theta: 0.5           # Turning speed (rad/s)
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 0.3               # Conservative acceleration for balance
      acc_lim_y: 0.1
      acc_lim_theta: 0.5
      decel_lim_x: -0.5
      decel_lim_y: -0.1
      decel_lim_theta: -0.5

      # Trajectory scoring
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint",
                "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      ObstacleFootprint.scale: 0.01
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

recovery_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]

    spin:
      plugin: "nav2_recoveries/Spin"

    backup:
      plugin: "nav2_recoveries/BackUp"

    wait:
      plugin: "nav2_recoveries/Wait"
```

## Translating cmd_vel to Bipedal Gait

Nav2 outputs `geometry_msgs/Twist` messages on `/cmd_vel`. Wheeled robots apply these directly as wheel velocities. A humanoid must translate them into footstep patterns.

### The Velocity-to-Gait Bridge

```python
# humanoid_navigation/cmd_vel_to_gait.py
"""Translate Nav2 cmd_vel commands into bipedal gait parameters."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class CmdVelToGait(Node):
    """Converts continuous velocity commands to discrete gait parameters."""

    # Gait parameters
    MAX_STEP_LENGTH = 0.30     # meters
    MAX_STEP_WIDTH = 0.10      # meters (lateral)
    MAX_STEP_HEIGHT = 0.05     # meters (foot clearance)
    STEP_DURATION = 0.8        # seconds per step
    STANCE_WIDTH = 0.20        # meters (distance between feet)

    def __init__(self):
        super().__init__('cmd_vel_to_gait')

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.gait_pub = self.create_publisher(String, '/humanoid/gait_command', 10)

        self.get_logger().info('Velocity-to-gait bridge active')

    def on_cmd_vel(self, msg: Twist):
        vx = msg.linear.x     # Forward velocity
        vy = msg.linear.y     # Lateral velocity
        wz = msg.angular.z    # Turning rate

        speed = math.sqrt(vx ** 2 + vy ** 2)

        if speed < 0.01 and abs(wz) < 0.01:
            gait_mode = 'STAND'
            step_length = 0.0
            step_width = 0.0
            turn_angle = 0.0
        elif abs(wz) > 0.3 and speed < 0.05:
            gait_mode = 'TURN_IN_PLACE'
            step_length = 0.0
            step_width = 0.0
            turn_angle = min(abs(wz) * self.STEP_DURATION, math.radians(30))
            turn_angle = math.copysign(turn_angle, wz)
        elif speed > 0.3:
            gait_mode = 'WALK_FAST'
            step_length = min(vx * self.STEP_DURATION, self.MAX_STEP_LENGTH)
            step_width = min(abs(vy) * self.STEP_DURATION, self.MAX_STEP_WIDTH)
            step_width = math.copysign(step_width, vy)
            turn_angle = wz * self.STEP_DURATION * 0.5
        else:
            gait_mode = 'WALK_SLOW'
            step_length = vx * self.STEP_DURATION * 0.7
            step_width = vy * self.STEP_DURATION * 0.5
            turn_angle = wz * self.STEP_DURATION * 0.3

        # Publish gait command
        cmd = String()
        cmd.data = (
            f'{gait_mode}:'
            f'step_length={step_length:.3f},'
            f'step_width={step_width:.3f},'
            f'turn_angle={math.degrees(turn_angle):.1f},'
            f'step_height={self.MAX_STEP_HEIGHT:.3f},'
            f'duration={self.STEP_DURATION:.2f}'
        )
        self.gait_pub.publish(cmd)

        self.get_logger().info(
            f'{gait_mode}: forward={step_length:.2f}m, '
            f'lateral={step_width:.2f}m, '
            f'turn={math.degrees(turn_angle):.1f}deg'
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CmdVelToGait())
    rclpy.shutdown()
```

## Footstep Planning

For complex terrain (stairs, stepping stones, narrow passages), discrete footstep planning is more appropriate than continuous velocity control.

### Footstep Planner Node

```python
# humanoid_navigation/footstep_planner.py
"""Discrete footstep planner for humanoid navigation."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Path
import math
import numpy as np


class FootstepPlanner(Node):
    """Plans a sequence of footstep placements from current position to goal."""

    # Bipedal constraints
    MAX_STEP_LENGTH = 0.30     # meters forward
    MIN_STEP_LENGTH = 0.10     # meters forward
    MAX_STEP_WIDTH = 0.10      # meters lateral
    NOMINAL_STANCE_WIDTH = 0.20  # meters between feet
    MAX_STEP_TURN = 0.35       # radians (~20 degrees) per step
    FOOT_LENGTH = 0.25         # meters
    FOOT_WIDTH = 0.10          # meters

    def __init__(self):
        super().__init__('footstep_planner')

        self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal, 10
        )
        self.create_subscription(
            Path, '/plan', self.on_global_path, 10
        )

        self.footstep_pub = self.create_publisher(
            PoseArray, '/humanoid/footstep_plan', 10
        )

        self.current_path = None
        self.get_logger().info('Footstep planner ready')

    def on_global_path(self, msg: Path):
        """Store the global path from Nav2 planner."""
        self.current_path = msg

    def on_goal(self, msg: PoseStamped):
        """Plan footsteps along the global path to the goal."""
        if self.current_path is None or len(self.current_path.poses) < 2:
            self.get_logger().warn('No global path available for footstep planning')
            return

        footsteps = self.plan_footsteps(self.current_path)

        # Publish footstep plan
        plan_msg = PoseArray()
        plan_msg.header.stamp = self.get_clock().now().to_msg()
        plan_msg.header.frame_id = 'map'
        plan_msg.poses = footsteps

        self.footstep_pub.publish(plan_msg)
        self.get_logger().info(f'Published {len(footsteps)} footsteps')

    def plan_footsteps(self, path: Path) -> list:
        """Generate alternating left-right footstep placements along a path."""
        footsteps = []
        is_left_foot = True  # Start with left foot

        # Sample waypoints from the global path
        waypoints = [(p.pose.position.x, p.pose.position.y) for p in path.poses]

        # Walk along the path, placing footsteps
        current_x, current_y = waypoints[0]
        current_heading = 0.0
        path_idx = 1

        while path_idx < len(waypoints):
            target_x, target_y = waypoints[path_idx]
            dx = target_x - current_x
            dy = target_y - current_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < self.MIN_STEP_LENGTH:
                path_idx += 1
                continue

            # Compute heading to target
            target_heading = math.atan2(dy, dx)
            heading_diff = self.normalize_angle(target_heading - current_heading)

            # Limit turn per step
            step_turn = max(-self.MAX_STEP_TURN, min(self.MAX_STEP_TURN, heading_diff))
            current_heading += step_turn

            # Compute step length (limited by max)
            step_length = min(self.MAX_STEP_LENGTH, dist)

            # Compute foot placement
            # Lateral offset alternates left/right
            lateral_offset = self.NOMINAL_STANCE_WIDTH / 2
            if not is_left_foot:
                lateral_offset = -lateral_offset

            foot_x = current_x + step_length * math.cos(current_heading) \
                      - lateral_offset * math.sin(current_heading)
            foot_y = current_y + step_length * math.sin(current_heading) \
                      + lateral_offset * math.cos(current_heading)

            # Create foot pose
            foot_pose = Pose()
            foot_pose.position = Point(x=foot_x, y=foot_y, z=0.0)
            foot_pose.orientation = self.yaw_to_quaternion(current_heading)
            footsteps.append(foot_pose)

            # Advance
            current_x = current_x + step_length * math.cos(current_heading)
            current_y = current_y + step_length * math.sin(current_heading)
            is_left_foot = not is_left_foot

            if dist <= step_length:
                path_idx += 1

        return footsteps

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        return Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2),
            w=math.cos(yaw / 2),
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FootstepPlanner())
    rclpy.shutdown()
```

## Dynamic Obstacle Avoidance

Humanoid robots operate around people. Dynamic obstacle avoidance must account for human motion patterns.

### People Tracking and Prediction

```python
# humanoid_navigation/people_tracker.py
"""Track detected people and predict their future positions."""
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import math


class PeopleTracker(Node):
    """Tracks people using detections and predicts motion for Nav2 avoidance."""

    PREDICTION_HORIZON = 3.0   # seconds into the future
    PREDICTION_STEPS = 6       # number of predicted positions
    AVERAGE_WALK_SPEED = 1.2   # m/s typical human walking speed

    def __init__(self):
        super().__init__('people_tracker')

        self.create_subscription(
            Detection2DArray,
            '/humanoid/detectnet/detections',
            self.on_detections,
            10,
        )

        self.tracked_pub = self.create_publisher(
            PoseArray, '/humanoid/tracked_people', 10
        )
        self.predicted_pub = self.create_publisher(
            PoseArray, '/humanoid/predicted_people', 10
        )

        # Simple tracking state: {id: [(x, y, timestamp), ...]}
        self.tracks = {}
        self.next_id = 0

        self.get_logger().info('People tracker active')

    def on_detections(self, msg: Detection2DArray):
        current_time = self.get_clock().now().nanoseconds / 1e9
        current_people = []
        predicted_people = []

        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id != 'person':
                    continue
                if result.hypothesis.score < 0.6:
                    continue

                # Estimate world position from bounding box
                # (simplified — use depth + camera model in production)
                bbox = detection.bbox
                estimated_distance = 500.0 / max(bbox.size_y, 1)  # Rough depth from height
                estimated_x = estimated_distance
                estimated_y = (bbox.center.position.x - 320) / 320.0 * estimated_distance * 0.5

                current_people.append(
                    Pose(
                        position=Point(x=estimated_x, y=estimated_y, z=0.0),
                        orientation=Quaternion(w=1.0),
                    )
                )

                # Linear motion prediction
                vel_x = -self.AVERAGE_WALK_SPEED * 0.5  # Assume approaching
                for step in range(self.PREDICTION_STEPS):
                    dt = (step + 1) * self.PREDICTION_HORIZON / self.PREDICTION_STEPS
                    predicted_people.append(
                        Pose(
                            position=Point(
                                x=estimated_x + vel_x * dt,
                                y=estimated_y,
                                z=0.0,
                            ),
                            orientation=Quaternion(w=1.0),
                        )
                    )

        # Publish current tracked positions
        tracked_msg = PoseArray()
        tracked_msg.header.stamp = self.get_clock().now().to_msg()
        tracked_msg.header.frame_id = 'base_link'
        tracked_msg.poses = current_people
        self.tracked_pub.publish(tracked_msg)

        # Publish predicted future positions
        predicted_msg = PoseArray()
        predicted_msg.header.stamp = self.get_clock().now().to_msg()
        predicted_msg.header.frame_id = 'base_link'
        predicted_msg.poses = predicted_people
        self.predicted_pub.publish(predicted_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PeopleTracker())
    rclpy.shutdown()
```

## Launching the Complete Navigation Stack

```python
# launch/humanoid_navigation.launch.py
"""Complete Nav2-based navigation for the humanoid."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_pkg = get_package_share_directory('humanoid_navigation')
    perception_pkg = get_package_share_directory('humanoid_perception')

    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_humanoid_params.yaml')

    return LaunchDescription([
        # Perception stack (SLAM + detection)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_pkg, 'launch', 'humanoid_perception.launch.py')
            )
        ),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py'
                )
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'true',
            }.items(),
        ),

        # Velocity to gait bridge
        Node(
            package='humanoid_navigation',
            executable='cmd_vel_to_gait',
            name='cmd_vel_to_gait',
            output='screen',
        ),

        # Footstep planner
        Node(
            package='humanoid_navigation',
            executable='footstep_planner',
            name='footstep_planner',
            output='screen',
        ),

        # People tracker
        Node(
            package='humanoid_navigation',
            executable='people_tracker',
            name='people_tracker',
            output='screen',
        ),

        # RViz with navigation config
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(nav_pkg, 'config', 'navigation.rviz')],
            output='screen',
        ),
    ])
```

### Sending Navigation Goals

```bash
# Launch the navigation stack
ros2 launch humanoid_navigation humanoid_navigation.launch.py

# Send a goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Or use RViz "2D Goal Pose" tool to click a destination
```

## Bipedal-Specific Recovery Behaviors

When standard recovery fails, humanoids have unique options:

| Recovery Behavior | When to Use | Implementation |
|-------------------|-------------|---------------|
| **Widen stance** | Instability detected during navigation | Increase step width temporarily |
| **Side step** | Narrow passage, cannot turn | Lateral stepping gait |
| **Step over** | Low obstacle in path (< 15 cm) | High-step gait pattern |
| **Step back** | Dead end or too close to obstacle | Backward stepping gait |
| **Crouch** | Low overhead clearance | Lower center of mass |
| **Turn in place** | Need to change direction in tight space | Pivot on one foot |

---

## Exercises

### Exercise 1: Costmap Tuning
Configure the Nav2 costmap for your humanoid in Gazebo. Place obstacles at varying heights (5 cm, 10 cm, 15 cm, 20 cm, 30 cm). Verify that the costmap correctly marks obstacles below 15 cm as traversable (cost < 100) and obstacles above 15 cm as lethal (cost = 100).

### Exercise 2: Navigate a Corridor
Build a 15-meter corridor in Gazebo with 3 static obstacles and 2 narrow doorways. Configure Nav2 and send a navigation goal to the end of the corridor. Verify that the robot plans a path through all doorways and avoids obstacles. Log the footstep commands generated by the velocity-to-gait bridge.

### Exercise 3: Dynamic Avoidance
Add a simulated human character walking toward the robot in the corridor. Implement the people tracker and verify that Nav2 replans when the human blocks the current path. Measure the replanning time and verify the robot maintains a minimum 0.5 m safety distance from the human at all times.

---

:::tip Key Takeaway
Nav2 provides the navigation framework, but humanoid robots require a **translation layer** between continuous velocity commands and discrete bipedal gaits. The key adaptations are costmaps that encode step height and slope constraints, a velocity-to-gait bridge that converts `cmd_vel` to footstep parameters, and dynamic obstacle avoidance that predicts human motion. These adaptations sit on top of Nav2 — you get the full power of the navigation stack while respecting the physics of bipedal locomotion.
:::
