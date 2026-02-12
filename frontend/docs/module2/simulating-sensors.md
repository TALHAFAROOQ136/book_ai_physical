---
sidebar_position: 4
---

# Simulating Sensors

**LiDAR, Cameras, IMUs, and Force-Torque — Building Trust Before Real Hardware**

---

## Overview

Your humanoid robot perceives the world through sensors: cameras for vision, LiDAR for 3D spatial mapping, IMUs for balance and orientation, and force-torque sensors for detecting contact forces. In simulation, these sensors produce **synthetic data** that closely matches real hardware output. This chapter teaches you to configure every major sensor type in Gazebo, add realistic noise models, and build perception pipelines that transfer directly from simulation to physical hardware.

## Why Simulated Sensors Matter

The sensor pipeline is the most critical component to get right before deploying to real hardware:

```
┌───────────────────────────────────────────────────────┐
│               PERCEPTION PIPELINE                      │
│                                                         │
│   Sensor ──► Raw Data ──► Processing ──► Decision      │
│                                                         │
│   In simulation, we control EVERY stage:               │
│   • Sensor parameters (resolution, FOV, noise)         │
│   • Ground truth (exact positions, labels)             │
│   • Environmental conditions (lighting, weather)       │
│   • Edge cases (sensor failure, occlusion)             │
└───────────────────────────────────────────────────────┘
```

If your perception code works with simulated sensor data, it will work with real sensor data — **provided the simulation is realistic enough**. This chapter focuses on making that gap as small as possible.

## Camera Simulation

### RGB Camera in Gazebo

Add a camera sensor to your humanoid's head link:

```xml
<!-- Add to your URDF inside the head_link -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head_cam">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- Realistic camera noise -->
      </noise>
    </camera>

    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera (RGB-D)

Depth cameras like the Intel RealSense are essential for 3D perception:

```xml
<gazebo reference="depth_camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>15.0</update_rate>
    <camera name="depth_cam">
      <horizontal_fov>1.2112</horizontal_fov>  <!-- 69.4 degrees (RealSense D435) -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.3</near>   <!-- RealSense minimum range -->
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </camera>

    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>image_raw:=depth/image_raw</remapping>
        <remapping>camera_info:=depth/camera_info</remapping>
        <remapping>points:=depth/points</remapping>
      </ros>
      <frame_name>depth_camera_link</frame_name>
      <min_depth>0.3</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### Processing Camera Data in ROS 2

```python
# humanoid_perception/image_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.on_rgb_image,
            10
        )

        self.create_subscription(
            Image,
            '/humanoid/depth/image_raw',
            self.on_depth_image,
            10
        )

        self.get_logger().info('Image processor started')

    def on_rgb_image(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Example: simple edge detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        self.get_logger().info(
            f'RGB frame: {cv_image.shape[1]}x{cv_image.shape[0]}, '
            f'edges detected: {np.count_nonzero(edges)} pixels'
        )

    def on_depth_image(self, msg: Image):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Calculate distance statistics
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            self.get_logger().info(
                f'Depth frame: min={np.min(valid_depths):.2f}m, '
                f'max={np.max(valid_depths):.2f}m, '
                f'mean={np.mean(valid_depths):.2f}m'
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImageProcessor())
    rclpy.shutdown()
```

## LiDAR Simulation

### 2D LiDAR (Laser Scanner)

A 2D LiDAR is commonly mounted at the robot's torso for obstacle detection and navigation:

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_2d">
    <update_rate>10.0</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>           <!-- 0.5 degree resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>   <!-- Full 360-degree scan -->
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>    <!-- 12 cm minimum range -->
        <max>12.0</max>     <!-- 12 m maximum range -->
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1 cm noise standard deviation -->
      </noise>
    </ray>

    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### 3D LiDAR (Point Cloud)

For full 3D spatial awareness, add a multi-layer LiDAR:

```xml
<gazebo reference="lidar_3d_link">
  <sensor type="ray" name="lidar_3d">
    <update_rate>10.0</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>          <!-- 0.2 degree horizontal resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>            <!-- 16 vertical layers (like Velodyne VLP-16) -->
          <resolution>1</resolution>
          <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
          <max_angle>0.261799</max_angle>   <!-- +15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.5</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </ray>

    <plugin name="lidar_3d_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>lidar_3d_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Processing LiDAR Data

```python
# humanoid_perception/lidar_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import numpy as np
import math


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        self.create_subscription(LaserScan, '/humanoid/scan', self.on_scan, 10)
        self.create_subscription(PointCloud2, '/humanoid/points', self.on_points, 10)

        self.get_logger().info('LiDAR processor started')

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Filter valid readings
        valid = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]

        if len(valid) == 0:
            return

        # Find the closest obstacle
        min_range = np.min(valid)
        min_idx = np.argmin(
            np.where(
                (ranges > msg.range_min) & (ranges < msg.range_max),
                ranges,
                float('inf')
            )
        )
        obstacle_angle = msg.angle_min + min_idx * msg.angle_increment

        self.get_logger().info(
            f'Closest obstacle: {min_range:.2f}m at {math.degrees(obstacle_angle):.1f} deg'
        )

        # Safety check: warn if obstacle is within 0.5m
        if min_range < 0.5:
            self.get_logger().warn(
                f'PROXIMITY WARNING: Object at {min_range:.2f}m!'
            )

    def on_points(self, msg: PointCloud2):
        self.get_logger().info(
            f'Point cloud: {msg.width * msg.height} points, '
            f'frame={msg.header.frame_id}'
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LidarProcessor())
    rclpy.shutdown()
```

## IMU Simulation

The Inertial Measurement Unit is critical for humanoid balance. It measures:

- **Linear acceleration** (accelerometer) — including gravity
- **Angular velocity** (gyroscope) — rotation rates
- **Orientation** (magnetometer + sensor fusion) — absolute heading

### Adding an IMU to Your URDF

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="body_imu">
    <update_rate>200.0</update_rate>  <!-- 200 Hz for balance control -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>    <!-- Gyro noise -->
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>     <!-- Accelerometer noise -->
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Noise Parameters Explained

| Parameter | Real Sensor Value | Effect on Simulation |
|-----------|------------------|---------------------|
| **Gyro noise** (`stddev`) | 0.0001-0.001 rad/s | Random jitter in angular velocity readings |
| **Gyro bias** | 0.00001-0.0001 rad/s | Slow drift — orientation error accumulates over time |
| **Accel noise** (`stddev`) | 0.01-0.05 m/s² | Random jitter in acceleration readings |
| **Accel bias** | 0.01-0.5 m/s² | Constant offset — affects tilt estimation |

These values should be matched to your actual IMU's datasheet for realistic sim-to-real transfer.

### Balance Controller Using IMU Data

```python
# humanoid_control/balance_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math


class BalanceController(Node):
    """Simple PD balance controller using IMU feedback."""

    def __init__(self):
        super().__init__('balance_controller')

        self.create_subscription(Imu, '/humanoid/imu/data', self.on_imu, 10)
        self.cmd_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)

        # PD gains for balance correction
        self.kp_pitch = 2.0   # Proportional gain for forward/backward tilt
        self.kd_pitch = 0.5   # Derivative gain
        self.kp_roll = 2.0    # Proportional gain for side-to-side tilt
        self.kd_roll = 0.5

        self.get_logger().info('Balance controller active')

    def on_imu(self, msg: Imu):
        # Extract orientation as pitch and roll
        q = msg.orientation
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )

        # Angular velocities (derivative term)
        pitch_rate = msg.angular_velocity.y
        roll_rate = msg.angular_velocity.x

        # PD control to correct tilt
        correction = Twist()
        correction.linear.x = -(self.kp_pitch * pitch + self.kd_pitch * pitch_rate)
        correction.linear.y = -(self.kp_roll * roll + self.kd_roll * roll_rate)

        self.cmd_pub.publish(correction)

        # Log warnings for excessive tilt
        tilt_deg = math.degrees(math.sqrt(pitch ** 2 + roll ** 2))
        if tilt_deg > 5.0:
            self.get_logger().warn(f'Tilt warning: {tilt_deg:.1f} degrees')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BalanceController())
    rclpy.shutdown()
```

## Force-Torque Sensors

Force-torque (F/T) sensors measure forces and torques at joints. They are essential for:

- Detecting contact during grasping
- Measuring ground reaction forces during walking
- Implementing compliant (force-controlled) behaviors

### Adding F/T Sensors in Gazebo

```xml
<gazebo reference="left_wrist_joint">
  <sensor type="force_torque" name="left_wrist_ft">
    <update_rate>500.0</update_rate>  <!-- High rate for force control -->
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.1</stddev>  <!-- 0.1 N noise -->
      </noise>
    </force_torque>

    <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>wrench:=left_wrist/ft</remapping>
      </ros>
      <body_name>left_wrist_link</body_name>
      <frame_name>left_wrist_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Foot Force Sensors for Walking

Ground reaction force measurement is critical for walking algorithms:

```xml
<!-- Add to each foot link -->
<gazebo reference="left_ankle_joint">
  <sensor type="force_torque" name="left_foot_ft">
    <update_rate>1000.0</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>

    <plugin name="left_foot_ft_plugin" filename="libgazebo_ros_ft_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>wrench:=left_foot/ft</remapping>
      </ros>
      <body_name>left_foot_link</body_name>
      <frame_name>left_foot_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Using Force Data for Gait Detection

```python
# humanoid_control/gait_detector.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String


class GaitDetector(Node):
    """Detects gait phase using foot force-torque sensors."""

    CONTACT_THRESHOLD = 10.0  # Newtons — foot is on ground if Fz > threshold

    def __init__(self):
        super().__init__('gait_detector')

        self.left_fz = 0.0
        self.right_fz = 0.0

        self.create_subscription(
            WrenchStamped, '/humanoid/left_foot/ft', self.on_left_ft, 10
        )
        self.create_subscription(
            WrenchStamped, '/humanoid/right_foot/ft', self.on_right_ft, 10
        )

        self.phase_pub = self.create_publisher(String, '/humanoid/gait_phase', 10)
        self.create_timer(0.01, self.detect_phase)  # 100 Hz

        self.get_logger().info('Gait detector started')

    def on_left_ft(self, msg: WrenchStamped):
        self.left_fz = abs(msg.wrench.force.z)

    def on_right_ft(self, msg: WrenchStamped):
        self.right_fz = abs(msg.wrench.force.z)

    def detect_phase(self):
        left_contact = self.left_fz > self.CONTACT_THRESHOLD
        right_contact = self.right_fz > self.CONTACT_THRESHOLD

        if left_contact and right_contact:
            phase = 'DOUBLE_SUPPORT'
        elif left_contact:
            phase = 'LEFT_STANCE'
        elif right_contact:
            phase = 'RIGHT_STANCE'
        else:
            phase = 'FLIGHT'  # Both feet off ground (jumping or falling)

        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GaitDetector())
    rclpy.shutdown()
```

## Sensor Noise Models

Realistic simulation requires accurate noise. Here is a comparison of noise-free vs. realistic sensor data:

### Types of Sensor Noise

| Noise Type | Description | Affected Sensors |
|------------|-------------|-----------------|
| **Gaussian** | Random per-reading noise | All sensors |
| **Bias** | Constant offset that drifts slowly | IMU, F/T sensors |
| **Quantization** | Discrete step resolution | Encoders, low-res cameras |
| **Dropout** | Occasional missing readings | LiDAR (glass, black surfaces) |
| **Motion blur** | Smearing from fast movement | Cameras |

### Matching Noise to Real Hardware

The process for tuning noise parameters:

1. **Read the datasheet** — Every sensor specifies noise characteristics (e.g., "gyro noise density: 0.01 deg/s/sqrt(Hz)")
2. **Record real data** — Collect sensor data with the robot stationary
3. **Compute statistics** — Calculate mean (bias), standard deviation (noise), and Allan variance (drift)
4. **Set simulation parameters** — Match the Gazebo noise model to measured values

```python
# scripts/compute_noise_params.py
"""Compute noise parameters from recorded IMU bag data."""
import numpy as np


def compute_imu_noise(gyro_data: np.ndarray, accel_data: np.ndarray, dt: float):
    """
    Compute noise parameters from stationary IMU recordings.

    Args:
        gyro_data: array of shape (N, 3) — angular velocity readings
        accel_data: array of shape (N, 3) — linear acceleration readings
        dt: time between samples in seconds

    Returns:
        Dictionary of noise parameters for Gazebo configuration
    """
    results = {}

    # Gyroscope noise and bias
    gyro_mean = np.mean(gyro_data, axis=0)       # Bias
    gyro_std = np.std(gyro_data, axis=0)          # Noise
    results['gyro_bias'] = gyro_mean.tolist()
    results['gyro_noise_stddev'] = gyro_std.tolist()

    # Accelerometer noise and bias
    # Remove gravity from z-axis (assuming robot is upright)
    accel_corrected = accel_data.copy()
    accel_corrected[:, 2] -= 9.81
    accel_mean = np.mean(accel_corrected, axis=0)
    accel_std = np.std(accel_corrected, axis=0)
    results['accel_bias'] = accel_mean.tolist()
    results['accel_noise_stddev'] = accel_std.tolist()

    return results
```

## Launching a Full Sensor Suite

Combine all sensors in a single launch file:

```python
# launch/humanoid_sensors.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('humanoid_simulation')

    return LaunchDescription([
        # Start Gazebo with the robot (includes sensors from URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'humanoid_gazebo.launch.py')
            )
        ),

        # Image processor
        Node(
            package='humanoid_perception',
            executable='image_processor',
            name='image_processor',
            output='screen',
        ),

        # LiDAR processor
        Node(
            package='humanoid_perception',
            executable='lidar_processor',
            name='lidar_processor',
            output='screen',
        ),

        # Balance controller (uses IMU)
        Node(
            package='humanoid_control',
            executable='balance_controller',
            name='balance_controller',
            output='screen',
        ),

        # Gait detector (uses F/T sensors)
        Node(
            package='humanoid_control',
            executable='gait_detector',
            name='gait_detector',
            output='screen',
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'sensors.rviz')],
            output='screen',
        ),
    ])
```

```bash
# Launch the complete sensor suite
ros2 launch humanoid_simulation humanoid_sensors.launch.py

# Verify all topics are publishing
ros2 topic list | grep humanoid
# /humanoid/camera/image_raw
# /humanoid/depth/image_raw
# /humanoid/depth/points
# /humanoid/scan
# /humanoid/points
# /humanoid/imu/data
# /humanoid/left_foot/ft
# /humanoid/right_foot/ft
# /humanoid/left_wrist/ft
```

## Sensor Fusion: Combining Multiple Sensors

Real robots never rely on a single sensor. Sensor fusion combines multiple data sources for robust perception:

```python
# humanoid_perception/sensor_fusion.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import WrenchStamped, PoseStamped
import math


class SimpleSensorFusion(Node):
    """Fuses IMU + LiDAR + F/T for situational awareness."""

    def __init__(self):
        super().__init__('sensor_fusion')

        self.imu_data = None
        self.scan_data = None
        self.left_ft = None
        self.right_ft = None

        self.create_subscription(Imu, '/humanoid/imu/data', self.on_imu, 10)
        self.create_subscription(LaserScan, '/humanoid/scan', self.on_scan, 10)
        self.create_subscription(
            WrenchStamped, '/humanoid/left_foot/ft', self.on_left_ft, 10
        )
        self.create_subscription(
            WrenchStamped, '/humanoid/right_foot/ft', self.on_right_ft, 10
        )

        self.status_pub = self.create_publisher(PoseStamped, '/humanoid/fused_state', 10)
        self.create_timer(0.02, self.fuse_and_publish)  # 50 Hz

    def on_imu(self, msg):
        self.imu_data = msg

    def on_scan(self, msg):
        self.scan_data = msg

    def on_left_ft(self, msg):
        self.left_ft = msg

    def on_right_ft(self, msg):
        self.right_ft = msg

    def fuse_and_publish(self):
        if self.imu_data is None:
            return

        # Extract tilt from IMU
        q = self.imu_data.orientation
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )

        # Nearest obstacle from LiDAR
        nearest_obstacle = float('inf')
        if self.scan_data:
            import numpy as np
            ranges = np.array(self.scan_data.ranges)
            valid = ranges[
                (ranges > self.scan_data.range_min) &
                (ranges < self.scan_data.range_max)
            ]
            if len(valid) > 0:
                nearest_obstacle = float(np.min(valid))

        # Ground contact from F/T
        on_ground = False
        if self.left_ft and self.right_ft:
            total_fz = (
                abs(self.left_ft.wrench.force.z) +
                abs(self.right_ft.wrench.force.z)
            )
            on_ground = total_fz > 20.0  # At least 20N total contact force

        # Log fused state
        tilt_deg = math.degrees(math.sqrt(pitch ** 2 + roll ** 2))
        self.get_logger().info(
            f'Tilt: {tilt_deg:.1f}deg | '
            f'Nearest: {nearest_obstacle:.2f}m | '
            f'Ground: {"YES" if on_ground else "NO"}'
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SimpleSensorFusion())
    rclpy.shutdown()
```

---

## Exercises

### Exercise 1: Camera Pipeline
Add an RGB and depth camera to your humanoid's head in Gazebo. Write a ROS 2 node that subscribes to both streams, converts the depth image to a colorized visualization, and publishes it as a new image topic. Verify in RViz that the color and depth images are aligned.

### Exercise 2: LiDAR Obstacle Map
Configure a 360-degree 2D LiDAR on your humanoid. Write a node that processes `LaserScan` data to create a simple occupancy grid — mark cells as free, occupied, or unknown. Publish this grid on a `nav_msgs/OccupancyGrid` topic and visualize it in RViz.

### Exercise 3: Noise Characterization
Run your simulated IMU with three different noise configurations: no noise, moderate noise (datasheet values), and extreme noise (10x datasheet). Record 60 seconds of stationary data for each. Plot the gyroscope readings over time and compute the drift in estimated orientation. Document how noise levels affect the balance controller's performance.

---

:::tip Key Takeaway
Simulated sensors are the bridge between your algorithms and physical hardware. The key to successful sim-to-real transfer is **accurate noise modeling** — match your simulation noise parameters to real sensor datasheets, and your perception code will transfer directly. Always develop with noise enabled, never with perfect sensors, because the real world is never perfect.
:::
