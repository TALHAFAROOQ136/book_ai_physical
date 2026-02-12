---
sidebar_position: 3
---

# Isaac ROS & Visual SLAM

**GPU-Accelerated Perception — From Raw Pixels to Real-Time Localization**

---

## Overview

Your humanoid robot has cameras, LiDAR, and IMUs streaming data at high rates. Processing this data fast enough for real-time control is impossible on CPU alone. Isaac ROS is NVIDIA's collection of GPU-accelerated ROS 2 packages that run on Jetson or discrete NVIDIA GPUs, delivering 10-50x speedups over CPU equivalents. In this chapter, you will deploy cuVSLAM for visual SLAM, accelerate your camera pipeline, run DNN inference for object detection, and integrate everything into a unified perception stack.

## The Isaac ROS Architecture

Isaac ROS packages are **drop-in replacements** for standard ROS 2 perception packages. They subscribe to the same topics and publish the same message types — the only difference is speed.

```
┌─────────────────────────────────────────────────────────┐
│                    ROBOT PERCEPTION STACK                 │
│                                                           │
│   Cameras ──► /image_raw ──┬──► isaac_ros_image_pipeline │
│                             │      (GPU debayer/rectify)  │
│                             │              │               │
│                             │              ▼               │
│                             ├──► isaac_ros_visual_slam    │
│                             │      (cuVSLAM on GPU)       │
│                             │              │               │
│                             │              ▼               │
│                             │      /visual_slam/tracking  │
│                             │      /tf (odom → base)      │
│                             │                              │
│                             └──► isaac_ros_dnn_inference  │
│                                    (TensorRT on GPU)      │
│                                           │                │
│                                           ▼                │
│                                   /detections2d            │
│                                                           │
│   LiDAR ──► /scan ──► Nav2 costmap (CPU)                 │
│   IMU ──► /imu ──► EKF sensor fusion (CPU)               │
└─────────────────────────────────────────────────────────┘
```

## GPU-Accelerated Image Pipeline

### The Problem with CPU Image Processing

A stereo camera produces raw Bayer-pattern images that must be:

1. **Debayered** — Converting Bayer mosaic to full RGB
2. **Rectified** — Removing lens distortion
3. **Resized** — Scaling for different consumers (SLAM needs 640x480, detection needs 300x300)

On CPU, this takes **15-30ms per frame** for a 1280x720 stereo pair. At 30 fps, image processing alone consumes 45-90% of a CPU core.

### Isaac ROS Image Pipeline

```bash
# Launch the GPU-accelerated image pipeline
ros2 launch isaac_ros_image_pipeline isaac_ros_image_pipeline.launch.py \
  camera_namespace:=/humanoid/camera \
  image_width:=1280 \
  image_height:=720
```

The launch file chains three GPU-accelerated nodes:

```python
# launch/image_pipeline.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='image_pipeline_container',
        namespace='/humanoid',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # GPU debayer
            ComposableNode(
                package='isaac_ros_image_pipeline',
                plugin='nvidia::isaac_ros::image_pipeline::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', '/humanoid/camera/image_raw'),
                    ('camera_info', '/humanoid/camera/camera_info'),
                    ('image_rect', '/humanoid/camera/image_rect'),
                ],
            ),
            # GPU resize for DNN input
            ComposableNode(
                package='isaac_ros_image_pipeline',
                plugin='nvidia::isaac_ros::image_pipeline::ResizeNode',
                name='resize_node',
                parameters=[{
                    'output_width': 300,
                    'output_height': 300,
                    'keep_aspect_ratio': False,
                }],
                remappings=[
                    ('image', '/humanoid/camera/image_rect'),
                    ('camera_info', '/humanoid/camera/camera_info'),
                    ('resize/image', '/humanoid/camera/image_resized'),
                    ('resize/camera_info', '/humanoid/camera/camera_info_resized'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

### Performance Comparison

| Operation | CPU (i7-12700) | Jetson Orin NX (GPU) | Speedup |
|-----------|---------------|---------------------|---------|
| Debayer (1280x720) | 8.2 ms | 0.3 ms | 27x |
| Rectify (1280x720) | 6.5 ms | 0.4 ms | 16x |
| Resize (1280x720 → 300x300) | 2.1 ms | 0.1 ms | 21x |
| **Total pipeline** | **16.8 ms** | **0.8 ms** | **21x** |

GPU processing frees the CPU entirely for control and planning.

## Visual SLAM with cuVSLAM

### What is Visual SLAM?

SLAM (Simultaneous Localization and Mapping) answers two questions simultaneously:

1. **Where am I?** — Localization within the environment
2. **What does the environment look like?** — Building a map

Visual SLAM uses camera images (instead of LiDAR) to achieve both. cuVSLAM is NVIDIA's GPU-accelerated implementation that runs in real time on Jetson.

```
┌──────────────────────────────────────────────────┐
│                    cuVSLAM PIPELINE               │
│                                                    │
│  Stereo Images ──► Feature Extraction (GPU)       │
│                          │                         │
│                          ▼                         │
│                    Feature Matching (GPU)          │
│                          │                         │
│                          ▼                         │
│                    Pose Estimation (GPU)           │
│                          │                         │
│                    ┌─────┴─────┐                   │
│                    ▼           ▼                    │
│               /visual_slam  3D Map                 │
│               /tracking/    (Landmarks)            │
│               odometry                             │
│                                                    │
│  Outputs:                                         │
│  • 6-DOF pose at camera frame rate (30+ Hz)       │
│  • Visual odometry (incremental motion)           │
│  • Loop closure (corrects accumulated drift)      │
│  • 3D landmark map                                 │
└──────────────────────────────────────────────────┘
```

### Launching cuVSLAM

```python
# launch/visual_slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # Camera configuration
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_image_denoising': False,

            # SLAM parameters
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,

            # Performance tuning
            'num_cameras': 2,  # Stereo
            'min_num_images': 2,
            'enable_localization_n_mapping': True,
            'enable_imu_fusion': True,  # Fuse with IMU for better accuracy

            # Output frames
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_optical_frames': [
                'left_camera_optical_frame',
                'right_camera_optical_frame',
            ],
        }],
        remappings=[
            ('stereo_camera/left/image', '/humanoid/left_camera/image_rect'),
            ('stereo_camera/left/camera_info', '/humanoid/left_camera/camera_info'),
            ('stereo_camera/right/image', '/humanoid/right_camera/image_rect'),
            ('stereo_camera/right/camera_info', '/humanoid/right_camera/camera_info'),
            ('visual_slam/imu', '/humanoid/imu/data'),
        ],
    )

    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return LaunchDescription([container])
```

### Verifying SLAM Output

```bash
# Launch SLAM
ros2 launch humanoid_perception visual_slam.launch.py

# Check the pose output
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose
# position:
#   x: 1.234
#   y: 0.567
#   z: 1.050
# orientation:
#   x: 0.001
#   y: 0.003
#   z: 0.012
#   w: 0.999

# Verify TF tree
ros2 run tf2_tools view_frames
# Should show: map → odom → base_link chain

# Check frame rate
ros2 topic hz /visual_slam/tracking/odometry
# average rate: 30.0 Hz
```

### Integrating SLAM with the Robot's TF Tree

cuVSLAM publishes the transform from `odom` to `base_link`. Connect this with your robot's existing URDF transforms:

```python
# humanoid_perception/slam_tf_bridge.py
"""Bridge cuVSLAM odometry into the full robot TF tree."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SlamTfBridge(Node):
    def __init__(self):
        super().__init__('slam_tf_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.on_odom,
            10,
        )
        self.get_logger().info('SLAM TF bridge active')

    def on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlamTfBridge())
    rclpy.shutdown()
```

## DNN Inference with TensorRT

### Object Detection Pipeline

Isaac ROS runs neural networks through TensorRT — NVIDIA's inference optimizer that can be 5-20x faster than CPU frameworks:

```python
# launch/object_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='detection_container',
        namespace='/humanoid',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Encoder — preprocess image for the DNN
            ComposableNode(
                package='isaac_ros_dnn_image_encoder',
                plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                name='dnn_image_encoder',
                parameters=[{
                    'input_image_width': 640,
                    'input_image_height': 480,
                    'network_image_width': 300,
                    'network_image_height': 300,
                    'image_mean': [0.485, 0.456, 0.406],
                    'image_stddev': [0.229, 0.224, 0.225],
                }],
                remappings=[
                    ('image', '/humanoid/camera/image_rect'),
                    ('encoded_tensor', 'tensor_pub'),
                ],
            ),
            # TensorRT inference
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt_node',
                parameters=[{
                    'model_file_path': '/home/user/models/ssd_mobilenet_v2.onnx',
                    'engine_file_path': '/home/user/models/ssd_mobilenet_v2.engine',
                    'input_tensor_names': ['input'],
                    'input_binding_names': ['input'],
                    'output_tensor_names': ['scores', 'boxes'],
                    'output_binding_names': ['scores', 'boxes'],
                    'force_engine_update': False,
                    'verbose': False,
                }],
            ),
            # Decoder — convert tensors to detection messages
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetDecoderNode',
                name='detectnet_decoder',
                parameters=[{
                    'label_list': ['person', 'chair', 'table', 'door', 'robot'],
                    'confidence_threshold': 0.5,
                }],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

### Processing Detections in Your Nodes

```python
# humanoid_perception/detection_processor.py
"""Process object detections for humanoid awareness."""
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class DetectionProcessor(Node):
    def __init__(self):
        super().__init__('detection_processor')

        self.create_subscription(
            Detection2DArray,
            '/humanoid/detectnet/detections',
            self.on_detections,
            10,
        )
        self.get_logger().info('Detection processor started')

    def on_detections(self, msg: Detection2DArray):
        for detection in msg.detections:
            for result in detection.results:
                class_id = result.hypothesis.class_id
                score = result.hypothesis.score
                bbox = detection.bbox

                self.get_logger().info(
                    f'Detected {class_id} (conf={score:.2f}) at '
                    f'center=({bbox.center.position.x:.0f}, '
                    f'{bbox.center.position.y:.0f}), '
                    f'size=({bbox.size_x:.0f}x{bbox.size_y:.0f})'
                )

                # Alert if a person is detected nearby (large bounding box)
                if class_id == 'person' and bbox.size_y > 200:
                    self.get_logger().warn(
                        'Person detected at close range — activating safety mode'
                    )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DetectionProcessor())
    rclpy.shutdown()
```

### Model Optimization for Jetson

Converting a model to TensorRT for maximum inference speed:

```bash
# Convert ONNX model to TensorRT engine (run on target Jetson)
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/user/models/ssd_mobilenet_v2.onnx \
  --saveEngine=/home/user/models/ssd_mobilenet_v2.engine \
  --fp16 \                  # Use FP16 precision (2x faster, minimal accuracy loss)
  --workspace=2048 \        # GPU workspace in MB
  --verbose

# Expected output:
# [TensorRT] Serialized engine: ssd_mobilenet_v2.engine
# [TensorRT] Latency: min=3.2ms, max=4.1ms, mean=3.5ms
```

| Precision | Latency (Jetson Orin NX) | Accuracy Impact |
|-----------|-------------------------|-----------------|
| FP32 | 8.5 ms | Baseline |
| FP16 | 3.5 ms | < 0.5% mAP drop |
| INT8 | 1.8 ms | 1-2% mAP drop (requires calibration) |

## AprilTag Detection

Fiducial markers provide ground-truth localization for calibrating and validating SLAM:

```python
# launch/apriltag_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='/humanoid',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag_node',
                parameters=[{
                    'size': 0.166,          # Tag size in meters
                    'max_tags': 16,
                    'tile_size': 4,         # GPU tile size for detection
                }],
                remappings=[
                    ('image', '/humanoid/camera/image_rect'),
                    ('camera_info', '/humanoid/camera/camera_info'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

```bash
# View detected tags
ros2 topic echo /humanoid/apriltag/detections
# family: 36h11
# id: 5
# center: {x: 320.5, y: 240.2}
# corners: [{x: 280, y: 200}, {x: 360, y: 200}, ...]
# pose: {position: {x: 1.5, y: 0.0, z: 0.0}, ...}
```

## The Complete Perception Launch

Bring everything together in a single launch file:

```python
# launch/humanoid_perception.launch.py
"""Complete Isaac ROS perception stack for the humanoid."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('humanoid_perception')

    return LaunchDescription([
        # GPU image pipeline
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'image_pipeline.launch.py')
            )
        ),

        # Visual SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'visual_slam.launch.py')
            )
        ),

        # Object detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'object_detection.launch.py')
            )
        ),

        # AprilTag detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'apriltag_detection.launch.py')
            )
        ),

        # SLAM TF bridge
        Node(
            package='humanoid_perception',
            executable='slam_tf_bridge',
            name='slam_tf_bridge',
            output='screen',
        ),

        # Detection processor
        Node(
            package='humanoid_perception',
            executable='detection_processor',
            name='detection_processor',
            output='screen',
        ),
    ])
```

```bash
# Launch the full perception stack
ros2 launch humanoid_perception humanoid_perception.launch.py

# Monitor GPU utilization
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv -l 1
# 45%, 3200 MiB  — leaving headroom for control and planning
```

---

## Exercises

### Exercise 1: Image Pipeline Benchmark
Launch the Isaac ROS image pipeline with your humanoid's camera. Measure the end-to-end latency from raw image capture to rectified output using ROS 2 message timestamps. Compare against a CPU-based pipeline using the standard `image_proc` package.

### Exercise 2: Visual SLAM Mapping
Run cuVSLAM while commanding your humanoid to walk a 10-meter loop in Gazebo or Isaac Sim. Save the generated map and trajectory. Verify loop closure by checking that the start and end positions match within 5 cm. Visualize the landmark map in RViz.

### Exercise 3: Detection and Response
Deploy the TensorRT object detection pipeline. Place 5 objects (person, chair, table, door, cup) in the simulation. Write a ROS 2 node that logs all detections, filters by confidence > 0.7, and publishes a warning when a person is detected within 2 meters (estimated from bounding box size).

---

:::tip Key Takeaway
Isaac ROS transforms your robot's perception from a CPU bottleneck into a GPU-accelerated pipeline. The critical insight is that these are **drop-in replacements** — your existing ROS 2 nodes do not change. You swap CPU image processing for GPU image processing, CPU SLAM for cuVSLAM, and CPU inference for TensorRT. The same topics, the same messages, but 10-50x faster.
:::
