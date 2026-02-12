---
sidebar_position: 2
---

# Isaac Sim & Synthetic Data Generation

**Photorealistic Worlds, Parallel Training, and Automated Dataset Pipelines**

---

## Overview

Isaac Sim is NVIDIA's flagship robotics simulator, built on the Omniverse platform. It combines PhysX 5 GPU-accelerated physics with real-time ray-traced rendering to produce simulation environments that are visually indistinguishable from reality. In this chapter, you will load your humanoid URDF into Isaac Sim, build photorealistic training environments, run thousands of parallel simulations, and use Replicator to generate labeled datasets that train your robot's perception models.

## Understanding USD: The Scene Format

Isaac Sim uses **Universal Scene Description (USD)**, originally developed by Pixar. USD is to 3D scenes what Git is to source code — a composable, layerable, versionable format.

### USD Key Concepts

| Concept | Description | Robotics Example |
|---------|-------------|-----------------|
| **Prim** | Any object in the scene | A robot link, a table, a light |
| **Stage** | The complete scene | Your simulation world |
| **Layer** | An overlay of changes | Physics properties on top of visual scene |
| **Reference** | Reusable asset link | Same robot model in 100 environments |
| **Variant** | Switchable alternatives | Same hallway with different wall textures |

### Why USD Matters for Robotics

```
┌──────────────────────────────────────────────┐
│              USD COMPOSITION                  │
│                                                │
│   base_world.usd          (floor, walls, lights)
│       ↓ reference                              │
│   robot.usd               (humanoid model)     │
│       ↓ reference                              │
│   furniture_v1.usd        (tables, chairs)     │
│       ↓ variant                                │
│   furniture_v2.usd        (different layout)   │
│       ↓ layer                                  │
│   physics_override.usd    (friction, mass)     │
│                                                │
│   = Complete training scene, composed from     │
│     reusable, versioned pieces                 │
└──────────────────────────────────────────────┘
```

You build a scene once, then create thousands of variations by swapping layers and variants — without duplicating data.

## Loading Your Humanoid into Isaac Sim

### URDF to USD Conversion

Isaac Sim converts URDF files to USD format for use in Omniverse:

```python
# scripts/convert_urdf_to_usd.py
"""Convert a humanoid URDF to USD for Isaac Sim."""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.kit.commands
from pxr import UsdPhysics, Gf

# Import the URDF
status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/home/user/ros2_ws/src/humanoid_description/urdf/humanoid.urdf",
    import_config={
        "merge_fixed_joints": False,
        "fix_base": False,              # Humanoid is free-standing
        "import_inertia_tensor": True,
        "distance_scale": 1.0,
        "density": 0.0,                 # Use URDF-specified masses
        "default_drive_type": "position",
        "default_drive_strength": 1e4,
        "default_position_drive_damping": 1e3,
        "self_collision": False,
        "create_physics_scene": True,
    },
)

print(f"Robot imported at: {prim_path}")

# Save as USD
stage = omni.usd.get_context().get_stage()
stage.Export("/home/user/isaac_assets/humanoid.usd")

simulation_app.close()
```

### Interactive Import via GUI

1. Open Isaac Sim
2. Go to **Isaac Utils → URDF Importer**
3. Select your URDF file
4. Configure import settings:
   - **Fix Base Link**: Unchecked (humanoid is free-standing)
   - **Joint Drive Type**: Position
   - **Self Collision**: Enabled for adjacent links

### Verifying the Import

After import, check that all joints are functional:

```python
# scripts/verify_humanoid.py
"""Verify humanoid joint articulation in Isaac Sim."""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import numpy as np

world = World(stage_units_in_meters=1.0)

# Load the humanoid
world.scene.add_default_ground_plane()
robot = world.scene.add(
    Robot(
        prim_path="/World/humanoid",
        usd_path="/home/user/isaac_assets/humanoid.usd",
        name="humanoid",
        position=np.array([0.0, 0.0, 1.05]),
    )
)

world.reset()

# Print all joint names and limits
joint_names = robot.dof_names
print(f"Found {len(joint_names)} joints:")
for i, name in enumerate(joint_names):
    lower = robot.dof_properties["lower"][i]
    upper = robot.dof_properties["upper"][i]
    print(f"  {name}: [{np.degrees(lower):.1f}, {np.degrees(upper):.1f}] deg")

# Run simulation for 5 seconds
for _ in range(5000):
    world.step(render=True)

simulation_app.close()
```

## Building Photorealistic Environments

### Creating a Hospital Corridor

A common deployment environment for humanoid robots — hospitals, offices, and retail spaces:

```python
# scripts/build_hospital_world.py
"""Build a photorealistic hospital corridor in Isaac Sim."""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom, UsdShade, Sdf, Gf

stage = omni.usd.get_context().get_stage()

# Create the corridor structure
def create_corridor(length=20.0, width=3.0, height=3.0):
    """Build a corridor with floor, walls, and ceiling."""

    # Floor
    floor = UsdGeom.Mesh.Define(stage, "/World/corridor/floor")
    floor.CreatePointsAttr([
        Gf.Vec3f(0, 0, 0), Gf.Vec3f(length, 0, 0),
        Gf.Vec3f(length, width, 0), Gf.Vec3f(0, width, 0),
    ])
    floor.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    floor.CreateFaceVertexCountsAttr([4])

    # Left wall
    left_wall = UsdGeom.Mesh.Define(stage, "/World/corridor/left_wall")
    left_wall.CreatePointsAttr([
        Gf.Vec3f(0, 0, 0), Gf.Vec3f(length, 0, 0),
        Gf.Vec3f(length, 0, height), Gf.Vec3f(0, 0, height),
    ])
    left_wall.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    left_wall.CreateFaceVertexCountsAttr([4])

    # Right wall
    right_wall = UsdGeom.Mesh.Define(stage, "/World/corridor/right_wall")
    right_wall.CreatePointsAttr([
        Gf.Vec3f(0, width, 0), Gf.Vec3f(length, width, 0),
        Gf.Vec3f(length, width, height), Gf.Vec3f(0, width, height),
    ])
    right_wall.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    right_wall.CreateFaceVertexCountsAttr([4])

create_corridor()

# Add fluorescent ceiling lights
for i in range(5):
    light_path = f"/World/corridor/light_{i}"
    light = UsdGeom.RectLight.Define(stage, light_path) if hasattr(UsdGeom, 'RectLight') else None
    # Fallback to basic lighting
    from pxr import UsdLux
    rect_light = UsdLux.RectLight.Define(stage, light_path)
    rect_light.CreateWidthAttr(1.5)
    rect_light.CreateHeightAttr(0.3)
    rect_light.CreateIntensityAttr(5000)
    rect_light.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))  # Cool white
    UsdGeom.Xformable(rect_light).AddTranslateOp().Set(
        Gf.Vec3d(2.0 + i * 4.0, 1.5, 2.9)
    )

# Save the world
stage.Export("/home/user/isaac_assets/hospital_corridor.usd")
print("Hospital corridor world created")

simulation_app.close()
```

### Adding Photorealistic Materials

Isaac Sim includes NVIDIA's MDL material library with thousands of physically-based materials:

```python
# scripts/apply_materials.py
"""Apply photorealistic materials to the corridor."""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from omni.isaac.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()

# Apply materials from NVIDIA asset library
material_assignments = {
    "/World/corridor/floor": f"{assets_root}/NVIDIA/Materials/vMaterials_2/Floor/VinylTile_Grey.mdl",
    "/World/corridor/left_wall": f"{assets_root}/NVIDIA/Materials/vMaterials_2/Paint/Plaster_White.mdl",
    "/World/corridor/right_wall": f"{assets_root}/NVIDIA/Materials/vMaterials_2/Paint/Plaster_White.mdl",
}

for prim_path, material_path in material_assignments.items():
    omni.kit.commands.execute(
        "CreateMdlMaterialPrim",
        mtl_url=material_path,
        mtl_name="material",
        mtl_path=f"{prim_path}/material",
    )
    omni.kit.commands.execute(
        "BindMaterial",
        prim_path=prim_path,
        material_path=f"{prim_path}/material",
    )

print("Materials applied")
simulation_app.close()
```

## Parallel Environments with Isaac Gym

For reinforcement learning, run thousands of environments simultaneously:

```python
# scripts/parallel_humanoid_training.py
"""Run 1024 parallel humanoid environments for RL training."""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.gym.vec_env import VecEnvBase
from omni.isaac.core.robots import Robot
import numpy as np
import torch


class HumanoidStandingEnv(VecEnvBase):
    """Humanoid standing balance task with 1024 parallel envs."""

    def __init__(self, num_envs=1024, env_spacing=3.0):
        super().__init__(num_envs=num_envs, env_spacing=env_spacing)

    def set_up_scene(self):
        super().set_up_scene()

        # Add humanoid to each environment
        self._robots = self._scene.add(
            Robot(
                prim_path="/World/envs/env_0/humanoid",
                usd_path="/home/user/isaac_assets/humanoid.usd",
                name="humanoid",
                translation=np.array([0.0, 0.0, 1.05]),
            )
        )

        # Clone across all environments
        self.clone_environments()

    def get_observations(self) -> dict:
        """Observe joint positions, velocities, and IMU data."""
        joint_positions = self._robots.get_joint_positions()
        joint_velocities = self._robots.get_joint_velocities()
        root_orientations = self._robots.get_world_poses()[1]  # Quaternions

        return {
            "joint_pos": joint_positions,
            "joint_vel": joint_velocities,
            "orientation": root_orientations,
        }

    def compute_reward(self, obs: dict) -> torch.Tensor:
        """Reward for staying upright."""
        # Extract z-component of up vector from quaternion
        quat = obs["orientation"]
        # For a unit quaternion, the z-component of the rotated up vector is:
        # 1 - 2*(qx^2 + qy^2)
        upright = 1.0 - 2.0 * (quat[:, 0] ** 2 + quat[:, 1] ** 2)

        # Penalize large joint velocities (energy efficiency)
        velocity_penalty = -0.01 * torch.sum(obs["joint_vel"] ** 2, dim=1)

        # Reward: stay upright with minimal movement
        reward = upright + velocity_penalty

        return reward

    def is_done(self, obs: dict) -> torch.Tensor:
        """Episode ends if robot falls below height threshold."""
        heights = self._robots.get_world_poses()[0][:, 2]  # Z position
        fallen = heights < 0.5  # Below 0.5m means fallen
        return fallen

    def pre_physics_step(self, actions: torch.Tensor):
        """Apply joint position targets from the policy."""
        self._robots.set_joint_position_targets(actions)


# Training loop
env = HumanoidStandingEnv(num_envs=1024)
env.set_up_scene()

print(f"Running {env.num_envs} parallel environments")
print(f"Observation space: {env.get_observations()}")

for step in range(10000):
    # Random actions for demonstration (replace with RL policy)
    actions = torch.zeros(1024, env._robots.num_dof)
    env.pre_physics_step(actions)
    env.world.step(render=False)

    obs = env.get_observations()
    rewards = env.compute_reward(obs)
    dones = env.is_done(obs)

    if step % 1000 == 0:
        print(f"Step {step}: mean reward={rewards.mean():.3f}, "
              f"fallen={dones.sum()}/{env.num_envs}")

simulation_app.close()
```

### Training Performance Comparison

| Setup | Environments | Steps/Second | Time for 1M Steps |
|-------|-------------|-------------|-------------------|
| Gazebo (CPU) | 1 | ~1,000 | ~17 minutes |
| Isaac Sim (RTX 3070) | 256 | ~50,000 | ~20 seconds |
| Isaac Sim (RTX 4090) | 4,096 | ~500,000 | ~2 seconds |
| Isaac Sim (A100 cloud) | 8,192 | ~1,000,000 | ~1 second |

This is why Isaac Sim is essential for training humanoid locomotion policies — the scale difference is three orders of magnitude.

## Synthetic Data Generation with Replicator

NVIDIA Replicator automates the creation of labeled datasets for training perception models.

### Setting Up a Data Generation Pipeline

```python
# scripts/generate_synthetic_dataset.py
"""Generate labeled synthetic data using NVIDIA Replicator."""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from omni.isaac.core import World
import numpy as np

world = World()

# Load the hospital corridor scene
omni.usd.get_context().open_stage("/home/user/isaac_assets/hospital_corridor.usd")

# Define randomization groups
with rep.new_layer():

    # Camera placement — varies viewpoint for each frame
    camera = rep.create.camera(
        position=rep.distribution.uniform(
            lower=(1.0, 0.5, 1.2),
            upper=(18.0, 2.5, 1.8)
        ),
        look_at="/World/humanoid/base_link",
    )

    # Lighting randomization
    lights = rep.get.prims(path_pattern="/World/corridor/light_*")
    with lights:
        rep.modify.attribute("intensity", rep.distribution.uniform(2000, 8000))
        rep.modify.attribute(
            "color",
            rep.distribution.uniform(
                lower=(0.85, 0.85, 0.90),
                upper=(1.0, 1.0, 1.0)
            ),
        )

    # Scatter distractor objects (chairs, carts, signs)
    distractors = rep.create.from_usd(
        usd_paths=[
            "/home/user/isaac_assets/props/office_chair.usd",
            "/home/user/isaac_assets/props/medical_cart.usd",
            "/home/user/isaac_assets/props/sign_board.usd",
        ],
        count=rep.distribution.choice([2, 3, 4, 5]),
    )
    with distractors:
        rep.modify.pose(
            position=rep.distribution.uniform(
                lower=(0.5, 0.2, 0.0),
                upper=(19.0, 2.8, 0.0)
            ),
            rotation=rep.distribution.uniform(
                lower=(0, 0, 0),
                upper=(0, 0, 360)
            ),
        )

    # Floor material randomization
    floor = rep.get.prim_at_path("/World/corridor/floor")
    with floor:
        rep.randomizer.materials(
            materials=[
                "/NVIDIA/Materials/vMaterials_2/Floor/VinylTile_Grey.mdl",
                "/NVIDIA/Materials/vMaterials_2/Floor/VinylTile_Beige.mdl",
                "/NVIDIA/Materials/vMaterials_2/Floor/Linoleum_Blue.mdl",
            ]
        )

# Configure output — annotators define what labels to capture
render_product = rep.create.render_product(camera, (640, 480))

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/home/user/datasets/humanoid_perception",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    depth=True,
    instance_segmentation=True,
)
writer.attach([render_product])

# Generate 5000 frames
rep.orchestrator.run_until_complete(num_frames=5000)

print("Dataset generation complete: 5000 labeled frames")
simulation_app.close()
```

### Replicator Output Structure

```
humanoid_perception/
├── rgb/
│   ├── frame_000000.png
│   ├── frame_000001.png
│   └── ...
├── bounding_box_2d_tight/
│   ├── frame_000000.npy      # [class_id, x_min, y_min, x_max, y_max]
│   └── ...
├── semantic_segmentation/
│   ├── frame_000000.png      # Per-pixel class labels
│   └── ...
├── depth/
│   ├── frame_000000.npy      # Float32 depth in meters
│   └── ...
├── instance_segmentation/
│   ├── frame_000000.png      # Per-pixel instance IDs
│   └── ...
└── metadata.json              # Camera intrinsics, class definitions
```

### Supported Annotation Types

| Annotation | Format | Use Case |
|-----------|--------|----------|
| **RGB** | PNG | Input images for any vision model |
| **Bounding Box 2D** | NumPy array | Object detection (YOLO, SSD, DETR) |
| **Semantic Segmentation** | PNG mask | Per-pixel classification |
| **Instance Segmentation** | PNG mask | Distinguishing individual objects |
| **Depth** | Float32 NumPy | 3D reconstruction, obstacle distance |
| **3D Bounding Box** | NumPy array | 3D object detection |
| **Skeleton/Keypoints** | JSON | Human pose estimation |
| **Normals** | Float32 NumPy | Surface orientation analysis |
| **Optical Flow** | Float32 NumPy | Motion estimation between frames |

## Connecting Isaac Sim to ROS 2

### The OmniGraph ROS 2 Bridge

Isaac Sim connects to ROS 2 through OmniGraph action graphs:

```python
# scripts/isaac_sim_ros2_bridge.py
"""Set up ROS 2 bridge for Isaac Sim humanoid."""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import numpy as np

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load humanoid
robot = world.scene.add(
    Robot(
        prim_path="/World/humanoid",
        usd_path="/home/user/isaac_assets/humanoid.usd",
        name="humanoid",
        position=np.array([0.0, 0.0, 1.05]),
    )
)

# Create ROS 2 joint state publisher graph
og.Controller.edit(
    {"graph_path": "/ROS2JointStateGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishJointState.inputs:targetPrim", "/World/humanoid"),
            ("PublishJointState.inputs:topicName", "/joint_states"),
        ],
    },
)

# Create ROS 2 camera publisher graph
og.Controller.edit(
    {"graph_path": "/ROS2CameraGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
            ("PublishRGB", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishRGB.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "PublishRGB.inputs:renderProductPath"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("CreateRenderProduct.inputs:cameraPrim", "/World/humanoid/head_link/camera"),
            ("CreateRenderProduct.inputs:width", 640),
            ("CreateRenderProduct.inputs:height", 480),
            ("PublishRGB.inputs:topicName", "/humanoid/camera/image_raw"),
            ("PublishRGB.inputs:type", "rgb"),
            ("PublishRGB.inputs:frameId", "camera_link"),
        ],
    },
)

print("ROS 2 bridge configured — publishing /joint_states and /humanoid/camera/image_raw")

world.reset()
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

### Verifying the ROS 2 Connection

```bash
# In a separate terminal, verify topics are publishing
ros2 topic list
# /joint_states
# /humanoid/camera/image_raw
# /humanoid/camera/camera_info

ros2 topic hz /joint_states
# average rate: 60.0 Hz

ros2 topic hz /humanoid/camera/image_raw
# average rate: 30.0 Hz

# View the camera stream
ros2 run rqt_image_view rqt_image_view /humanoid/camera/image_raw
```

## Domain Randomization at Scale

### Structured Randomization Strategy

Effective domain randomization is not random — it is systematically structured:

| Level | What Changes | Purpose |
|-------|-------------|---------|
| **Visual** | Textures, colors, lighting | Camera robustness |
| **Geometric** | Object positions, sizes, shapes | Spatial perception |
| **Physical** | Friction, mass, damping | Control robustness |
| **Sensor** | Noise levels, camera intrinsics | Sensor model transfer |
| **Scenario** | Number of people, object types | Behavioral diversity |

### Implementation Pattern

```python
# scripts/structured_randomization.py
"""Structured domain randomization for humanoid training."""
import omni.replicator.core as rep
import random


def randomize_visual():
    """Randomize visual appearance — textures, lighting, colors."""
    with rep.get.prims(semantics=[("class", "wall")]):
        rep.randomizer.materials(
            materials=rep.distribution.choice([
                "/Materials/Paint_White.mdl",
                "/Materials/Paint_Cream.mdl",
                "/Materials/Wallpaper_Stripe.mdl",
            ])
        )

    with rep.get.prims(semantics=[("class", "light")]):
        rep.modify.attribute("intensity", rep.distribution.normal(5000, 1500))
        rep.modify.attribute(
            "color",
            rep.distribution.uniform((0.8, 0.8, 0.9), (1.0, 1.0, 1.0))
        )


def randomize_geometric():
    """Randomize object placement and scene geometry."""
    with rep.get.prims(semantics=[("class", "obstacle")]):
        rep.modify.pose(
            position=rep.distribution.uniform((-5, -3, 0), (5, 3, 0)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
        )


def randomize_physical():
    """Randomize physics properties for control robustness."""
    # Applied through Isaac Sim APIs, not Replicator
    friction_values = [0.6, 0.8, 1.0, 1.2, 1.5]
    mass_scale = random.uniform(0.9, 1.1)  # ±10% mass variation
    return {
        "floor_friction": random.choice(friction_values),
        "mass_scale": mass_scale,
    }


# Compose randomizations
with rep.trigger.on_frame():
    randomize_visual()
    randomize_geometric()
```

---

## Exercises

### Exercise 1: URDF to USD Pipeline
Convert your humanoid URDF to USD using the Isaac Sim URDF importer. Verify that all joints, masses, and collision geometries transferred correctly. Compare joint behavior in Gazebo vs Isaac Sim by commanding the same standing pose in both simulators.

### Exercise 2: Photorealistic Environment
Build a hospital corridor environment in Isaac Sim with PBR materials, fluorescent lighting, and at least 5 props (chairs, carts, signs). Capture 10 screenshots with different camera angles and compare visual quality against your Gazebo world.

### Exercise 3: Synthetic Dataset Generation
Use Replicator to generate a 1,000-image dataset of the humanoid robot in the hospital corridor. Include RGB images, 2D bounding boxes, and semantic segmentation labels. Apply domain randomization to lighting (3 levels), floor texture (3 variants), and object placement (random per frame). Verify label accuracy on 20 randomly sampled frames.

---

:::tip Key Takeaway
Isaac Sim closes the two biggest gaps in robotics simulation: **visual realism** and **training scale**. Replicator automates what would take weeks of manual labeling, and GPU-parallel environments compress months of training into hours. The investment in learning USD and the Omniverse ecosystem pays off when your perception models transfer to real cameras with minimal fine-tuning.
:::
