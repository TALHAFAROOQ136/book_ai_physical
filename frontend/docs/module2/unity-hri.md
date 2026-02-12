---
sidebar_position: 3
---

# Unity for Human-Robot Interaction

**Photorealistic Simulation for Perception, Interaction, and Synthetic Data**

---

## Overview

Gazebo excels at physics accuracy, but when your humanoid robot needs to recognize human faces, interpret gestures, navigate visually complex environments, or generate training data for deep learning — you need photorealistic rendering. Unity is a real-time 3D engine used across gaming, film, and increasingly, robotics research. In this chapter, you will import your humanoid URDF into Unity, connect it to ROS 2, simulate human-robot interaction scenarios, and generate synthetic datasets for training perception models.

## Why Unity for Robotics?

Unity brings capabilities that traditional robotics simulators lack:

- **Photorealistic rendering** — High Definition Render Pipeline (HDRP) with ray tracing produces images nearly indistinguishable from real photographs
- **Human character animation** — Extensive animation systems and character controllers for simulating realistic human behavior
- **Asset ecosystem** — Thousands of pre-built environments, furniture, objects, and human models in the Unity Asset Store
- **GPU acceleration** — Leverages modern GPUs for real-time rendering and parallel environment execution
- **Cross-platform deployment** — Simulations run on Windows, Linux, macOS, and cloud instances

### Where Unity Fits in the Robotics Pipeline

```
┌──────────────────────────────────────────────────────────┐
│                SIMULATION STACK                           │
│                                                           │
│  Gazebo                          Unity                    │
│  ┌─────────────────┐            ┌─────────────────┐      │
│  │ Physics accuracy │            │ Visual fidelity  │     │
│  │ ROS 2 native     │            │ Human simulation │     │
│  │ Sensor models    │            │ Synthetic data   │     │
│  │ Control testing  │            │ HRI research     │     │
│  └─────────────────┘            └─────────────────┘      │
│           │                              │                │
│           └──────────┬───────────────────┘                │
│                      ▼                                    │
│              ROS 2 Message Layer                          │
│              (Same nodes, same code)                      │
└──────────────────────────────────────────────────────────┘
```

Your ROS 2 nodes do not care whether sensor data comes from Gazebo or Unity — the message interfaces are identical.

## Setting Up the Unity Robotics Environment

### Installing Required Packages

After creating a new Unity project with the **3D (HDRP)** template:

1. Open **Window → Package Manager**
2. Click **+ → Add package from git URL** and add:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

3. Add the URDF Importer:

```
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

### The ROS-TCP Bridge Architecture

Unity communicates with ROS 2 through a TCP bridge:

```
┌──────────┐    TCP     ┌──────────────┐    ROS 2    ┌──────────┐
│  Unity   │◄─────────►│  ROS-TCP-    │◄───────────►│  ROS 2   │
│  Scene   │  Messages  │  Endpoint    │   Topics    │  Nodes   │
└──────────┘            └──────────────┘             └──────────┘
```

Start the ROS 2 endpoint node:

```bash
# Install the ROS-TCP-Endpoint package
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b ROS2v0.7.0

cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

# Launch the endpoint (default port 10000)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

In Unity, configure the connection:

1. Go to **Robotics → ROS Settings**
2. Set **ROS IP Address** to `127.0.0.1` (or your ROS machine IP)
3. Set **ROS Port** to `10000`

## Importing Your Humanoid URDF

### Loading the Robot Model

Unity can directly import URDF files and convert them into a fully articulated Unity GameObject hierarchy:

1. Copy your humanoid URDF and mesh files into `Assets/URDF/`
2. Right-click the `.urdf` file → **Import Robot from URDF**
3. In the import dialog:
   - **Axis Type**: Y Axis (Unity uses Y-up, ROS uses Z-up — the importer handles conversion)
   - **Mesh Decomposer**: VHACD (for accurate convex collision meshes)

### The Imported Hierarchy

After import, Unity creates a GameObject hierarchy matching your URDF:

```
humanoid (root)
├── base_link
│   ├── torso_link
│   │   ├── left_shoulder_link
│   │   │   └── left_elbow_link
│   │   │       └── left_wrist_link
│   │   ├── right_shoulder_link
│   │   │   └── right_elbow_link
│   │   │       └── right_wrist_link
│   │   └── head_link
│   │       └── camera_link
│   ├── left_hip_link
│   │   └── left_knee_link
│   │       └── left_ankle_link
│   │           └── left_foot_link
│   └── right_hip_link
│       └── right_knee_link
│           └── right_ankle_link
│               └── right_foot_link
```

Each link becomes a GameObject with:
- **ArticulationBody** component for physics simulation
- **Mesh renderers** for visual geometry
- **Colliders** for collision detection

### Enhancing Visual Quality

The default URDF materials are plain. Upgrade them for photorealistic rendering:

```csharp
// Scripts/RobotMaterialSetup.cs
using UnityEngine;

public class RobotMaterialSetup : MonoBehaviour
{
    public Material robotBodyMaterial;    // Metallic PBR material
    public Material jointCoverMaterial;   // Rubber/plastic material
    public Material sensorHousingMaterial; // Matte black plastic

    void Start()
    {
        // Apply materials based on link naming conventions
        foreach (var renderer in GetComponentsInChildren<MeshRenderer>())
        {
            string name = renderer.gameObject.name.ToLower();

            if (name.Contains("camera") || name.Contains("sensor"))
                renderer.material = sensorHousingMaterial;
            else if (name.Contains("joint") || name.Contains("cover"))
                renderer.material = jointCoverMaterial;
            else
                renderer.material = robotBodyMaterial;
        }
    }
}
```

## Connecting Unity to ROS 2

### Publishing Joint States from Unity

Send the robot's current joint positions to ROS 2 so your existing nodes can process them:

```csharp
// Scripts/JointStatePublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System.Linq;

public class JointStatePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/joint_states";
    public float publishRate = 50f; // 50 Hz

    ArticulationBody[] joints;
    string[] jointNames;
    float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);

        // Collect all articulation joints (skip the root fixed joint)
        joints = GetComponentsInChildren<ArticulationBody>()
            .Where(ab => ab.jointType != ArticulationJointType.FixedJoint)
            .ToArray();

        jointNames = joints.Select(j => j.gameObject.name).ToArray();
    }

    void FixedUpdate()
    {
        timer += Time.fixedDeltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        var msg = new JointStateMsg
        {
            header = new HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "base_link"
            },
            name = jointNames,
            position = joints.Select(j => (double)j.jointPosition[0]).ToArray(),
            velocity = joints.Select(j => (double)j.jointVelocity[0]).ToArray(),
            effort = joints.Select(j => (double)j.jointForce[0]).ToArray()
        };

        ros.Publish(topicName, msg);
    }
}
```

### Subscribing to Joint Commands from ROS 2

Receive motion commands from your ROS 2 controllers and apply them in Unity:

```csharp
// Scripts/JointCommandSubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Trajectory;
using System.Collections.Generic;

public class JointCommandSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/position_controller/joint_trajectory";

    Dictionary<string, ArticulationBody> jointMap;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointTrajectoryMsg>(topicName, OnTrajectoryReceived);

        // Build a lookup from joint name to ArticulationBody
        jointMap = new Dictionary<string, ArticulationBody>();
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            jointMap[ab.gameObject.name] = ab;
        }
    }

    void OnTrajectoryReceived(JointTrajectoryMsg msg)
    {
        if (msg.points.Length == 0) return;

        // Apply the first trajectory point (simplification for demonstration)
        var point = msg.points[0];

        for (int i = 0; i < msg.joint_names.Length; i++)
        {
            string jointName = msg.joint_names[i];
            if (jointMap.TryGetValue(jointName, out var joint))
            {
                var drive = joint.xDrive;
                drive.target = (float)point.positions[i] * Mathf.Rad2Deg;
                joint.xDrive = drive;
            }
        }
    }
}
```

## Simulating Camera Sensors

### RGB Camera with ROS 2 Publishing

Attach a virtual camera to the robot's head and stream images to ROS 2:

```csharp
// Scripts/RosCameraPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class RosCameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public Camera robotCamera;
    public string topicName = "/humanoid/camera/image_raw";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float publishRate = 30f;

    RenderTexture renderTexture;
    Texture2D texture2D;
    float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        robotCamera.targetTexture = renderTexture;
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        // Capture the camera render
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        byte[] imageData = texture2D.GetRawTextureData();

        var msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "camera_link"
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(imageWidth * 3),
            data = imageData
        };

        ros.Publish(topicName, msg);
    }

    void OnDestroy()
    {
        if (renderTexture != null) renderTexture.Release();
    }
}
```

### Depth Camera

Unity's HDRP supports depth rendering for simulating RGB-D cameras:

```csharp
// Scripts/DepthCameraPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class DepthCameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public Camera depthCamera;
    public string topicName = "/humanoid/depth/image_raw";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float maxDepth = 10f;
    public float publishRate = 15f;

    RenderTexture depthTexture;
    Texture2D readTexture;
    float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Create depth render texture (single channel float)
        depthTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.Depth);
        depthCamera.targetTexture = depthTexture;
        depthCamera.depthTextureMode = DepthTextureMode.Depth;

        readTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        RenderTexture.active = depthTexture;
        readTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        readTexture.Apply();
        RenderTexture.active = null;

        byte[] depthData = readTexture.GetRawTextureData();

        var msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "depth_camera_link"
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "32FC1",
            is_bigendian = 0,
            step = (uint)(imageWidth * 4),
            data = depthData
        };

        ros.Publish(topicName, msg);
    }
}
```

## Simulating Human Characters

This is where Unity truly shines for robotics — simulating realistic human behavior for HRI research.

### Adding Human Characters

Unity's animation system (Mecanim) provides a framework for realistic human motion:

```csharp
// Scripts/SimulatedHuman.cs
using UnityEngine;
using UnityEngine.AI;

public class SimulatedHuman : MonoBehaviour
{
    public enum HumanBehavior { Idle, Walking, Waving, Approaching, Avoiding }

    Animator animator;
    NavMeshAgent navAgent;
    public HumanBehavior currentBehavior = HumanBehavior.Idle;

    public Transform robotTransform;
    public float interactionDistance = 2.0f;
    public float avoidanceDistance = 1.0f;

    void Start()
    {
        animator = GetComponent<Animator>();
        navAgent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        float distToRobot = Vector3.Distance(transform.position, robotTransform.position);

        switch (currentBehavior)
        {
            case HumanBehavior.Walking:
                animator.SetFloat("Speed", navAgent.velocity.magnitude);
                break;

            case HumanBehavior.Approaching:
                if (distToRobot > interactionDistance)
                {
                    navAgent.SetDestination(robotTransform.position);
                    animator.SetFloat("Speed", navAgent.velocity.magnitude);
                }
                else
                {
                    navAgent.ResetPath();
                    animator.SetFloat("Speed", 0f);
                    animator.SetTrigger("Wave");
                }
                break;

            case HumanBehavior.Avoiding:
                if (distToRobot < avoidanceDistance)
                {
                    Vector3 awayDir = (transform.position - robotTransform.position).normalized;
                    navAgent.SetDestination(transform.position + awayDir * 3f);
                }
                animator.SetFloat("Speed", navAgent.velocity.magnitude);
                break;
        }
    }

    public void SetBehavior(HumanBehavior behavior)
    {
        currentBehavior = behavior;
    }
}
```

### Publishing Human Pose to ROS 2

Your robot's perception system needs to track human positions and poses. Publish this from Unity:

```csharp
// Scripts/HumanPosePublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class HumanPosePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/detected_humans/pose";
    public float publishRate = 10f;

    float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        // Convert Unity coordinates (Y-up, left-handed)
        // to ROS coordinates (Z-up, right-handed)
        Vector3 pos = transform.position;
        Quaternion rot = transform.rotation;

        var msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "world"
            },
            pose = new PoseMsg
            {
                position = new PointMsg(pos.z, -pos.x, pos.y),
                orientation = new QuaternionMsg(-rot.z, rot.x, -rot.y, rot.w)
            }
        };

        ros.Publish(topicName, msg);
    }
}
```

## HRI Scenario Design

### Building an Interaction Scenario

A complete HRI scenario combines environment, human behavior, and robot response:

```csharp
// Scripts/HRIScenarioManager.cs
using UnityEngine;
using System.Collections;

public class HRIScenarioManager : MonoBehaviour
{
    public SimulatedHuman[] humans;
    public Transform robotTransform;
    public Transform[] waypoints;

    [Header("Scenario Settings")]
    public float scenarioDuration = 60f;
    public int humanCount = 3;

    void Start()
    {
        StartCoroutine(RunGreetingScenario());
    }

    IEnumerator RunGreetingScenario()
    {
        Debug.Log("=== HRI Scenario: Greeting in Lobby ===");

        // Phase 1: Humans walk around naturally
        foreach (var human in humans)
        {
            human.SetBehavior(SimulatedHuman.HumanBehavior.Walking);
        }
        yield return new WaitForSeconds(10f);

        // Phase 2: One human approaches the robot
        humans[0].SetBehavior(SimulatedHuman.HumanBehavior.Approaching);
        yield return new WaitForSeconds(15f);

        // Phase 3: Other humans continue walking, some avoid the robot
        humans[1].SetBehavior(SimulatedHuman.HumanBehavior.Avoiding);
        yield return new WaitForSeconds(15f);

        // Phase 4: Multiple humans approach simultaneously
        foreach (var human in humans)
        {
            human.SetBehavior(SimulatedHuman.HumanBehavior.Approaching);
        }
        yield return new WaitForSeconds(20f);

        Debug.Log("=== Scenario Complete ===");
    }
}
```

### Common HRI Scenarios

| Scenario | Description | Research Focus |
|----------|-------------|----------------|
| **Lobby Greeting** | Robot greets visitors in a building lobby | Social navigation, gesture recognition |
| **Collaborative Assembly** | Robot and human assemble an object together | Handover, shared workspace safety |
| **Guided Tour** | Robot leads a group through a space | Path planning, group dynamics |
| **Emergency Response** | Robot assists during building evacuation | Human detection, verbal communication |
| **Elderly Care** | Robot assists with daily tasks in a home | Object handover, fall detection |

## Generating Synthetic Training Data

### Domain Randomization

To train robust perception models, vary the environment systematically:

```csharp
// Scripts/DomainRandomizer.cs
using UnityEngine;

public class DomainRandomizer : MonoBehaviour
{
    [Header("Lighting")]
    public Light[] sceneLights;
    public float minIntensity = 0.3f;
    public float maxIntensity = 2.0f;

    [Header("Materials")]
    public Renderer[] wallRenderers;
    public Renderer[] floorRenderers;
    public Texture2D[] wallTextures;
    public Texture2D[] floorTextures;

    [Header("Object Placement")]
    public GameObject[] distractorObjects;
    public Bounds spawnArea;

    public void Randomize()
    {
        RandomizeLighting();
        RandomizeMaterials();
        RandomizeObjects();
    }

    void RandomizeLighting()
    {
        foreach (var light in sceneLights)
        {
            light.intensity = Random.Range(minIntensity, maxIntensity);
            light.color = Color.Lerp(Color.white, Random.ColorHSV(0.05f, 0.15f, 0.2f, 0.5f, 0.8f, 1f), 0.3f);
            light.transform.rotation = Quaternion.Euler(
                Random.Range(20f, 80f),
                Random.Range(0f, 360f),
                0f
            );
        }
    }

    void RandomizeMaterials()
    {
        foreach (var wall in wallRenderers)
        {
            if (wallTextures.Length > 0)
            {
                wall.material.mainTexture = wallTextures[Random.Range(0, wallTextures.Length)];
            }
            wall.material.color = Random.ColorHSV(0f, 1f, 0.1f, 0.4f, 0.5f, 1f);
        }

        foreach (var floor in floorRenderers)
        {
            if (floorTextures.Length > 0)
            {
                floor.material.mainTexture = floorTextures[Random.Range(0, floorTextures.Length)];
            }
        }
    }

    void RandomizeObjects()
    {
        foreach (var obj in distractorObjects)
        {
            obj.transform.position = new Vector3(
                Random.Range(spawnArea.min.x, spawnArea.max.x),
                0f,
                Random.Range(spawnArea.min.z, spawnArea.max.z)
            );
            obj.transform.rotation = Quaternion.Euler(0, Random.Range(0, 360), 0);
            obj.SetActive(Random.value > 0.3f);
        }
    }
}
```

### Automated Dataset Collection

Capture labeled images automatically for training object detection and human pose estimation:

```csharp
// Scripts/DatasetCollector.cs
using UnityEngine;
using System.IO;

public class DatasetCollector : MonoBehaviour
{
    public Camera captureCamera;
    public DomainRandomizer randomizer;
    public string outputPath = "Datasets/hri_training";

    public int totalSamples = 10000;
    public int imageWidth = 640;
    public int imageHeight = 480;

    int sampleCount = 0;
    RenderTexture renderTexture;
    Texture2D captureTexture;

    void Start()
    {
        Directory.CreateDirectory(outputPath);
        Directory.CreateDirectory(Path.Combine(outputPath, "images"));
        Directory.CreateDirectory(Path.Combine(outputPath, "labels"));

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        captureCamera.targetTexture = renderTexture;
        captureTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (sampleCount >= totalSamples) return;

        // Randomize environment every frame
        randomizer.Randomize();

        // Wait one frame for rendering to complete
        if (Time.frameCount % 2 == 0) return;

        CaptureFrame();
        sampleCount++;

        if (sampleCount % 100 == 0)
            Debug.Log($"Captured {sampleCount}/{totalSamples} samples");
    }

    void CaptureFrame()
    {
        // Capture image
        RenderTexture.active = renderTexture;
        captureTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        captureTexture.Apply();
        RenderTexture.active = null;

        // Save image
        byte[] png = captureTexture.EncodeToPNG();
        string imagePath = Path.Combine(outputPath, "images", $"frame_{sampleCount:D6}.png");
        File.WriteAllBytes(imagePath, png);

        // Generate and save labels (bounding boxes for humans)
        string labelPath = Path.Combine(outputPath, "labels", $"frame_{sampleCount:D6}.txt");
        GenerateLabels(labelPath);
    }

    void GenerateLabels(string path)
    {
        using var writer = new StreamWriter(path);
        foreach (var human in FindObjectsOfType<SimulatedHuman>())
        {
            Vector3 screenPos = captureCamera.WorldToViewportPoint(human.transform.position);
            if (screenPos.z > 0 && screenPos.x > 0 && screenPos.x < 1
                && screenPos.y > 0 && screenPos.y < 1)
            {
                // YOLO format: class x_center y_center width height (normalized)
                float bboxWidth = 0.1f;   // Approximate — use proper bounds in production
                float bboxHeight = 0.3f;
                writer.WriteLine($"0 {screenPos.x:F4} {screenPos.y:F4} {bboxWidth:F4} {bboxHeight:F4}");
            }
        }
    }
}
```

## Performance Optimization

Running photorealistic simulation at interactive frame rates requires optimization:

| Technique | Implementation | Impact |
|-----------|---------------|--------|
| **LOD Groups** | Reduce mesh detail at distance | 2-3x render performance |
| **Occlusion Culling** | Skip rendering hidden objects | 30-50% fewer draw calls |
| **Baked Lighting** | Pre-compute static light | Eliminates real-time light cost |
| **GPU Instancing** | Render duplicate objects in one call | Major gain for crowd scenes |
| **Fixed Timestep** | Decouple physics from render rate | Consistent simulation at any FPS |
| **Headless Mode** | Render only to texture (no display) | 2x throughput for data generation |

```bash
# Run Unity in headless batch mode for dataset generation
Unity -batchmode -nographics -executeMethod DatasetCollector.RunBatch -quit
```

---

## Exercises

### Exercise 1: Import and Visualize
Import your humanoid URDF into Unity using the URDF Importer. Apply PBR materials to give the robot a realistic metallic appearance. Verify that all joints articulate correctly by manually adjusting ArticulationBody drive targets in the Inspector.

### Exercise 2: ROS 2 Round-Trip
Connect Unity to ROS 2 via the TCP bridge. Publish joint states from Unity and subscribe to joint commands from a ROS 2 node. Verify round-trip communication by commanding the robot to wave from a Python script.

### Exercise 3: HRI Data Collection
Create a lobby environment with 3 animated human characters. Implement domain randomization for lighting and wall textures. Capture 1,000 labeled images of humans at varying distances from the robot. Verify label accuracy by visualizing bounding boxes on a sample of 10 images.

---

:::tip Key Takeaway
Unity extends your simulation capabilities beyond physics into the visual domain. For humanoid robots that must perceive, recognize, and interact with humans, Unity provides the photorealistic rendering and human simulation tools that Gazebo lacks. The key insight is that **both simulators connect through ROS 2** — your perception and control code does not change when you switch between them.
:::
