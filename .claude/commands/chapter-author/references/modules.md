# Module Reference

Quick-reference for module-specific context when generating chapters.

## Module 1: The Robotic Nervous System (ROS 2)

**Chapters**: 3-4 | **Languages**: Python (rclpy), XML (URDF/launch), YAML (config)

**Learning Objectives**:
- ROS 2 middleware architecture for robot control
- Nodes, topics, services, and actions
- Bridge Python AI agents to ROS controllers via rclpy
- URDF for humanoid robot descriptions

**Content Must Cover**:
- ROS 2 concepts with practical examples
- Step-by-step ROS 2 package creation
- Node communication code examples
- URDF modeling exercises for humanoid robots
- AI agent ↔ ROS 2 integration patterns

**Key Packages**: rclpy, sensor_msgs, geometry_msgs, tf2, urdf, launch_ros
**Tested On**: ROS 2 Humble, Ubuntu 22.04, Python 3.10+

---

## Module 2: The Digital Twin (Gazebo & Unity)

**Chapters**: 2-3 | **Languages**: Python, SDF/XML, C# (Unity)

**Learning Objectives**:
- Physics simulations (gravity, collision)
- High-fidelity rendering environments
- Sensor simulation (LiDAR, depth cameras, IMUs)
- Digital twin concepts

**Content Must Cover**:
- Gazebo setup and configuration
- Physics engine fundamentals
- Unity integration for visualization
- Sensor simulation tutorials
- Environment building and world creation

**Key Tools**: Gazebo Fortress/Garden, Unity 2022 LTS, gz-sim, ros_gz_bridge
**Tested On**: Ubuntu 22.04, NVIDIA RTX 4070 Ti+

---

## Module 3: The AI-Robot Brain (NVIDIA Isaac)

**Chapters**: 3-4 | **Languages**: Python, USD (Universal Scene Description)

**Learning Objectives**:
- NVIDIA Isaac Sim for photorealistic simulation
- Hardware-accelerated VSLAM and navigation
- Synthetic training data generation
- Path planning for bipedal humanoid movement

**Content Must Cover**:
- Isaac SDK installation and setup
- Isaac Sim workflows
- Isaac ROS perception pipelines
- Nav2 integration for humanoid navigation
- Sim-to-real transfer techniques

**Key Tools**: Isaac Sim 4.x, Isaac ROS, Nav2, nvblox, cuMotion
**Tested On**: Ubuntu 22.04, NVIDIA RTX 4070 Ti (12GB VRAM), Jetson Orin

---

## Module 4: Vision-Language-Action (VLA)

**Chapters**: 2-3 | **Languages**: Python

**Learning Objectives**:
- Voice commands via OpenAI Whisper
- LLMs for cognitive planning and NLU
- Natural language → ROS 2 action sequences
- End-to-end autonomous humanoid systems

**Content Must Cover**:
- Voice interface integration
- LLM-to-robot action translation
- Multi-modal interaction design
- Complete capstone project walkthrough

**Key Tools**: OpenAI Whisper, LangChain/LlamaIndex, ROS 2 action servers
**Security Note**: API keys MUST use .env pattern. Never hardcode.

---

## Special Chapters

### Introduction
- Physical AI landscape overview
- Course roadmap and learning paths
- Target audience guidance
- How to use this textbook

### Setup
- Hardware requirements (workstation + edge kit + robot options + cloud)
- Software installation guides (ROS 2, Gazebo, Isaac, Python env)
- Verification steps for each installation
- Troubleshooting common setup issues

### Capstone
- Autonomous humanoid with voice commands (integrates all 4 modules)
- Both simulation and hardware deployment paths
- Progressive build: skeleton → perception → planning → voice → integration

### Appendices
- Troubleshooting reference
- Glossary of terms
- Hardware purchasing guide
- Community resources
