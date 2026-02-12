import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  textbookSidebar: [
    "intro",
    {
      type: "category",
      label: "Module 1: The Robotic Nervous System (ROS 2)",
      items: [
        "module1/intro-to-ros2",
        "module1/nodes-topics-services",
        "module1/rclpy-python-agents",
        "module1/urdf-humanoids",
      ],
    },
    {
      type: "category",
      label: "Module 2: The Digital Twin (Gazebo & Unity)",
      items: [
        "module2/intro-to-simulation",
        "module2/gazebo-physics",
        "module2/unity-hri",
        "module2/simulating-sensors",
      ],
    },
    {
      type: "category",
      label: "Module 3: The AI-Robot Brain (NVIDIA Isaac)",
      items: [
        "module3/intro-to-isaac",
        "module3/isaac-sim-synthetic-data",
        "module3/isaac-ros-vslam",
        "module3/nav2-bipedal",
      ],
    },
    {
      type: "category",
      label: "Module 4: Vision-Language-Action (VLA)",
      items: [
        "module4/intro-to-vla",
        "module4/voice-to-action",
        "module4/cognitive-planning",
        "module4/capstone-autonomous-humanoid",
      ],
    },
  ],
};

export default sidebars;
