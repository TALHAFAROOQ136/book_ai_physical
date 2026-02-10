import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  textbookSidebar: [
    "intro",
    {
      type: "category",
      label: "Module 1: The Robotic Nervous System (ROS 2)",
      items: ["module1/intro-to-ros2"],
    },
    {
      type: "category",
      label: "Module 2: The Digital Twin (Gazebo & Unity)",
      items: ["module2/intro-to-simulation"],
    },
    {
      type: "category",
      label: "Module 3: The AI-Robot Brain (NVIDIA Isaac)",
      items: ["module3/intro-to-isaac"],
    },
    {
      type: "category",
      label: "Module 4: Vision-Language-Action (VLA)",
      items: ["module4/intro-to-vla"],
    },
  ],
};

export default sidebars;
