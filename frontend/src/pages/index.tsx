import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";

const modules = [
  {
    title: "Module 1: The Robotic Nervous System",
    description:
      "Master ROS 2 fundamentals — nodes, topics, services, and actions that form the communication backbone of every modern robot.",
    link: "/chapters/module1/intro-to-ros2",
    icon: "🤖",
  },
  {
    title: "Module 2: The Digital Twin",
    description:
      "Build and test robots in Gazebo and Unity simulations before deploying to real hardware.",
    link: "/chapters/module2/intro-to-simulation",
    icon: "🌐",
  },
  {
    title: "Module 3: The AI-Robot Brain",
    description:
      "Leverage NVIDIA Isaac for GPU-accelerated perception, planning, and reinforcement learning on robotic platforms.",
    link: "/chapters/module3/intro-to-isaac",
    icon: "🧠",
  },
  {
    title: "Module 4: Vision-Language-Action",
    description:
      "Explore cutting-edge VLA models that unify vision, language understanding, and robotic action in a single architecture.",
    link: "/chapters/module4/intro-to-vla",
    icon: "👁️",
  },
];

function HeroBanner() {
  return (
    <header
      style={{
        padding: "4rem 2rem",
        textAlign: "center",
        background: "linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%)",
        color: "#fff",
      }}
    >
      <h1 style={{ fontSize: "2.8rem", marginBottom: "1rem", fontWeight: 800 }}>
        Physical AI &amp; Humanoid Robotics
      </h1>
      <p
        style={{
          fontSize: "1.25rem",
          maxWidth: 700,
          margin: "0 auto 2rem",
          opacity: 0.9,
          lineHeight: 1.6,
        }}
      >
        From Digital AI to Embodied Intelligence. A hands-on textbook covering
        ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action models.
      </p>
      <div style={{ display: "flex", gap: "1rem", justifyContent: "center", flexWrap: "wrap" }}>
        <Link
          to="/chapters/"
          style={{
            padding: "0.75rem 2rem",
            borderRadius: 8,
            background: "#25c2a0",
            color: "#fff",
            fontWeight: 600,
            fontSize: "1.05rem",
            textDecoration: "none",
          }}
        >
          Start Reading
        </Link>
        <Link
          to="/auth/signup"
          style={{
            padding: "0.75rem 2rem",
            borderRadius: 8,
            background: "transparent",
            border: "2px solid #25c2a0",
            color: "#25c2a0",
            fontWeight: 600,
            fontSize: "1.05rem",
            textDecoration: "none",
          }}
        >
          Sign Up Free
        </Link>
      </div>
    </header>
  );
}

function ModuleCard({ title, description, link, icon }: (typeof modules)[0]) {
  return (
    <Link
      to={link}
      style={{
        flex: "1 1 280px",
        maxWidth: 340,
        padding: "1.5rem",
        borderRadius: 12,
        border: "1px solid var(--ifm-color-emphasis-300)",
        textDecoration: "none",
        color: "inherit",
        transition: "box-shadow 0.2s, transform 0.2s",
      }}
      onMouseEnter={(e) => {
        (e.currentTarget as HTMLElement).style.boxShadow = "0 4px 20px rgba(0,0,0,0.12)";
        (e.currentTarget as HTMLElement).style.transform = "translateY(-2px)";
      }}
      onMouseLeave={(e) => {
        (e.currentTarget as HTMLElement).style.boxShadow = "none";
        (e.currentTarget as HTMLElement).style.transform = "none";
      }}
    >
      <div style={{ fontSize: "2rem", marginBottom: "0.75rem" }}>{icon}</div>
      <h3 style={{ fontSize: "1.1rem", marginBottom: "0.5rem" }}>{title}</h3>
      <p style={{ fontSize: "0.95rem", opacity: 0.8, margin: 0, lineHeight: 1.5 }}>
        {description}
      </p>
    </Link>
  );
}

function Features() {
  return (
    <section style={{ padding: "3rem 2rem", maxWidth: 1200, margin: "0 auto" }}>
      <h2 style={{ textAlign: "center", marginBottom: "2rem", fontSize: "1.8rem" }}>
        What You'll Learn
      </h2>
      <div
        style={{
          display: "flex",
          flexWrap: "wrap",
          gap: "1.5rem",
          justifyContent: "center",
        }}
      >
        {modules.map((mod) => (
          <ModuleCard key={mod.title} {...mod} />
        ))}
      </div>
    </section>
  );
}

function WhySection() {
  return (
    <section
      style={{
        padding: "3rem 2rem",
        background: "var(--ifm-color-emphasis-100)",
      }}
    >
      <div style={{ maxWidth: 800, margin: "0 auto", textAlign: "center" }}>
        <h2 style={{ fontSize: "1.8rem", marginBottom: "1.5rem" }}>
          Why This Textbook?
        </h2>
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "repeat(auto-fit, minmax(200px, 1fr))",
            gap: "1.5rem",
            textAlign: "left",
          }}
        >
          <div>
            <h4 style={{ marginBottom: "0.5rem" }}>Hands-On First</h4>
            <p style={{ fontSize: "0.95rem", opacity: 0.8 }}>
              Every chapter includes runnable code, simulation exercises, and real-world projects.
            </p>
          </div>
          <div>
            <h4 style={{ marginBottom: "0.5rem" }}>Personalized Path</h4>
            <p style={{ fontSize: "0.95rem", opacity: 0.8 }}>
              Take an assessment to get a learning path tailored to your experience level.
            </p>
          </div>
          <div>
            <h4 style={{ marginBottom: "0.5rem" }}>Track Progress</h4>
            <p style={{ fontSize: "0.95rem", opacity: 0.8 }}>
              Mark chapters complete, monitor your progress, and pick up where you left off.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  return (
    <Layout
      title="Home"
      description="Physical AI & Humanoid Robotics — From Digital AI to Embodied Intelligence"
    >
      <HeroBanner />
      <Features />
      <WhySection />
    </Layout>
  );
}
