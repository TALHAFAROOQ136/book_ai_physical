import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useInView } from '../hooks/useInView';
import styles from './index.module.css';

const modules = [
  {
    title: 'Module 1: The Robotic Nervous System',
    description:
      'Master ROS 2 fundamentals — nodes, topics, services, and actions that form the communication backbone of every modern robot.',
    link: '/chapters/module1/intro-to-ros2',
    icon: '\u{1F916}',
    num: 1,
    chapters: 4,
    techTags: ['ROS 2', 'Python', 'URDF'],
  },
  {
    title: 'Module 2: The Digital Twin',
    description:
      'Build and test robots in Gazebo and Unity simulations before deploying to real hardware.',
    link: '/chapters/module2/intro-to-simulation',
    icon: '\u{1F310}',
    num: 2,
    chapters: 4,
    techTags: ['Gazebo', 'Unity', 'Sensors'],
  },
  {
    title: 'Module 3: The AI-Robot Brain',
    description:
      'Leverage NVIDIA Isaac for GPU-accelerated perception, planning, and reinforcement learning on robotic platforms.',
    link: '/chapters/module3/intro-to-isaac',
    icon: '\u{1F9E0}',
    num: 3,
    chapters: 4,
    techTags: ['NVIDIA Isaac', 'VSLAM', 'Nav2'],
  },
  {
    title: 'Module 4: Vision-Language-Action',
    description:
      'Explore cutting-edge VLA models that unify vision, language understanding, and robotic action in a single architecture.',
    link: '/chapters/module4/intro-to-vla',
    icon: '\u{1F441}\u{FE0F}',
    num: 4,
    chapters: 4,
    techTags: ['VLA Models', 'LLMs', 'Multimodal'],
  },
];

const stats = [
  { value: '4', label: 'Core Modules' },
  { value: '16+', label: 'In-Depth Chapters' },
  { value: '5', label: 'Technologies' },
  { value: '100%', label: 'Hands-On' },
];

const whyItems = [
  {
    iconSymbol: '{ }',
    title: 'Hands-On First',
    text: 'Every chapter includes runnable code, simulation exercises, and real-world projects.',
    color: 'var(--module-1-color)',
  },
  {
    iconSymbol: '\u2794',
    title: 'Personalized Path',
    text: 'Take an assessment to get a learning path tailored to your experience level.',
    color: 'var(--module-2-color)',
  },
  {
    iconSymbol: '\u2713',
    title: 'Track Progress',
    text: 'Mark chapters complete, monitor your progress, and pick up where you left off.',
    color: 'var(--module-3-color)',
  },
];

const technologies = [
  { name: 'ROS 2', bg: '#22314e', text: '#fff', abbrev: 'ROS' },
  { name: 'Gazebo', bg: '#f58220', text: '#fff', abbrev: 'GZ' },
  { name: 'Unity', bg: '#222222', text: '#fff', abbrev: 'U3D' },
  { name: 'NVIDIA Isaac', bg: '#76b900', text: '#fff', abbrev: 'NV' },
  { name: 'Python', bg: '#3776ab', text: '#ffd43b', abbrev: 'Py' },
];

const staggerClass = [styles.stagger1, styles.stagger2, styles.stagger3, styles.stagger4];

function HeroBanner() {
  return (
    <header className={styles.hero}>
      <h1 className={styles.heroTitle}>
        Physical AI &amp; Humanoid Robotics
      </h1>
      <p className={styles.heroSubtitle}>
        From Digital AI to Embodied Intelligence. A hands-on textbook covering
        ROS&nbsp;2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action models.
      </p>
      <div className={styles.heroButtons}>
        <Link to="/chapters/" className={styles.btnPrimary}>
          Start Reading &rarr;
        </Link>
        <Link to="/auth/signup" className={styles.btnSecondary}>
          Sign Up Free
        </Link>
      </div>

      <div className={styles.heroDecorations} aria-hidden="true">
        <div className={clsx(styles.geoShape, styles.geoHexagon)} />
        <div className={clsx(styles.geoShape, styles.geoCircle)} />
        <div className={clsx(styles.geoShape, styles.geoRing)} />
        <div className={clsx(styles.geoShape, styles.geoDot1)} />
        <div className={clsx(styles.geoShape, styles.geoDot2)} />
        <div className={clsx(styles.geoShape, styles.geoDot3)} />
        <div className={clsx(styles.geoShape, styles.geoLine1)} />
        <div className={clsx(styles.geoShape, styles.geoLine2)} />
        <svg className={styles.geoCircuitSvg} viewBox="0 0 200 200" xmlns="http://www.w3.org/2000/svg">
          <path d="M10 100 H80 L100 80 H190" stroke="rgba(37,194,160,0.15)" strokeWidth="1" fill="none" />
          <path d="M10 130 H60 L80 150 H190" stroke="rgba(108,99,255,0.12)" strokeWidth="1" fill="none" />
          <circle cx="100" cy="80" r="3" fill="rgba(37,194,160,0.3)" />
          <circle cx="80" cy="150" r="3" fill="rgba(108,99,255,0.2)" />
        </svg>
      </div>
    </header>
  );
}

function StatsBar() {
  return (
    <section className={styles.statsBar}>
      <div className={styles.statsInner}>
        {stats.map((stat) => (
          <div key={stat.label} className={styles.statItem}>
            <span className={styles.statValue}>{stat.value}</span>
            <span className={styles.statLabel}>{stat.label}</span>
          </div>
        ))}
      </div>
    </section>
  );
}

function ModuleCard({
  title,
  description,
  link,
  icon,
  num,
  index,
  inView,
  chapters,
  techTags,
}: {
  title: string;
  description: string;
  link: string;
  icon: string;
  num: number;
  index: number;
  inView: boolean;
  chapters: number;
  techTags: string[];
}) {
  return (
    <Link
      to={link}
      className={clsx(
        styles.card,
        styles.animateHidden,
        inView && styles.animateVisible,
        staggerClass[index],
      )}
      style={{ '--card-accent': `var(--module-${num}-color)` } as React.CSSProperties}
    >
      <div className={styles.cardPattern} aria-hidden="true" />
      <div className={styles.cardHeader}>
        <div className={styles.cardIconContainer}>
          <span className={styles.cardIcon}>{icon}</span>
        </div>
        <div className={styles.cardNumber}>{num}</div>
      </div>
      <h3 className={styles.cardTitle}>{title}</h3>
      <p className={styles.cardDescription}>{description}</p>
      <div className={styles.cardTags}>
        {techTags.map((tag) => (
          <span key={tag} className={styles.cardTag}>{tag}</span>
        ))}
      </div>
      <div className={styles.cardFooter}>
        <span className={styles.cardChapterCount}>{chapters} chapters</span>
        <span className={styles.cardArrow}>Explore &rarr;</span>
      </div>
    </Link>
  );
}

function Features() {
  const { ref, inView } = useInView();

  return (
    <section className={styles.section} ref={ref}>
      <h2 className={styles.sectionTitle}>What You&apos;ll Learn</h2>
      <div className={styles.sectionDivider} />
      <p className={styles.sectionSubtitle}>
        Four progressive modules taking you from ROS fundamentals to cutting-edge VLA architectures.
      </p>
      <div className={styles.cardsGrid}>
        {modules.map((mod, i) => (
          <ModuleCard key={mod.title} {...mod} index={i} inView={inView} />
        ))}
      </div>
    </section>
  );
}

function Roadmap() {
  const { ref, inView } = useInView();

  return (
    <section className={styles.roadmapSection} ref={ref}>
      <h2 className={clsx(styles.sectionTitle, styles.animateHidden, inView && styles.animateVisible)}>
        Your Learning Journey
      </h2>
      <div className={styles.sectionDivider} />
      <p className={clsx(styles.sectionSubtitle, styles.animateHidden, inView && styles.animateVisible, styles.stagger1)}>
        A structured path from foundational robotics to cutting-edge embodied AI
      </p>
      <div className={styles.roadmap}>
        <div className={styles.roadmapLine} aria-hidden="true" />
        {modules.map((mod, i) => (
          <div
            key={mod.num}
            className={clsx(styles.roadmapNode, styles.animateHidden, inView && styles.animateVisible, staggerClass[i])}
            style={{ '--node-color': `var(--module-${mod.num}-color)` } as React.CSSProperties}
          >
            <div className={styles.roadmapDot}>
              <span className={styles.roadmapDotInner}>{mod.num}</span>
            </div>
            <div className={styles.roadmapContent}>
              <h4 className={styles.roadmapTitle}>
                {mod.title.replace(/Module \d: /, '')}
              </h4>
              <p className={styles.roadmapDesc}>
                {mod.description.length > 80
                  ? mod.description.slice(0, 80) + '...'
                  : mod.description}
              </p>
            </div>
          </div>
        ))}
      </div>
    </section>
  );
}

function TechStack() {
  return (
    <section className={styles.techSection}>
      <h2 className={styles.sectionTitle}>Technologies You&apos;ll Master</h2>
      <div className={styles.sectionDivider} />
      <div className={styles.techGrid}>
        {technologies.map((tech) => (
          <div
            key={tech.name}
            className={styles.techItem}
            style={{
              '--tech-bg': tech.bg,
              '--tech-text': tech.text,
            } as React.CSSProperties}
          >
            <div className={styles.techLogo}>
              <span className={styles.techAbbrev}>{tech.abbrev}</span>
            </div>
            <span className={styles.techName}>{tech.name}</span>
          </div>
        ))}
      </div>
    </section>
  );
}

function WhySection() {
  const { ref, inView } = useInView();

  return (
    <section className={styles.whySection} ref={ref}>
      <div className={styles.whyInner}>
        <h2
          className={clsx(
            styles.sectionTitle,
            styles.animateHidden,
            inView && styles.animateVisible,
          )}
        >
          Why This Textbook?
        </h2>
        <div className={styles.sectionDivider} />
        <div className={styles.whyGrid}>
          {whyItems.map((item, i) => (
            <div
              key={item.title}
              className={clsx(
                styles.whyCard,
                styles.animateHidden,
                inView && styles.animateVisible,
                staggerClass[i],
              )}
              style={{ '--why-accent': item.color } as React.CSSProperties}
            >
              <div className={styles.whyIconContainer}>
                <span className={styles.whyIconSymbol}>{item.iconSymbol}</span>
              </div>
              <h4 className={styles.whyCardTitle}>{item.title}</h4>
              <p className={styles.whyCardText}>{item.text}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CtaBanner() {
  const { ref, inView } = useInView();

  return (
    <section
      className={clsx(styles.ctaBanner, styles.animateHidden, inView && styles.animateVisible)}
      ref={ref}
    >
      <h2 className={styles.ctaTitle}>Ready to Build Intelligent Robots?</h2>
      <p className={styles.ctaText}>
        Start your journey from digital AI to physical embodied intelligence today.
      </p>
      <Link to="/chapters/" className={styles.btnPrimary}>
        Get Started &rarr;
      </Link>
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
      <StatsBar />
      <Features />
      <Roadmap />
      <TechStack />
      <WhySection />
      <CtaBanner />
    </Layout>
  );
}
