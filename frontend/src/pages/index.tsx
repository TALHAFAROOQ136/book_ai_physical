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
  },
  {
    title: 'Module 2: The Digital Twin',
    description:
      'Build and test robots in Gazebo and Unity simulations before deploying to real hardware.',
    link: '/chapters/module2/intro-to-simulation',
    icon: '\u{1F310}',
    num: 2,
  },
  {
    title: 'Module 3: The AI-Robot Brain',
    description:
      'Leverage NVIDIA Isaac for GPU-accelerated perception, planning, and reinforcement learning on robotic platforms.',
    link: '/chapters/module3/intro-to-isaac',
    icon: '\u{1F9E0}',
    num: 3,
  },
  {
    title: 'Module 4: Vision-Language-Action',
    description:
      'Explore cutting-edge VLA models that unify vision, language understanding, and robotic action in a single architecture.',
    link: '/chapters/module4/intro-to-vla',
    icon: '\u{1F441}\u{FE0F}',
    num: 4,
  },
];

const whyItems = [
  {
    icon: '\u{1F6E0}\u{FE0F}',
    title: 'Hands-On First',
    text: 'Every chapter includes runnable code, simulation exercises, and real-world projects.',
  },
  {
    icon: '\u{1F3AF}',
    title: 'Personalized Path',
    text: 'Take an assessment to get a learning path tailored to your experience level.',
  },
  {
    icon: '\u{1F4CA}',
    title: 'Track Progress',
    text: 'Mark chapters complete, monitor your progress, and pick up where you left off.',
  },
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
    </header>
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
}: {
  title: string;
  description: string;
  link: string;
  icon: string;
  num: number;
  index: number;
  inView: boolean;
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
      <div className={styles.cardHeader}>
        <div className={styles.cardNumber}>{num}</div>
        <span className={styles.cardIcon}>{icon}</span>
      </div>
      <h3 className={styles.cardTitle}>{title}</h3>
      <p className={styles.cardDescription}>{description}</p>
      <span className={styles.cardArrow}>Explore module &rarr;</span>
    </Link>
  );
}

function Features() {
  const { ref, inView } = useInView();

  return (
    <section className={styles.section} ref={ref}>
      <h2 className={styles.sectionTitle}>What You&apos;ll Learn</h2>
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
            >
              <span className={styles.whyIcon}>{item.icon}</span>
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
      <Features />
      <WhySection />
      <CtaBanner />
    </Layout>
  );
}
