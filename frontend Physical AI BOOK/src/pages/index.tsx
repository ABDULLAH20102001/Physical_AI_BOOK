import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              Master Physical AI & Humanoid Robotics
            </Heading>
            <p className="hero__subtitle">
              The definitive guide to building intelligent physical systems that interact with the real world
            </p>
            <div className={styles.heroButtons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning Now
              </Link>
              <Link
                className="button button--primary button--outline button--lg"
                to="/docs/01-introduction-to-physical-ai/intro-to-physical-ai">
                Read the Book
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <div className={styles.robotVisual}></div>
          </div>
        </div>
      </div>
    </header>
  );
}

function AboutSection() {
  return (
    <section className={styles.section}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            About the Physical AI Book
          </Heading>
          <p className={styles.sectionSubtitle}>
            Dive deep into the intersection of artificial intelligence and physical systems
          </p>
        </div>
        <div className={styles.aboutGrid}>
          <div className={styles.aboutCard}>
            <h3>Real-World Applications</h3>
            <p>
              Learn how AI algorithms work in physical environments, from humanoid robots to autonomous vehicles,
              with hands-on projects that bridge the gap between simulation and reality.
            </p>
          </div>
          <div className={styles.aboutCard}>
            <h3>Advanced Robotics Concepts</h3>
            <p>
              Master cutting-edge techniques in sensor fusion, motor control, computer vision, and embodied AI
              that power the next generation of intelligent machines.
            </p>
          </div>
          <div className={styles.aboutCard}>
            <h3>Industry-Ready Skills</h3>
            <p>
              Develop expertise in ROS2, Gazebo simulation, Unity physics engine, and NVIDIA Isaac - the tools
              used by leading robotics companies worldwide.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function ValuePropositionSection() {
  return (
    <section className={clsx(styles.section, styles.darkSection)}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Why This Course Stands Apart
          </Heading>
          <p className={styles.sectionSubtitle}>
            Designed for the future of robotics and AI
          </p>
        </div>
        <div className={styles.valueGrid}>
          <div className={styles.valueItem}>
            <div className={styles.valueIcon}>🤖</div>
            <h3>Hands-On Learning</h3>
            <p>
              Build actual robots, not just simulations. Work with real hardware and deploy your AI models
              on physical platforms.
            </p>
          </div>
          <div className={styles.valueItem}>
            <div className={styles.valueIcon}>🏢</div>
            <h3>Industry Relevant</h3>
            <p>
              Curriculum designed with input from top robotics companies, ensuring you learn skills that
              matter in the job market.
            </p>
          </div>
          <div className={styles.valueItem}>
            <div className={styles.valueIcon}>🚀</div>
            <h3>Cutting-Edge Tech</h3>
            <p>
              Access to the latest tools, frameworks, and methodologies in Physical AI and humanoid robotics.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function CurriculumSection() {
  return (
    <section className={styles.section}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Comprehensive Curriculum
          </Heading>
          <p className={styles.sectionSubtitle}>
            From fundamentals to advanced applications
          </p>
        </div>
        <div className={styles.curriculumGrid}>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>01</span>
            <h3>Introduction to Physical AI</h3>
            <p>Fundamentals of AI in physical systems and embodied cognition</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>02</span>
            <h3>Humanoid Robots in the Physical World</h3>
            <p>Understanding locomotion, balance, and interaction dynamics</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>03</span>
            <h3>Sensors & Hardware Stack</h3>
            <p>LiDAR, cameras, IMUs, and sensor fusion techniques</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>04</span>
            <h3>ROS2 Fundamentals</h3>
            <p>Robot Operating System for distributed robotics</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>05</span>
            <h3>ROS2 Python Control</h3>
            <p>Controlling robots with Python and real-time systems</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>06</span>
            <h3>URDF Robot Description</h3>
            <p>Modeling robots in simulation and reality</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>07</span>
            <h3>Gazebo Simulation Physics</h3>
            <p>Physics engines and realistic robot simulation</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>08</span>
            <h3>Digital Twin & Unity</h3>
            <p>Unity-based digital twins for robot development</p>
          </div>
          <div className={styles.curriculumItem}>
            <span className={styles.curriculumIndex}>09</span>
            <h3>NVIDIA Isaac Simulation</h3>
            <p>Advanced GPU-accelerated physics simulation</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function TechStackSection() {
  return (
    <section className={clsx(styles.section, styles.darkSection)}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Advanced Technology Stack
          </Heading>
          <p className={styles.sectionSubtitle}>
            Master industry-standard tools and frameworks
          </p>
        </div>
        <div className={styles.techGrid}>
          <div className={styles.techItem}>
            <h3>AI & Machine Learning</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>TensorFlow</span>
              <span className={styles.techTag}>PyTorch</span>
              <span className={styles.techTag}>Reinforcement Learning</span>
              <span className={styles.techTag}>Computer Vision</span>
            </div>
          </div>
          <div className={styles.techItem}>
            <h3>Robotics Frameworks</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>ROS2</span>
              <span className={styles.techTag}>Gazebo</span>
              <span className={styles.techTag}>MoveIt!</span>
              <span className={styles.techTag}>OpenRAVE</span>
            </div>
          </div>
          <div className={styles.techItem}>
            <h3>Simulation Platforms</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>Unity</span>
              <span className={styles.techTag}>NVIDIA Isaac</span>
              <span className={styles.techTag}>Mujoco</span>
              <span className={styles.techTag}>PyBullet</span>
            </div>
          </div>
          <div className={styles.techItem}>
            <h3>Hardware Integration</h3>
            <div className={styles.techTags}>
              <span className={styles.techTag}>Arduino</span>
              <span className={styles.techTag}>Raspberry Pi</span>
              <span className={styles.techTag}>NVIDIA Jetson</span>
              <span className={styles.techTag}>RealSense</span>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CommunitySection() {
  return (
    <section className={styles.section}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Join Our Community
          </Heading>
          <p className={styles.sectionSubtitle}>
            Contribute, collaborate, and grow with fellow AI & robotics enthusiasts
          </p>
        </div>
        <div className={styles.communityGrid}>
          <div className={styles.communityCard}>
            <h3>Open Source</h3>
            <p>Contribute to cutting-edge robotics projects and enhance your portfolio</p>
            <Link className="button button--primary button--sm" to="https://github.com">
              Contribute Now
            </Link>
          </div>
          <div className={styles.communityCard}>
            <h3>Learning Groups</h3>
            <p>Join study groups and collaborative learning sessions</p>
            <Link className="button button--primary button--sm" to="/community">
              Find a Group
            </Link>
          </div>
          <div className={styles.communityCard}>
            <h3>Project Showcase</h3>
            <p>Share your creations and get feedback from experts</p>
            <Link className="button button--primary button--sm" to="/showcase">
              Share Your Work
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

function FinalCTASection() {
  return (
    <section className={clsx(styles.section, styles.ctaSection)}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaTitle}>
            Ready to Shape the Future of Robotics?
          </Heading>
          <p className={styles.ctaSubtitle}>
            Join thousands of students mastering Physical AI and building the next generation of intelligent machines
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning Today
            </Link>
            <Link
              className="button button--secondary button--outline button--lg"
              to="/contact">
              Contact Us
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics Course`}
      description="Master the intersection of artificial intelligence and physical systems">
      <HomepageHeader />
      <main>
        <AboutSection />
        <ValuePropositionSection />
        <CurriculumSection />
        <TechStackSection />
        <CommunitySection />
        <FinalCTASection />
      </main>
    </Layout>
  );
}
