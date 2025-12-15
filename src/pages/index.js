import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/chapters/c1-foundations-physical-ai">
            Read the Book 📚
          </Link>
          <Link
            className="button button--outline button--lg"
            to="/chapters">
            Browse Chapters
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics: A University-Level Textbook on Physical AI, ROS 2, and Humanoid Systems">
      <HomepageHeader />
      <main>
        <section className={styles.featuresSection}>
          <div className="container">
            <div className="row">
              <div className="col col--6">
                <h2>About This Book</h2>
                <p>
                  This comprehensive textbook covers the foundations of Physical AI and humanoid robotics,
                  bridging the gap between theoretical concepts and practical implementation. Designed for
                  university-level students and professionals, it provides hands-on experience with modern
                  robotics frameworks and simulation environments.
                </p>
                <h3>Key Topics Include:</h3>
                <ul>
                  <li>Foundations of Physical AI and embodied intelligence</li>
                  <li>ROS 2 architecture, actions, and robot description (URDF)</li>
                  <li>Simulation environments (Gazebo, NVIDIA Isaac Sim, Unity)</li>
                  <li>Real-time control and sensor fusion</li>
                  <li>Humanoid integration and walking algorithms</li>
                </ul>
              </div>
              <div className="col col--6">
                <h2>Book Structure</h2>
                <div className={styles.modules}>
                  <div className={styles.module}>
                    <h3>Module 1: The Robotic Nervous System (ROS 2)</h3>
                    <ul>
                      <li>Foundations of Physical AI</li>
                      <li>ROS 2 Architecture</li>
                      <li>ROS 2 Actions</li>
                      <li>URDF Robot Description</li>
                    </ul>
                  </div>
                  <div className={styles.module}>
                    <h3>Module 2: Simulation Environments</h3>
                    <ul>
                      <li>Gazebo Simulation</li>
                      <li>NVIDIA Isaac Sim</li>
                      <li>Unity Simulation</li>
                      <li>Advanced Simulation Techniques</li>
                    </ul>
                  </div>
                  <div className={styles.module}>
                    <h3>Module 3: Edge Computing and Embedded Systems</h3>
                    <ul>
                      <li>Real-Time Control</li>
                      <li>Real-Time Algorithms</li>
                      <li>Sensor Fusion</li>
                    </ul>
                  </div>
                  <div className={styles.module}>
                    <h3>Module 4: Humanoid Integration</h3>
                    <ul>
                      <li>Whole Body Control</li>
                      <li>ZMP Walking</li>
                      <li>Humanoid Integration</li>
                    </ul>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}