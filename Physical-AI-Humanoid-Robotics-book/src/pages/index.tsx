import React, { JSX } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import AnimatedSection from '../components/Animated/AnimatedSection';
import AnimatedCard from '../components/Animated/AnimatedCard';
import FloatingLogo from '../components/Animated/FloatingLogo';
import Heading from '@theme/Heading';

import LogoSvg from '@site/static/img/logo.svg';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <AnimatedSection className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <div className="text--center">
              <FloatingLogo
                SvgComponent={LogoSvg}
                alt="Physical AI Humanoid Robotics Logo"
                size="lg"
                className="margin-bottom--lg"
              />
            </div>
          </div>
          <div className="col col--6">
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning - 10min read ⏱️
              </Link>
            </div>
          </div>
        </div>
      </div>
    </AnimatedSection>
  );
}

function HomepageFeaturesExtended() {
  const features = [
    {
      title: 'Physical AI Concepts',
      description: 'Learn fundamental concepts of Physical AI and embodied intelligence, understanding how AI systems interact with the physical world through robots.',
      icon: '🧠'
    },
    {
      title: 'ROS 2 & Simulation',
      description: 'Master ROS 2, the Robot Operating System, and explore simulation environments like Gazebo, Unity, and NVIDIA Isaac for robotics development.',
      icon: '🤖'
    },
    {
      title: 'Vision-Language-Action Models',
      description: 'Discover cutting-edge VLA models that integrate perception, reasoning, and action for advanced humanoid robotics applications.',
      icon: '👁️'
    }
  ];

  return (
    <AnimatedSection className={styles.features}>
      <div className="container">
        <div className={styles.featureGrid}>
          {features.map((feature, index) => (
            <div
              key={index}
              className={styles.featureCard}
            >
              <span className={styles.featureCardIcon}>{feature.icon}</span>
              <h3>{feature.title}</h3>
              <p>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </AnimatedSection>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive educational book about Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeaturesExtended />
        <section className={styles.ctaSection}>
          <div className="container padding-vert--xl text--center">
            <h2 className="padding-bottom--sm">Ready to dive deeper?</h2>
            <p className="padding-bottom--lg">
              Explore our comprehensive documentation and start building the future of robotics
            </p>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Explore Documentation
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}