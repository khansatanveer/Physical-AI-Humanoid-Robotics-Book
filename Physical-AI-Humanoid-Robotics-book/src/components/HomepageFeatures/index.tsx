import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import FeatureCard from '@site/src/components/Animated/FeatureCard';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Concepts',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn fundamental concepts of Physical AI and embodied intelligence,
        understanding how AI systems interact with the physical world through robots.
      </>
    ),
  },
  {
    title: 'ROS 2 & Simulation',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master ROS 2, the Robot Operating System, and explore simulation environments
        like Gazebo, Unity, and NVIDIA Isaac for robotics development.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action Models',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Discover cutting-edge VLA models that integrate perception, reasoning,
        and action for advanced humanoid robotics applications.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4', 'margin-bottom--lg')}>
      <FeatureCard
        title={title}
        description={description}
        icon={<Svg className={styles.featureSvg} role="img" />}
        variant="floating"
      />
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
