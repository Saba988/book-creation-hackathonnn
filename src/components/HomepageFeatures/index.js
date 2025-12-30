import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Comprehensive Modules',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Four comprehensive modules covering everything from ROS 2 basics to advanced
        Vision-Language-Action robotics, providing a complete learning path for
        humanoid robotics development.
      </>
    ),
  },
  {
    title: 'Hands-on Learning',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Practical exercises and real-world applications across all modules. Learn by
        doing with simulation environments, AI integration, and robotics control systems.
      </>
    ),
  },
  {
    title: 'Cutting-Edge Technology',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Leverage state-of-the-art tools like NVIDIA Isaac, Gazebo, Unity, and
        Vision-Language-Action systems to build intelligent humanoid robots.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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
