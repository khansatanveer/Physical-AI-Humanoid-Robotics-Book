import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for the Physical AI Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'ROS 2',
      items: [
        'ros2/nodes-topics',
        'ros2/architecture',
        'ros2/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Fundamentals',
      items: [
        'fundamentals/kinematics',
        'fundamentals/sensors',
        'fundamentals/control-systems',
      ],
    },
    {
      type: 'category',
      label: 'Simulation',
      items: [
        'simulation/gazebo',
        'simulation/unity',
        'simulation/isaac',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin (Gazebo & Unity)',
      items: [
        'module-3-digital-twin/chapter-1-gazebo-physics',
        'module-3-digital-twin/chapter-1-exercises',
        'module-3-digital-twin/chapter-2-unity-visualization',
        'module-3-digital-twin/chapter-2-exercises',
        'module-3-digital-twin/chapter-3-sensor-simulation',
        'module-3-digital-twin/chapter-3-exercises',
        'module-3-digital-twin/workflow-sketch',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) Models',
      items: [
        'vla/introduction',
        'vla/integration',
      ],
    },
    {
      type: 'category',
      label: 'Projects',
      items: [
        'projects/first-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'advanced/future-directions',
      ],
    },
  ],
};

export default sidebars;
