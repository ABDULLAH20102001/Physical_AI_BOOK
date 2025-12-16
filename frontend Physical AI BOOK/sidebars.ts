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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: [
        'intro',
        {
          type: 'category',
          label: 'Foundation',
          items: [
            'introduction-to-physical-ai/intro-to-physical-ai',
            'humanoid-robots-physical-world/humanoid-robots-physical-world',
            'sensors-hardware-stack/sensors-hardware-stack'
          ],
        },
        {
          type: 'category',
          label: 'ROS 2 & Control',
          items: [
            'ros2-fundamentals/ros2-fundamentals',
            'ros2-python-control/ros2-python-control',
            'urdf-robot-description/urdf-robot-description'
          ],
        },
        {
          type: 'category',
          label: 'Simulation & Environment',
          items: [
            'gazebo-simulation-physics/gazebo-simulation-physics',
            'digital-twin-unity/digital-twin-unity',
            'nvidia-isaac-simulation/nvidia-isaac-simulation'
          ],
        },
        {
          type: 'category',
          label: 'Navigation & Mapping',
          items: [
            'navigation-slam-movement/navigation-slam-movement'
          ],
        },
        {
          type: 'category',
          label: 'AI & Perception',
          items: [
            'vision-language-action/vision-language-action'
          ],
        },
        {
          type: 'category',
          label: 'Capstone Project',
          items: [
            'capstone-autonomous-humanoid/capstone-autonomous-humanoid'
          ],
        }
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
