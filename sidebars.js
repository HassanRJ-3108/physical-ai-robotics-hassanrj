/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapter 1: Physical AI Fundamentals',
      collapsed: false,
      items: [
        'chapter-01-physical-ai/index',
        'chapter-01-physical-ai/core-principles',
        'chapter-01-physical-ai/embodiment',
        'chapter-01-physical-ai/sensory-perception',
        'chapter-01-physical-ai/motor-action',
        'chapter-01-physical-ai/learning-from-interaction',
        'chapter-01-physical-ai/autonomy',
        'chapter-01-physical-ai/digital-to-physical',
        'chapter-01-physical-ai/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS 2',
      collapsed: true,
      items: [
        'chapter-02-ros2/index',
        'chapter-02-ros2/core-architecture',
        'chapter-02-ros2/dds',
        'chapter-02-ros2/nodes',
        'chapter-02-ros2/ros-graph',
        'chapter-02-ros2/communication-patterns',
        'chapter-02-ros2/topics',
        'chapter-02-ros2/services',
        'chapter-02-ros2/actions',
        'chapter-02-ros2/python-integration',
        'chapter-02-ros2/urdf',
        'chapter-02-ros2/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Robot Simulation',
      collapsed: true,
      items: [
        'chapter-03-simulation/index',
        'chapter-03-simulation/platforms',
        'chapter-03-simulation/gazebo',
        'chapter-03-simulation/unity',
        'chapter-03-simulation/sensor-simulation',
        'chapter-03-simulation/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: NVIDIA Isaac Platform',
      collapsed: true,
      items: [
        'chapter-04-nvidia-isaac/index',
        'chapter-04-nvidia-isaac/isaac-ros',
        'chapter-04-nvidia-isaac/isaac-sim',
        'chapter-04-nvidia-isaac/sim-to-real',
        'chapter-04-nvidia-isaac/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Vision-Language-Action',
      collapsed: true,
      items: [
        'chapter-05-vla/index',
        'chapter-05-vla/vla-pipeline',
        'chapter-05-vla/llms-in-robotics',
        'chapter-05-vla/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 6: Humanoid Robotics',
      collapsed: true,
      items: [
        'chapter-06-humanoid/index',
        'chapter-06-humanoid/kinematics-dynamics',
        'chapter-06-humanoid/bipedal-locomotion',
        'chapter-06-humanoid/manipulation',
        'chapter-06-humanoid/hri',
        'chapter-06-humanoid/conclusion',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 7: Hardware Setup',
      collapsed: true,
      items: [
        'chapter-07-hardware/index',
        'chapter-07-hardware/workstation',
        'chapter-07-hardware/edge-computing',
        'chapter-07-hardware/sensors-actuators',
        'chapter-07-hardware/lab-infrastructure',
        'chapter-07-hardware/conclusion',
      ],
    },
  ],
  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Tutorial',
      items: ['hello'],
    },
  ],
   */
};

module.exports = sidebars;
