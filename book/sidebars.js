// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Custom sidebar with explicit structure for better organization
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: ['intro', 'glossary'],
    },
    {
      type: 'category',
      label: 'Core Concepts',
      items: [
        'implementation-patterns',
        'spec-driven-development',
        'tools-technologies',
        'ai-features', // Our new AI features documentation
      ],
    },
    {
      type: 'category',
      label: 'Robotics Curriculum',
      items: [
        'module-1-ros2',
        'module-2-gazebo-unity',
        'module-3-isaac',
        'module-4-vla',
        'physical-ai-humanoid', // Our new curriculum documentation
      ],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'case-studies',
        'capstone-project',
        'appendix-hardware',
        'appendix-ros2-commands',
      ],
    },
    'references',
  ],
};

export default sidebars;
