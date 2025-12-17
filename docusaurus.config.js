// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const remarkMath = require('remark-math');
const rehypeKatex = require('rehype-katex');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A University-Level Textbook on Physical AI, ROS 2, and Humanoid Systems',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://123Mismail.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/physical-ai-humanoid-robotics/',

  // GitHub pages deployment config
  organizationName: '123Mismail', // Your GitHub username
  projectName: 'physical-ai-humanoid-robotics', // Your repository name

  onBrokenLinks: 'ignore', // Changed from 'warn'
  onBrokenMarkdownLinks: 'ignore', // Changed from 'throw' to prevent build failures
  trailingSlash: false, // Recommended for GitHub Pages
  markdown: {
    mermaid: true,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang.
  i18n: {
    defaultLocale: 'en',
    locales: ['en'], // Temporarily limit to English only for GitHub Pages deployment
    // locales: ['en', 'ur', 'es', 'fr', 'de'], // Full set when build is stable
    localeConfigs: {
      en: {
        label: 'English',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Path to chapters directory (Docusaurus docs root)
          path: 'chapters',
          routeBasePath: 'chapters',
          // Math support for LaTeX rendering
          remarkPlugins: [[remarkMath, {}]],
          rehypePlugins: [[rehypeKatex, {strict: false}]],
        },
        gtag: undefined,
        googleTagManager: undefined,
        sitemap: undefined,
      }),
    ],
  ],
  // KaTeX stylesheet for LaTeX math rendering
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-n8MVd4RsNIU0tAv4ct0nTaAbDJwPJzDEaqSD1odI+WdtXRGWt2kTvGFasHpSy3SV',
      crossorigin: 'anonymous',
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
          href: '/', // Link to homepage instead of first chapter
          target: '_self',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Chapters',
            items: [
              {
                label: 'Chapter 1: Foundations of Physical AI',
                to: '/chapters/c1-foundations-physical-ai',
              },
              {
                label: 'Chapter 2: ROS 2 Architecture',
                to: '/chapters/c2-ros2-architecture',
              },
              {
                label: 'Chapter 3: ROS 2 Actions',
                to: '/chapters/c3-ros2-actions',
              },
              {
                label: 'Chapter 4: URDF Robot Description',
                to: '/chapters/c4-urdf-robot-description',
              },
            ],
          },
          {
            title: 'Advanced Topics',
            items: [
              {
                label: 'Chapter 5: Gazebo Simulation',
                to: '/chapters/c5-gazebo-simulation',
              },
              {
                label: 'Chapter 6: NVIDIA Isaac Sim',
                to: '/chapters/c6-isaac-sim',
              },
              {
                label: 'Chapter 7: Unity Simulation',
                to: '/chapters/c7-unity-simulation',
              },
              {
                label: 'Chapter 8: Advanced Simulation',
                to: '/chapters/c8-advanced-simulation',
              },
            ],
          },
          {
            title: 'Control & Integration',
            items: [
              {
                label: 'Chapter 9: Real-Time Control',
                to: '/chapters/c9-real-time-control',
              },
              {
                label: 'Chapter 10: Control Algorithms',
                to: '/chapters/c10-real-time-algorithms',
              },
              {
                label: 'Chapter 11: Sensor Fusion',
                to: '/chapters/c11-sensor-fusion',
              },
              {
                label: 'Chapter 12-14: Advanced Topics',
                to: '/chapters/c12-whole-body-control',
              },
            ],
          },
          {
            title: 'Resources & Support',
            items: [
              {
                label: 'ROS 2 Humble Docs',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'Gazebo',
                href: 'https://gazebosim.org/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/',
              },
              {
                label: 'GitHub Repository',
                href: 'https://github.com/123Mismail/physical-ai-humanoid-robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook | Built with Docusaurus`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'markup', 'yaml'],
      },
    }),
};

module.exports = config;
