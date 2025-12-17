// @ts-check

/** * @type {import('@docusaurus/types').Config} */
async function createConfig() {
  // We use dynamic imports to handle ESM-only plugins in a CJS environment
  const remarkMath = (await import('remark-math')).default;
  const rehypeKatex = (await import('rehype-katex')).default;

  return {
    title: 'Physical AI & Humanoid Robotics',
    tagline: 'A University-Level Textbook on Physical AI, ROS 2, and Humanoid Systems',
    favicon: 'img/favicon.ico',

    url: 'https://123Mismail.github.io',
    baseUrl: '/physical-ai-humanoid-robotics/',

    organizationName: '123Mismail',
    projectName: 'physical-ai-humanoid-robotics',

    onBrokenLinks: 'ignore',
    trailingSlash: false,
    
    markdown: {
      mermaid: true,
      onBrokenMarkdownLinks: 'ignore',
      mdx1Compat: {
        comments: true,
        admonitions: true,
        headingIds: true,
      },
    },

    i18n: {
      defaultLocale: 'en',
      locales: ['en'],
    },

    presets: [
      [
        'classic',
        /** @type {import('@docusaurus/preset-classic').Options} */
        ({
          docs: {
            sidebarPath: require.resolve('./sidebars.js'),
            path: 'chapters',
            routeBasePath: 'chapters',
            // Corrected: Passing the imported functions directly
            remarkPlugins: [remarkMath],
            rehypePlugins: [[rehypeKatex, { strict: false }]],
          },
          theme: {
            customCss: require.resolve('./src/css/custom.css'),
          },
        }),
      ],
    ],

    stylesheets: [
      {
        href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
        type: 'text/css',
        integrity: 'sha384-n8MVd4RsNIU0tAv4ct0nTaAbDJwPJzDEaqSD1odI+WdtXRGWt2kTvGFasHpSy3SV',
        crossorigin: 'anonymous',
      },
    ],

    themeConfig:
      /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
      ({
        image: 'img/docusaurus-social-card.jpg',
        navbar: {
          title: 'Physical AI Textbook',
          logo: {
            alt: 'Physical AI Logo',
            src: 'img/logo.svg',
            href: '/',
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
                { label: 'Chapter 1: Foundations', to: '/chapters/c1-foundations-physical-ai' },
                { label: 'Chapter 2: ROS 2 Architecture', to: '/chapters/c2-ros2-architecture' },
              ],
            },
            {
              title: 'Community',
              items: [
                { label: 'GitHub', href: 'https://github.com/123Mismail/physical-ai-humanoid-robotics' },
              ],
            },
          ],
          copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook`,
        },
        prism: {
          theme: require('prism-react-renderer').themes.github,
          darkTheme: require('prism-react-renderer').themes.dracula,
          additionalLanguages: ['python', 'bash', 'yaml'],
        },
      }),
  };
}

module.exports = createConfig;