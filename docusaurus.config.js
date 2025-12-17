

/** @type {import('@docusaurus/types').Config} */
async function createConfig() {
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
   
    onBrokenMarkdownLinks: 'ignore', 
    
    trailingSlash: false,
    
    markdown: {
      mermaid: true,
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
            remarkPlugins: [remarkMath],
            rehypePlugins: [[rehypeKatex, { strict: false }]],
          },
          theme: {
            customCss: require.resolve('./src/css/custom-rtl.css'),
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
        navbar: {
          title: 'Physical AI Textbook',
          items: [
            { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Chapters' },
          ],
        },
        footer: {
          style: 'dark',
          copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook`,
        },
      }),
  };
}

module.exports = createConfig;