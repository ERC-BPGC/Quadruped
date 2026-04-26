// @ts-check

const config = {
  title: 'ERC Quadruped',
  tagline: 'A research-oriented quadruped robotics platform by ERC-BPGC.',
  favicon: 'img/genesis-teaser.png',

  url: 'https://ERC-BPGC.github.io',
  baseUrl: '/Quadruped/',
  organizationName: 'ERC-BPGC',
  projectName: 'Quadruped',
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  future: {
    experimental_faster: {
      rspackBundler: true,
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: 'docs',
          editUrl: 'https://github.com/ERC-BPGC/Quadruped/tree/main/',
        },
        blog: {
          showReadingTime: true,
          routeBasePath: 'updates',
          onUntruncatedBlogPosts: 'ignore',
          editUrl: 'https://github.com/ERC-BPGC/Quadruped/tree/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/genesis-teaser.png',
      metadata: [
        {
          name: 'keywords',
          content:
            'quadruped robot, robotics, legged locomotion, ERC BPGC, controls, embedded systems',
        },
      ],
      navbar: {
        title: 'ERC Quadruped',
        logo: {
          alt: 'ERC Quadruped',
          src: 'img/genesis-teaser.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'projectSidebar',
            position: 'left',
            label: 'Docs',
          },
          { to: '/updates', label: 'Updates', position: 'left' },
          {
            href: 'https://github.com/ERC-BPGC/Quadruped',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Project',
            items: [
              { label: 'Overview', to: '/docs/intro' },
              { label: 'Roadmap', to: '/docs/roadmap' },
              { label: 'Contributing', to: '/docs/contribute' },
            ],
          },
          {
            title: 'Subsystems',
            items: [
              { label: 'Mechanical', to: '/docs/mechanical/overview' },
              { label: 'Electronics', to: '/docs/electronics/overview' },
              { label: 'Software', to: '/docs/software/overview' },
              { label: 'Controls', to: '/docs/controls/overview' },
            ],
          },
          {
            title: 'Links',
            items: [
              {
                label: 'Repository',
                href: 'https://github.com/ERC-BPGC/Quadruped',
              },
              {
                label: 'ERC-BPGC',
                href: 'https://github.com/ERC-BPGC',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} ERC-BPGC contributors.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;
