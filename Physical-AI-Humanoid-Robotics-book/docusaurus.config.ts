import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI Humanoid Robotics',
  tagline: 'A comprehensive guide to Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics-book-gamma.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'physical-ai-humanoid-robotics-book', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/physical-ai-humanoid-robotics-book/physical-ai-humanoid-robotics-book/tree/main/Physical-AI-Humanoid-Robotics-book/docs/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/physical-ai-humanoid-robotics-book/physical-ai-humanoid-robotics-book/tree/main/Physical-AI-Humanoid-Robotics-book/blog/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Humanoid Robotics',
      logo: {
        alt: 'Physical AI Humanoid Robotics Logo',
        src: 'img/logo.svg',
        srcDark: 'img/logo.svg', // Add dark mode logo support
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/khansatanveer/Physical-AI-Humanoid-Robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
      // Mobile responsive settings
      hideOnScroll: false,
    },
    // Enable local search functionality
    algolia: undefined, // Disable Algolia search
    // For local search, no additional configuration is needed
    // Docusaurus will automatically provide search functionality
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Sections',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Fundamentals',
              to: '/docs/fundamentals/kinematics',
            },
            {
              label: 'ROS 2',
              to: '/docs/ros2/architecture',
            },
            {
              label: 'Simulation',
              to: '/docs/simulation/gazebo',
            },
            {
              label: 'Digital Twin',
              to: '/docs/module-3-digital-twin/chapter-1-gazebo-physics',
            },
            {
              label: 'VLA Models',
              to: '/docs/vla/introduction',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/physical-ai-humanoid-robotics-book/physical-ai-humanoid-robotics-book',
            },
            {
              label: 'Docusaurus',
              href: 'https://docusaurus.io',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/khansatanveer/Physical-AI-Humanoid-Robotics-bookk',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Humanoid Robotics Book. Built with Docusaurus.`,
    },
    // Accessibility and RAG chatbot compatibility settings
    metadata: [
      {name: 'keywords', content: 'physical ai, humanoid robotics, ros2, simulation, vla models, robotics education, ai robotics, embodied ai, machine learning, artificial intelligence'},
      {name: 'description', content: 'A comprehensive educational book about Physical AI and Humanoid Robotics, covering fundamental concepts, ROS 2, simulation environments, and Vision-Language-Action models.'},
      {name: 'og:title', content: 'Physical AI Humanoid Robotics Book'},
      {name: 'og:description', content: 'A comprehensive educational book about Physical AI and Humanoid Robotics'},
      {name: 'og:type', content: 'website'},
      {name: 'og:url', content: 'https://physical-ai-humanoid-robotics-book.github.io'},
      {name: 'og:site_name', content: 'Physical AI Humanoid Robotics Book'},
      {name: 'twitter:card', content: 'summary_large_image'},
      {name: 'twitter:title', content: 'Physical AI Humanoid Robotics Book'},
      {name: 'twitter:description', content: 'Educational resource for Physical AI and Humanoid Robotics'},
      // Additional metadata for RAG systems
      {name: 'docusaurus:version', content: 'current'},
      {name: 'docusaurus:section', content: 'education'},
      {name: 'docusaurus:category', content: 'robotics,ai,education'},
    ],
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,


  // Add plugin to handle Node.js core modules for Webpack 5
  plugins: [
    // Add webpack configuration to handle Node.js core modules
    async function myPlugin(context, options) {
      return {
        name: 'webpack-config-plugin',
        configureWebpack(config, isServer) {
          return {
            resolve: {
              fallback: {
                path: require.resolve('path-browserify'),
                fs: false, // Disable fs since it's not available in browsers
                os: require.resolve('os-browserify/browser'),
                crypto: require.resolve('crypto-browserify'),
                stream: require.resolve('stream-browserify'),
                util: require.resolve('util'),
                buffer: require.resolve('buffer'),
                process: require.resolve('process/browser'),
                assert: require.resolve('assert'),
                constants: require.resolve('constants-browserify'),
                tty: require.resolve('tty-browserify'),
                url: require.resolve('url/'),
                vm: require.resolve('vm-browserify'),
                zlib: require.resolve('browserify-zlib'),
                dns: false, // Disable dns since it's not available in browsers
                net: false, // Disable net since it's not available in browsers
                tls: false, // Disable tls since it's not available in browsers
                child_process: false, // Disable child_process since it's not available in browsers
                readline: false, // Disable readline since it's not available in browsers
                perf_hooks: false, // Disable perf_hooks since it's not available in browsers
                v8: false, // Disable v8 since it's not available in browsers
                module: false, // Disable module since it's not available in browsers
                http: require.resolve('stream-http'),
                https: require.resolve('https-browserify'),
                querystring: require.resolve('querystring-es3'),
                string_decoder: require.resolve('string_decoder/'),
                punycode: require.resolve('punycode/'),
                domain: require.resolve('domain-browser'),
                events: require.resolve('events/'),
                async_hooks: false, // Disable async_hooks since it's not available in browsers
              },
            },
            plugins: [
              ...(config.plugins || []),
              // Add DefinePlugin to ensure process is available in browser
              new (require('webpack').DefinePlugin)({
                'process.env.NODE_ENV': JSON.stringify(process.env.NODE_ENV || 'development'),
              }),
            ],
          };
        },
      };
    },
  ],
};

export default config;
