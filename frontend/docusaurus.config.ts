import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "From Digital AI to Embodied Intelligence",
  favicon: "img/favicon.ico",
  url: "https://your-site.github.io",
  baseUrl: "/",
  organizationName: "your-org",
  projectName: "physical-ai-textbook",
  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          routeBasePath: "chapters",
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    navbar: {
      title: "Physical AI Textbook",
      items: [
        {
          type: "docSidebar",
          sidebarId: "textbookSidebar",
          position: "left",
          label: "Chapters",
        },
        {
          to: "/dashboard",
          label: "Dashboard",
          position: "left",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Content",
          items: [
            { label: "Introduction", to: "/chapters/" },
            { label: "Module 1: ROS 2", to: "/chapters/module1/intro-to-ros2" },
            { label: "Module 2: Simulation", to: "/chapters/module2/intro-to-simulation" },
            { label: "Module 3: Isaac", to: "/chapters/module3/intro-to-isaac" },
            { label: "Module 4: VLA", to: "/chapters/module4/intro-to-vla" },
          ],
        },
        {
          title: "Resources",
          items: [
            { label: "Dashboard", to: "/dashboard" },
            { label: "Assessment", to: "/assessment" },
          ],
        },
        {
          title: "Community",
          items: [
            { label: "GitHub", href: "https://github.com/TALHAFAROOQ136" },
          ],
        },
      ],
      copyright: `Copyright \u00A9 ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ["bash", "python", "yaml"],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
