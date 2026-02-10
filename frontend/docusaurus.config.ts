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
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ["bash", "python", "yaml", "xml"],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
