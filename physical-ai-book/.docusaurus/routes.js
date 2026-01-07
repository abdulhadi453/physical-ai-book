import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/hackathon-book/__docusaurus/debug',
    component: ComponentCreator('/hackathon-book/__docusaurus/debug', 'c29'),
    exact: true
  },
  {
    path: '/hackathon-book/__docusaurus/debug/config',
    component: ComponentCreator('/hackathon-book/__docusaurus/debug/config', 'ead'),
    exact: true
  },
  {
    path: '/hackathon-book/__docusaurus/debug/content',
    component: ComponentCreator('/hackathon-book/__docusaurus/debug/content', '336'),
    exact: true
  },
  {
    path: '/hackathon-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/hackathon-book/__docusaurus/debug/globalData', 'bb6'),
    exact: true
  },
  {
    path: '/hackathon-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/hackathon-book/__docusaurus/debug/metadata', '644'),
    exact: true
  },
  {
    path: '/hackathon-book/__docusaurus/debug/registry',
    component: ComponentCreator('/hackathon-book/__docusaurus/debug/registry', 'e50'),
    exact: true
  },
  {
    path: '/hackathon-book/__docusaurus/debug/routes',
    component: ComponentCreator('/hackathon-book/__docusaurus/debug/routes', 'e1b'),
    exact: true
  },
  {
    path: '/hackathon-book/blog',
    component: ComponentCreator('/hackathon-book/blog', '99e'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/archive',
    component: ComponentCreator('/hackathon-book/blog/archive', 'e0e'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/authors',
    component: ComponentCreator('/hackathon-book/blog/authors', 'bc9'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/hackathon-book/blog/authors/all-sebastien-lorber-articles', '89a'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/authors/yangshun',
    component: ComponentCreator('/hackathon-book/blog/authors/yangshun', '113'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/first-blog-post',
    component: ComponentCreator('/hackathon-book/blog/first-blog-post', '3f3'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/long-blog-post',
    component: ComponentCreator('/hackathon-book/blog/long-blog-post', 'ba2'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/mdx-blog-post',
    component: ComponentCreator('/hackathon-book/blog/mdx-blog-post', '675'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/tags',
    component: ComponentCreator('/hackathon-book/blog/tags', '8cf'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/tags/docusaurus',
    component: ComponentCreator('/hackathon-book/blog/tags/docusaurus', '4bf'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/tags/facebook',
    component: ComponentCreator('/hackathon-book/blog/tags/facebook', 'eed'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/tags/hello',
    component: ComponentCreator('/hackathon-book/blog/tags/hello', 'f53'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/tags/hola',
    component: ComponentCreator('/hackathon-book/blog/tags/hola', 'b1c'),
    exact: true
  },
  {
    path: '/hackathon-book/blog/welcome',
    component: ComponentCreator('/hackathon-book/blog/welcome', 'ab1'),
    exact: true
  },
  {
    path: '/hackathon-book/markdown-page',
    component: ComponentCreator('/hackathon-book/markdown-page', '474'),
    exact: true
  },
  {
    path: '/hackathon-book/docs',
    component: ComponentCreator('/hackathon-book/docs', '506'),
    routes: [
      {
        path: '/hackathon-book/docs',
        component: ComponentCreator('/hackathon-book/docs', 'a48'),
        routes: [
          {
            path: '/hackathon-book/docs/tags',
            component: ComponentCreator('/hackathon-book/docs/tags', 'f74'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/actions',
            component: ComponentCreator('/hackathon-book/docs/tags/actions', 'b95'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/advanced',
            component: ComponentCreator('/hackathon-book/docs/tags/advanced', 'e4f'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/ai',
            component: ComponentCreator('/hackathon-book/docs/tags/ai', 'b48'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/ai-integration',
            component: ComponentCreator('/hackathon-book/docs/tags/ai-integration', 'da8'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/applications',
            component: ComponentCreator('/hackathon-book/docs/tags/applications', 'b4e'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/architecture',
            component: ComponentCreator('/hackathon-book/docs/tags/architecture', 'cc6'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/communication',
            component: ComponentCreator('/hackathon-book/docs/tags/communication', '14d'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/control',
            component: ComponentCreator('/hackathon-book/docs/tags/control', '903'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/data-flow',
            component: ComponentCreator('/hackathon-book/docs/tags/data-flow', 'ba2'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/humanoid',
            component: ComponentCreator('/hackathon-book/docs/tags/humanoid', '27d'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/integration',
            component: ComponentCreator('/hackathon-book/docs/tags/integration', 'fdc'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/middleware',
            component: ComponentCreator('/hackathon-book/docs/tags/middleware', 'ffe'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/nodes',
            component: ComponentCreator('/hackathon-book/docs/tags/nodes', 'd3c'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/python',
            component: ComponentCreator('/hackathon-book/docs/tags/python', '780'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/rclpy',
            component: ComponentCreator('/hackathon-book/docs/tags/rclpy', '8d3'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/robot-description',
            component: ComponentCreator('/hackathon-book/docs/tags/robot-description', 'c86'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/ros-2',
            component: ComponentCreator('/hackathon-book/docs/tags/ros-2', '144'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/services',
            component: ComponentCreator('/hackathon-book/docs/tags/services', '65a'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/topics',
            component: ComponentCreator('/hackathon-book/docs/tags/topics', '671'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/urdf',
            component: ComponentCreator('/hackathon-book/docs/tags/urdf', 'f78'),
            exact: true
          },
          {
            path: '/hackathon-book/docs/tags/xml',
            component: ComponentCreator('/hackathon-book/docs/tags/xml', '6a0'),
            exact: true
          },
          {
            path: '/hackathon-book/docs',
            component: ComponentCreator('/hackathon-book/docs', '75a'),
            routes: [
              {
                path: '/hackathon-book/docs/chapter-1/ai-integration',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/ai-integration', 'c46'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/educator-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/educator-guide', '9c4'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/intro',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/intro', '69c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-1',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-1', '593'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-1-perception-action',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-1-perception-action', '722'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-2',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-2', '4b5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-2-math-foundations',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-2-math-foundations', 'efd'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-3',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-3', '3fb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-3-embodiment',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-3-embodiment', '435'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-4',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-4', 'bdf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-5',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-5', '63c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-1/lesson-6',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/lesson-6', 'b1b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-1/module-assessment',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/module-assessment', '8e2'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/progress-tracking-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/progress-tracking-guide', '6f4'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/quality-validation',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/quality-validation', '060'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-1/researcher-resources',
                component: ComponentCreator('/hackathon-book/docs/chapter-1/researcher-resources', 'ae8'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-2/ai-integration',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/ai-integration', 'f48'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/assessment-framework',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/assessment-framework', '502'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-2/assessments',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/assessments', '625'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/cross-platform-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/cross-platform-guide', '42f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/educator-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/educator-guide', 'bcc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/exercise-1',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/exercise-1', 'aa4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/exercise-2',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/exercise-2', 'ea2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/exercise-3',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/exercise-3', '604'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/exercise-4',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/exercise-4', '160'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/exercise-framework',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/exercise-framework', '999'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-2/intro',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/intro', '92b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-1',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-1', 'bc6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-2',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-2', '1d4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-3',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-3', 'd56'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-4',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-4', 'f6f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-5',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-5', '36e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-6',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-6', '2da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-7',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-7', '9ad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-8',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-8', '9c1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/lesson-9',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/lesson-9', 'd6e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/platform-differences',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/platform-differences', '88e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/prerequisite-check',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/prerequisite-check', '41b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/progress-tracking-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/progress-tracking-guide', 'ca5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/quality-validation',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/quality-validation', '8ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/references',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/references', 'db5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/researcher-resources',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/researcher-resources', '867'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/sensor-assessments',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/sensor-assessments', 'a3b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/sensor-validation-tests',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/sensor-validation-tests', '40d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-2/tests',
                component: ComponentCreator('/hackathon-book/docs/chapter-2/tests', '1ce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/assessment',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/assessment', '26a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/assessments',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/assessments', 'b1f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/communication-protocols',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/communication-protocols', '7fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/educator-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/educator-guide', '027'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/exercises',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/exercises', '0e0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/intro',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/intro', 'fb6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/isaac-ros-setup',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/isaac-ros-setup', 'be4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/lesson-1',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/lesson-1', '048'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/lesson-2',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/lesson-2', '6e9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/lesson-3',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/lesson-3', '72d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/lesson-4',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/lesson-4', '601'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/model-deployment',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/model-deployment', 'beb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/nav2-setup',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/nav2-setup', '693'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/perception-pipeline',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/perception-pipeline', 'dee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/performance-monitoring',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/performance-monitoring', 'b4b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/progress-tracking-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/progress-tracking-guide', '08d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/researcher-resources',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/researcher-resources', '320'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/safety-framework',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/safety-framework', '132'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/sensor-integration',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/sensor-integration', '48f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/setup',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/setup', '2a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/simulation-setup',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/simulation-setup', 'f36'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-3/summary',
                component: ComponentCreator('/hackathon-book/docs/chapter-3/summary', '0b4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/assessments',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/assessments', 'ff4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/capstone-project',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/capstone-project', 'd31'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/educator-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/educator-guide', '3e9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/exercises',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/exercises', '6a0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/glossary',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/glossary', '416'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-4/integration-guide',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/integration-guide', '881'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/intro',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/intro', '522'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/lesson-1',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/lesson-1', '6a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/lesson-2',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/lesson-2', 'cbd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/lesson-3',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/lesson-3', 'd44'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/lesson-4',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/lesson-4', '7f1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/lesson-5',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/lesson-5', '33c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/performance-monitoring',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/performance-monitoring', '4aa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/reference',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/reference', '685'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-4/researcher-resources',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/researcher-resources', '391'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/security-considerations',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/security-considerations', 'fa0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/setup',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/setup', 'e82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/chapter-4/summary',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/summary', 'e2b'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/chapter-4/troubleshooting',
                component: ComponentCreator('/hackathon-book/docs/chapter-4/troubleshooting', '481'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/contributing',
                component: ComponentCreator('/hackathon-book/docs/contributing', '0b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/intro',
                component: ComponentCreator('/hackathon-book/docs/intro', '30c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/hackathon-book/docs/tutorial-extras/manage-docs-versions', '189'),
                exact: true
              },
              {
                path: '/hackathon-book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/hackathon-book/docs/tutorial-extras/translate-your-site', 'e86'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/hackathon-book/',
    component: ComponentCreator('/hackathon-book/', '32a'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
