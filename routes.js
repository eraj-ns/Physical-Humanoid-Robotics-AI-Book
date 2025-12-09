import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'fbe'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'b14'),
        routes: [
          {
            path: '/docs/tags',
            component: ComponentCreator('/docs/tags', 'fce'),
            exact: true
          },
          {
            path: '/docs/tags/collisions',
            component: ComponentCreator('/docs/tags/collisions', '691'),
            exact: true
          },
          {
            path: '/docs/tags/digital-twin',
            component: ComponentCreator('/docs/tags/digital-twin', 'd18'),
            exact: true
          },
          {
            path: '/docs/tags/gazebo',
            component: ComponentCreator('/docs/tags/gazebo', 'a4c'),
            exact: true
          },
          {
            path: '/docs/tags/physics',
            component: ComponentCreator('/docs/tags/physics', '134'),
            exact: true
          },
          {
            path: '/docs/tags/ros-2',
            component: ComponentCreator('/docs/tags/ros-2', '361'),
            exact: true
          },
          {
            path: '/docs/tags/sdf',
            component: ComponentCreator('/docs/tags/sdf', '2fe'),
            exact: true
          },
          {
            path: '/docs/tags/urdf',
            component: ComponentCreator('/docs/tags/urdf', 'f09'),
            exact: true
          },
          {
            path: '/docs',
            component: ComponentCreator('/docs', '252'),
            routes: [
              {
                path: '/docs/Module1/',
                component: ComponentCreator('/docs/Module1/', '5f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module1/ch01-ros2-architecture',
                component: ComponentCreator('/docs/Module1/ch01-ros2-architecture', 'eb1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module1/ch02-nodes-topics-services',
                component: ComponentCreator('/docs/Module1/ch02-nodes-topics-services', '4e0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module1/ch03-python-agents-rclpy',
                component: ComponentCreator('/docs/Module1/ch03-python-agents-rclpy', 'e03'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module1/ch04-urdf-for-humanoids',
                component: ComponentCreator('/docs/Module1/ch04-urdf-for-humanoids', '9b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module1/intro',
                component: ComponentCreator('/docs/Module1/intro', '1d2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module2/',
                component: ComponentCreator('/docs/Module2/', '312'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module2/ch01-gazebo-physics-and-collisions',
                component: ComponentCreator('/docs/Module2/ch01-gazebo-physics-and-collisions', '4e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module2/ch02-digital-twin-environment-design',
                component: ComponentCreator('/docs/Module2/ch02-digital-twin-environment-design', '576'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module2/intro',
                component: ComponentCreator('/docs/Module2/intro', '97b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module3/',
                component: ComponentCreator('/docs/Module3/', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module3/Introduction-to-AI-Robot-Brain',
                component: ComponentCreator('/docs/Module3/Introduction-to-AI-Robot-Brain', '344'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module3/Isaac-ROS-VSLAM-and-Nav2-Planning',
                component: ComponentCreator('/docs/Module3/Isaac-ROS-VSLAM-and-Nav2-Planning', '47f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module3/Isaac-Sim-Simulation',
                component: ComponentCreator('/docs/Module3/Isaac-Sim-Simulation', 'ebb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module4/',
                component: ComponentCreator('/docs/Module4/', 'f0f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module4/ch01-voice-to-action',
                component: ComponentCreator('/docs/Module4/ch01-voice-to-action', 'be2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module4/ch02-llm-cognitive-planning',
                component: ComponentCreator('/docs/Module4/ch02-llm-cognitive-planning', '9d9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module4/ch03-vision-and-navigation',
                component: ComponentCreator('/docs/Module4/ch03-vision-and-navigation', '152'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module4/ch04-capstone-autonomous-humanoid',
                component: ComponentCreator('/docs/Module4/ch04-capstone-autonomous-humanoid', '5b3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module4/intro',
                component: ComponentCreator('/docs/Module4/intro', 'da8'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
