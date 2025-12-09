# Physical AI & Humanoid Robotics Textbook

Welcome to the comprehensive textbook for teaching Physical AI & Humanoid Robotics. This course bridges the gap between artificial intelligence and physical embodiment, preparing students to build autonomous robots that understand and interact with the physical world.

## 📚 Course Highlights

- **4 Core Modules** covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models
- **Practical Python Implementation** with real code examples
- **Hands-on Exercises** integrated throughout
- **Capstone Project**: Build an autonomous humanoid with voice commands
- **Deployed on Docusaurus** with interactive documentation

## 🚀 Quick Links

- **[Start Reading](/docs/physical-ai/intro)** - Begin with course introduction
- **[Chapter 1: Introduction to Physical AI](/docs/physical-ai/chapter-1-intro)** - Understand embodied intelligence
- **[Chapter 2: ROS 2 Architecture](/docs/physical-ai/chapter-2-ros2-architecture)** - Learn the robotic nervous system
- **[Capstone Project](/docs/physical-ai/capstone)** - Build your autonomous humanoid

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
