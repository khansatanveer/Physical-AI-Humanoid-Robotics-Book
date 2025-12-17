# Physical AI Humanoid Robotics Book

This educational resource is built using [Docusaurus](https://docusaurus.io/), a modern static website generator, to provide a comprehensive guide to Physical AI and Humanoid Robotics.

## About This Book

This book serves as an educational resource for beginners to intermediate learners interested in humanoid robotics and AI. It covers fundamental concepts, ROS 2, simulation environments, and Vision-Language-Action models, with a focus on practical applications and hands-on learning.

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Book Structure

The book is organized into the following sections:
- Introduction to Physical AI and Humanoid Robotics
- Fundamentals (kinematics, sensors, control systems)
- ROS 2 (architecture, nodes, topics, URDF)
- Simulation (Gazebo, Unity, Isaac)
- Vision-Language-Action (VLA) Models
- Projects and practical applications
- Advanced topics and future directions

## Contributing

We welcome contributions to improve this educational resource. Please feel free to submit issues or pull requests to enhance the content, examples, or interactive elements.

## License

This educational content is made available under [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).
