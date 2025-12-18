# AI-Native Textbook for Physical AI & Humanoid Robotics

This AI-native textbook is designed for the competitive hackathon focused on creating AI-enhanced educational materials for Physical AI and Humanoid Robotics. The project integrates robotics simulation (ROS 2, Gazebo, NVIDIA Isaac), embodied intelligence, and large language models to create an interactive learning platform.

## About This Textbook

This textbook teaches Physical AI & Humanoid Robotics through:
- **Robotics Simulation**: Using ROS 2, Gazebo, and NVIDIA Isaac platforms
- **Embodied Intelligence**: Understanding how AI systems interact with physical environments
- **Vision-Language-Action Systems**: Building robots that can see, understand, and act
- **Humanoid Robotics**: Advanced systems that mimic human-like movement and interaction
- **AI-Enhanced Learning**: RAG-based chatbots, personalization, and multilingual support

## Installation

```bash
# Navigate to the project directory
cd physical-ai-book

# Install dependencies
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
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Project Structure

- `docs/` - Contains all textbook content organized by chapters
  - `chapter-1/` - Module 1: Foundations of Physical AI (perception-action loops, mathematical foundations, embodiment principles)
  - `chapter-2/` - Simulation and Control Systems (ROS 2, Gazebo, NVIDIA Isaac)
  - `chapter-3/` - Vision-Language-Action Integration
  - `chapter-4/` - Humanoid Systems and Locomotion
  - `chapter-5/` - Advanced Applications and Deployment
- `src/` - Custom React components for interactive learning
- `static/` - Static assets like images and documents

## Contributing

We welcome contributions to improve this AI-native textbook! Please see our [Contributing Guide](./docs/contributing.md) for detailed information on how to contribute content, technical improvements, or community support.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
