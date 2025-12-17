# Quickstart Guide: The AI-Robot Brain (NVIDIA Isaac)

**Feature**: 004-isaac-robot
**Date**: 2025-12-15

## Overview

This quickstart guide provides the essential steps to begin working with the NVIDIA Isaac educational module. It covers the basic setup needed to run Isaac Sim, Isaac ROS pipelines, and Nav2 navigation examples.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 LTS or Windows 10/11 with WSL2
- **CPU**: Multi-core processor (Intel i7 or equivalent recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA GPU with CUDA support (RTX 2060 or better recommended)
- **Storage**: 50GB free space for Isaac Sim and dependencies

### Software Dependencies
- NVIDIA GPU drivers (latest version)
- CUDA Toolkit 11.8 or later
- Docker and Docker Compose
- Python 3.8 or later
- Git

## Installation Steps

### 1. Install NVIDIA Isaac Sim
```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow installation instructions for your platform
# Verify installation:
isaac-sim --version
```

### 2. Set up Isaac ROS Workspace
```bash
# Clone Isaac ROS repositories
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark.git

# Build the workspace
cd isaac_ros_workspace
colcon build
source install/setup.bash
```

### 3. Install Navigation2 (Nav2)
```bash
# Install Nav2 from source or package
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 4. Set up Docusaurus Documentation
```bash
# Navigate to project root
cd Physical-AI-Humanoid-Robotics-Book

# Install dependencies
npm install

# Start documentation server
npm start
```

## First Run: Basic Isaac Sim Scene

### 1. Launch Isaac Sim
```bash
# Start Isaac Sim application
isaac-sim
```

### 2. Create Basic Scene
- Open Isaac Sim
- Create a new scene with basic objects
- Add a simple robot model
- Configure basic sensors

### 3. Run Simulation
- Press play to start the simulation
- Observe the basic robot behavior
- Verify sensors are collecting data

## Running the Examples

### Isaac Sim Examples
```bash
# Navigate to Isaac Sim examples
cd isaac-sim-examples/basic-simulation

# Run basic simulation example
python basic_simulation.py
```

### VSLAM Pipeline Example
```bash
# Navigate to VSLAM examples
cd isaac-ros-examples/vslam-basics

# Run VSLAM pipeline
python vslam_pipeline.py
```

### Nav2 Navigation Example
```bash
# Launch Nav2 navigation
ros2 launch nav2_bringup navigation_launch.py
```

## Common Issues and Solutions

### GPU Memory Issues
- Reduce scene complexity for lower-end GPUs
- Close other GPU-intensive applications
- Adjust Isaac Sim quality settings

### ROS Communication Issues
- Verify ROS_DOMAIN_ID is consistent across terminals
- Check network configuration for multi-machine setups
- Ensure all required packages are sourced

### Docusaurus Build Issues
- Clear npm cache: `npm cache clean --force`
- Delete node_modules and reinstall: `rm -rf node_modules && npm install`
- Check Node.js version compatibility

## Next Steps

1. Complete the Isaac Sim chapter to learn photorealistic simulation
2. Proceed to Isaac ROS chapter for VSLAM and navigation
3. Explore Nav2 chapter for advanced path planning
4. Integrate everything in the complete AI navigation loop chapter

## Getting Help

- Check the troubleshooting section of each chapter
- Review the FAQ in the documentation
- Join the community forums for additional support